# Camera Feeder 設計提案：讓 ROS 與 WebRTC 共用同一顆 camera

狀態：**提案**（2026-10-07）
範圍：`SyncAI-Robot-Workspace`、`SyncAI-WebRTC-Worker`、`SyncAI-Robot-Backend`，以及本 repo

## 1. 問題

機器人上的 `/dev/syncai/camera0`（TechNexion VCS-AR0234-C）有兩個都想串流的使用者：

- **WebRTC worker**（`SyncAI-WebRTC-Worker`，由 backend 以 `dlopen` 載入）：操作員的即時畫面。
- **ROS camera driver**（`vizionsdk_ros2`，由 `syncai_bringup` 啟動）：發布 `image_raw/compressed`。

V4L2 的 capture device 同時只允許**一個** streaming opener。第二個開啟的會在 `REQBUFS` / `STREAMON` 拿到 `EBUSY`，表現出來是：

- worker 這邊：`POST /api/v1/webrtc/whep` 回 502，內容是 GStreamer 的錯誤句子，沒有 `code`，前端分不出是 device busy 還是別的 pipeline 錯誤（`SyncAI-Robot-Backend/README.md` 的 WebRTC 段已記錄）。
- ROS 這邊：`VxStartStreaming failed`，節點在 constructor 就 throw。

backend 的 `WebRtcGateway` 只在**自己的** `whep` / `whip` / `duplex` session 之間做 device 搶佔，管不到別的 process。這份提案要解決的是跨 process 的共用。

## 2. 現況盤點

以下是從三個 repo 讀出來的事實，提案建立在這些事實上。

### 2.1 Camera 硬體

- Jetson 上有**兩顆不同型號**的 camera：usb-1 的 **VCS-AR0234-C**（`/dev/syncai/camera0`）和 usb-2 的 **VCI-AR0144-C**（`/dev/syncai/camera1`，最高 1280x800）。
- `/dev/video<N>` 的編號不穩定（每顆兩個 node，probe 順序決定誰拿到哪個），穩定的是 `syncai_sys_manager` 的 udev rule 建出來的 `/dev/syncai/*` symlink。
- AR0234 在 720p 只提供 **60 和 120 fps**，沒有 30；AR0144 是 30 和 60。兩顆都接受的只有 60。
- UVC 的 ISP 狀態（曝光、白平衡、gain…）**存在 camera firmware 裡，開關 device 不會重置**。誰最後寫，畫面就長誰的樣子。

### 2.2 WebRTC worker（`origin/feat/robot`，`b0f4126`）

> 注意：本機 checkout 的 `feat/robot` 落後 origin 兩個 commit，還是寫死 `/dev/syncai_camera` 的舊版。以下以 origin 為準。

- `internal/proc/video.go` 的 `BuildVideoPipelineString`：
  ```
  v4l2src device=$VIDEO_DEVICE io-mode=2 extra-controls="c,auto_exposure=…,white_balance_automatic=0,white_balance_temperature=5200,…" do-timestamp=true
    ! image/jpeg,width=1280,height=720,framerate=60/1
    ! queue leaky=downstream ! nvjpegdec ! video/x-raw(memory:NVMM)
    ! nvvidconv ! nvv4l2h264enc … ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=$VIDEO_RTP_PORT
  ```
- `internal/config/config.go` 的 `VideoConfig`：`VIDEO_DEVICE`（預設 `/dev/syncai/camera0`）、`VIDEO_FORMAT`（`mjpeg` / `uyvy`）、`VIDEO_WIDTH/HEIGHT/FRAMERATE`（1280x720@60）、`VIDEO_BITRATE`，以及一整組 sensor control（`VIDEO_AUTO_WB=0`、`VIDEO_WB_TEMP=5200`、`VIDEO_POWER_LINE_FREQ=1` 等，附實測理由）。
- pipeline 是**每個 WHEP session 建一條**，session 結束就釋放 camera；沒有 session 時 camera 是空的。
- worker 內部本來就用 `127.0.0.1` 上的 RTP（`udpsink` → `udpsrc`）把 GStreamer 接到 pion。

### 2.3 ROS camera driver（`SyncAI-Robot-Workspace/src/third-party/vizionsdk-ros2`）

- 上游 TechNexion 的 vendor package，透過閉源 VizionSDK 開 camera：`VxDiscoverCameraDevices` → `VxInitialCameraDevice(device_index)` → `VxOpen` → `VxIsVizionCamera`（`src/vizionsdk_camera_node.cpp:114-132`）。SDK 掃 `/sys/class/video4linux` 找 TechNexion 的實體裝置，**不是一般的 V4L2 client，讀不了虛擬 device**。
- `syncai_bringup/launch/bringup.launch.py`：`use_camera` 預設 **false**，`camera_device_index` 預設 0（= AR0234 = camera0）。header 註解明寫：預設關閉是為了不跟 host 上的 `scripts/publish_camera_crop.sh` 搶 `/dev/video0`。
- `syncai_bringup/params/bringup.yaml`（`/**/vizionsdk_camera`）：`publish_imu: false`（這顆不支援 IMU，開了會 throw 再 segfault）、`publish_image: true`、`MJPG 1280x720@60`、`isp.whitebalance_mode: 1`（**自動白平衡**）。intrinsics 也讀不到，所以不發 `camera_info`。
- 在這顆 camera 上，這個節點**實際只提供兩樣東西：image 和 ISP 參數**。
- 同一個 yaml 記錄了第三個搶 device 的：host 上 `~/controller/src/controller/HTML_joy_controller.py` 用 OpenCV 開 `/dev/video0`。
- 影像 stamp 用 `this->now()`，不是曝光時間。
- workspace 內**沒有找到任何訂閱 `image_raw/compressed` 的節點**。

### 2.4 ISP 已經在打架

| 來源 | 白平衡設定 |
|---|---|
| `bringup.yaml` | `isp.whitebalance_mode: 1`（自動） |
| worker `config.go` | `VIDEO_AUTO_WB=0` + `VIDEO_WB_TEMP=5200`（手動，實測值） |

兩邊寫的是同一顆 firmware 裡的同一個 control。就算 device 能共用，畫面顏色也取決於誰最後啟動。這點跟 device 衝突無關，但任何共用方案都要順手解掉。

### 2.5 兩個 container 的共同點

- 都是 **host network**（DDS 走 `lo`），所以 `127.0.0.1` 上的 UDP 跨 container 可達。
- 都有 **`ipc: host`**，共用 host 的 `/dev/shm`。
- backend 的 compose 用 nvidia runtime 注入 Tegra GStreamer plugin（`nvjpegdec` / `nvvidconv` / `nvv4l2h264enc`）；硬體 JPEG 解碼和 H.264 編碼都在 worker 這邊。

## 3. 先決問題：ROS 真的需要 camera0 的影像嗎？

這個問題決定要不要做後面的任何事，目前**沒有答案**。

| 情況 | 該做什麼 |
|---|---|
| ROS 沒有任何東西訂閱 `image_raw/compressed`（目前 workspace 內的狀況） | **什麼都不用做。** `use_camera` 維持 `false`，WebRTC 獨占 camera0。開 image 當初是 IMU 方案的副產品，而 IMU 不能用 |
| ROS 需要影像，但不在乎是哪顆鏡頭 | ROS 用 `camera_device_index:=1`（AR0144，`bringup.yaml` 註明 MJPG 可用），零程式改動 |
| ROS 需要 **camera0** 的影像，且要和 WebRTC **同時** | 才需要第 5 節的設計 |

> 待決：請在 §10 填上誰要用這個影像、做什麼用、需要多少 fps。

## 4. 方案比較

前提：兩個 consumer 都需要 camera0 同時串流。所有可行方案都是同一個形狀 ——**一個 owner 開 device，其餘 consumer 從 owner 分流**。差別在 owner 是誰、分流走什麼。

| 方案 | 做法 | 優點 | 缺點 |
|---|---|---|---|
| A. 分開用兩顆 camera | ROS 用 camera1 | 零改動 | 不符合前提（需要同一顆） |
| B. `v4l2loopback` | host 上一條 feeder 寫進虛擬 `/dev/videoN`，consumer 當普通 V4L2 device 讀 | consumer 幾乎不用改 | **`vizionsdk_ros2` 讀不了虛擬 device**（§2.3），ROS 端還是得換節點；要在 L4T 上裝 kernel module（DKMS、JetPack 升級重編）；要 host 上的 systemd service + udev rule；owner 重啟時 consumer 要重開 |
| C. `shmsink` / `shmsrc` | owner 用 GStreamer `shmsink` 寫 `/dev/shm`，consumer 用 `shmsrc` | userspace，`ipc: host` 已經有 | `shmsrc` 沒有 caps 協商要手寫；**shm 滿了會 block 所有人**，一個卡住的 reader 拖垮另一個；socket 消失時 consumer 要重建 |
| D. **localhost RTP/JPEG**（建議） | owner 用 `tee` 一路 `rtpjpegpay ! udpsink 127.0.0.1`，worker 用 `udpsrc ! rtpjpegdepay` | userspace；consumer 彼此獨立（UDP 不等人）；owner 重啟 consumer 不用重建；worker 本來就在用這個模式 | worker 要多一個 source 分支；RTP/JPEG 有格式限制（§8）；多一跳延遲（約一個 frame） |
| E. worker 當 owner | worker 常駐開 camera，分流給 ROS | 硬體解碼已在 worker | worker 的 pipeline 是 per-session 的，要大改成常駐；backend 是 operator-facing process，不該變成 sensor driver；worker 沒有 ROS，ROS 端還是要一個 bridge |

**選 D。** 理由集中在三點：不碰 kernel、consumer 互不拖累、owner 重啟後 WebRTC 自己恢復（`switch_mode` 重建 byobu session 的那幾秒就是這個情況）。

## 5. 建議設計

### 5.1 資料流

```
robot container（syncai_bringup window 0）
  camera feeder 節點 = /dev/syncai/camera0 的唯一 opener
    v4l2src device=/dev/syncai/camera0 io-mode=2 extra-controls="<ISP>" do-timestamp=true
      ! image/jpeg,width=1280,height=720,framerate=60/1
      ! tee name=t
          t. ! queue leaky=downstream ! rtpjpegpay ! udpsink host=127.0.0.1 port=5008   → WebRTC worker
          t. ! queue leaky=downstream ! [videorate ! image/jpeg,framerate=N/1] ! appsink → image_raw/compressed

backend container
  WebRTC worker（每個 WHEP session）
    udpsrc port=5008 caps="application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000"
      ! rtpjitterbuffer ! rtpjpegdepay
      ! queue leaky=downstream ! nvjpegdec ! video/x-raw(memory:NVMM)
      ! nvvidconv ! nvv4l2h264enc … ! h264parse ! rtph264pay ! udpsink …   （nvjpegdec 之後完全不變）
```

重點：

- **全程 MJPEG，不解碼。** feeder 只是搬 JPEG bytes；唯一的解碼仍是 worker 的 `nvjpegdec`（硬體）。ROS 分支發 `CompressedImage`（`format: jpeg`），不經 CPU。
- **解析度全系統一組**（1280x720），由 feeder 決定。**fps 可以分開**：WebRTC 分支維持 60，ROS 分支可用 `videorate` 降到需要的值。
- **ISP 只在 feeder 寫一次**，用 worker 那組實測值（AE 自動、AWB 手動 5200 K、50 Hz 抗閃爍…）。worker 的 `rtpjpeg` 分支不套 `extra-controls`，`bringup.yaml` 的 `isp.*` 區塊退役。
- **port 命名**：worker 現有 `AUDIO_RTP_PORT=5004`、`VIDEO_RTP_PORT=5006`、`VIDEO_AUDIO_RTP_PORT=5007` 是它內部用的；新增 `VIDEO_RTP_IN_PORT=5008` 作為 camera 進來的 port，兩邊 compose 都要對齊。
- 兩個 consumer 互不影響：ROS 端 subscriber 再慢，`udpsink` 不等人；worker 沒有 session 時封包就丟掉，不佔資源。

### 5.2 本 repo 的角色

`SyncAI-Robot-Camera-Feeder` 放的是 **feeder 節點本身**：一個 ROS 2 package（暫名 `syncai_camera_feeder`），和 `syncai_backend` 一樣以 vcs-import 進 workspace 的 `src/`，由 `syncai_bringup` 啟動。

節點實作有兩條路，建議先走第一條：

| | (1) 包 `gscam2` | (2) 自寫節點（Python + `python3-gst-1.0`） |
|---|---|---|
| 程式量 | 幾乎沒有：launch + params，pipeline 字串放 `gscam_config`，`tee` 寫在字串裡 | 約 150 行 |
| 依賴 | `gscam2` + `ros2_shared`，Humble 的 apt 可能沒有，要進 `third-party.repos` 從原始碼編 | 只有 apt 的 `python3-gst-1.0`、`gstreamer1.0-plugins-good`（已有） |
| JPEG 直通 | `image_encoding: jpeg` 直接發 `CompressedImage`（**待確認 gscam2 保留了這個模式**） | 自己掌握 |
| 線上調 ISP | 無 | 可做成 ROS 參數 → `v4l2-ctl` |
| 風險 | 第三方打包、JPEG 模式不如預期 | 程式歸我們維護 |

決定點：(1) 的打包和 JPEG 直通在 robot 上驗證通過就用 (1)；任一不通就直接做 (2)，不要花時間修 gscam2。

### 5.3 ROS 介面（維持相容）

- topic：`<robot_id>/image_raw/compressed`（`sensor_msgs/CompressedImage`，`format: jpeg`），和現在 `vizionsdk_ros2` 發的一樣。
- `frame_id`：`<robot_id>/camera_optical_frame`，照 bringup 現在的做法由 launch 帶 `robot_id`。
- `camera_info`：不發（現在也沒有，intrinsics 讀不到）。
- stamp：節點收到 frame 的時間。精度和現在的 `this->now()` 相同。

## 6. 各 repo 的改動

### 6.1 本 repo（新）

- `syncai_camera_feeder` package：`package.xml`、launch、params（device、解析度、fps、ISP controls、RTP port、ROS 分支 fps）。
- README 記錄：為什麼是 RTP 不是 loopback、ISP 值的來源（引用 worker `config.go` 的量測註解）、port 表。

### 6.2 `SyncAI-Robot-Workspace`

- `third-party.repos` / 一個新的 repos 檔：加入本 repo（以及 `gscam2`，若走 §5.2 的 (1)）。
- `Dockerfile`：補 GStreamer 相關 apt（`gstreamer1.0-plugins-good` 的 `rtpjpegpay`、`v4l2src`；走 (2) 則加 `python3-gst-1.0`）。VizionSDK 的 `.deb` 安裝段可以保留給 camera1 或未來支援 IMU 的模組，但 bringup 不再預設啟動 `vizionsdk_camera_node`。
- `syncai_bringup/launch/bringup.launch.py`：`use_camera` 改成啟動 feeder 節點，預設翻成 **true**（原本預設關閉的理由 —— 跟 `publish_camera_crop.sh` 搶 device —— 在這個設計下消失），加 `respawn=True`。
- `syncai_bringup/params/bringup.yaml`：`vizionsdk_camera` 區塊退役，換成 feeder 的參數。
- `scripts/publish_camera_crop.sh`：退役（§8 第 1 點）。

### 6.3 `SyncAI-WebRTC-Worker`

- `internal/config/config.go`：`VIDEO_FORMAT` 新增 `rtpjpeg`；新增 `VIDEO_RTP_IN_PORT`（預設 5008）。
- `internal/proc/video.go`：`BuildVideoPipelineString` 第三個 source 分支（§5.1），不套 `extra-controls`。
- `internal/proc/video_test.go`：pipeline 字串的測試。
- README：source 分支說明；`mjpeg` / `uyvy` 兩個直接開 device 的分支保留，給沒有 feeder 的部署。

### 6.4 `SyncAI-Robot-Backend`

- `docker-compose.yml`：環境變數 `VIDEO_FORMAT=rtpjpeg`、`VIDEO_RTP_IN_PORT=5008`；**移除** `/dev/video0..3`、`/dev/syncai` 的 passthrough 和 `VIDEO_GID`（backend 不再碰 camera）。nvidia runtime 保留（`nvjpegdec` / `nvv4l2h264enc` 還在這邊）。
- `README.md`：WebRTC 段改寫「camera 被別人占用是 502」那段為新架構；部署段的 `--device` 說明對應更新。
- `CLAUDE.md`：WebRTC 一段補一句 camera 來源。
- Python 程式**不用改**：`gateways/webrtc` 的 slot 搶佔邏輯不變（camera slot 現在代表「這條 RTP 的 consumer」，語意仍成立）。

### 6.5 `syncai_sys_manager`

- 不用動。

## 7. 實施順序

1. **回答 §3。** 答案是「不需要」或「不挑鏡頭」就到此為止。
2. **在 robot 上手動驗證**（不改任何 repo）：
   - 停掉 `publish_camera_crop.sh` 和搖桿 UI，`fuser -v /dev/video*` 確認沒人占著。
   - `gst-launch-1.0` 跑 §5.1 的 feeder 字串（`tee` 到 `udpsink` + `fakesink`）。
   - 另一個 shell 跑 worker 那段 `udpsrc … ! rtpjpegdepay ! nvjpegdec ! … ! fakesink`，確認 `nvjpegdec` 吃得下 depay 出來的 JPEG（§8 第 2 點）。
   - 殺掉 feeder 再重啟，確認 `udpsrc` 那條會自己恢復畫面。
   - 若走 gscam2：建起來，確認 `image_encoding: jpeg` 真的發 `CompressedImage` 且 CPU 沒有明顯上升。
3. **worker**：`rtpjpeg` 分支 + 測試。獨立、最確定要做，可以先動。
4. **本 repo**：feeder package。
5. **workspace**：repos、Dockerfile、bringup、yaml、退役 script。
6. **backend**：compose、README、CLAUDE.md。
7. 整合測試：WHEP 開著的同時 `ros2 topic hz image_raw/compressed`；`switch_mode` 一次，確認 WebRTC 畫面在 bringup 回來後自己恢復。

## 8. 風險與驗證清單

1. **另外兩個搶 device 的要先清掉。** host 的 `scripts/publish_camera_crop.sh` 和 `~/controller/.../HTML_joy_controller.py`（OpenCV 開 `/dev/video0`）。owner 模型只有在它們都停掉時才成立；任何一個還在跑，feeder 就會像現在的 `vizionsdk_ros2` 一樣啟動失敗。
2. **RTP/JPEG（RFC 2435）的格式限制。** 只支援 baseline JPEG、邊長 ≤ 2040（1280x720 沒問題）。UVC 的 MJPEG 通常不帶 Huffman table，`rtpjpegpay` 會略過、`rtpjpegdepay` 補回標準表 —— **要實測 `nvjpegdec` 接受補表後的串流**。若不行，退路是 feeder 用 `jpegparse` 先正規化，或改走 §4 的 C。
3. **feeder 是單點。** 它一掛，ROS 和 WebRTC 一起沒畫面。緩解：launch `respawn=True`；worker 的 `udpsrc` 分支不會因為沒封包而出錯，feeder 回來就恢復；ROS subscriber 本來就能容忍 topic 中斷。和現在相比不算變差 —— `vizionsdk_ros2` 掛了也不會自己恢復。
4. **mode switch 期間 WebRTC 會黑掉幾秒。** `switch_mode` 砍掉 byobu session 重建，feeder 跟著重啟。現在 worker 直接開 device 沒有這個現象。接受這個代價，換來 owner 在對的 container 裡。
5. **UDP 在 `lo` 上的掉包。** 720p@60 的 MJPEG 約 50–100 Mbit/s，每 frame 70 個左右 1400 B 的封包，`lo` 承受得住；`udpsrc` 設 `buffer-size` 保險。backend `CLAUDE.md` 記錄的 UDP 緩衝溢位是 16–45 MB 的單一 PointCloud2，和這裡量級不同。
6. **gscam2 的打包與 JPEG 直通**（若走 §5.2 的 (1)）。Humble 的 apt 有沒有要查；`image_encoding: jpeg` 要確認沒有先解碼再壓縮，否則 Jetson CPU 跑 720p@60 的軟體 JPEG 會很重。
7. **失去 `vizionsdk_ros2` 的功能**：`ros2 param set …/vizionsdk_camera isp.*` 線上調參、`camera/status` 診斷 topic、vendor 擴充的 eHDR / denoise / flip。今天只用了 `isp.whitebalance_mode` 一項，等於沒有損失；IMU 和 intrinsics 這顆本來就讀不到。若日後換成支援 IMU 的模組，VizionSDK 只讀 IMU、不開串流，理論上能和 feeder 共存，**未驗證**。
8. **延遲**：多一跳約一個 frame（60 fps 下 ≈ 17 ms）。stamp 精度與現在相同。

## 9. 跟原本 `v4l2loopback` 構想的差異

這份提案取代了稍早口頭討論的 `v4l2loopback` 方案。改變的原因是後來查到的三個事實：

1. `vizionsdk_ros2` 讀不了虛擬 device（§2.3），所以「兩邊都讀 loopback、誰都不用改」不成立，ROS 端無論如何要換節點。
2. 兩個 container 都是 host network 且共用 `/dev/shm`（§2.5），userspace 的分流在現有部署下就能跑。
3. worker 內部本來就用 `127.0.0.1` 的 RTP 接 pipeline，`udpsrc` 分支是它既有模式的延伸，不是新概念。

既然 ROS 端一定要換節點，loopback 剩下的唯一好處（consumer 不用改）就消失了，而它的成本（kernel module、host service、udev rule）全在。

## 10. 待決事項

| # | 問題 | 誰決定 | 影響 |
|---|---|---|---|
| 1 | ROS 端誰要用 `image_raw/compressed`？做什麼？需要幾 fps？ | — | 決定要不要做這整件事（§3） |
| 2 | `publish_camera_crop.sh` 和搖桿 UI 還在用嗎？ | — | §8 第 1 點 |
| 3 | feeder 走 gscam2 還是自寫節點？ | 驗證後定 | §5.2 |
| 4 | ISP 採用 worker 的實測值（AWB 手動 5200 K）還是自動白平衡？ | — | 建議採 worker 的值，理由在 worker `config.go` 的量測註解 |
| 5 | `VIDEO_RTP_IN_PORT` 定 5008？ | — | 兩邊 compose 對齊即可 |
