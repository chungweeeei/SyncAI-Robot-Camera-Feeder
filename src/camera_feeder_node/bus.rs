use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::JoinHandle;

use gstreamer as gst;
use gstreamer::prelude::*;
use rclrs::*;

/// How long each bus poll blocks before the watcher re-checks whether it should stop. Short enough
/// that dropping the node is not perceptibly delayed, long enough not to spin.
const POLL_INTERVAL: gst::ClockTime = gst::ClockTime::from_mseconds(200);

const WARNING_THROTTLE: std::time::Duration = std::time::Duration::from_secs(5);

/// Watches the pipeline bus and ends the process on a fatal error.
///
/// Most of what goes wrong with a capture pipeline goes wrong *after* `set_state(Playing)` has
/// returned: a device another process already holds (`EBUSY`), a size/rate pair the sensor does
/// not advertise (`not-negotiated`), the camera unplugged mid-stream. Those arrive only as ERROR
/// messages on the bus.
///
/// On ERROR or EOS the process exits non-zero rather than trying to rebuild the pipeline in place:
/// the launch file's `respawn=True` already restarts the node, and a fresh process is the one
/// restart path that is certain to have released the device. Polling on its own thread rather
/// than `Bus::add_watch`, which needs a GLib main loop this process does not run.
pub struct BusWatch {
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    logger: Logger,
}

impl BusWatch {
    pub fn start(pipeline: gst::Pipeline, logger: Logger) -> Result<Self, std::io::Error> {
        let stop = Arc::new(AtomicBool::new(false));
        let stop_flag = Arc::clone(&stop);
        let thread = std::thread::Builder::new()
            .name("gst-bus".to_string())
            .spawn({
                let logger = logger.clone();
                move || watch(&pipeline, &logger, &stop_flag)
            })?;
        Ok(Self {
            stop,
            thread: Some(thread),
            logger,
        })
    }
}

impl Drop for BusWatch {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(thread) = self.thread.take() {
            if thread.join().is_err() {
                log_error!(
                    &self.logger,
                    "[BusWatch] the watcher thread panicked; errors after that went unreported"
                );
            }
        }
    }
}

fn watch(pipeline: &gst::Pipeline, logger: &Logger, stop: &AtomicBool) {
    let Some(bus) = pipeline.bus() else {
        log_error!(
            logger,
            "[BusWatch] pipeline has no bus; errors will go unreported"
        );
        return;
    };

    while !stop.load(Ordering::Relaxed) {
        let Some(message) = bus.timed_pop_filtered(
            POLL_INTERVAL,
            &[
                gst::MessageType::Error,
                gst::MessageType::Warning,
                gst::MessageType::Eos,
            ],
        ) else {
            continue;
        };

        let source = message
            .src()
            .map_or_else(|| "<unknown>".into(), |s| s.path_string());
        match message.view() {
            gst::MessageView::Warning(warning) => {
                // Throttled: a warning about the stream (rtpjpegpay rejecting a frame, say) repeats
                // for every frame, 60 times a second.
                log_warn!(
                    logger.throttle(WARNING_THROTTLE),
                    "[BusWatch] warning from {source}: {} ({})",
                    warning.error(),
                    warning.debug().as_deref().unwrap_or("no debug info"),
                );
            }
            gst::MessageView::Error(error) => {
                log_error!(
                    logger,
                    "[BusWatch] error from {source}: {} ({})",
                    error.error(),
                    error.debug().as_deref().unwrap_or("no debug info"),
                );
                exit(pipeline, logger);
            }
            gst::MessageView::Eos(_) => {
                log_error!(logger, "[BusWatch] end of stream: capture has stopped");
                exit(pipeline, logger);
            }
            _ => {}
        }
    }
}

/// Releases the device, then ends the process for `respawn` to restart.
fn exit(pipeline: &gst::Pipeline, logger: &Logger) -> ! {
    // NULL first: it is what closes the V4L2 fd. The process exit would close it too, but this way
    // the camera is free before the respawned node tries to open it.
    if let Err(err) = pipeline.set_state(gst::State::Null) {
        log_error!(logger, "[BusWatch] pipeline did not reach NULL: {err}");
    }
    log_error!(
        logger,
        "[BusWatch] exiting so the launch file can respawn the node"
    );
    std::process::exit(1);
}
