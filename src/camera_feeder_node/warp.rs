//! Warping I420 planes by a homography: bilinear inverse mapping, one destination row per rayon
//! task.
//!
//! The homography changes every frame (it carries the frame's own rotation), so there is no
//! lookup table to cache; instead the projective coordinates of a destination row are linear in
//! the column, and the inner loop is three additions, one division and one bilinear fetch per
//! pixel.
//!
//! Pure: plain byte slices in and out, so the kernel is tested on synthetic images.

use rayon::prelude::*;

use super::math::Mat3;

/// A read-only 8-bit plane. `stride` is the distance between rows in bytes (>= `width`).
#[derive(Clone, Copy, Debug)]
pub struct Plane<'a> {
    pub data: &'a [u8],
    pub width: usize,
    pub height: usize,
    pub stride: usize,
}

/// A writable 8-bit plane.
#[derive(Debug)]
pub struct PlaneMut<'a> {
    pub data: &'a mut [u8],
    pub width: usize,
    pub height: usize,
    pub stride: usize,
}

/// I420 black: luma 0 and neutral chroma.
pub const FILL_Y: u8 = 0;
pub const FILL_UV: u8 = 128;

/// Fills `dst` from `src` through `inverse`, which maps DESTINATION pixel centres to SOURCE pixel
/// centres (pixel `(u, v)` has its centre at `(u + 0.5, v + 0.5)`). Destination pixels whose
/// source lies outside the plane, or behind the camera, get `fill`.
pub fn warp_plane(src: &Plane<'_>, dst: &mut PlaneMut<'_>, inverse: &Mat3, fill: u8) {
    let (dst_width, dst_height, dst_stride) = (dst.width, dst.height, dst.stride);
    if dst_width == 0 || dst_height == 0 {
        return;
    }
    let src = *src;
    let m = inverse.0;
    dst.data
        .par_chunks_mut(dst_stride)
        .take(dst_height)
        .enumerate()
        .for_each(|(v, row)| {
            let row = &mut row[..dst_width];
            let vc = v as f64 + 0.5;
            // The projective coordinates at the row's first pixel centre, and their change per
            // column (the first column of the matrix).
            let mut x = m[0][0] * 0.5 + m[0][1] * vc + m[0][2];
            let mut y = m[1][0] * 0.5 + m[1][1] * vc + m[1][2];
            let mut w = m[2][0] * 0.5 + m[2][1] * vc + m[2][2];
            for out in row.iter_mut() {
                *out = sample(&src, x, y, w, fill);
                x += m[0][0];
                y += m[1][0];
                w += m[2][0];
            }
        });
}

/// The bilinear sample at the projective point `(x, y, w)`, in pixel-centre coordinates.
#[inline]
fn sample(src: &Plane<'_>, x: f64, y: f64, w: f64, fill: u8) -> u8 {
    if src.width < 2 || src.height < 2 || w <= 0.0 {
        return fill;
    }
    // Back to the pixel grid: pixel i covers [i, i + 1), its centre is i + 0.5.
    let sx = x / w - 0.5;
    let sy = y / w - 0.5;
    let (max_x, max_y) = ((src.width - 1) as f64, (src.height - 1) as f64);
    if !(0.0..=max_x).contains(&sx) || !(0.0..=max_y).contains(&sy) {
        return fill;
    }
    // Clamp the cell so that the last row/column interpolates within the plane.
    let x0 = (sx as usize).min(src.width - 2);
    let y0 = (sy as usize).min(src.height - 2);
    let fx = sx - x0 as f64;
    let fy = sy - y0 as f64;
    let at = |xi: usize, yi: usize| f64::from(src.data[yi * src.stride + xi]);
    let top = at(x0, y0) * (1.0 - fx) + at(x0 + 1, y0) * fx;
    let bottom = at(x0, y0 + 1) * (1.0 - fx) + at(x0 + 1, y0 + 1) * fx;
    (top * (1.0 - fy) + bottom * fy + 0.5) as u8
}

/// Warps the three I420 planes. `inverse_luma` is the luma-plane inverse homography; the chroma
/// planes, at half resolution, use [`chroma_homography`] of it.
pub fn warp_i420(src: [&Plane<'_>; 3], dst: [&mut PlaneMut<'_>; 3], inverse_luma: &Mat3) {
    let inverse_chroma = chroma_homography(inverse_luma);
    let [src_y, src_u, src_v] = src;
    let [dst_y, dst_u, dst_v] = dst;
    warp_plane(src_y, dst_y, inverse_luma, FILL_Y);
    warp_plane(src_u, dst_u, &inverse_chroma, FILL_UV);
    warp_plane(src_v, dst_v, &inverse_chroma, FILL_UV);
}

/// The same mapping expressed in half-resolution (4:2:0 chroma) pixel-centre coordinates:
/// `S · H · S⁻¹` with `S = diag(½, ½, 1)`. JPEG sites chroma samples between their four luma
/// pixels, which is exactly what halving the centre coordinates gives.
#[must_use]
pub fn chroma_homography(luma: &Mat3) -> Mat3 {
    let h = luma.0;
    Mat3([
        [h[0][0], h[0][1], h[0][2] / 2.0],
        [h[1][0], h[1][1], h[1][2] / 2.0],
        [2.0 * h[2][0], 2.0 * h[2][1], h[2][2]],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    const W: usize = 16;
    const H: usize = 8;

    /// A horizontal gradient: pixel (u, v) = 10·u + v.
    fn gradient() -> Vec<u8> {
        (0..H)
            .flat_map(|v| (0..W).map(move |u| (10 * u + v) as u8))
            .collect()
    }

    fn plane(data: &[u8]) -> Plane<'_> {
        Plane {
            data,
            width: W,
            height: H,
            stride: W,
        }
    }

    fn warp(inverse: &Mat3) -> Vec<u8> {
        let src_data = gradient();
        let mut out = vec![0xAA; W * H];
        warp_plane(
            &plane(&src_data),
            &mut PlaneMut {
                data: &mut out,
                width: W,
                height: H,
                stride: W,
            },
            inverse,
            FILL_Y,
        );
        out
    }

    fn translation(dx: f64, dy: f64) -> Mat3 {
        Mat3([[1.0, 0.0, dx], [0.0, 1.0, dy], [0.0, 0.0, 1.0]])
    }

    #[test]
    fn the_identity_is_byte_exact() {
        assert_eq!(warp(&Mat3::IDENTITY), gradient());
    }

    #[test]
    fn an_integer_shift_moves_the_image_and_fills_the_vacated_edge() {
        // Destination (u, v) reads source (u + 3, v + 1).
        let out = warp(&translation(3.0, 1.0));
        let src = gradient();
        for v in 0..H - 1 {
            for u in 0..W - 3 {
                assert_eq!(out[v * W + u], src[(v + 1) * W + u + 3], "({u},{v})");
            }
            for u in W - 3..W {
                assert_eq!(out[v * W + u], FILL_Y, "({u},{v})");
            }
        }
        assert!(out[(H - 1) * W..].iter().all(|&p| p == FILL_Y));
    }

    #[test]
    fn a_half_pixel_shift_averages_horizontal_neighbours() {
        let out = warp(&translation(0.5, 0.0));
        // Source pixels 4 and 5 on row 0 are 40 and 50: the midpoint is 45.
        assert_eq!(out[4], 45);
        // The last column has nothing to its right: filled.
        assert_eq!(out[W - 1], FILL_Y);
    }

    #[test]
    fn the_last_source_column_is_still_sampled_exactly() {
        // Destination u = W - 1 reads source centre at exactly W - 1 (no shift): no fill.
        let out = warp(&Mat3::IDENTITY);
        assert_eq!(out[W - 1], gradient()[W - 1]);
    }

    #[test]
    fn points_behind_the_camera_are_filled() {
        let out = warp(&Mat3([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]));
        assert!(out.iter().all(|&p| p == FILL_Y));
    }

    #[test]
    fn a_two_times_zoom_leaves_the_centre_pixel_cell_in_place() {
        // Inverse of a 2x zoom about the image centre (W/2, H/2): source = centre + (dst - centre)/2.
        let (cx, cy) = (W as f64 / 2.0, H as f64 / 2.0);
        let inverse = Mat3([[0.5, 0.0, cx / 2.0], [0.0, 0.5, cy / 2.0], [0.0, 0.0, 1.0]]);
        let out = warp(&inverse);
        let src = gradient();
        // The destination pixel just below-right of the centre (8, 4) has its centre at
        // (8.5, 4.5) -> source centre (8.25, 4.25), i.e. pixel coordinates (7.75, 3.75): three
        // quarters of the way from column 7 to 8 and from row 3 to 4.
        let expected = {
            let top = f64::from(src[3 * W + 7]) * 0.25 + f64::from(src[3 * W + 8]) * 0.75;
            let bottom = f64::from(src[4 * W + 7]) * 0.25 + f64::from(src[4 * W + 8]) * 0.75;
            (top * 0.25 + bottom * 0.75 + 0.5) as u8
        };
        assert_eq!(out[4 * W + 8], expected);
        // And nothing is filled: the zoomed-in view lies entirely inside the source.
        assert!(out.iter().all(|&p| p != 0xAA));
    }

    #[test]
    fn chroma_uses_half_the_translation() {
        let luma = translation(2.0, 4.0);
        let chroma = chroma_homography(&luma);
        assert_eq!(chroma, translation(1.0, 2.0));
        // A pure rotation-like homography keeps its scale: only the translations and the
        // perspective row change.
        let h = Mat3([[1.0, 0.1, 6.0], [-0.1, 1.0, 8.0], [0.001, 0.002, 1.0]]);
        let c = chroma_homography(&h);
        assert_eq!(c.0[0], [1.0, 0.1, 3.0]);
        assert_eq!(c.0[2], [0.002, 0.004, 1.0]);
    }

    #[test]
    fn i420_warps_all_three_planes() {
        let y = gradient();
        let u = vec![100u8; (W / 2) * (H / 2)];
        let v = vec![200u8; (W / 2) * (H / 2)];
        let (mut oy, mut ou, mut ov) = (vec![0; W * H], vec![0; W * H / 4], vec![0; W * H / 4]);
        fn half(data: &[u8]) -> Plane<'_> {
            Plane {
                data,
                width: W / 2,
                height: H / 2,
                stride: W / 2,
            }
        }
        warp_i420(
            [&plane(&y), &half(&u), &half(&v)],
            [
                &mut PlaneMut {
                    data: &mut oy,
                    width: W,
                    height: H,
                    stride: W,
                },
                &mut PlaneMut {
                    data: &mut ou,
                    width: W / 2,
                    height: H / 2,
                    stride: W / 2,
                },
                &mut PlaneMut {
                    data: &mut ov,
                    width: W / 2,
                    height: H / 2,
                    stride: W / 2,
                },
            ],
            &translation(2.0, 0.0),
        );
        assert_eq!(oy[0], y[2]);
        // Chroma shifted by one: the last chroma column is filled with neutral grey.
        assert_eq!(ou[0], 100);
        assert_eq!(ou[W / 2 - 1], FILL_UV);
        assert_eq!(ov[0], 200);
    }

    #[test]
    fn strides_wider_than_the_width_are_respected() {
        let stride = W + 4;
        let mut src_data = vec![0u8; stride * H];
        for v in 0..H {
            for u in 0..W {
                src_data[v * stride + u] = (10 * u + v) as u8;
            }
        }
        let mut out = vec![0xAA; stride * H];
        warp_plane(
            &Plane {
                data: &src_data,
                width: W,
                height: H,
                stride,
            },
            &mut PlaneMut {
                data: &mut out,
                width: W,
                height: H,
                stride,
            },
            &Mat3::IDENTITY,
            FILL_Y,
        );
        for v in 0..H {
            assert_eq!(
                &out[v * stride..v * stride + W],
                &src_data[v * stride..v * stride + W]
            );
            // Padding is untouched.
            assert!(
                out[v * stride + W..(v + 1) * stride]
                    .iter()
                    .all(|&p| p == 0xAA)
            );
        }
    }
}
