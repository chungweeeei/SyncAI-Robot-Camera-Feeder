//! The camera's calibration, read from the YAML that `camera_calibration` writes (its
//! `ost.yaml`), for the `camera_info` published beside every image.
//!
//! The file is per camera, not per package: the intrinsics belong to one physical lens and
//! sensor, so they live outside this repo (the launch file points at the robot's own copy) and
//! the params file only names the path. The format is the calibrator's own rather than ROS
//! parameters, so a fresh calibration is dropped in as it comes out, the same file
//! `camera_info_manager` reads in C++.
//!
//! Every check here guards a `camera_info` that would be wrong rather than absent, and wrong is
//! the worse of the two: a consumer projecting points with a K for another resolution or a
//! truncated distortion vector mislabels pixels without any error. So a file that fails one is
//! rejected whole, and the node publishes no `camera_info` at all.
//!
//! Pure: no ROS or GStreamer types; tested on calibration text.

use serde::Deserialize;

/// A validated calibration, in `sensor_msgs/CameraInfo`'s layout.
#[derive(Clone, Debug, PartialEq)]
pub struct Calibration {
    pub width: u32,
    pub height: u32,
    pub distortion_model: String,
    pub d: Vec<f64>,
    pub k: [f64; 9],
    pub r: [f64; 9],
    pub p: [f64; 12],
}

/// The calibrator's file. Unknown keys (`camera_name`) are ignored.
#[derive(Deserialize)]
struct CalibrationFile {
    image_width: u32,
    image_height: u32,
    distortion_model: String,
    camera_matrix: Matrix,
    distortion_coefficients: Matrix,
    rectification_matrix: Matrix,
    projection_matrix: Matrix,
}

#[derive(Deserialize)]
struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

/// Parses calibration text and checks it against the resolution the pipeline captures at.
///
/// The resolution check is the one that matters most in practice: a calibration taken at another
/// resolution or crop (the AR0234 also offers 1280x800, and 720 is a crop of it, which moves `cy`)
/// parses fine and is wrong everywhere.
pub fn parse(text: &str, width: u32, height: u32) -> Result<Calibration, String> {
    let file: CalibrationFile =
        serde_yaml::from_str(text).map_err(|err| format!("not a calibration file: {err}"))?;

    if (file.image_width, file.image_height) != (width, height) {
        return Err(format!(
            "calibrated at {}x{}, but the camera captures at {width}x{height}",
            file.image_width, file.image_height
        ));
    }

    let k: [f64; 9] = matrix("camera_matrix", &file.camera_matrix, 3, 3)?;
    let r: [f64; 9] = matrix("rectification_matrix", &file.rectification_matrix, 3, 3)?;
    let p: [f64; 12] = matrix("projection_matrix", &file.projection_matrix, 3, 4)?;

    let (fx, fy) = (k[0], k[4]);
    if fx <= 0.0 || fy <= 0.0 {
        return Err(format!(
            "camera_matrix has a non-positive focal length (fx {fx}, fy {fy})"
        ));
    }

    let expected = distortion_len(&file.distortion_model).ok_or_else(|| {
        format!(
            "distortion_model {:?} is not one of plumb_bob, rational_polynomial, equidistant",
            file.distortion_model
        )
    })?;
    let d = &file.distortion_coefficients;
    if d.rows * d.cols != d.data.len() || d.data.len() != expected {
        return Err(format!(
            "distortion_coefficients has {} values ({}x{}); {} takes {expected}",
            d.data.len(),
            d.rows,
            d.cols,
            file.distortion_model
        ));
    }
    if !d.data.iter().all(|v| v.is_finite()) {
        return Err("distortion_coefficients has a non-finite value".to_string());
    }

    Ok(Calibration {
        width: file.image_width,
        height: file.image_height,
        distortion_model: file.distortion_model,
        d: file.distortion_coefficients.data,
        k,
        r,
        p,
    })
}

/// The coefficient count of each model in `sensor_msgs/distortion_models.hpp`.
fn distortion_len(model: &str) -> Option<usize> {
    match model {
        "plumb_bob" => Some(5),
        "rational_polynomial" => Some(8),
        "equidistant" => Some(4),
        _ => None,
    }
}

/// A `rows x cols` matrix as its row-major array. The declared shape, the data length and the
/// expected shape must all agree: a short `data` would otherwise leave zeros in the message.
fn matrix<const N: usize>(
    name: &str,
    m: &Matrix,
    rows: usize,
    cols: usize,
) -> Result<[f64; N], String> {
    if (m.rows, m.cols) != (rows, cols) || m.data.len() != N {
        return Err(format!(
            "{name} is {}x{} with {} values; expected {rows}x{cols}",
            m.rows,
            m.cols,
            m.data.len()
        ));
    }
    if !m.data.iter().all(|v| v.is_finite()) {
        return Err(format!("{name} has a non-finite value"));
    }
    let mut out = [0.0; N];
    out.copy_from_slice(&m.data);
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// What camera_calibration writes, with integers where it writes them.
    const OST: &str = "\
image_width: 1280
image_height: 720
camera_name: camera0
camera_matrix:
  rows: 3
  cols: 3
  data: [953.875, 0, 690.458, 0, 947.329, 377.627, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.411, 0.292, -0.0009, -0.0018, -0.0705]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [953.875, 0, 690.458, 0, 0, 947.329, 377.627, 0, 0, 0, 1, 0]
";

    #[test]
    fn the_calibrators_output_parses() {
        let c = parse(OST, 1280, 720).unwrap();
        assert_eq!((c.width, c.height), (1280, 720));
        assert_eq!(c.distortion_model, "plumb_bob");
        assert_eq!(c.d, vec![-0.411, 0.292, -0.0009, -0.0018, -0.0705]);
        assert_eq!(c.k[0], 953.875);
        assert_eq!(c.k[5], 377.627);
        assert_eq!(c.k[8], 1.0);
        assert_eq!(c.r, [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        assert_eq!(c.p[3], 0.0);
        assert_eq!(c.p[10], 1.0);
    }

    #[test]
    fn another_resolution_is_rejected() {
        let err = parse(OST, 1280, 800).unwrap_err();
        assert!(
            err.contains("1280x720") && err.contains("1280x800"),
            "{err}"
        );
    }

    #[test]
    fn a_short_matrix_is_rejected_rather_than_zero_filled() {
        let text = OST.replace(
            "data: [953.875, 0, 690.458, 0, 947.329, 377.627, 0, 0, 1]",
            "data: [953.875, 0, 690.458, 0, 947.329, 377.627, 0, 0]",
        );
        let err = parse(&text, 1280, 720).unwrap_err();
        assert!(err.contains("camera_matrix"), "{err}");
    }

    #[test]
    fn the_distortion_length_must_match_the_model() {
        let text = OST.replace(
            "distortion_model: plumb_bob",
            "distortion_model: equidistant",
        );
        let err = parse(&text, 1280, 720).unwrap_err();
        assert!(err.contains("equidistant takes 4"), "{err}");

        let fisheye = OST
            .replace(
                "distortion_model: plumb_bob",
                "distortion_model: equidistant",
            )
            .replace("cols: 5", "cols: 4")
            .replace(
                "[-0.411, 0.292, -0.0009, -0.0018, -0.0705]",
                "[-0.05, -0.2, 0.9, -0.7]",
            );
        assert_eq!(parse(&fisheye, 1280, 720).unwrap().d.len(), 4);
    }

    #[test]
    fn an_unknown_model_is_rejected() {
        let text = OST.replace("distortion_model: plumb_bob", "distortion_model: fisheye");
        let err = parse(&text, 1280, 720).unwrap_err();
        assert!(err.contains("fisheye"), "{err}");
    }

    #[test]
    fn a_non_positive_focal_length_is_rejected() {
        let text = OST.replace("data: [953.875, 0, 690.458,", "data: [0, 0, 690.458,");
        let err = parse(&text, 1280, 720).unwrap_err();
        assert!(err.contains("focal length"), "{err}");
    }

    #[test]
    fn non_finite_values_are_rejected() {
        let text = OST.replace("-0.0705]", ".nan]");
        assert!(parse(&text, 1280, 720).is_err());
    }

    #[test]
    fn a_file_that_is_not_a_calibration_is_rejected() {
        let err = parse("image_width: 1280\n", 1280, 720).unwrap_err();
        assert!(err.starts_with("not a calibration file"), "{err}");
        assert!(parse("", 1280, 720).is_err());
    }
}
