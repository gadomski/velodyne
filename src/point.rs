//! Measured data points.

use chrono::{Duration, UTC};

/// A three-dimensional Velodyne point.
#[derive(Clone, Copy, Debug)]
pub struct Point {
    /// The x coordinate.
    pub x: f32,
    /// The y coordinate.
    pub y: f32,
    /// The z coordinate.
    pub z: f32,
    /// The calibrated reflectivity of the point.
    pub reflectivity: u8,
    /// The laser channel.
    pub channel: u8,
    //return_type: ReturnType,
    /// The azimuth measurement.
    pub azimuth: Azimuth, 
    //time: Time,
}

/// The type of laser return.
#[derive(Clone, Copy, Debug)]
pub enum ReturnType {
    /// The strongest return.
    Strongest,
    /// The last return.
    ///
    /// Can be the strongest, but doesn't have to be.
    Last,
    /// This is either the strongest return or, if the strongest return was the last return, the
    /// second-strongest.
    Secondary,
}

/// The type of azimuth measurement.
#[derive(Clone, Copy, Debug)]
pub enum Azimuth {
    /// The azimuth was provided as part of the data packet.
    Measured(f32),
    /// The azimuth was interpolated.
    Interpolated(f32),
    /// The azimuth was extrapolated.
    Extrapolated(f32),
}

/// The type of time measurement.
#[derive(Clone, Copy, Debug)]
pub enum Time {
    /// The timestamp provided in the data packet, which is an offset from the last hour.
    Offset(Duration),
    /// The absolute time of the point, as calcualted from the offset and a GPS-provided time
    /// value.
    Absolute(UTC),
}
