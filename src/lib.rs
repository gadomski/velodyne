//! Read data from Velodyne LiDAR sensors.
//!
//! As of now, only supports the VLP-16.

#![deny(missing_docs,
        missing_debug_implementations, missing_copy_implementations,
        trivial_casts, trivial_numeric_casts,
        unsafe_code,
        unstable_features,
        unused_import_braces, unused_qualifications)]

extern crate byteorder;
extern crate chrono;
extern crate pcap;

pub mod fixtures;
pub mod io;
pub mod nmea;
pub mod point;
pub mod vlp_16;

pub use point::Point;

/// Our crate-specific error enum.
#[derive(Debug)]
pub enum Error {
    /// Wrapper around `chrono::ParseError`.
    ChronoParse(chrono::ParseError),
    /// Invalid sensor code.
    InvalidSensor(u8),
    /// Invalid start identifier for a data block.
    InvalidStartIdentifier(u16),
    /// Invalid return mode code.
    InvalidReturnMode(u8),
    /// Wrapper around `std::io::Error`.
    Io(std::io::Error),
    /// Something went wrong when parsing a NMEA string.
    Nmea(String),
    /// Wrapper around `std::num::ParseFloatError`.
    ParseFloat(std::num::ParseFloatError),
    /// Wrapper around `pcap::Error`.
    Pcap(pcap::Error),
}

impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Error {
        Error::Io(err)
    }
}

impl From<std::num::ParseFloatError> for Error {
    fn from(err: std::num::ParseFloatError) -> Error {
        Error::ParseFloat(err)
    }
}

impl From<chrono::ParseError> for Error {
    fn from(err: chrono::ParseError) -> Error {
        Error::ChronoParse(err)
    }
}

impl From<pcap::Error> for Error {
    fn from(err: pcap::Error) -> Error {
        Error::Pcap(err)
    }
}

/// Our crate-specific result type.
pub type Result<T> = std::result::Result<T, Error>;
