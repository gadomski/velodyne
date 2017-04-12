//! Read data from Velodyne VLP-16 and HDL-32E sensors.

#![deny(missing_docs,
        missing_debug_implementations, missing_copy_implementations,
        trivial_casts, trivial_numeric_casts,
        unsafe_code,
        unstable_features,
        unused_import_braces, unused_qualifications)]

extern crate byteorder;
extern crate chrono;
extern crate pcap;

mod packet;

pub use packet::Packet;

/// Our crate-specific error enum.
#[derive(Debug)]
pub enum Error {
    /// Invalid sensor code.
    InvalidSensor(u8),
    /// Invalid start identifier for a data block.
    InvalidStartIdentifier(u16),
    /// Invalid return mode code.
    InvalidReturnMode(u8),
    /// Wrapper around `std::io::Error`.
    Io(std::io::Error),
    /// Wrapper around `pcap::Error`.
    Pcap(pcap::Error),
}

impl From<std::io::Error> for Error {
    fn from(err: std::io::Error) -> Error {
        Error::Io(err)
    }
}

impl From<pcap::Error> for Error {
    fn from(err: pcap::Error) -> Error {
        Error::Pcap(err)
    }
}

/// Our crate-specific result type.
pub type Result<T> = std::result::Result<T, Error>;
