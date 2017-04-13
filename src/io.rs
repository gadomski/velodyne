//! Read Velodyne data from sources.

use Result;
use pcap::{self, Capture, Offline};
use std::path::Path;

/// A trait for things that can produce Velodyne packets.
pub trait Read {
    /// Get the next group of bytes that can be turned into Velodyne data.
    ///
    /// # Examples
    ///
    /// `Pcap` implements `Read`:
    ///
    /// ```
    /// use velodyne::io::{Pcap, Read};
    /// let mut pcap = Pcap::open("data/single.pcap").unwrap();
    /// let bytes = pcap.read().unwrap().unwrap();
    /// ```
    fn read(&mut self) -> Option<Result<&[u8]>>;
}

/// Reads Velodyne data from pcap files.
#[allow(missing_debug_implementations)]
pub struct Pcap {
    capture: Capture<Offline>,
}

impl Pcap {
    /// Opens a pcap file for reading.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::io::Pcap;
    /// let reader = Pcap::open("data/single.pcap").unwrap();
    /// ```
    pub fn open<P: AsRef<Path>>(path: P) -> Result<Pcap> {
        Ok(Pcap { capture: Capture::from_file(path)? })
    }
}

impl Read for Pcap {
    fn read(&mut self) -> Option<Result<&[u8]>> {
        match self.capture.next() {
            Ok(packet) => Some(Ok(packet.data)),
            Err(err) => {
                match err {
                    pcap::Error::NoMorePackets => None,
                    _ => Some(Err(err.into())),
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pcap_single() {
        Pcap::open("data/single.pcap").unwrap();
    }

    #[test]
    fn pcap_invalid_file() {
        assert!(Pcap::open("notafile").is_err());
    }
}
