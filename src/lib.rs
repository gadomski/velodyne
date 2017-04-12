extern crate byteorder;
extern crate chrono;
extern crate pcap;

mod packet;

pub use packet::Packet;

#[derive(Debug)]
pub enum Error {
    InvalidStartIdentifier(u16),
    Io(std::io::Error),
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

pub type Result<T> = std::result::Result<T, Error>;
