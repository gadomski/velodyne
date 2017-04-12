extern crate pcap;

mod packet;

pub use packet::Packet;

#[derive(Debug)]
pub enum Error {
    Pcap(pcap::Error),
}

impl From<pcap::Error> for Error {
    fn from(err: pcap::Error) -> Error {
        Error::Pcap(err)
    }
}

pub type Result<T> = std::result::Result<T, Error>;
