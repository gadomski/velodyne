mod packet;

pub use packet::Packet;

#[derive(Debug)]
pub enum Error {}

pub type Result<T> = std::result::Result<T, Error>;
