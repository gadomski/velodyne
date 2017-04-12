use std::path::Path;
use Result;
use pcap::{self, Capture};

pub enum Packet {
    Data,
    Position,
}

impl Packet {
    /// Reads one or more packets from a pcap file, as specified by its path.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vlp::Packet;
    /// let packets = Packet::from_pcap_path("data/single.pcap").unwrap();
    /// ```
    pub fn from_pcap_path<P: AsRef<Path>>(path: P) -> Result<Vec<Packet>> {
        let mut capture = Capture::from_file(path)?;
        let mut packets = Vec::new();
        loop {
            match capture.next() {
                Ok(packet) => packets.push(Packet::from_pcap_packet(packet)?),
                Err(err) => {
                    match err {
                        pcap::Error::NoMorePackets => break,
                        _ => return Err(err.into()),
                    }
                }
            }
        }
        Ok(packets)
    }

    fn from_pcap_packet(packet: pcap::Packet) -> Result<Packet> {
        Ok(Packet::Data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn read_one_data_packet() {
        let packets = Packet::from_pcap_path("data/single.pcap").unwrap();
        assert_eq!(1, packets.len());
    }

    #[test]
    fn read_one_position_packet() {
        let packets = Packet::from_pcap_path("data/position.pcap").unwrap();
        assert_eq!(1, packets.len());
    }
}
