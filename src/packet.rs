use std::path::Path;
use Result;

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
        unimplemented!()
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
