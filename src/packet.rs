use std::io::{Cursor, Read};
use std::path::Path;
use chrono::Duration;
use {Error, Result};
use pcap::{self, Capture};
use byteorder::{ReadBytesExt, LittleEndian};

const AZIMUTH_SCALE_FACTOR: f32 = 100.;
const DISTANCE_SCALE_FACTOR: f32 = 0.002;
const NUM_DATA_BLOCKS: usize = 12;
const NUM_DATA_RECORDS: usize = 16;
const PACKET_HEADER_LEN: usize = 42;
const START_IDENTIFIER: u16 = 0xeeff;

#[derive(Clone, Copy, Debug)]
pub enum Packet {
    Data {
        data_blocks: [DataBlock; NUM_DATA_BLOCKS],
        timestamp: Duration,
    },
    Position,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DataBlock {
    pub azimuth: f32,
    pub data_records: [[DataRecord; NUM_DATA_RECORDS]; 2],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DataRecord {
    pub return_distance: f32,
    pub calibrated_reflectivity: u8,
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
                    return match err {
                               pcap::Error::NoMorePackets => Ok(packets),
                               _ => Err(err.into()),
                           };
                }
            }
        }
    }

    /// Returns this packet's data blocks, or none if it is a position packet.
    ///
    /// # Examples
    ///
    /// ```
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// assert!(packet.data_blocks().is_some());
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// assert!(packet.data_blocks().is_none());
    /// ```
    pub fn data_blocks(&self) -> Option<[DataBlock; 12]> {
        match *self {
            Packet::Data { data_blocks, .. } => Some(data_blocks),
            Packet::Position => None,
        }
    }

    /// Returns this packet's timestamp.
    ///
    /// # Examples
    ///
    /// ```
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// assert_eq!(2467108343, packet.timestamp());
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// unimplemented!()
    /// ```
    pub fn timestamp(&self) -> Duration {
        match *self {
            Packet::Data { data_blocks: _, timestamp } => timestamp,
            Packet::Position => unimplemented!(),
        }
    }

    fn from_pcap_packet(packet: pcap::Packet) -> Result<Packet> {
        let mut data_blocks: [DataBlock; NUM_DATA_BLOCKS] = Default::default();
        let mut cursor = Cursor::new(&packet.data[PACKET_HEADER_LEN..]);
        for mut data_block in &mut data_blocks {
            *data_block = DataBlock::read_from(&mut cursor)?;
        }
        let timestamp = Duration::microseconds(cursor.read_u32::<LittleEndian>()? as i64);
        Ok(Packet::Data {
               data_blocks: data_blocks,
               timestamp: timestamp,
           })
    }
}

impl DataBlock {
    fn read_from<R: Read>(mut read: R) -> Result<DataBlock> {
        let start_identifier = read.read_u16::<LittleEndian>()?;
        if start_identifier != START_IDENTIFIER {
            return Err(Error::InvalidStartIdentifier(start_identifier));
        }
        let azimuth = read.read_u16::<LittleEndian>()? as f32 / AZIMUTH_SCALE_FACTOR;
        let mut data_records: [[DataRecord; NUM_DATA_RECORDS]; 2] = Default::default();
        for data_set in &mut data_records {
            for mut data_record in data_set {
                *data_record = DataRecord::read_from(&mut read)?;
            }
        }
        Ok(DataBlock {
               azimuth: azimuth,
               data_records: data_records,
           })
    }
}

impl DataRecord {
    fn read_from<R: Read>(mut read: R) -> Result<DataRecord> {
        Ok(DataRecord {
               return_distance: read.read_u16::<LittleEndian>()? as f32 * DISTANCE_SCALE_FACTOR,
               calibrated_reflectivity: read.read_u8()?,
           })
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

    #[test]
    fn first_data_block() {
        let data_block = Packet::from_pcap_path("data/single.pcap")
            .unwrap()
            .pop()
            .unwrap()
            .data_blocks()
            .unwrap()
            [0];
        assert_eq!(229.7, data_block.azimuth);
        let data_record = data_block.data_records[0][0];
        assert_eq!(6.524, data_record.return_distance);
        assert_eq!(4, data_record.calibrated_reflectivity);
    }

    #[test]
    fn data_metadata() {
        let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
        assert_eq!(Duration::microseconds(2467108343), packet.timestamp());
    }
}
