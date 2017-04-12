use {Error, Result};
use byteorder::{ReadBytesExt, LittleEndian};
use chrono::Duration;
use pcap::{self, Capture};
use std::io::{Cursor, Read};
use std::path::Path;

const AZIMUTH_SCALE_FACTOR: f32 = 100.;
const DISTANCE_SCALE_FACTOR: f32 = 0.002;
const NUM_DATA_BLOCKS: usize = 12;
const NUM_DATA_RECORDS: usize = 16;
const PACKET_HEADER_LEN: usize = 42;
const START_IDENTIFIER: u16 = 0xeeff;

/// A Velodyne information packet.
#[derive(Clone, Debug)]
pub enum Packet {
    /// Data packets contain laser range measurements.
    Data {
        /// A fixed-size array of data blocks.
        data_blocks: [DataBlock; NUM_DATA_BLOCKS],
        /// The duration from the top of the hour to the first laser firing in the packet.
        timestamp: Duration,
        /// The return mode of the sensor.
        return_mode: ReturnMode,
        /// The sensor type.
        sensor: Sensor,
    },
    /// A position measurement, really just an echoing of information from a GNSS system.
    Position {
        /// The duration from the top of the hour that the NMEA string was received.
        timestamp: Duration,
        /// The NMA $GPRMC message as received from an external GNSS system.
        nmea: String,
    },
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

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReturnMode {
    StrongestReturn,
    LastReturn,
    DualReturn,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(non_camel_case_types)]
pub enum Sensor {
    HDL_32E,
    VLP_16,
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
    /// # use vlp::Packet;
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// assert!(packet.data_blocks().is_some());
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// assert!(packet.data_blocks().is_none());
    /// ```
    pub fn data_blocks(&self) -> Option<[DataBlock; 12]> {
        match *self {
            Packet::Data { data_blocks, .. } => Some(data_blocks),
            Packet::Position { .. } => None,
        }
    }

    /// Returns this packet's timestamp.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vlp::Packet;
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// let timestamp = packet.timestamp();
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// let timestamp = packet.timestamp();
    /// ```
    pub fn timestamp(&self) -> Duration {
        match *self {
            Packet::Data { timestamp, .. } => timestamp,
            Packet::Position { timestamp, .. } => timestamp,
        }
    }

    /// Returns this packet's return mode, or none if it's a position packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vlp::Packet;
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// assert!(packet.return_mode().is_some());
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// assert!(packet.return_mode().is_none());
    /// ```
    pub fn return_mode(&self) -> Option<ReturnMode> {
        match *self {
            Packet::Data { return_mode, .. } => Some(return_mode),
            Packet::Position { .. } => None,
        }
    }

    /// Returns this packet's sensor, or none if it's a position packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vlp::Packet;
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// assert!(packet.sensor().is_some());
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// assert!(packet.sensor().is_none());
    /// ```
    pub fn sensor(&self) -> Option<Sensor> {
        match *self {
            Packet::Data { sensor, .. } => Some(sensor),
            Packet::Position { .. } => None,
        }
    }

    /// Returns this packet's NMEA string, or none if it's a data packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vlp::Packet;
    /// let packet = Packet::from_pcap_path("data/single.pcap").unwrap().pop().unwrap();
    /// assert!(packet.nmea().is_none());
    /// let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
    /// assert!(packet.nmea().is_some());
    /// ```
    pub fn nmea(&self) -> Option<&str> {
        match *self {
            Packet::Data { .. } => None,
            Packet::Position { ref nmea, .. } => Some(nmea),
        }
    }

    fn from_pcap_packet(packet: pcap::Packet) -> Result<Packet> {
        if packet.data[PACKET_HEADER_LEN..PACKET_HEADER_LEN + 198].iter().all(|&n| n == 0) &&
           &packet.data[PACKET_HEADER_LEN + 206..PACKET_HEADER_LEN + 212] == b"$GPRMC" {
            Packet::position_from_pcap_packet(packet)
        } else {
            Packet::data_from_pcap_packet(packet)
        }
    }

    fn position_from_pcap_packet(packet: pcap::Packet) -> Result<Packet> {
        let mut cursor = Cursor::new(&packet.data[PACKET_HEADER_LEN + 198..]);
        let timestamp = Duration::microseconds(cursor.read_u32::<LittleEndian>()? as i64);
        let mut nmea = String::new();
        cursor.set_position(8);
        cursor.take(72).read_to_string(&mut nmea)?;
        Ok(Packet::Position {
               timestamp: timestamp,
               nmea: nmea,
           })
    }

    fn data_from_pcap_packet(packet: pcap::Packet) -> Result<Packet> {
        let mut data_blocks: [DataBlock; NUM_DATA_BLOCKS] = Default::default();
        let mut cursor = Cursor::new(&packet.data[PACKET_HEADER_LEN..]);
        for mut data_block in &mut data_blocks {
            *data_block = DataBlock::read_from(&mut cursor)?;
        }
        let timestamp = Duration::microseconds(cursor.read_u32::<LittleEndian>()? as i64);
        let return_mode = ReturnMode::from_u8(cursor.read_u8()?)?;
        let sensor = Sensor::from_u8(cursor.read_u8()?)?;
        Ok(Packet::Data {
               data_blocks: data_blocks,
               timestamp: timestamp,
               return_mode: return_mode,
               sensor: sensor,
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

impl ReturnMode {
    fn from_u8(n: u8) -> Result<ReturnMode> {
        match n {
            0x37 => Ok(ReturnMode::StrongestReturn),
            0x38 => Ok(ReturnMode::LastReturn),
            0x39 => Ok(ReturnMode::DualReturn),
            _ => Err(Error::InvalidReturnMode(n)),
        }
    }
}

impl Sensor {
    fn from_u8(n: u8) -> Result<Sensor> {
        match n {
            0x21 => Ok(Sensor::HDL_32E),
            0x22 => Ok(Sensor::VLP_16),
            _ => Err(Error::InvalidSensor(n)),
        }
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
        assert_eq!(ReturnMode::StrongestReturn, packet.return_mode().unwrap());
        assert_eq!(Sensor::VLP_16, packet.sensor().unwrap());
    }

    #[test]
    fn position() {
        let packet = Packet::from_pcap_path("data/position.pcap").unwrap().pop().unwrap();
        assert_eq!(Duration::microseconds(2467110195), packet.timestamp());
        assert_eq!("$GPRMC,214106,A,3707.8178,N,12139.2690,W,010.3,188.2,230715,013.8,E,D*05",
                   packet.nmea().unwrap());
    }
}
