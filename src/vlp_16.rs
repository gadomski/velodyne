//! Velodyne Puck 16.

use {Error, Result, Point};
use byteorder::{ReadBytesExt, LittleEndian};
use chrono::Duration;
use point::{Azimuth, Time, ReturnType};
use std::f32;
use std::io::{Cursor, Read};

const AZIMUTH_SCALE_FACTOR: f32 = 100.;
const DISTANCE_SCALE_FACTOR: f32 = 0.002;
const NUM_LASERS: usize = 16;
const NUM_DATA_BLOCKS: usize = 12;
const PACKET_HEADER_LEN: usize = 42;
const START_IDENTIFIER: u16 = 0xeeff;

/// A Velodyne information packet.
#[derive(Clone, Debug)]
pub enum Packet {
    /// Data packets contain laser range measurements.
    Data {
        /// A fixed-size array of data blocks.
        data_blocks: Box<[DataBlock; NUM_DATA_BLOCKS]>,
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

/// A block of laser measurements.
#[derive(Clone, Copy, Debug, Default)]
pub struct DataBlock {
    /// The reported azimuth assocaited with the first laser shot.
    ///
    /// This value often needs to be interpolated for the second set of data records.
    pub azimuth: f32,
    /// Two sets of sixteen data records.
    ///
    /// Each laser has it's value recorded twice in each data block.
    pub data_records: [[DataRecord; NUM_LASERS]; 2],
}

/// A measurement of range and reflectivity.
#[derive(Clone, Copy, Debug, Default)]
pub struct DataRecord {
    /// The distance of the reflective object.
    pub return_distance: f32,
    /// The calibrated reflectivity.
    ///
    /// A black, absorbent diffuse reflector is zero. A white, reflective diffuse reflector is 100.
    /// A retro-reflector covered with a semi-transparent white surface is 101. A retro-reflector
    /// without any coverage is 255.
    pub calibrated_reflectivity: u8,
}

/// The modes by which the instrument can report reutrns.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ReturnMode {
    /// The strongest return by light energy.
    StrongestReturn,
    /// The last return for that laser pulse.
    LastReturn,
    /// The strongest return and the last return.
    ///
    /// If the last return is the strongest, returns the second-strongest return and the last
    /// return.
    DualReturn,
}

/// The sensor that produced the data.
#[derive(Clone, Copy, Debug, PartialEq)]
#[allow(non_camel_case_types)]
pub enum Sensor {
    /// HDL-32E.
    HDL_32E,
    /// VLP-16.
    VLP_16,
}

impl Packet {
    /// Creates a new packet from bytes.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::vlp_16::Packet;
    /// use velodyne::fixtures::VLP_16_DATA_PACKET;
    /// let packet = Packet::new(&VLP_16_DATA_PACKET).unwrap();
    /// ```
    pub fn new(bytes: &[u8]) -> Result<Packet> {
        if &bytes[248..254] == b"$GPRMC" {
            Packet::new_position(bytes)
        } else {
            Packet::new_data(bytes)
        }
    }

    /// Returns true if this is a data packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::vlp_16::Packet;
    /// use velodyne::fixtures::{VLP_16_DATA_PACKET, VLP_16_POSITION_PACKET};
    /// assert!(Packet::new(&VLP_16_DATA_PACKET).unwrap().is_data());
    /// assert!(!Packet::new(&VLP_16_POSITION_PACKET).unwrap().is_data());
    /// ```
    pub fn is_data(&self) -> bool {
        match *self {
            Packet::Data { .. } => true,
            Packet::Position { .. } => false,
        }
    }

    /// Returns true if this is a position packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::vlp_16::Packet;
    /// use velodyne::fixtures::{VLP_16_DATA_PACKET, VLP_16_POSITION_PACKET};
    /// assert!(!Packet::new(&VLP_16_DATA_PACKET).unwrap().is_position());
    /// assert!(Packet::new(&VLP_16_POSITION_PACKET).unwrap().is_position());
    /// ```
    pub fn is_position(&self) -> bool {
        !self.is_data()
    }

    /// Returns this packet's data blocks, or none if it is a position packet.
    pub fn data_blocks(&self) -> Option<[DataBlock; 12]> {
        match *self {
            Packet::Data { ref data_blocks, .. } => Some(**data_blocks),
            Packet::Position { .. } => None,
        }
    }

    /// Returns this packet's timestamp.
    ///
    /// A timestamp is a duration from the last UTC hour.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::vlp_16::Packet;
    /// # use velodyne::fixtures::VLP_16_DATA_PACKET;
    /// let packet = Packet::new(&VLP_16_DATA_PACKET).unwrap();
    /// let timestamp = packet.timestamp();
    /// ```
    pub fn timestamp(&self) -> Duration {
        match *self {
            Packet::Data { timestamp, .. } |
            Packet::Position { timestamp, .. } => timestamp,
        }
    }

    /// Returns this packet's return mode, or none if it's a position packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::vlp_16::Packet;
    /// # use velodyne::fixtures::VLP_16_DATA_PACKET;
    /// let packet = Packet::new(&VLP_16_DATA_PACKET).unwrap();
    /// let return_mode = packet.return_mode().unwrap();
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
    /// # use velodyne::vlp_16::Packet;
    /// # use velodyne::fixtures::VLP_16_DATA_PACKET;
    /// let packet = Packet::new(&VLP_16_DATA_PACKET).unwrap();
    /// let sensor = packet.sensor().unwrap();
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
    /// # use velodyne::vlp_16::Packet;
    /// # use velodyne::fixtures::VLP_16_POSITION_PACKET;
    /// let packet = Packet::new(&VLP_16_POSITION_PACKET).unwrap();
    /// let nmea = packet.nmea().unwrap();
    /// ```
    pub fn nmea(&self) -> Option<&str> {
        match *self {
            Packet::Data { .. } => None,
            Packet::Position { ref nmea, .. } => Some(nmea),
        }
    }

    /// Returns the points contained within this data packet.
    ///
    /// Returns `None` if this is a position packet.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::vlp_16::Packet;
    /// # use velodyne::fixtures::VLP_16_DATA_PACKET;
    /// let packet = Packet::new(&VLP_16_DATA_PACKET).unwrap();
    /// let points = packet.points().unwrap();
    /// ```
    pub fn points(&self) -> Option<Vec<Point>> {
        match *self {
            Packet::Data { ref data_blocks, timestamp, return_mode, .. } => {
                let azimuth_model = AzimuthModel::new(**data_blocks);
                let mut points = Vec::new();
                for (i, data_block) in data_blocks.iter().enumerate() {
                    for (j, sequence) in data_block.data_records.iter().enumerate() {
                        for (channel, data_record) in sequence.iter().enumerate() {
                            let azimuth = azimuth_model.predict(i, j, channel);
                            let vertical_angle = vertical_angle(channel);
                            points.push(Point {
                                            x: data_record.return_distance * vertical_angle.cos() *
                                               azimuth.sin(),
                                            y: data_record.return_distance * vertical_angle.cos() *
                                               azimuth.cos(),
                                            z: data_record.return_distance * vertical_angle.sin(),
                                            reflectivity: data_record.calibrated_reflectivity,
                                            channel: channel as u8,
                                        });
                        }
                    }
                }
                Some(points)
            }
            Packet::Position { .. } => None,
        }
    }

    fn new_position(bytes: &[u8]) -> Result<Packet> {
        let mut cursor = Cursor::new(&bytes[PACKET_HEADER_LEN + 198..]);
        let timestamp = Duration::microseconds(cursor.read_u32::<LittleEndian>()? as i64);
        let mut nmea = String::new();
        cursor.set_position(8);
        cursor.take(72).read_to_string(&mut nmea)?;
        Ok(Packet::Position {
               timestamp: timestamp,
               nmea: nmea,
           })
    }

    fn new_data(bytes: &[u8]) -> Result<Packet> {
        let mut data_blocks: [DataBlock; NUM_DATA_BLOCKS] = Default::default();
        let mut cursor = Cursor::new(&bytes[PACKET_HEADER_LEN..]);
        for mut data_block in &mut data_blocks {
            *data_block = DataBlock::read_from(&mut cursor)?;
        }
        let timestamp = Duration::microseconds(cursor.read_u32::<LittleEndian>()? as i64);
        let return_mode = ReturnMode::from_u8(cursor.read_u8()?)?;
        let sensor = Sensor::from_u8(cursor.read_u8()?)?;
        Ok(Packet::Data {
               data_blocks: Box::new(data_blocks),
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
        let mut data_records: [[DataRecord; NUM_LASERS]; 2] = Default::default();
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

fn vertical_angle(channel: usize) -> f32 {
    if channel > 15 {
        panic!("Channel should never be above 15: {}", channel);
    }
    (if channel % 2 == 1 {
             channel as f32
         } else {
             -15. + channel as f32
         })
        .to_radians()
}

struct AzimuthModel {
    data_blocks: [DataBlock; NUM_DATA_BLOCKS],
}

impl AzimuthModel {
    fn new(data_blocks: [DataBlock; NUM_DATA_BLOCKS]) -> AzimuthModel {
        AzimuthModel { data_blocks: data_blocks }
    }

    fn predict(&self, data_block: usize, sequence: usize, channel: usize) -> f32 {
        if data_block != NUM_DATA_BLOCKS - 1 {
            self.interpolate(data_block, sequence, channel)
        } else {
            self.extrapolate(data_block, sequence, channel)
        }
    }

    fn interpolate(&self, data_block: usize, sequence: usize, channel: usize) -> f32 {
        unimplemented!()
    }

    fn extrapolate(&self, data_block: usize, sequence: usize, channel: usize) -> f32 {
        unimplemented!()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fixtures::{VLP_16_DATA_PACKET, VLP_16_POSITION_PACKET};

    #[test]
    fn data_packet() {
        let packet = Packet::new(&VLP_16_DATA_PACKET).unwrap();
        assert!(packet.is_data());
    }

    #[test]
    fn position_packet() {
        let packet = Packet::new(&VLP_16_POSITION_PACKET).unwrap();
        assert!(packet.is_position());
    }

    #[test]
    fn azimuth() {
        let data_blocks = Packet::new(&VLP_16_DATA_PACKET).unwrap().data_blocks().unwrap();
        assert_eq!(229.70, data_blocks[0].azimuth);
        assert_eq!(234.08, data_blocks[11].azimuth);
    }

    #[test]
    fn data_record() {
        let data_blocks = Packet::new(&VLP_16_DATA_PACKET).unwrap().data_blocks().unwrap();
        let data_record = data_blocks[0].data_records[0][0];
        assert_eq!(6.524, data_record.return_distance);
        assert_eq!(4, data_record.calibrated_reflectivity);
        let data_record = data_blocks[11].data_records[1][12];
        assert_eq!(51.470, data_record.return_distance);
        assert_eq!(9, data_record.calibrated_reflectivity);
    }
}
