//! NMEA parsing.
//!
//! So far, all we need to do is parse $GPRMC messages.

use {Error, Result};
use chrono::{DateTime, TimeZone, UTC};

/// A position measurement from a $GPRMC message.
#[derive(Clone, Copy, Debug)]
pub struct Position {
    /// The date and time of the position information.
    pub datetime: DateTime<UTC>,
    /// Is this position valid?
    pub valid: bool,
    /// The latitude. negative numbers are south.
    pub latitude: f64,
    /// The longitude, negative numbers are west.
    pub longitude: f64,
    /// The speed, in knots.
    pub speed: Knots,
    /// The true course, in degrees.
    pub true_course: Degrees,
    /// Magnetic variation, negative numbers are west.
    pub variation: f32,
}

impl Position {
    /// Parses a position from a NMEA $GPRMC string.
    ///
    /// # Examples
    ///
    /// ```
    /// # use velodyne::nmea::Position;
    /// let nmea = "$GPRMC,214106,A,3707.8178,N,12139.2690,W,010.3,188.2,230715,013.8,E,D*05";
    /// let position = Position::new(nmea).unwrap();
    /// ```
    pub fn new(nmea: &str) -> Result<Position> {
        let words = nmea.split(',').collect::<Vec<_>>();
        if words.len() != 13 {
            return Err(Error::Nmea(format!("$GPRMC should have 13 words, only has {}",
                                           words.len())));
        }
        if words[0] != "$GPRMC" {
            return Err(Error::Nmea(format!("Positions can only be created from $GPRMC messages, not {}",
                                           words[0])));
        }
        let last_star_position = if let Some(index) = nmea.rfind('*') {
            index
        } else {
            return Err(Error::Nmea("No ending star to delineate checksum".to_string()));
        };
        let expected_checksum = &nmea[nmea.len() - 2..];
        let calculated_checksum =
            format!("{:02x}",
                    nmea[1..last_star_position].bytes().fold(0, |acc, n| acc ^ n));
        if expected_checksum != calculated_checksum {
            return Err(Error::Nmea(format!("Invalid checksum, expected {}, got {}",
                                           expected_checksum,
                                           calculated_checksum)));
        }
        let latitude = to_dd(words[3].parse()?) * if words[4] == "S" { -1. } else { 1. };
        let longitude = to_dd(words[5].parse()?) * if words[6] == "W" { -1. } else { 1. };
        let variation = words[10].parse::<f32>()? * if words[11] == "W" { -1. } else { 1. };
        Ok(Position {
               datetime: UTC.datetime_from_str(&format!("{}{}", words[9], words[1]),
                                               "%d%m%y%H%M%S")?,
               valid: words[2] == "A",
               latitude: latitude,
               longitude: longitude,
               speed: Knots(words[7].parse()?),
               true_course: Degrees(words[8].parse()?),
               variation: variation,
           })
    }
}

fn to_dd(n: f64) -> f64 {
    let degrees = (n / 100.).round();
    let decimal = (n / 100.).fract() * 100. / 60.;
    degrees + decimal
}

/// Knots.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Knots(pub f32);

/// Degrees.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Degrees(pub f32);

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::{UTC, TimeZone};

    #[test]
    fn position() {
        let nmea = "$GPRMC,214106,A,3707.8178,N,12139.2690,W,010.3,188.2,230715,013.8,E,D*05";
        let position = Position::new(nmea).unwrap();
        println!("{:?}", position);
        assert_eq!(UTC.ymd(2015, 7, 23).and_hms(21, 41, 6), position.datetime);
        assert!(position.valid);
        assert!((37.1303 - position.latitude).abs() < 1e-4);
        assert!((-121.6545 - position.longitude).abs() < 1e-4);
        assert_eq!(Knots(10.3), position.speed);
        assert_eq!(Degrees(188.2), position.true_course);
        assert_eq!(13.8, position.variation);
    }

    #[test]
    fn bad_checksum() {
        let nmea = "$GPRMC,214106,A,3707.8178,N,12139.2690,W,010.3,188.2,230715,013.9,E,D*05";
        assert!(Position::new(nmea).is_err());
    }

    #[test]
    fn too_few_words() {
        let nmea = "$GPRMC,214106,A,3707.8178,N,12139.2690,W,010.3,188.2,230715,013.9,E";
        assert!(Position::new(nmea).is_err());
    }

    #[test]
    fn not_gprmc() {
        let nmea = "$GPRMZ,214106,A,3707.8178,N,12139.2690,W,010.3,188.2,230715,013.9,E,D*05";
        assert!(Position::new(nmea).is_err());
    }
}
