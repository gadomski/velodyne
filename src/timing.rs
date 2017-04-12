use NUM_LASERS;
use chrono::Duration;

// These values are from the VLP-16 manual.
const FIRING_DURATION: i64 = 2_304; // nanoseconds
const SEQUENCE_DURATION: i64 = 55_296; // nanoseconds

pub fn firing_time(timestamp: Duration, sequence_index: i64, data_point_index: i64) -> Duration {
    timestamp + Duration::nanoseconds(FIRING_DURATION * data_point_index) +
    Duration::nanoseconds(SEQUENCE_DURATION * sequence_index)
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Duration;

    #[test]
    fn from_manual() {
        let sequence_index = 23;
        let data_point_index = 15;
        let timestamp = Duration::microseconds(45_231_878);
        assert_eq!(45_233_184_368,
                   firing_time(timestamp, sequence_index, data_point_index)
                       .num_nanoseconds()
                       .unwrap());
    }
}
