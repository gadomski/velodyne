//! Sources of Velodyne data.

use Point;

/// A source of Velodyne data.
#[derive(Clone, Copy, Debug)]
pub struct Source;

impl Source {
    /// Returns an interator over this source's points.
    pub fn points(&mut self) -> Points {
        unimplemented!()
    }
}

/// An iterator over a source's points.
#[derive(Clone, Copy, Debug)]
pub struct Points;

impl Iterator for Points {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        unimplemented!()
    }
}
