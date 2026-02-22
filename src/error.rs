use crate::coord::Coord2;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TesseraError {
    OutOfBounds { coord: Coord2, width: u32, height: u32 },
    GridSizeMismatch { expected: usize, got: usize },
    InvalidConfig(&'static str),
    EmptyGoals,
}
