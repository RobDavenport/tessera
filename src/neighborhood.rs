use crate::coord::Coord2;
use crate::hex::HexCoord;

/// 4-directional movement (Von Neumann neighborhood).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dir4 {
    North,
    East,
    South,
    West,
}

impl Dir4 {
    pub const ALL: [Dir4; 4] = [Dir4::North, Dir4::East, Dir4::South, Dir4::West];

    #[inline]
    pub const fn offset(self) -> Coord2 {
        match self {
            Dir4::North => Coord2::new(0, -1),
            Dir4::East => Coord2::new(1, 0),
            Dir4::South => Coord2::new(0, 1),
            Dir4::West => Coord2::new(-1, 0),
        }
    }

    #[inline]
    pub const fn opposite(self) -> Dir4 {
        match self {
            Dir4::North => Dir4::South,
            Dir4::East => Dir4::West,
            Dir4::South => Dir4::North,
            Dir4::West => Dir4::East,
        }
    }

    #[inline]
    pub const fn cw(self) -> Dir4 {
        match self {
            Dir4::North => Dir4::East,
            Dir4::East => Dir4::South,
            Dir4::South => Dir4::West,
            Dir4::West => Dir4::North,
        }
    }

    #[inline]
    pub const fn ccw(self) -> Dir4 {
        match self {
            Dir4::North => Dir4::West,
            Dir4::East => Dir4::North,
            Dir4::South => Dir4::East,
            Dir4::West => Dir4::South,
        }
    }
}

/// 8-directional movement (Moore neighborhood).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dir8 {
    N,
    NE,
    E,
    SE,
    S,
    SW,
    W,
    NW,
}

impl Dir8 {
    pub const ALL: [Dir8; 8] = [
        Dir8::N,
        Dir8::NE,
        Dir8::E,
        Dir8::SE,
        Dir8::S,
        Dir8::SW,
        Dir8::W,
        Dir8::NW,
    ];

    #[inline]
    pub const fn offset(self) -> Coord2 {
        match self {
            Dir8::N => Coord2::new(0, -1),
            Dir8::NE => Coord2::new(1, -1),
            Dir8::E => Coord2::new(1, 0),
            Dir8::SE => Coord2::new(1, 1),
            Dir8::S => Coord2::new(0, 1),
            Dir8::SW => Coord2::new(-1, 1),
            Dir8::W => Coord2::new(-1, 0),
            Dir8::NW => Coord2::new(-1, -1),
        }
    }

    #[inline]
    pub const fn is_diagonal(self) -> bool {
        matches!(self, Dir8::NE | Dir8::SE | Dir8::SW | Dir8::NW)
    }

    #[inline]
    pub const fn cost(self) -> u32 {
        if self.is_diagonal() { 14 } else { 10 }
    }

    #[inline]
    pub const fn opposite(self) -> Dir8 {
        match self {
            Dir8::N => Dir8::S,
            Dir8::NE => Dir8::SW,
            Dir8::E => Dir8::W,
            Dir8::SE => Dir8::NW,
            Dir8::S => Dir8::N,
            Dir8::SW => Dir8::NE,
            Dir8::W => Dir8::E,
            Dir8::NW => Dir8::SE,
        }
    }
}

/// 6-directional movement for hex grids.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dir6 {
    E = 0,
    NE = 1,
    NW = 2,
    W = 3,
    SW = 4,
    SE = 5,
}

impl Dir6 {
    pub const ALL: [Dir6; 6] = [Dir6::E, Dir6::NE, Dir6::NW, Dir6::W, Dir6::SW, Dir6::SE];

    #[inline]
    pub const fn offset(self) -> HexCoord {
        HexCoord::NEIGHBOR_OFFSETS[self as usize]
    }

    #[inline]
    pub const fn opposite(self) -> Dir6 {
        match self {
            Dir6::E => Dir6::W,
            Dir6::NE => Dir6::SW,
            Dir6::NW => Dir6::SE,
            Dir6::W => Dir6::E,
            Dir6::SW => Dir6::NE,
            Dir6::SE => Dir6::NW,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Neighborhood {
    VonNeumann,
    Moore,
    Hex,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dir4_opposite() {
        assert_eq!(Dir4::North.opposite(), Dir4::South);
        assert_eq!(Dir4::East.opposite(), Dir4::West);
    }

    #[test]
    fn dir8_diagonal_cost() {
        assert_eq!(Dir8::NE.cost(), 14);
        assert_eq!(Dir8::E.cost(), 10);
    }

    #[test]
    fn dir6_offset() {
        assert_eq!(Dir6::E.offset(), HexCoord::new(1, 0));
        assert_eq!(Dir6::NW.offset(), HexCoord::new(0, -1));
    }
}
