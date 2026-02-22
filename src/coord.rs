use core::ops::{Add, Mul, Neg, Sub};

/// 2D integer coordinate for grid positions.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Coord2 {
    pub x: i32,
    pub y: i32,
}

impl Coord2 {
    pub const ZERO: Coord2 = Coord2 { x: 0, y: 0 };

    #[inline]
    pub const fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn manhattan_distance(self, other: Coord2) -> u32 {
        (self.x - other.x).unsigned_abs() + (self.y - other.y).unsigned_abs()
    }

    #[inline]
    pub fn chebyshev_distance(self, other: Coord2) -> u32 {
        (self.x - other.x)
            .unsigned_abs()
            .max((self.y - other.y).unsigned_abs())
    }

    #[inline]
    pub fn octile_distance(self, other: Coord2) -> u32 {
        let dx = (self.x - other.x).unsigned_abs();
        let dy = (self.y - other.y).unsigned_abs();
        let (big, small) = if dx > dy { (dx, dy) } else { (dy, dx) };
        big * 10 + small * 4
    }

    #[inline]
    pub fn to_index(self, width: u32) -> Option<usize> {
        if width == 0 || self.x < 0 || self.y < 0 || self.x >= width as i32 {
            return None;
        }
        Some((self.y as usize) * (width as usize) + (self.x as usize))
    }

    #[inline]
    pub fn from_index(index: usize, width: u32) -> Self {
        if width == 0 {
            return Coord2::ZERO;
        }
        Self {
            x: (index % width as usize) as i32,
            y: (index / width as usize) as i32,
        }
    }
}

impl Add for Coord2 {
    type Output = Coord2;

    fn add(self, rhs: Self) -> Self::Output {
        Coord2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Coord2 {
    type Output = Coord2;

    fn sub(self, rhs: Self) -> Self::Output {
        Coord2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Neg for Coord2 {
    type Output = Coord2;

    fn neg(self) -> Self::Output {
        Coord2::new(-self.x, -self.y)
    }
}

impl Mul<i32> for Coord2 {
    type Output = Coord2;

    fn mul(self, rhs: i32) -> Self::Output {
        Coord2::new(self.x * rhs, self.y * rhs)
    }
}

/// 3D integer coordinate for volumetric grids.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Coord3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Coord3 {
    pub const ZERO: Coord3 = Coord3 { x: 0, y: 0, z: 0 };

    #[inline]
    pub const fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    #[inline]
    pub fn manhattan_distance(self, other: Coord3) -> u32 {
        (self.x - other.x).unsigned_abs()
            + (self.y - other.y).unsigned_abs()
            + (self.z - other.z).unsigned_abs()
    }

    #[inline]
    pub fn to_index(self, width: u32, height: u32) -> Option<usize> {
        if width == 0
            || height == 0
            || self.x < 0
            || self.y < 0
            || self.z < 0
            || self.x >= width as i32
            || self.y >= height as i32
        {
            return None;
        }
        Some(
            (self.z as usize) * (width as usize * height as usize)
                + (self.y as usize) * (width as usize)
                + (self.x as usize),
        )
    }

    #[inline]
    pub fn from_index(index: usize, width: u32, height: u32) -> Self {
        if width == 0 || height == 0 {
            return Coord3::ZERO;
        }
        let layer = width as usize * height as usize;
        let z = index / layer;
        let rem = index % layer;
        let y = rem / width as usize;
        let x = rem % width as usize;
        Coord3::new(x as i32, y as i32, z as i32)
    }
}

impl Add for Coord3 {
    type Output = Coord3;

    fn add(self, rhs: Self) -> Self::Output {
        Coord3::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl Sub for Coord3 {
    type Output = Coord3;

    fn sub(self, rhs: Self) -> Self::Output {
        Coord3::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl Neg for Coord3 {
    type Output = Coord3;

    fn neg(self) -> Self::Output {
        Coord3::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<i32> for Coord3 {
    type Output = Coord3;

    fn mul(self, rhs: i32) -> Self::Output {
        Coord3::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn coord2_add() {
        assert_eq!(Coord2::new(1, 2) + Coord2::new(3, 4), Coord2::new(4, 6));
    }

    #[test]
    fn coord2_sub() {
        assert_eq!(Coord2::new(5, 6) - Coord2::new(1, 4), Coord2::new(4, 2));
    }

    #[test]
    fn coord2_neg() {
        assert_eq!(-Coord2::new(2, -3), Coord2::new(-2, 3));
    }

    #[test]
    fn coord2_manhattan() {
        assert_eq!(Coord2::new(0, 0).manhattan_distance(Coord2::new(3, 4)), 7);
    }

    #[test]
    fn coord2_chebyshev() {
        assert_eq!(Coord2::new(0, 0).chebyshev_distance(Coord2::new(3, 4)), 4);
    }

    #[test]
    fn coord2_octile() {
        assert_eq!(Coord2::new(0, 0).octile_distance(Coord2::new(3, 4)), 52);
    }

    #[test]
    fn coord2_index_roundtrip() {
        let c = Coord2::new(3, 5);
        let idx = c.to_index(10).unwrap();
        assert_eq!(Coord2::from_index(idx, 10), c);
    }

    #[test]
    fn coord2_out_of_bounds() {
        assert_eq!(Coord2::new(-1, 0).to_index(10), None);
        assert_eq!(Coord2::new(10, 0).to_index(10), None);
    }

    #[test]
    fn coord3_index_roundtrip() {
        let c = Coord3::new(2, 3, 4);
        let idx = c.to_index(8, 7).unwrap();
        assert_eq!(Coord3::from_index(idx, 8, 7), c);
    }
}
