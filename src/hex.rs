use alloc::vec::Vec;
use core::ops::{Add, Mul, Neg, Sub};

use crate::grid_trait::SpatialGrid;

/// Hexagonal coordinate using axial (q, r) system.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct HexCoord {
    pub q: i32,
    pub r: i32,
}

impl HexCoord {
    pub const ZERO: HexCoord = HexCoord { q: 0, r: 0 };

    #[inline]
    pub const fn new(q: i32, r: i32) -> Self {
        Self { q, r }
    }

    #[inline]
    pub const fn s(self) -> i32 {
        -self.q - self.r
    }

    #[inline]
    pub fn hex_distance(self, other: HexCoord) -> u32 {
        let dq = (self.q - other.q).unsigned_abs();
        let dr = (self.r - other.r).unsigned_abs();
        let ds = (self.s() - other.s()).unsigned_abs();
        dq.max(dr).max(ds)
    }

    pub const NEIGHBOR_OFFSETS: [HexCoord; 6] = [
        HexCoord { q: 1, r: 0 },
        HexCoord { q: 1, r: -1 },
        HexCoord { q: 0, r: -1 },
        HexCoord { q: -1, r: 0 },
        HexCoord { q: -1, r: 1 },
        HexCoord { q: 0, r: 1 },
    ];

    pub fn neighbors(self) -> [HexCoord; 6] {
        let mut result = [HexCoord::ZERO; 6];
        let mut i = 0;
        while i < 6 {
            let o = Self::NEIGHBOR_OFFSETS[i];
            result[i] = HexCoord::new(self.q + o.q, self.r + o.r);
            i += 1;
        }
        result
    }

    pub fn ring(self, radius: u32) -> Vec<HexCoord> {
        if radius == 0 {
            return alloc::vec![self];
        }
        let r = radius as i32;
        let mut out = Vec::with_capacity((radius * 6) as usize);
        let mut cursor = self + HexCoord::NEIGHBOR_OFFSETS[4] * r;
        for dir in HexCoord::NEIGHBOR_OFFSETS {
            for _ in 0..radius {
                out.push(cursor);
                cursor = cursor + dir;
            }
        }
        out
    }

    pub fn spiral(self, radius: u32) -> Vec<HexCoord> {
        let mut out = Vec::with_capacity(HexGrid::<u8>::cell_count(radius));
        out.push(self);
        for r in 1..=radius {
            out.extend(self.ring(r));
        }
        out
    }
}

impl Add for HexCoord {
    type Output = HexCoord;

    fn add(self, rhs: Self) -> Self::Output {
        HexCoord::new(self.q + rhs.q, self.r + rhs.r)
    }
}

impl Sub for HexCoord {
    type Output = HexCoord;

    fn sub(self, rhs: Self) -> Self::Output {
        HexCoord::new(self.q - rhs.q, self.r - rhs.r)
    }
}

impl Neg for HexCoord {
    type Output = HexCoord;

    fn neg(self) -> Self::Output {
        HexCoord::new(-self.q, -self.r)
    }
}

impl Mul<i32> for HexCoord {
    type Output = HexCoord;

    fn mul(self, rhs: i32) -> Self::Output {
        HexCoord::new(self.q * rhs, self.r * rhs)
    }
}

/// Hexagonal grid using axial coordinates in a fixed radius around origin.
#[derive(Clone, Debug)]
pub struct HexGrid<T> {
    cells: Vec<T>,
    coords: Vec<HexCoord>,
    radius: u32,
}

impl<T: Clone> HexGrid<T> {
    pub fn new_filled(radius: u32, fill: T) -> Self {
        let coords = Self::collect_coords(radius);
        let cells = alloc::vec![fill; coords.len()];
        Self {
            cells,
            coords,
            radius,
        }
    }

    #[inline]
    pub const fn radius(&self) -> u32 {
        self.radius
    }

    pub const fn cell_count(radius: u32) -> usize {
        let r = radius as usize;
        3 * r * (r + 1) + 1
    }

    fn collect_coords(radius: u32) -> Vec<HexCoord> {
        let mut coords = Vec::with_capacity(Self::cell_count(radius));
        let r = radius as i32;
        for q in -r..=r {
            let r_min = (-r).max(-q - r);
            let r_max = r.min(-q + r);
            for rr in r_min..=r_max {
                coords.push(HexCoord::new(q, rr));
            }
        }
        coords
    }

    fn axial_to_index(&self, coord: HexCoord) -> Option<usize> {
        self.coords.iter().position(|c| *c == coord)
    }

    fn index_to_axial(&self, index: usize) -> HexCoord {
        self.coords[index]
    }
}

impl<T: Clone> SpatialGrid for HexGrid<T> {
    type Cell = T;
    type Coord = HexCoord;

    fn width(&self) -> u32 {
        self.radius * 2 + 1
    }

    fn height(&self) -> u32 {
        self.radius * 2 + 1
    }

    fn len(&self) -> usize {
        self.cells.len()
    }

    fn in_bounds(&self, coord: Self::Coord) -> bool {
        HexCoord::ZERO.hex_distance(coord) <= self.radius
    }

    fn get(&self, coord: Self::Coord) -> Option<&Self::Cell> {
        self.axial_to_index(coord).and_then(|i| self.cells.get(i))
    }

    fn get_mut(&mut self, coord: Self::Coord) -> Option<&mut Self::Cell> {
        self.axial_to_index(coord)
            .and_then(move |i| self.cells.get_mut(i))
    }

    fn set(&mut self, coord: Self::Coord, value: Self::Cell) {
        if let Some(index) = self.axial_to_index(coord) {
            self.cells[index] = value;
        }
    }

    fn index_to_coord(&self, index: usize) -> Self::Coord {
        self.index_to_axial(index)
    }

    fn coord_to_index(&self, coord: Self::Coord) -> Option<usize> {
        self.axial_to_index(coord)
    }

    fn neighbors(&self, coord: Self::Coord) -> Vec<Self::Coord> {
        let mut out = Vec::with_capacity(6);
        for n in coord.neighbors() {
            if self.in_bounds(n) {
                out.push(n);
            }
        }
        out
    }

    fn move_cost(&self, from: Self::Coord, to: Self::Coord) -> Option<u32> {
        if !self.in_bounds(from) || !self.in_bounds(to) {
            return None;
        }
        if from == to {
            Some(0)
        } else if from.hex_distance(to) == 1 {
            Some(10)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid_trait::SpatialGrid;

    #[test]
    fn hex_distance_adjacent() {
        assert_eq!(HexCoord::new(0, 0).hex_distance(HexCoord::new(1, 0)), 1);
    }

    #[test]
    fn hex_ring_count() {
        assert_eq!(HexCoord::ZERO.ring(0).len(), 1);
        assert_eq!(HexCoord::ZERO.ring(2).len(), 12);
    }

    #[test]
    fn hex_spiral_count() {
        assert_eq!(HexCoord::ZERO.spiral(0).len(), 1);
        assert_eq!(HexCoord::ZERO.spiral(3).len(), 37);
    }

    #[test]
    fn hexgrid_cell_count() {
        assert_eq!(HexGrid::<u8>::cell_count(2), 19);
    }

    #[test]
    fn hexgrid_neighbor_count() {
        let grid = HexGrid::new_filled(2, 0u8);
        assert_eq!(grid.neighbors(HexCoord::ZERO).len(), 6);
    }

    #[test]
    fn hexgrid_boundary() {
        let grid = HexGrid::new_filled(2, 0u8);
        assert!(grid.neighbors(HexCoord::new(2, 0)).len() < 6);
    }

    #[test]
    fn hexgrid_get_set() {
        let mut grid = HexGrid::new_filled(2, 0u8);
        grid.set(HexCoord::new(1, -1), 7);
        assert_eq!(grid.get(HexCoord::new(1, -1)), Some(&7));
    }
}
