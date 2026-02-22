use alloc::vec::Vec;

use crate::coord::Coord3;
use crate::grid_trait::SpatialGrid;

/// 3D volumetric grid backed by a flat Vec.
#[derive(Clone, Debug)]
pub struct Grid3<T> {
    cells: Vec<T>,
    width: u32,
    height: u32,
    depth: u32,
}

impl<T: Clone> Grid3<T> {
    pub fn new_filled(width: u32, height: u32, depth: u32, fill: T) -> Self {
        let len = width as usize * height as usize * depth as usize;
        Self {
            cells: alloc::vec![fill; len],
            width,
            height,
            depth,
        }
    }

    pub fn from_fn(
        width: u32,
        height: u32,
        depth: u32,
        mut f: impl FnMut(i32, i32, i32) -> T,
    ) -> Self {
        let mut cells = Vec::with_capacity(width as usize * height as usize * depth as usize);
        for z in 0..depth as i32 {
            for y in 0..height as i32 {
                for x in 0..width as i32 {
                    cells.push(f(x, y, z));
                }
            }
        }
        Self {
            cells,
            width,
            height,
            depth,
        }
    }

    #[inline]
    pub const fn depth(&self) -> u32 {
        self.depth
    }

    #[inline]
    pub const fn width(&self) -> u32 {
        self.width
    }

    #[inline]
    pub const fn height(&self) -> u32 {
        self.height
    }

    pub fn get_3d(&self, coord: Coord3) -> Option<&T> {
        self.coord_to_index(coord).and_then(|i| self.cells.get(i))
    }

    pub fn set_3d(&mut self, coord: Coord3, value: T) {
        if let Some(index) = self.coord_to_index(coord) {
            self.cells[index] = value;
        }
    }

    pub fn slice(&self, z: u32) -> &[T] {
        assert!(z < self.depth, "z out of bounds");
        let layer = self.width as usize * self.height as usize;
        let start = z as usize * layer;
        &self.cells[start..start + layer]
    }
}

impl<T: Clone> SpatialGrid for Grid3<T> {
    type Cell = T;
    type Coord = Coord3;

    fn width(&self) -> u32 {
        self.width
    }

    fn height(&self) -> u32 {
        self.height
    }

    fn len(&self) -> usize {
        self.cells.len()
    }

    fn in_bounds(&self, coord: Self::Coord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && coord.z >= 0
            && coord.x < self.width as i32
            && coord.y < self.height as i32
            && coord.z < self.depth as i32
    }

    fn get(&self, coord: Self::Coord) -> Option<&Self::Cell> {
        self.get_3d(coord)
    }

    fn get_mut(&mut self, coord: Self::Coord) -> Option<&mut Self::Cell> {
        self.coord_to_index(coord)
            .and_then(move |i| self.cells.get_mut(i))
    }

    fn set(&mut self, coord: Self::Coord, value: Self::Cell) {
        self.set_3d(coord, value);
    }

    fn index_to_coord(&self, index: usize) -> Self::Coord {
        let layer = self.width as usize * self.height as usize;
        let z = index / layer;
        let rem = index % layer;
        let y = rem / self.width as usize;
        let x = rem % self.width as usize;
        Coord3::new(x as i32, y as i32, z as i32)
    }

    fn coord_to_index(&self, coord: Self::Coord) -> Option<usize> {
        if !self.in_bounds(coord) {
            return None;
        }
        Some(
            coord.z as usize * (self.width as usize * self.height as usize)
                + coord.y as usize * self.width as usize
                + coord.x as usize,
        )
    }

    fn neighbors(&self, coord: Self::Coord) -> Vec<Self::Coord> {
        let candidates = [
            Coord3::new(coord.x + 1, coord.y, coord.z),
            Coord3::new(coord.x - 1, coord.y, coord.z),
            Coord3::new(coord.x, coord.y + 1, coord.z),
            Coord3::new(coord.x, coord.y - 1, coord.z),
            Coord3::new(coord.x, coord.y, coord.z + 1),
            Coord3::new(coord.x, coord.y, coord.z - 1),
        ];
        let mut out = Vec::with_capacity(6);
        for c in candidates {
            if self.in_bounds(c) {
                out.push(c);
            }
        }
        out
    }

    fn move_cost(&self, from: Self::Coord, to: Self::Coord) -> Option<u32> {
        if !self.in_bounds(from) || !self.in_bounds(to) {
            return None;
        }
        let dx = (to.x - from.x).abs();
        let dy = (to.y - from.y).abs();
        let dz = (to.z - from.z).abs();
        if dx + dy + dz == 1 {
            Some(10)
        } else if dx == 0 && dy == 0 && dz == 0 {
            Some(0)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid3_3d_indexing() {
        let g = Grid3::from_fn(3, 3, 2, |x, y, z| x + y * 10 + z * 100);
        assert_eq!(g.get_3d(Coord3::new(2, 1, 0)), Some(&12));
        assert_eq!(g.get_3d(Coord3::new(2, 1, 1)), Some(&112));
    }

    #[test]
    fn grid3_layers_independent() {
        let mut g = Grid3::new_filled(2, 2, 2, 0u8);
        g.set_3d(Coord3::new(1, 1, 0), 1);
        g.set_3d(Coord3::new(1, 1, 1), 2);
        assert_eq!(g.get_3d(Coord3::new(1, 1, 0)), Some(&1));
        assert_eq!(g.get_3d(Coord3::new(1, 1, 1)), Some(&2));
    }

    #[test]
    fn grid3_out_of_bounds() {
        let g = Grid3::new_filled(2, 2, 2, 0u8);
        assert_eq!(g.get_3d(Coord3::new(2, 0, 0)), None);
    }
}
