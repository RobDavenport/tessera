use alloc::vec::Vec;

use crate::coord::Coord2;
use crate::grid_trait::SpatialGrid;
use crate::neighborhood::Dir8;

/// 2D rectangular grid backed by a flat Vec.
#[derive(Clone, Debug)]
pub struct Grid<T> {
    cells: Vec<T>,
    width: u32,
    height: u32,
}

impl<T: Clone> Grid<T> {
    pub fn new_filled(width: u32, height: u32, fill: T) -> Self {
        let len = width as usize * height as usize;
        Self {
            cells: alloc::vec![fill; len],
            width,
            height,
        }
    }

    pub fn from_fn(width: u32, height: u32, mut f: impl FnMut(i32, i32) -> T) -> Self {
        let mut cells = Vec::with_capacity(width as usize * height as usize);
        for y in 0..height as i32 {
            for x in 0..width as i32 {
                cells.push(f(x, y));
            }
        }
        Self {
            cells,
            width,
            height,
        }
    }

    pub fn from_vec(width: u32, height: u32, cells: Vec<T>) -> Self {
        let expected = width as usize * height as usize;
        assert_eq!(cells.len(), expected, "cell vector size mismatch");
        Self {
            cells,
            width,
            height,
        }
    }

    #[inline]
    pub const fn width(&self) -> u32 {
        self.width
    }

    #[inline]
    pub const fn height(&self) -> u32 {
        self.height
    }

    pub fn row(&self, y: u32) -> &[T] {
        assert!(y < self.height, "row out of bounds");
        let start = y as usize * self.width as usize;
        &self.cells[start..start + self.width as usize]
    }

    pub fn row_mut(&mut self, y: u32) -> &mut [T] {
        assert!(y < self.height, "row out of bounds");
        let start = y as usize * self.width as usize;
        let width = self.width as usize;
        &mut self.cells[start..start + width]
    }

    pub fn iter_coords(&self) -> impl Iterator<Item = (Coord2, &T)> {
        self.cells
            .iter()
            .enumerate()
            .map(move |(index, cell)| (Coord2::from_index(index, self.width), cell))
    }

    #[inline]
    pub fn cells(&self) -> &[T] {
        &self.cells
    }

    #[inline]
    pub fn cells_mut(&mut self) -> &mut [T] {
        &mut self.cells
    }
}

impl<T: Clone> SpatialGrid for Grid<T> {
    type Cell = T;
    type Coord = Coord2;

    #[inline]
    fn width(&self) -> u32 {
        self.width
    }

    #[inline]
    fn height(&self) -> u32 {
        self.height
    }

    #[inline]
    fn len(&self) -> usize {
        self.cells.len()
    }

    #[inline]
    fn in_bounds(&self, coord: Self::Coord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && coord.x < self.width as i32
            && coord.y < self.height as i32
    }

    fn get(&self, coord: Self::Coord) -> Option<&Self::Cell> {
        self.coord_to_index(coord).and_then(|i| self.cells.get(i))
    }

    fn get_mut(&mut self, coord: Self::Coord) -> Option<&mut Self::Cell> {
        self.coord_to_index(coord)
            .and_then(move |i| self.cells.get_mut(i))
    }

    fn set(&mut self, coord: Self::Coord, value: Self::Cell) {
        if let Some(index) = self.coord_to_index(coord) {
            self.cells[index] = value;
        }
    }

    #[inline]
    fn index_to_coord(&self, index: usize) -> Self::Coord {
        Coord2::from_index(index, self.width)
    }

    fn coord_to_index(&self, coord: Self::Coord) -> Option<usize> {
        if !self.in_bounds(coord) {
            return None;
        }
        Some(coord.y as usize * self.width as usize + coord.x as usize)
    }

    fn neighbors(&self, coord: Self::Coord) -> Vec<Self::Coord> {
        let mut out = Vec::with_capacity(8);
        for dir in Dir8::ALL {
            let next = coord + dir.offset();
            if self.in_bounds(next) {
                out.push(next);
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
        if dx == 0 && dy == 0 {
            return Some(0);
        }
        if dx <= 1 && dy <= 1 {
            return if dx == 1 && dy == 1 {
                Some(14)
            } else {
                Some(10)
            };
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_new_filled() {
        let g = Grid::new_filled(3, 2, 9u8);
        assert_eq!(g.cells(), &[9, 9, 9, 9, 9, 9]);
    }

    #[test]
    fn grid_from_fn() {
        let g = Grid::from_fn(3, 2, |x, y| x + y * 10);
        assert_eq!(g.get(Coord2::new(2, 1)), Some(&12));
    }

    #[test]
    fn grid_set_get() {
        let mut g = Grid::new_filled(2, 2, false);
        g.set(Coord2::new(1, 1), true);
        assert_eq!(g.get(Coord2::new(1, 1)), Some(&true));
    }

    #[test]
    fn grid_out_of_bounds() {
        let g = Grid::new_filled(2, 2, 0u8);
        assert_eq!(g.get(Coord2::new(2, 0)), None);
    }

    #[test]
    fn grid_row_slice() {
        let g = Grid::from_fn(4, 3, |x, y| x + y * 10);
        assert_eq!(g.row(1), &[10, 11, 12, 13]);
    }

    #[test]
    fn grid_neighbors_corner() {
        let g = Grid::new_filled(3, 3, 0u8);
        assert_eq!(g.neighbors(Coord2::new(0, 0)).len(), 3);
    }

    #[test]
    fn grid_neighbors_center() {
        let g = Grid::new_filled(3, 3, 0u8);
        assert_eq!(g.neighbors(Coord2::new(1, 1)).len(), 8);
    }

    #[test]
    fn grid_iter_coords() {
        let g = Grid::from_fn(2, 2, |x, y| x + y * 2);
        let coords: Vec<(Coord2, i32)> = g.iter_coords().map(|(c, v)| (c, *v)).collect();
        assert_eq!(
            coords,
            alloc::vec![
                (Coord2::new(0, 0), 0),
                (Coord2::new(1, 0), 1),
                (Coord2::new(0, 1), 2),
                (Coord2::new(1, 1), 3)
            ]
        );
    }
}
