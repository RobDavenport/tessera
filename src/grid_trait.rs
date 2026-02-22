use alloc::vec::Vec;

/// Trait unifying all grid types for generic algorithms.
pub trait SpatialGrid {
    type Cell: Clone;
    type Coord: Copy + Eq;

    fn width(&self) -> u32;
    fn height(&self) -> u32;
    fn len(&self) -> usize;

    fn is_empty(&self) -> bool {
        self.len() == 0
    }

    fn in_bounds(&self, coord: Self::Coord) -> bool;

    fn get(&self, coord: Self::Coord) -> Option<&Self::Cell>;
    fn get_mut(&mut self, coord: Self::Coord) -> Option<&mut Self::Cell>;
    fn set(&mut self, coord: Self::Coord, value: Self::Cell);

    fn index_to_coord(&self, index: usize) -> Self::Coord;
    fn coord_to_index(&self, coord: Self::Coord) -> Option<usize>;

    fn neighbors(&self, coord: Self::Coord) -> Vec<Self::Coord>;

    fn move_cost(&self, from: Self::Coord, to: Self::Coord) -> Option<u32>;
}
