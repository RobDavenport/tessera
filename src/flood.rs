use alloc::vec::Vec;

use crate::coord::Coord2;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;
use crate::neighborhood::Dir4;

#[derive(Clone, Debug)]
pub struct FloodResult {
    pub cells: Vec<Coord2>,
    pub visited: Grid<bool>,
}

pub fn flood_fill<F>(grid: &Grid<bool>, start: Coord2, mut is_passable: F) -> FloodResult
where
    F: FnMut(Coord2) -> bool,
{
    let mut visited = Grid::new_filled(grid.width(), grid.height(), false);
    let mut cells = Vec::new();

    if !grid.in_bounds(start) || !is_passable(start) {
        return FloodResult { cells, visited };
    }

    let mut stack = alloc::vec![start];
    visited.set(start, true);

    while let Some(current) = stack.pop() {
        cells.push(current);
        for dir in Dir4::ALL {
            let next = current + dir.offset();
            if !grid.in_bounds(next) {
                continue;
            }
            if visited.get(next).copied().unwrap_or(false) {
                continue;
            }
            if !is_passable(next) {
                continue;
            }
            visited.set(next, true);
            stack.push(next);
        }
    }

    FloodResult { cells, visited }
}

pub fn is_connected(grid: &Grid<bool>) -> bool {
    let mut first = None;
    let mut passable_total = 0usize;

    for y in 0..grid.height() as i32 {
        for x in 0..grid.width() as i32 {
            let c = Coord2::new(x, y);
            if grid.get(c).copied().unwrap_or(false) {
                passable_total += 1;
                if first.is_none() {
                    first = Some(c);
                }
            }
        }
    }

    if passable_total <= 1 {
        return true;
    }
    let start = match first {
        Some(c) => c,
        None => return true,
    };
    let region = flood_fill(grid, start, |c| grid.get(c).copied().unwrap_or(false));
    region.cells.len() == passable_total
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flood_fill_room() {
        let g = Grid::new_filled(5, 5, true);
        let res = flood_fill(&g, Coord2::new(2, 2), |c| g.get(c).copied().unwrap_or(false));
        assert_eq!(res.cells.len(), 25);
    }

    #[test]
    fn flood_fill_walls_block() {
        let mut g = Grid::new_filled(3, 3, true);
        g.set(Coord2::new(1, 0), false);
        g.set(Coord2::new(1, 1), false);
        g.set(Coord2::new(1, 2), false);
        let res = flood_fill(&g, Coord2::new(0, 1), |c| g.get(c).copied().unwrap_or(false));
        assert_eq!(res.cells.len(), 3);
    }

    #[test]
    fn is_connected_test() {
        let mut g = Grid::new_filled(3, 3, true);
        assert!(is_connected(&g));
        g.set(Coord2::new(1, 0), false);
        g.set(Coord2::new(1, 1), false);
        g.set(Coord2::new(1, 2), false);
        assert!(!is_connected(&g));
    }
}
