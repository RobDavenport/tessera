use alloc::vec::Vec;

use crate::coord::Coord2;
use crate::dijkstra::DijkstraMap;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;
use crate::neighborhood::Dir8;

/// Flow field represented as direction indices per cell.
/// `-1` means no direction / unreachable / goal.
#[derive(Clone, Debug)]
pub struct FlowFieldGrid {
    directions: Vec<i8>,
    width: u32,
    height: u32,
}

impl FlowFieldGrid {
    pub fn new_empty(width: u32, height: u32) -> Self {
        Self {
            directions: alloc::vec![-1i8; width as usize * height as usize],
            width,
            height,
        }
    }

    pub fn from_grid(grid: &Grid<bool>, goals: &[Coord2]) -> Option<Self> {
        let map = DijkstraMap::from_grid(grid, goals)?;
        let mut directions = alloc::vec![-1i8; grid.len()];

        for y in 0..grid.height() as i32 {
            for x in 0..grid.width() as i32 {
                let c = Coord2::new(x, y);
                let idx = match grid.coord_to_index(c) {
                    Some(i) => i,
                    None => continue,
                };
                if !grid.get(c).copied().unwrap_or(false) {
                    continue;
                }

                let current = match map.get(c) {
                    Some(v) => v,
                    None => continue,
                };
                if current == 0 {
                    continue;
                }

                let mut best_dir = -1i8;
                let mut best_cost = current;
                for (dir_idx, dir) in Dir8::ALL.iter().copied().enumerate() {
                    let n = c + dir.offset();
                    if let Some(next_cost) = map.get(n) {
                        if next_cost < best_cost {
                            best_cost = next_cost;
                            best_dir = dir_idx as i8;
                        }
                    }
                }
                directions[idx] = best_dir;
            }
        }

        Some(Self {
            directions,
            width: grid.width(),
            height: grid.height(),
        })
    }

    #[inline]
    pub const fn width(&self) -> u32 {
        self.width
    }

    #[inline]
    pub const fn height(&self) -> u32 {
        self.height
    }

    pub fn direction_at(&self, coord: Coord2) -> Option<Coord2> {
        let idx = self.index_of(coord)?;
        let v = self.directions[idx];
        if v < 0 {
            return None;
        }
        Some(Dir8::ALL[v as usize].offset())
    }

    pub fn trace_path(&self, start: Coord2, max_steps: u32) -> Vec<Coord2> {
        let mut path = Vec::with_capacity(max_steps as usize + 1);
        let mut current = start;
        path.push(current);

        for _ in 0..max_steps {
            let dir = match self.direction_at(current) {
                Some(d) => d,
                None => break,
            };
            let next = current + dir;
            if self.index_of(next).is_none() {
                break;
            }
            if next == current {
                break;
            }
            current = next;
            path.push(current);
        }

        path
    }

    #[inline]
    pub fn directions(&self) -> &[i8] {
        &self.directions
    }

    fn index_of(&self, coord: Coord2) -> Option<usize> {
        if coord.x < 0
            || coord.y < 0
            || coord.x >= self.width as i32
            || coord.y >= self.height as i32
        {
            return None;
        }
        Some(coord.y as usize * self.width as usize + coord.x as usize)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn flow_field_all_reachable() {
        let g = Grid::new_filled(8, 8, true);
        let f = FlowFieldGrid::from_grid(&g, &[Coord2::new(4, 4)]).unwrap();
        let mut with_dir = 0usize;
        for y in 0..8 {
            for x in 0..8 {
                if f.direction_at(Coord2::new(x, y)).is_some() {
                    with_dir += 1;
                }
            }
        }
        assert!(with_dir >= 8 * 8 - 1);
    }

    #[test]
    fn flow_field_trace() {
        let g = Grid::new_filled(10, 10, true);
        let goal = Coord2::new(9, 9);
        let f = FlowFieldGrid::from_grid(&g, &[goal]).unwrap();
        let path = f.trace_path(Coord2::new(0, 0), 32);
        assert_eq!(path.last().copied(), Some(goal));
    }
}
