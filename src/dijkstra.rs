use alloc::vec::Vec;

use crate::coord::Coord2;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;

#[derive(Clone, Debug)]
pub struct DijkstraMap {
    width: u32,
    height: u32,
    values: Vec<u32>,
}

impl DijkstraMap {
    pub fn from_grid(grid: &Grid<bool>, goals: &[Coord2]) -> Option<Self> {
        if goals.is_empty() {
            return None;
        }
        let mut values = alloc::vec![u32::MAX; grid.len()];
        let mut open = Vec::<(u32, usize)>::new();

        for &goal in goals {
            let idx = grid.coord_to_index(goal)?;
            if !grid.get(goal).copied().unwrap_or(false) {
                continue;
            }
            values[idx] = 0;
            open.push((0, idx));
        }

        while let Some((dist, idx)) = pop_min(&mut open) {
            if dist != values[idx] {
                continue;
            }
            let coord = grid.index_to_coord(idx);
            for next in grid.neighbors(coord) {
                if !grid.get(next).copied().unwrap_or(false) {
                    continue;
                }
                let next_idx = match grid.coord_to_index(next) {
                    Some(i) => i,
                    None => continue,
                };
                let step = grid.move_cost(coord, next).unwrap_or(10);
                let tentative = dist.saturating_add(step);
                if tentative < values[next_idx] {
                    values[next_idx] = tentative;
                    open.push((tentative, next_idx));
                }
            }
        }

        Some(Self {
            width: grid.width(),
            height: grid.height(),
            values,
        })
    }

    #[inline]
    pub fn width(&self) -> u32 {
        self.width
    }

    #[inline]
    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn get(&self, coord: Coord2) -> Option<u32> {
        if coord.x < 0
            || coord.y < 0
            || coord.x >= self.width as i32
            || coord.y >= self.height as i32
        {
            return None;
        }
        let idx = coord.y as usize * self.width as usize + coord.x as usize;
        let v = self.values[idx];
        if v == u32::MAX { None } else { Some(v) }
    }

    pub fn downhill(&self, grid: &Grid<bool>, coord: Coord2) -> Option<Coord2> {
        let current = self.get(coord)?;
        let mut best = None;
        let mut best_cost = current;
        for n in grid.neighbors(coord) {
            if let Some(cost) = self.get(n) {
                if cost < best_cost {
                    best_cost = cost;
                    best = Some(n);
                }
            }
        }
        best
    }

    pub fn uphill(&self, grid: &Grid<bool>, coord: Coord2) -> Option<Coord2> {
        let current = self.get(coord)?;
        let mut best = None;
        let mut best_cost = current;
        for n in grid.neighbors(coord) {
            if let Some(cost) = self.get(n) {
                if cost > best_cost {
                    best_cost = cost;
                    best = Some(n);
                }
            }
        }
        best
    }

    #[inline]
    pub fn values(&self) -> &[u32] {
        &self.values
    }
}

fn pop_min(open: &mut Vec<(u32, usize)>) -> Option<(u32, usize)> {
    if open.is_empty() {
        return None;
    }
    let mut min_i = 0usize;
    let mut min_v = open[0];
    for (i, v) in open.iter().copied().enumerate().skip(1) {
        if v.0 < min_v.0 || (v.0 == min_v.0 && v.1 < min_v.1) {
            min_i = i;
            min_v = v;
        }
    }
    Some(open.swap_remove(min_i))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dijkstra_single_goal() {
        let g = Grid::new_filled(5, 5, true);
        let map = DijkstraMap::from_grid(&g, &[Coord2::new(2, 2)]).unwrap();
        assert_eq!(map.get(Coord2::new(2, 2)), Some(0));
        assert!(map.get(Coord2::new(0, 0)).unwrap() > 0);
    }

    #[test]
    fn dijkstra_multi_goal() {
        let g = Grid::new_filled(10, 1, true);
        let map = DijkstraMap::from_grid(&g, &[Coord2::new(0, 0), Coord2::new(9, 0)]).unwrap();
        assert_eq!(map.get(Coord2::new(0, 0)), Some(0));
        assert_eq!(map.get(Coord2::new(9, 0)), Some(0));
        assert_eq!(map.get(Coord2::new(5, 0)), Some(40));
    }

    #[test]
    fn dijkstra_downhill() {
        let g = Grid::new_filled(5, 5, true);
        let map = DijkstraMap::from_grid(&g, &[Coord2::new(2, 2)]).unwrap();
        let next = map.downhill(&g, Coord2::new(4, 4)).unwrap();
        assert!(map.get(next).unwrap() < map.get(Coord2::new(4, 4)).unwrap());
    }

    #[test]
    fn dijkstra_uphill() {
        let g = Grid::new_filled(5, 5, true);
        let map = DijkstraMap::from_grid(&g, &[Coord2::new(2, 2)]).unwrap();
        let next = map.uphill(&g, Coord2::new(2, 2)).unwrap();
        assert!(map.get(next).unwrap() > map.get(Coord2::new(2, 2)).unwrap());
    }
}
