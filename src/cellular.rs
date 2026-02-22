use alloc::vec::Vec;

use rand_core::RngCore;

use crate::coord::Coord2;
use crate::flood::flood_fill;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;
use crate::los::bresenham;
use crate::neighborhood::Dir8;

#[derive(Clone, Debug)]
pub struct CellularConfig {
    pub width: u32,
    pub height: u32,
    pub fill_chance: u32,
    pub birth_limit: u32,
    pub death_limit: u32,
    pub iterations: u32,
}

impl Default for CellularConfig {
    fn default() -> Self {
        Self {
            width: 64,
            height: 64,
            fill_chance: 50,
            birth_limit: 5,
            death_limit: 3,
            iterations: 5,
        }
    }
}

pub fn cellular_generate(config: &CellularConfig, rng: &mut impl RngCore) -> Grid<bool> {
    let mut grid = Grid::from_fn(config.width, config.height, |_, _| {
        (rng.next_u32() % 100) < config.fill_chance
    });

    for _ in 0..config.iterations {
        let mut next = Grid::new_filled(config.width, config.height, false);
        for y in 0..config.height as i32 {
            for x in 0..config.width as i32 {
                let c = Coord2::new(x, y);
                let walls = wall_neighbors(&grid, c);
                let current_floor = grid.get(c).copied().unwrap_or(false);
                let next_floor = if current_floor {
                    walls < config.birth_limit
                } else {
                    walls < config.death_limit
                };
                next.set(c, next_floor);
            }
        }
        grid = next;
    }

    let mut seen = Grid::new_filled(config.width, config.height, false);
    let mut regions = Vec::<Vec<Coord2>>::new();

    for y in 0..config.height as i32 {
        for x in 0..config.width as i32 {
            let c = Coord2::new(x, y);
            if !grid.get(c).copied().unwrap_or(false) || seen.get(c).copied().unwrap_or(false) {
                continue;
            }
            let region = flood_fill(&grid, c, |p| grid.get(p).copied().unwrap_or(false));
            for rc in &region.cells {
                seen.set(*rc, true);
            }
            regions.push(region.cells);
        }
    }

    if regions.len() > 1 {
        let mut largest_idx = 0usize;
        for i in 1..regions.len() {
            if regions[i].len() > regions[largest_idx].len() {
                largest_idx = i;
            }
        }
        let anchor = regions[largest_idx][0];
        for (i, region) in regions.iter().enumerate() {
            if i == largest_idx || region.is_empty() {
                continue;
            }
            let from = region[0];
            for p in bresenham(from, anchor) {
                if grid.in_bounds(p) {
                    grid.set(p, true);
                }
            }
        }
    }

    grid
}

fn wall_neighbors(grid: &Grid<bool>, coord: Coord2) -> u32 {
    let mut count = 0u32;
    for dir in Dir8::ALL {
        let n = coord + dir.offset();
        if !grid.in_bounds(n) {
            count += 1;
            continue;
        }
        if !grid.get(n).copied().unwrap_or(false) {
            count += 1;
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestRng(u64);

    impl RngCore for TestRng {
        fn next_u32(&mut self) -> u32 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 32) as u32
        }
        fn next_u64(&mut self) -> u64 {
            ((self.next_u32() as u64) << 32) | self.next_u32() as u64
        }
        fn fill_bytes(&mut self, dest: &mut [u8]) {
            rand_core::impls::fill_bytes_via_next(self, dest);
        }
        fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
            self.fill_bytes(dest);
            Ok(())
        }
    }

    #[test]
    fn cellular_floor_ratio() {
        let cfg = CellularConfig {
            width: 48,
            height: 48,
            fill_chance: 52,
            birth_limit: 5,
            death_limit: 3,
            iterations: 5,
        };
        let mut rng = TestRng(7);
        let grid = cellular_generate(&cfg, &mut rng);
        let floor = grid.cells().iter().filter(|v| **v).count();
        let ratio = floor as f32 / grid.len() as f32;
        assert!(ratio > 0.15 && ratio < 0.85);
    }

    #[test]
    fn cellular_largest_cave() {
        let cfg = CellularConfig {
            width: 40,
            height: 40,
            fill_chance: 50,
            birth_limit: 5,
            death_limit: 3,
            iterations: 5,
        };
        let mut rng = TestRng(42);
        let grid = cellular_generate(&cfg, &mut rng);

        let mut best = 0usize;
        let mut seen = Grid::new_filled(grid.width(), grid.height(), false);
        let mut total_floor = 0usize;
        for y in 0..grid.height() as i32 {
            for x in 0..grid.width() as i32 {
                let c = Coord2::new(x, y);
                if !grid.get(c).copied().unwrap_or(false) {
                    continue;
                }
                total_floor += 1;
                if seen.get(c).copied().unwrap_or(false) {
                    continue;
                }
                let region = flood_fill(&grid, c, |p| grid.get(p).copied().unwrap_or(false));
                for rc in &region.cells {
                    seen.set(*rc, true);
                }
                best = best.max(region.cells.len());
            }
        }
        if total_floor > 0 {
            assert!(best as f32 / total_floor as f32 > 0.35);
        }
    }
}
