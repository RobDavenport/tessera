use rand_core::RngCore;

use crate::coord::Coord2;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;
use crate::neighborhood::Dir4;

#[derive(Clone, Debug)]
pub struct DrunkardConfig {
    pub width: u32,
    pub height: u32,
    pub max_steps: u32,
    pub num_walkers: u32,
    pub turn_chance: u32,
    pub target_floor_ratio: f32,
}

impl Default for DrunkardConfig {
    fn default() -> Self {
        Self {
            width: 64,
            height: 64,
            max_steps: 800,
            num_walkers: 8,
            turn_chance: 35,
            target_floor_ratio: 0.40,
        }
    }
}

pub fn drunkard_generate(config: &DrunkardConfig, rng: &mut impl RngCore) -> Grid<bool> {
    let mut grid = Grid::new_filled(config.width, config.height, false);
    let target = (config.target_floor_ratio.clamp(0.0, 1.0) * grid.len() as f32) as usize;
    let mut carved = 0usize;

    let center = Coord2::new(config.width as i32 / 2, config.height as i32 / 2);
    for _ in 0..config.num_walkers {
        let mut pos = center;
        let mut dir = Dir4::ALL[(rng.next_u32() % 4) as usize];

        for _ in 0..config.max_steps {
            if !grid.get(pos).copied().unwrap_or(false) {
                grid.set(pos, true);
                carved += 1;
                if carved >= target {
                    return grid;
                }
            }

            if rng.next_u32() % 100 < config.turn_chance {
                dir = Dir4::ALL[(rng.next_u32() % 4) as usize];
            }

            let mut next = pos + dir.offset();
            next.x = next.x.clamp(1, config.width as i32 - 2);
            next.y = next.y.clamp(1, config.height as i32 - 2);
            pos = next;
        }
    }

    grid
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flood::flood_fill;

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
    fn drunkard_target_ratio() {
        let cfg = DrunkardConfig {
            width: 40,
            height: 40,
            max_steps: 1500,
            num_walkers: 10,
            turn_chance: 30,
            target_floor_ratio: 0.35,
        };
        let mut rng = TestRng(3);
        let grid = drunkard_generate(&cfg, &mut rng);
        let floors = grid.cells().iter().filter(|v| **v).count() as f32;
        let ratio = floors / grid.len() as f32;
        assert!((ratio - cfg.target_floor_ratio).abs() <= 0.05);
    }

    #[test]
    fn drunkard_connected() {
        let cfg = DrunkardConfig {
            width: 40,
            height: 40,
            max_steps: 1000,
            num_walkers: 8,
            turn_chance: 35,
            target_floor_ratio: 0.30,
        };
        let mut rng = TestRng(12);
        let grid = drunkard_generate(&cfg, &mut rng);
        let center = Coord2::new(cfg.width as i32 / 2, cfg.height as i32 / 2);
        let region = flood_fill(&grid, center, |c| grid.get(c).copied().unwrap_or(false));
        let total_floor = grid.cells().iter().filter(|v| **v).count();
        assert!(region.cells.len() >= total_floor / 2);
    }
}
