use alloc::vec::Vec;

use rand_core::RngCore;

#[derive(Clone, Debug)]
pub struct PoissonConfig {
    pub width: f32,
    pub height: f32,
    pub min_distance: f32,
    pub max_attempts: u32,
}

impl Default for PoissonConfig {
    fn default() -> Self {
        Self {
            width: 100.0,
            height: 100.0,
            min_distance: 8.0,
            max_attempts: 30,
        }
    }
}

pub fn poisson_generate(config: &PoissonConfig, rng: &mut impl RngCore) -> Vec<(f32, f32)> {
    if config.width <= 0.0 || config.height <= 0.0 || config.min_distance <= 0.0 {
        return Vec::new();
    }

    let cell_size = config.min_distance / libm::sqrtf(2.0);
    let grid_w = libm::ceilf(config.width / cell_size) as usize;
    let grid_h = libm::ceilf(config.height / cell_size) as usize;
    let mut bg = alloc::vec![usize::MAX; grid_w * grid_h];

    let mut points = Vec::<(f32, f32)>::new();
    let mut active = Vec::<usize>::new();

    let first = (rand01(rng) * config.width, rand01(rng) * config.height);
    points.push(first);
    active.push(0);
    place_in_grid(first, 0, &mut bg, grid_w, cell_size);

    while !active.is_empty() {
        let active_pos = (rng.next_u32() as usize) % active.len();
        let point_index = active[active_pos];
        let p = points[point_index];
        let mut accepted = false;

        for _ in 0..config.max_attempts {
            let angle = rand01(rng) * core::f32::consts::TAU;
            let distance = config.min_distance * (1.0 + rand01(rng));
            let c = (
                p.0 + libm::cosf(angle) * distance,
                p.1 + libm::sinf(angle) * distance,
            );

            if c.0 < 0.0 || c.1 < 0.0 || c.0 >= config.width || c.1 >= config.height {
                continue;
            }

            if is_valid(
                c,
                config.min_distance,
                &points,
                &bg,
                grid_w,
                grid_h,
                cell_size,
            ) {
                let idx = points.len();
                points.push(c);
                active.push(idx);
                place_in_grid(c, idx, &mut bg, grid_w, cell_size);
                accepted = true;
                break;
            }
        }

        if !accepted {
            active.swap_remove(active_pos);
        }
    }

    points
}

fn rand01(rng: &mut impl RngCore) -> f32 {
    (rng.next_u32() as f32) / (u32::MAX as f32)
}

fn place_in_grid(
    p: (f32, f32),
    point_index: usize,
    bg: &mut [usize],
    grid_w: usize,
    cell_size: f32,
) {
    let gx = (p.0 / cell_size) as usize;
    let gy = (p.1 / cell_size) as usize;
    let idx = gy * grid_w + gx;
    bg[idx] = point_index;
}

fn is_valid(
    c: (f32, f32),
    min_distance: f32,
    points: &[(f32, f32)],
    bg: &[usize],
    grid_w: usize,
    grid_h: usize,
    cell_size: f32,
) -> bool {
    let gx = (c.0 / cell_size) as i32;
    let gy = (c.1 / cell_size) as i32;
    let r2 = min_distance * min_distance;

    let min_x = (gx - 2).max(0);
    let max_x = (gx + 2).min(grid_w as i32 - 1);
    let min_y = (gy - 2).max(0);
    let max_y = (gy + 2).min(grid_h as i32 - 1);

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let bg_idx = y as usize * grid_w + x as usize;
            let p_idx = bg[bg_idx];
            if p_idx == usize::MAX {
                continue;
            }
            let p = points[p_idx];
            let dx = p.0 - c.0;
            let dy = p.1 - c.1;
            if dx * dx + dy * dy < r2 {
                return false;
            }
        }
    }
    true
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
    fn poisson_min_distance() {
        let cfg = PoissonConfig {
            width: 120.0,
            height: 80.0,
            min_distance: 7.0,
            max_attempts: 30,
        };
        let mut rng = TestRng(99);
        let points = poisson_generate(&cfg, &mut rng);
        for i in 0..points.len() {
            for j in (i + 1)..points.len() {
                let dx = points[i].0 - points[j].0;
                let dy = points[i].1 - points[j].1;
                let d = libm::sqrtf(dx * dx + dy * dy);
                assert!(d + 1e-4 >= cfg.min_distance);
            }
        }
    }

    #[test]
    fn poisson_coverage() {
        let cfg = PoissonConfig {
            width: 100.0,
            height: 100.0,
            min_distance: 12.0,
            max_attempts: 30,
        };
        let mut rng = TestRng(123);
        let points = poisson_generate(&cfg, &mut rng);
        assert!(!points.is_empty());
        let mut quadrants = [0usize; 4];
        for (x, y) in points {
            let q = match (x >= 50.0, y >= 50.0) {
                (false, false) => 0,
                (true, false) => 1,
                (false, true) => 2,
                (true, true) => 3,
            };
            quadrants[q] += 1;
        }
        assert!(quadrants.iter().all(|c| *c > 0));
    }
}
