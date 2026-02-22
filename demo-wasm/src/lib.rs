use rand_core::RngCore;
use tessera::{
    astar, bresenham, bsp_generate, cellular_generate, drunkard_generate, jps, shadowcast, BspConfig,
    CellularConfig, Coord2, DrunkardConfig, Grid, HexCoord, HexGrid, SpatialGrid,
};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct GridWorld {
    width: u32,
    height: u32,
    passable: Grid<bool>,
    start: Coord2,
    goal: Coord2,
    origin: Coord2,
    astar_cells: Vec<Coord2>,
    jps_cells: Vec<Coord2>,
    astar_cost: u32,
    jps_cost: u32,
    astar_nodes: u32,
    jps_nodes: u32,
}

#[wasm_bindgen]
impl GridWorld {
    #[wasm_bindgen(constructor)]
    pub fn new(width: u32, height: u32) -> GridWorld {
        let width = width.max(4);
        let height = height.max(4);
        let mut world = GridWorld {
            width,
            height,
            passable: Grid::new_filled(width, height, true),
            start: Coord2::new(1, 1),
            goal: Coord2::new(width as i32 - 2, height as i32 - 2),
            origin: Coord2::new(width as i32 / 2, height as i32 / 2),
            astar_cells: Vec::new(),
            jps_cells: Vec::new(),
            astar_cost: 0,
            jps_cost: 0,
            astar_nodes: 0,
            jps_nodes: 0,
        };
        world.recompute_paths();
        world
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn tick(&mut self) {
        self.recompute_paths();
    }

    pub fn clear_walls(&mut self) {
        self.passable = Grid::new_filled(self.width, self.height, true);
        self.normalize_points();
        self.recompute_paths();
    }

    pub fn toggle_wall(&mut self, x: i32, y: i32) {
        let c = Coord2::new(x, y);
        if !self.passable.in_bounds(c) || c == self.start || c == self.goal {
            return;
        }
        let v = self.passable.get(c).copied().unwrap_or(false);
        self.passable.set(c, !v);
        self.normalize_points();
        self.recompute_paths();
    }

    pub fn set_start(&mut self, x: i32, y: i32) {
        let c = Coord2::new(x, y);
        if self.passable.in_bounds(c) && self.is_passable(c) {
            self.start = c;
            self.recompute_paths();
        }
    }

    pub fn set_goal(&mut self, x: i32, y: i32) {
        let c = Coord2::new(x, y);
        if self.passable.in_bounds(c) && self.is_passable(c) {
            self.goal = c;
            self.recompute_paths();
        }
    }

    pub fn set_origin(&mut self, x: i32, y: i32) {
        let c = Coord2::new(x, y);
        if self.passable.in_bounds(c) {
            self.origin = c;
        }
    }

    pub fn astar_cost(&self) -> u32 {
        self.astar_cost
    }

    pub fn astar_nodes(&self) -> u32 {
        self.astar_nodes
    }

    pub fn jps_cost(&self) -> u32 {
        self.jps_cost
    }

    pub fn jps_nodes(&self) -> u32 {
        self.jps_nodes
    }

    pub fn astar_path(&self) -> Vec<i32> {
        flatten_coord2(&self.astar_cells)
    }

    pub fn jps_path(&self) -> Vec<i32> {
        flatten_coord2(&self.jps_cells)
    }

    pub fn render(&self) -> Vec<u8> {
        let mut out = vec![0u8; self.width as usize * self.height as usize];

        for y in 0..self.height as i32 {
            for x in 0..self.width as i32 {
                let c = Coord2::new(x, y);
                let idx = self.index_of(c).unwrap();
                out[idx] = if self.is_passable(c) { 0 } else { 1 };
            }
        }

        for c in &self.astar_cells {
            if let Some(idx) = self.index_of(*c) {
                out[idx] = 4;
            }
        }

        for c in &self.jps_cells {
            if let Some(idx) = self.index_of(*c) {
                out[idx] = if out[idx] == 4 { 6 } else { 5 };
            }
        }

        if let Some(idx) = self.index_of(self.origin) {
            out[idx] = 7;
        }
        if let Some(idx) = self.index_of(self.start) {
            out[idx] = 2;
        }
        if let Some(idx) = self.index_of(self.goal) {
            out[idx] = 3;
        }

        out
    }

    pub fn fov_mask(&self, radius: u32) -> Vec<u8> {
        let fov = shadowcast(&self.passable, self.origin, radius, |c| !self.is_passable(c));
        let mut out = vec![0u8; self.width as usize * self.height as usize];
        for y in 0..self.height as i32 {
            for x in 0..self.width as i32 {
                let c = Coord2::new(x, y);
                if let Some(idx) = self.index_of(c) {
                    out[idx] = if fov.is_visible(c) { 1 } else { 0 };
                }
            }
        }
        out
    }

    pub fn generate_bsp(&mut self, seed: u64) {
        let mut rng = SimpleRng::new(seed);
        let cfg = BspConfig {
            width: self.width,
            height: self.height,
            min_room_size: 5,
            max_room_size: 12,
            max_depth: 4,
        };
        let result = bsp_generate(&cfg, &mut rng);

        let mut grid = Grid::new_filled(self.width, self.height, false);
        for room in &result.rooms {
            for y in room.y..room.y + room.height as i32 {
                for x in room.x..room.x + room.width as i32 {
                    let c = Coord2::new(x, y);
                    if grid.in_bounds(c) {
                        grid.set(c, true);
                    }
                }
            }
        }

        for corridor in &result.corridors {
            for c in bresenham(corridor.from, corridor.to) {
                if grid.in_bounds(c) {
                    grid.set(c, true);
                }
            }
        }

        if let Some(first) = result.rooms.first() {
            self.start = first.center();
        }
        if let Some(last) = result.rooms.last() {
            self.goal = last.center();
        }

        self.apply_grid(grid);
    }

    pub fn generate_cellular(&mut self, seed: u64) {
        let mut rng = SimpleRng::new(seed);
        let cfg = CellularConfig {
            width: self.width,
            height: self.height,
            fill_chance: 50,
            birth_limit: 5,
            death_limit: 3,
            iterations: 5,
        };
        let grid = cellular_generate(&cfg, &mut rng);
        self.apply_grid(grid);
    }

    pub fn generate_drunkard(&mut self, seed: u64) {
        let mut rng = SimpleRng::new(seed);
        let cfg = DrunkardConfig {
            width: self.width,
            height: self.height,
            max_steps: 1200,
            num_walkers: 12,
            turn_chance: 35,
            target_floor_ratio: 0.4,
        };
        let grid = drunkard_generate(&cfg, &mut rng);
        self.apply_grid(grid);
    }

    pub fn hex_distance(&self, aq: i32, ar: i32, bq: i32, br: i32) -> u32 {
        HexCoord::new(aq, ar).hex_distance(HexCoord::new(bq, br))
    }

    pub fn hex_ring(&self, q: i32, r: i32, radius: u32) -> Vec<i32> {
        flatten_hex(&HexCoord::new(q, r).ring(radius))
    }

    pub fn hex_spiral(&self, q: i32, r: i32, radius: u32) -> Vec<i32> {
        flatten_hex(&HexCoord::new(q, r).spiral(radius))
    }

    pub fn hex_path(
        &self,
        radius: u32,
        start_q: i32,
        start_r: i32,
        goal_q: i32,
        goal_r: i32,
        blocked: Vec<i32>,
    ) -> Vec<i32> {
        let mut grid = HexGrid::new_filled(radius, true);
        for pair in blocked.chunks(2) {
            if pair.len() == 2 {
                grid.set(HexCoord::new(pair[0], pair[1]), false);
            }
        }
        let start = HexCoord::new(start_q, start_r);
        let goal = HexCoord::new(goal_q, goal_r);
        if let Some(result) = astar(&grid, start, goal, |c| grid.get(c).copied().unwrap_or(false)) {
            flatten_hex(&result.path)
        } else {
            Vec::new()
        }
    }
}

impl GridWorld {
    fn apply_grid(&mut self, grid: Grid<bool>) {
        self.passable = grid;
        self.normalize_points();
        self.recompute_paths();
    }

    fn normalize_points(&mut self) {
        let fallback = self
            .first_passable()
            .unwrap_or(Coord2::new(self.width as i32 / 2, self.height as i32 / 2));
        if !self.passable.in_bounds(self.start) || !self.is_passable(self.start) {
            self.start = fallback;
        }
        if !self.passable.in_bounds(self.goal) || !self.is_passable(self.goal) {
            self.goal = fallback;
        }
        if !self.passable.in_bounds(self.origin) {
            self.origin = fallback;
        }
    }

    fn recompute_paths(&mut self) {
        let a = astar(&self.passable, self.start, self.goal, |c| self.is_passable(c));
        if let Some(r) = a {
            self.astar_cells = r.path;
            self.astar_cost = r.cost;
            self.astar_nodes = r.nodes_expanded;
        } else {
            self.astar_cells.clear();
            self.astar_cost = 0;
            self.astar_nodes = 0;
        }

        let j = jps(&self.passable, self.start, self.goal, |c| self.is_passable(c));
        if let Some(r) = j {
            self.jps_cells = r.path;
            self.jps_cost = r.cost;
            self.jps_nodes = r.nodes_expanded;
        } else {
            self.jps_cells.clear();
            self.jps_cost = 0;
            self.jps_nodes = 0;
        }
    }

    fn first_passable(&self) -> Option<Coord2> {
        for y in 0..self.height as i32 {
            for x in 0..self.width as i32 {
                let c = Coord2::new(x, y);
                if self.is_passable(c) {
                    return Some(c);
                }
            }
        }
        None
    }

    fn index_of(&self, coord: Coord2) -> Option<usize> {
        if !self.passable.in_bounds(coord) {
            return None;
        }
        Some(coord.y as usize * self.width as usize + coord.x as usize)
    }

    fn is_passable(&self, coord: Coord2) -> bool {
        self.passable.get(coord).copied().unwrap_or(false)
    }
}

fn flatten_coord2(points: &[Coord2]) -> Vec<i32> {
    let mut out = Vec::with_capacity(points.len() * 2);
    for p in points {
        out.push(p.x);
        out.push(p.y);
    }
    out
}

fn flatten_hex(points: &[HexCoord]) -> Vec<i32> {
    let mut out = Vec::with_capacity(points.len() * 2);
    for p in points {
        out.push(p.q);
        out.push(p.r);
    }
    out
}

struct SimpleRng(u64);

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self(if seed == 0 { 0x9E3779B97F4A7C15 } else { seed })
    }
}

impl RngCore for SimpleRng {
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
