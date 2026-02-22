use alloc::vec::Vec;

use rand_core::RngCore;

use crate::coord::Coord2;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Room {
    pub x: i32,
    pub y: i32,
    pub width: u32,
    pub height: u32,
}

impl Room {
    pub fn new(x: i32, y: i32, width: u32, height: u32) -> Self {
        Self {
            x,
            y,
            width,
            height,
        }
    }

    #[inline]
    pub fn center(&self) -> Coord2 {
        Coord2::new(self.x + self.width as i32 / 2, self.y + self.height as i32 / 2)
    }

    #[inline]
    pub fn intersects(&self, other: &Room) -> bool {
        let a_left = self.x;
        let a_right = self.x + self.width as i32 - 1;
        let a_top = self.y;
        let a_bottom = self.y + self.height as i32 - 1;

        let b_left = other.x;
        let b_right = other.x + other.width as i32 - 1;
        let b_top = other.y;
        let b_bottom = other.y + other.height as i32 - 1;

        !(a_right < b_left || a_left > b_right || a_bottom < b_top || a_top > b_bottom)
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Corridor {
    pub from: Coord2,
    pub to: Coord2,
}

#[derive(Clone, Debug)]
pub struct BspConfig {
    pub width: u32,
    pub height: u32,
    pub min_room_size: u32,
    pub max_room_size: u32,
    pub max_depth: u32,
}

impl Default for BspConfig {
    fn default() -> Self {
        Self {
            width: 80,
            height: 50,
            min_room_size: 6,
            max_room_size: 14,
            max_depth: 4,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct BspResult {
    pub rooms: Vec<Room>,
    pub corridors: Vec<Corridor>,
}

#[derive(Copy, Clone)]
struct Rect {
    x: i32,
    y: i32,
    w: u32,
    h: u32,
}

pub fn bsp_generate(config: &BspConfig, rng: &mut impl RngCore) -> BspResult {
    let mut out = BspResult::default();
    let root = Rect {
        x: 0,
        y: 0,
        w: config.width,
        h: config.height,
    };
    split(
        root,
        0,
        config,
        rng,
        &mut out.rooms,
        &mut out.corridors,
    );
    out
}

fn split(
    rect: Rect,
    depth: u32,
    cfg: &BspConfig,
    rng: &mut impl RngCore,
    rooms: &mut Vec<Room>,
    corridors: &mut Vec<Corridor>,
) -> (usize, usize) {
    let min_split = cfg.min_room_size.saturating_mul(2).saturating_add(2);
    let can_split_h = rect.h >= min_split;
    let can_split_v = rect.w >= min_split;

    if depth >= cfg.max_depth || (!can_split_h && !can_split_v) {
        let room = carve_room(rect, cfg, rng);
        let idx = rooms.len();
        rooms.push(room);
        return (idx, idx);
    }

    let split_vertical = if can_split_v && can_split_h {
        if rect.w == rect.h {
            (rng.next_u32() & 1) == 0
        } else {
            rect.w > rect.h
        }
    } else {
        can_split_v
    };

    if split_vertical {
        let min = cfg.min_room_size + 1;
        let max = rect.w - cfg.min_room_size - 1;
        if min >= max {
            let room = carve_room(rect, cfg, rng);
            let idx = rooms.len();
            rooms.push(room);
            return (idx, idx);
        }
        let cut = rand_range_u32(rng, min, max);
        let left = Rect {
            x: rect.x,
            y: rect.y,
            w: cut,
            h: rect.h,
        };
        let right = Rect {
            x: rect.x + cut as i32,
            y: rect.y,
            w: rect.w - cut,
            h: rect.h,
        };
        let (l_first, l_last) = split(left, depth + 1, cfg, rng, rooms, corridors);
        let (r_first, r_last) = split(right, depth + 1, cfg, rng, rooms, corridors);
        connect_rooms(&rooms[l_last], &rooms[r_first], corridors);
        (l_first, r_last)
    } else {
        let min = cfg.min_room_size + 1;
        let max = rect.h - cfg.min_room_size - 1;
        if min >= max {
            let room = carve_room(rect, cfg, rng);
            let idx = rooms.len();
            rooms.push(room);
            return (idx, idx);
        }
        let cut = rand_range_u32(rng, min, max);
        let top = Rect {
            x: rect.x,
            y: rect.y,
            w: rect.w,
            h: cut,
        };
        let bottom = Rect {
            x: rect.x,
            y: rect.y + cut as i32,
            w: rect.w,
            h: rect.h - cut,
        };
        let (t_first, t_last) = split(top, depth + 1, cfg, rng, rooms, corridors);
        let (b_first, b_last) = split(bottom, depth + 1, cfg, rng, rooms, corridors);
        connect_rooms(&rooms[t_last], &rooms[b_first], corridors);
        (t_first, b_last)
    }
}

fn carve_room(rect: Rect, cfg: &BspConfig, rng: &mut impl RngCore) -> Room {
    let min_w = cfg.min_room_size.min(rect.w.max(1));
    let min_h = cfg.min_room_size.min(rect.h.max(1));
    let max_w = cfg.max_room_size.min(rect.w).max(min_w);
    let max_h = cfg.max_room_size.min(rect.h).max(min_h);

    let room_w = rand_range_u32(rng, min_w, max_w);
    let room_h = rand_range_u32(rng, min_h, max_h);

    let max_x_off = rect.w.saturating_sub(room_w);
    let max_y_off = rect.h.saturating_sub(room_h);
    let x_off = if max_x_off == 0 {
        0
    } else {
        rand_range_u32(rng, 0, max_x_off)
    };
    let y_off = if max_y_off == 0 {
        0
    } else {
        rand_range_u32(rng, 0, max_y_off)
    };

    Room::new(rect.x + x_off as i32, rect.y + y_off as i32, room_w, room_h)
}

fn connect_rooms(a: &Room, b: &Room, corridors: &mut Vec<Corridor>) {
    let ac = a.center();
    let bc = b.center();
    let turn = Coord2::new(bc.x, ac.y);
    corridors.push(Corridor { from: ac, to: turn });
    corridors.push(Corridor { from: turn, to: bc });
}

fn rand_range_u32(rng: &mut impl RngCore, min: u32, max: u32) -> u32 {
    if min >= max {
        return min;
    }
    let span = max - min + 1;
    min + (rng.next_u32() % span)
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
    fn bsp_rooms_no_overlap() {
        let cfg = BspConfig {
            width: 64,
            height: 48,
            min_room_size: 6,
            max_room_size: 10,
            max_depth: 4,
        };
        let mut rng = TestRng(12345);
        let result = bsp_generate(&cfg, &mut rng);
        for i in 0..result.rooms.len() {
            for j in (i + 1)..result.rooms.len() {
                assert!(!result.rooms[i].intersects(&result.rooms[j]));
            }
        }
    }

    #[test]
    fn bsp_all_connected() {
        let cfg = BspConfig {
            width: 64,
            height: 48,
            min_room_size: 6,
            max_room_size: 12,
            max_depth: 4,
        };
        let mut rng = TestRng(5);
        let result = bsp_generate(&cfg, &mut rng);
        if result.rooms.len() > 1 {
            assert!(!result.corridors.is_empty());
        }
    }

    #[test]
    fn bsp_min_size() {
        let cfg = BspConfig {
            width: 48,
            height: 48,
            min_room_size: 5,
            max_room_size: 9,
            max_depth: 3,
        };
        let mut rng = TestRng(9);
        let result = bsp_generate(&cfg, &mut rng);
        assert!(result
            .rooms
            .iter()
            .all(|r| r.width >= cfg.min_room_size && r.height >= cfg.min_room_size));
    }
}
