use crate::bsp::Room;
use crate::coord::Coord2;

pub trait PathfindObserver {
    fn on_expand(&mut self, _coord: Coord2, _g: u32, _f: u32) {}
    fn on_path_found(&mut self, _path: &[Coord2], _cost: u32) {}
    fn on_no_path(&mut self) {}
}

pub trait GenerationObserver {
    fn on_room_placed(&mut self, _room: &Room) {}
    fn on_corridor_carved(&mut self, _from: Coord2, _to: Coord2) {}
    fn on_iteration(&mut self, _iteration: u32) {}
}

pub struct NoOpPathfindObserver;
impl PathfindObserver for NoOpPathfindObserver {}

pub struct NoOpGenerationObserver;
impl GenerationObserver for NoOpGenerationObserver {}
