use alloc::vec::Vec;

use crate::bsp::{BspResult, Corridor, Room};
use crate::coord::Coord2;
use crate::flow::FlowFieldGrid;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;
use crate::spatial_hash::Aabb2;

pub fn grid_from_wfc(width: u32, height: u32, tile_ids: &[u32]) -> Grid<u32> {
    let expected = width as usize * height as usize;
    assert_eq!(
        tile_ids.len(),
        expected,
        "tile id length mismatch for grid dimensions"
    );
    Grid::from_vec(width, height, tile_ids.to_vec())
}

pub fn layout_from_graph(nodes: &[(i32, i32, u32, u32)], edges: &[(usize, usize)]) -> BspResult {
    let rooms: Vec<Room> = nodes
        .iter()
        .copied()
        .map(|(x, y, w, h)| Room::new(x, y, w, h))
        .collect();

    let mut corridors = Vec::new();
    for &(a, b) in edges {
        if a >= rooms.len() || b >= rooms.len() {
            continue;
        }
        let ac = rooms[a].center();
        let bc = rooms[b].center();
        let turn = Coord2::new(bc.x, ac.y);
        corridors.push(Corridor { from: ac, to: turn });
        corridors.push(Corridor { from: turn, to: bc });
    }

    BspResult { rooms, corridors }
}

pub fn grid_to_flow_field(grid: &Grid<bool>, goals: &[Coord2]) -> FlowFieldGrid {
    FlowFieldGrid::from_grid(grid, goals).unwrap_or_else(|| FlowFieldGrid::new_empty(grid.width(), grid.height()))
}

pub fn walls_to_aabbs(grid: &Grid<bool>, mut is_wall: impl FnMut(Coord2) -> bool) -> Vec<Aabb2> {
    let mut visited = Grid::new_filled(grid.width(), grid.height(), false);
    let mut out = Vec::new();

    for y in 0..grid.height() as i32 {
        for x in 0..grid.width() as i32 {
            let start = Coord2::new(x, y);
            if visited.get(start).copied().unwrap_or(false) || !is_wall(start) {
                continue;
            }

            let mut end_x = x;
            while end_x + 1 < grid.width() as i32 {
                let c = Coord2::new(end_x + 1, y);
                if visited.get(c).copied().unwrap_or(false) || !is_wall(c) {
                    break;
                }
                end_x += 1;
            }

            let mut end_y = y;
            'extend_down: while end_y + 1 < grid.height() as i32 {
                let ny = end_y + 1;
                for cx in x..=end_x {
                    let c = Coord2::new(cx, ny);
                    if visited.get(c).copied().unwrap_or(false) || !is_wall(c) {
                        break 'extend_down;
                    }
                }
                end_y += 1;
            }

            for cy in y..=end_y {
                for cx in x..=end_x {
                    visited.set(Coord2::new(cx, cy), true);
                }
            }
            out.push(Aabb2::new(Coord2::new(x, y), Coord2::new(end_x, end_y)));
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn grid_from_wfc_roundtrip() {
        let ids = [1, 2, 3, 4, 5, 6];
        let g = grid_from_wfc(3, 2, &ids);
        assert_eq!(g.cells(), &ids);
    }

    #[test]
    fn layout_from_graph_creates_corridors() {
        let nodes = [(0, 0, 4, 4), (10, 0, 4, 4)];
        let edges = [(0usize, 1usize)];
        let r = layout_from_graph(&nodes, &edges);
        assert_eq!(r.rooms.len(), 2);
        assert_eq!(r.corridors.len(), 2);
    }

    #[test]
    fn flow_field_from_grid() {
        let g = Grid::new_filled(6, 6, true);
        let f = grid_to_flow_field(&g, &[Coord2::new(5, 5)]);
        assert!(f.direction_at(Coord2::new(0, 0)).is_some());
    }

    #[test]
    fn walls_to_aabbs_merges() {
        let g = Grid::from_fn(2, 2, |_, _| false);
        let boxes = walls_to_aabbs(&g, |_| true);
        assert_eq!(boxes.len(), 1);
        assert_eq!(
            boxes[0],
            Aabb2::new(Coord2::new(0, 0), Coord2::new(1, 1))
        );
    }

    #[test]
    fn walls_to_aabbs_separate() {
        let g = Grid::new_filled(4, 1, false);
        let boxes = walls_to_aabbs(&g, |c| c.x == 0 || c.x == 3);
        assert_eq!(boxes.len(), 2);
    }

    #[test]
    fn walls_to_aabbs_l_shape() {
        let g = Grid::new_filled(3, 3, false);
        let boxes = walls_to_aabbs(&g, |c| (c.x == 0 && c.y <= 2) || (c.y == 2 && c.x <= 2));
        assert_eq!(boxes.len(), 2);
    }
}
