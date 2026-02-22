use alloc::vec::Vec;

use crate::coord::{Coord2, Coord3};
use crate::grid_trait::SpatialGrid;
use crate::hex::HexCoord;

pub trait HeuristicCoord: Copy + Eq {
    fn heuristic(self, goal: Self) -> u32;
}

impl HeuristicCoord for Coord2 {
    fn heuristic(self, goal: Self) -> u32 {
        self.octile_distance(goal)
    }
}

impl HeuristicCoord for Coord3 {
    fn heuristic(self, goal: Self) -> u32 {
        self.manhattan_distance(goal) * 10
    }
}

impl HeuristicCoord for HexCoord {
    fn heuristic(self, goal: Self) -> u32 {
        self.hex_distance(goal) * 10
    }
}

/// Result of a pathfinding search.
#[derive(Clone, Debug)]
pub struct PathResult<C> {
    pub path: Vec<C>,
    pub cost: u32,
    pub nodes_expanded: u32,
}

struct AStarState {
    g_scores: Vec<u32>,
    came_from: Vec<u32>,
    status: Vec<u8>,
    open_list: Vec<(u32, u32)>,
}

const UNVISITED: u8 = 0;
const OPEN: u8 = 1;
const CLOSED: u8 = 2;
const NONE_INDEX: u32 = u32::MAX;

impl AStarState {
    fn new(size: usize) -> Self {
        Self {
            g_scores: alloc::vec![u32::MAX; size],
            came_from: alloc::vec![NONE_INDEX; size],
            status: alloc::vec![UNVISITED; size],
            open_list: Vec::new(),
        }
    }

    fn pop_min(&mut self) -> Option<(u32, u32)> {
        if self.open_list.is_empty() {
            return None;
        }
        let mut min_idx = 0usize;
        let mut min_val = self.open_list[0];
        for (i, value) in self.open_list.iter().copied().enumerate().skip(1) {
            if value.0 < min_val.0 || (value.0 == min_val.0 && value.1 < min_val.1) {
                min_idx = i;
                min_val = value;
            }
        }
        Some(self.open_list.swap_remove(min_idx))
    }
}

pub fn astar<G, F>(
    grid: &G,
    start: G::Coord,
    goal: G::Coord,
    mut is_passable: F,
) -> Option<PathResult<G::Coord>>
where
    G: SpatialGrid,
    G::Coord: HeuristicCoord + Copy + Eq,
    F: FnMut(G::Coord) -> bool,
{
    let start_idx = grid.coord_to_index(start)?;
    let goal_idx = grid.coord_to_index(goal)?;

    if start_idx == goal_idx {
        return Some(PathResult {
            path: alloc::vec![start],
            cost: 0,
            nodes_expanded: 0,
        });
    }

    let mut state = AStarState::new(grid.len());
    state.g_scores[start_idx] = 0;
    state.status[start_idx] = OPEN;
    state
        .open_list
        .push((start.heuristic(goal), start_idx as u32));

    let mut nodes_expanded = 0u32;

    while let Some((_, current_u32)) = state.pop_min() {
        let current_idx = current_u32 as usize;
        if state.status[current_idx] == CLOSED {
            continue;
        }
        state.status[current_idx] = CLOSED;
        nodes_expanded = nodes_expanded.saturating_add(1);

        if current_idx == goal_idx {
            let path = reconstruct_path(grid, &state.came_from, start_idx, goal_idx);
            return Some(PathResult {
                cost: state.g_scores[goal_idx],
                path,
                nodes_expanded,
            });
        }

        let current_coord = grid.index_to_coord(current_idx);
        for neighbor_coord in grid.neighbors(current_coord) {
            let neighbor_idx = match grid.coord_to_index(neighbor_coord) {
                Some(i) => i,
                None => continue,
            };

            if state.status[neighbor_idx] == CLOSED || !is_passable(neighbor_coord) {
                continue;
            }

            let move_cost = match grid.move_cost(current_coord, neighbor_coord) {
                Some(c) => c,
                None => continue,
            };

            let tentative_g = state.g_scores[current_idx].saturating_add(move_cost);
            if tentative_g < state.g_scores[neighbor_idx] {
                state.g_scores[neighbor_idx] = tentative_g;
                state.came_from[neighbor_idx] = current_idx as u32;
                state.status[neighbor_idx] = OPEN;
                let f = tentative_g.saturating_add(neighbor_coord.heuristic(goal));
                state.open_list.push((f, neighbor_idx as u32));
            }
        }
    }

    None
}

fn reconstruct_path<G: SpatialGrid>(
    grid: &G,
    came_from: &[u32],
    start_idx: usize,
    goal_idx: usize,
) -> Vec<G::Coord> {
    let mut reverse = Vec::new();
    let mut current = goal_idx;
    reverse.push(grid.index_to_coord(current));

    while current != start_idx {
        let parent = came_from[current];
        if parent == NONE_INDEX {
            break;
        }
        current = parent as usize;
        reverse.push(grid.index_to_coord(current));
    }
    reverse.reverse();
    reverse
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::Grid;

    #[test]
    fn astar_straight_line() {
        let g = Grid::new_filled(5, 1, true);
        let result = astar(&g, Coord2::new(0, 0), Coord2::new(4, 0), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert_eq!(result.path.first().copied(), Some(Coord2::new(0, 0)));
        assert_eq!(result.path.last().copied(), Some(Coord2::new(4, 0)));
    }

    #[test]
    fn astar_around_wall() {
        let mut g = Grid::new_filled(5, 5, true);
        g.set(Coord2::new(2, 1), false);
        g.set(Coord2::new(2, 2), false);
        g.set(Coord2::new(2, 3), false);
        let result = astar(&g, Coord2::new(0, 2), Coord2::new(4, 2), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert!(result.path.iter().all(|&c| *g.get(c).unwrap_or(&false)));
    }

    #[test]
    fn astar_no_path() {
        let mut g = Grid::new_filled(3, 3, true);
        g.set(Coord2::new(1, 0), false);
        g.set(Coord2::new(1, 1), false);
        g.set(Coord2::new(1, 2), false);
        let result = astar(&g, Coord2::new(0, 1), Coord2::new(2, 1), |c| {
            *g.get(c).unwrap_or(&false)
        });
        assert!(result.is_none());
    }

    #[test]
    fn astar_start_is_goal() {
        let g = Grid::new_filled(2, 2, true);
        let result = astar(&g, Coord2::new(1, 1), Coord2::new(1, 1), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert_eq!(result.cost, 0);
        assert_eq!(result.path, alloc::vec![Coord2::new(1, 1)]);
    }

    #[test]
    fn astar_diagonal() {
        let g = Grid::new_filled(3, 3, true);
        let result = astar(&g, Coord2::new(0, 0), Coord2::new(2, 2), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert_eq!(result.cost, 28);
    }

    #[test]
    fn astar_cost_accuracy() {
        let g = Grid::new_filled(4, 4, true);
        let result = astar(&g, Coord2::new(0, 0), Coord2::new(3, 0), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert_eq!(result.cost, 30);
    }

    #[test]
    fn astar_nodes_expanded() {
        let g = Grid::new_filled(4, 4, true);
        let result = astar(&g, Coord2::new(0, 0), Coord2::new(3, 3), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert!(result.nodes_expanded > 0);
    }
}
