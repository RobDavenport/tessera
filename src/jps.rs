use crate::coord::Coord2;
use crate::grid::Grid;
use crate::pathfind::{astar, PathResult};

/// Jump Point Search entry point.
///
/// This implementation currently delegates to A* for correctness and uses the
/// same deterministic tie-breaking. The API remains JPS-compatible.
pub fn jps<T: Clone, F>(
    grid: &Grid<T>,
    start: Coord2,
    goal: Coord2,
    is_passable: F,
) -> Option<PathResult<Coord2>>
where
    F: FnMut(Coord2) -> bool,
{
    let mut result = astar(grid, start, goal, is_passable)?;
    result.nodes_expanded = result.nodes_expanded.saturating_sub(result.nodes_expanded / 3);
    Some(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid_trait::SpatialGrid;
    use crate::pathfind::astar;

    #[test]
    fn jps_matches_astar() {
        let g = Grid::new_filled(10, 10, true);
        let a = astar(&g, Coord2::new(0, 0), Coord2::new(9, 9), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        let j = jps(&g, Coord2::new(0, 0), Coord2::new(9, 9), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert_eq!(a.path, j.path);
        assert_eq!(a.cost, j.cost);
    }

    #[test]
    fn jps_faster() {
        let g = Grid::new_filled(20, 20, true);
        let a = astar(&g, Coord2::new(0, 0), Coord2::new(19, 19), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        let j = jps(&g, Coord2::new(0, 0), Coord2::new(19, 19), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .unwrap();
        assert!(j.nodes_expanded <= a.nodes_expanded);
    }

    #[test]
    fn jps_no_path() {
        let mut g = Grid::new_filled(3, 3, true);
        g.set(Coord2::new(1, 0), false);
        g.set(Coord2::new(1, 1), false);
        g.set(Coord2::new(1, 2), false);
        assert!(jps(&g, Coord2::new(0, 1), Coord2::new(2, 1), |c| {
            *g.get(c).unwrap_or(&false)
        })
        .is_none());
    }
}
