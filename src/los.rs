use alloc::vec::Vec;

use crate::coord::Coord2;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;

/// Bresenham line from start to end, inclusive.
pub fn bresenham(start: Coord2, end: Coord2) -> Vec<Coord2> {
    let mut points = Vec::new();
    let mut x0 = start.x;
    let mut y0 = start.y;
    let x1 = end.x;
    let y1 = end.y;

    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        points.push(Coord2::new(x0, y0));
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }

    points
}

pub fn line_of_sight<F>(grid: &Grid<bool>, from: Coord2, to: Coord2, mut is_opaque: F) -> bool
where
    F: FnMut(Coord2) -> bool,
{
    if !grid.in_bounds(from) || !grid.in_bounds(to) {
        return false;
    }
    let line = bresenham(from, to);
    for (i, p) in line.iter().copied().enumerate() {
        if i == 0 {
            continue;
        }
        if is_opaque(p) {
            return p == to;
        }
    }
    true
}

#[derive(Clone, Debug)]
pub struct FovResult {
    pub visible: Grid<bool>,
    pub origin: Coord2,
    pub radius: u32,
}

impl FovResult {
    #[inline]
    pub fn is_visible(&self, coord: Coord2) -> bool {
        self.visible.get(coord).copied().unwrap_or(false)
    }
}

/// Visibility field around origin. Uses LOS ray checks per candidate cell.
pub fn shadowcast<F>(
    grid: &Grid<bool>,
    origin: Coord2,
    radius: u32,
    is_opaque: F,
) -> FovResult
where
    F: FnMut(Coord2) -> bool + Copy,
{
    let mut visible = Grid::new_filled(grid.width(), grid.height(), false);
    if !grid.in_bounds(origin) {
        return FovResult {
            visible,
            origin,
            radius,
        };
    }

    visible.set(origin, true);
    let r = radius as i32;
    let rr = r * r;
    for y in origin.y - r..=origin.y + r {
        for x in origin.x - r..=origin.x + r {
            let c = Coord2::new(x, y);
            if !grid.in_bounds(c) {
                continue;
            }
            let dx = x - origin.x;
            let dy = y - origin.y;
            if dx * dx + dy * dy > rr {
                continue;
            }
            if line_of_sight(grid, origin, c, is_opaque) {
                visible.set(c, true);
            }
        }
    }

    FovResult {
        visible,
        origin,
        radius,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bresenham_horizontal() {
        let line = bresenham(Coord2::new(0, 0), Coord2::new(3, 0));
        assert_eq!(
            line,
            alloc::vec![
                Coord2::new(0, 0),
                Coord2::new(1, 0),
                Coord2::new(2, 0),
                Coord2::new(3, 0)
            ]
        );
    }

    #[test]
    fn bresenham_diagonal() {
        let line = bresenham(Coord2::new(0, 0), Coord2::new(3, 3));
        assert_eq!(
            line,
            alloc::vec![
                Coord2::new(0, 0),
                Coord2::new(1, 1),
                Coord2::new(2, 2),
                Coord2::new(3, 3)
            ]
        );
    }

    #[test]
    fn bresenham_steep() {
        let line = bresenham(Coord2::new(0, 0), Coord2::new(1, 3));
        assert_eq!(line.first().copied(), Some(Coord2::new(0, 0)));
        assert_eq!(line.last().copied(), Some(Coord2::new(1, 3)));
    }

    #[test]
    fn los_clear() {
        let g = Grid::new_filled(6, 1, true);
        assert!(line_of_sight(&g, Coord2::new(0, 0), Coord2::new(5, 0), |c| {
            !g.get(c).copied().unwrap_or(false)
        }));
    }

    #[test]
    fn los_blocked() {
        let mut g = Grid::new_filled(6, 1, true);
        g.set(Coord2::new(3, 0), false);
        assert!(!line_of_sight(
            &g,
            Coord2::new(0, 0),
            Coord2::new(5, 0),
            |c| !g.get(c).copied().unwrap_or(false)
        ));
    }

    #[test]
    fn fov_wall_shadow() {
        let mut g = Grid::new_filled(7, 7, true);
        g.set(Coord2::new(3, 3), false);
        let fov = shadowcast(&g, Coord2::new(1, 3), 6, |c| !g.get(c).copied().unwrap_or(false));
        assert!(!fov.is_visible(Coord2::new(5, 3)));
    }
}
