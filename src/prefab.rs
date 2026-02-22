use crate::coord::Coord2;
use crate::grid::Grid;
use crate::grid_trait::SpatialGrid;

#[derive(Clone, Debug)]
pub struct Prefab<T: Clone> {
    pub cells: Grid<T>,
    pub anchor: Coord2,
}

impl<T: Clone> Prefab<T> {
    pub fn new(cells: Grid<T>, anchor: Coord2) -> Self {
        Self { cells, anchor }
    }

    pub fn rotate_cw(&self) -> Self {
        let w = self.cells.width();
        let h = self.cells.height();
        let mut out = Grid::new_filled(
            h,
            w,
            self.cells.get(Coord2::new(0, 0)).unwrap().clone(),
        );

        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let src = Coord2::new(x, y);
                let dst = Coord2::new(h as i32 - 1 - y, x);
                out.set(dst, self.cells.get(src).unwrap().clone());
            }
        }

        let anchor = Coord2::new(h as i32 - 1 - self.anchor.y, self.anchor.x);
        Self::new(out, anchor)
    }

    pub fn rotate_ccw(&self) -> Self {
        let w = self.cells.width();
        let h = self.cells.height();
        let mut out = Grid::new_filled(
            h,
            w,
            self.cells.get(Coord2::new(0, 0)).unwrap().clone(),
        );

        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let src = Coord2::new(x, y);
                let dst = Coord2::new(y, w as i32 - 1 - x);
                out.set(dst, self.cells.get(src).unwrap().clone());
            }
        }

        let anchor = Coord2::new(self.anchor.y, w as i32 - 1 - self.anchor.x);
        Self::new(out, anchor)
    }

    pub fn flip_h(&self) -> Self {
        let w = self.cells.width();
        let h = self.cells.height();
        let mut out = Grid::new_filled(
            w,
            h,
            self.cells.get(Coord2::new(0, 0)).unwrap().clone(),
        );
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let src = Coord2::new(x, y);
                let dst = Coord2::new(w as i32 - 1 - x, y);
                out.set(dst, self.cells.get(src).unwrap().clone());
            }
        }
        let anchor = Coord2::new(w as i32 - 1 - self.anchor.x, self.anchor.y);
        Self::new(out, anchor)
    }

    pub fn flip_v(&self) -> Self {
        let w = self.cells.width();
        let h = self.cells.height();
        let mut out = Grid::new_filled(
            w,
            h,
            self.cells.get(Coord2::new(0, 0)).unwrap().clone(),
        );
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let src = Coord2::new(x, y);
                let dst = Coord2::new(x, h as i32 - 1 - y);
                out.set(dst, self.cells.get(src).unwrap().clone());
            }
        }
        let anchor = Coord2::new(self.anchor.x, h as i32 - 1 - self.anchor.y);
        Self::new(out, anchor)
    }

    pub fn stamp_onto(&self, target: &mut Grid<T>, position: Coord2) {
        for y in 0..self.cells.height() as i32 {
            for x in 0..self.cells.width() as i32 {
                let src = Coord2::new(x, y);
                let world = Coord2::new(
                    position.x + (x - self.anchor.x),
                    position.y + (y - self.anchor.y),
                );
                if target.in_bounds(world) {
                    target.set(world, self.cells.get(src).unwrap().clone());
                }
            }
        }
    }

    pub fn fits(&self, target: &Grid<T>, position: Coord2) -> bool {
        for y in 0..self.cells.height() as i32 {
            for x in 0..self.cells.width() as i32 {
                let world = Coord2::new(
                    position.x + (x - self.anchor.x),
                    position.y + (y - self.anchor.y),
                );
                if !target.in_bounds(world) {
                    return false;
                }
            }
        }
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefab_rotate_4x() {
        let grid = Grid::from_fn(3, 2, |x, y| x + y * 10);
        let prefab = Prefab::new(grid.clone(), Coord2::new(1, 0));
        let r4 = prefab.rotate_cw().rotate_cw().rotate_cw().rotate_cw();
        assert_eq!(r4.cells.cells(), grid.cells());
        assert_eq!(r4.anchor, prefab.anchor);
    }

    #[test]
    fn prefab_stamp() {
        let prefab = Prefab::new(Grid::new_filled(2, 2, 1u8), Coord2::new(0, 0));
        let mut target = Grid::new_filled(4, 4, 0u8);
        prefab.stamp_onto(&mut target, Coord2::new(1, 1));
        assert_eq!(target.get(Coord2::new(1, 1)), Some(&1));
        assert_eq!(target.get(Coord2::new(2, 2)), Some(&1));
    }

    #[test]
    fn prefab_fits() {
        let prefab = Prefab::new(Grid::new_filled(3, 3, true), Coord2::new(1, 1));
        let target = Grid::new_filled(5, 5, false);
        assert!(prefab.fits(&target, Coord2::new(2, 2)));
        assert!(!prefab.fits(&target, Coord2::new(0, 0)));
    }
}
