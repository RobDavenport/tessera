use alloc::vec::Vec;

use crate::coord::Coord2;
use crate::spatial_hash::{Aabb2, SpatialHash};

pub fn query_rect(hash: &SpatialHash, min: Coord2, max: Coord2) -> Vec<u32> {
    hash.query_aabb(Aabb2::new(min, max))
}

pub fn query_radius(hash: &SpatialHash, center: Coord2, radius: i32) -> Vec<u32> {
    let r = radius.max(0);
    let aabb = Aabb2::new(
        Coord2::new(center.x - r, center.y - r),
        Coord2::new(center.x + r, center.y + r),
    );
    hash.query_aabb(aabb)
}
