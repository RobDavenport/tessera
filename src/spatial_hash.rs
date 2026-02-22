use alloc::vec::Vec;

use crate::coord::Coord2;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Aabb2 {
    pub min: Coord2,
    pub max: Coord2,
}

impl Aabb2 {
    pub fn new(min: Coord2, max: Coord2) -> Self {
        Self {
            min: Coord2::new(min.x.min(max.x), min.y.min(max.y)),
            max: Coord2::new(min.x.max(max.x), min.y.max(max.y)),
        }
    }

    #[inline]
    pub fn intersects(&self, other: &Aabb2) -> bool {
        !(self.max.x < other.min.x
            || self.min.x > other.max.x
            || self.max.y < other.min.y
            || self.min.y > other.max.y)
    }

    #[inline]
    pub fn contains(&self, point: Coord2) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
    }
}

#[derive(Clone, Debug)]
struct Entry {
    id: u32,
    aabb: Aabb2,
    buckets: Vec<usize>,
}

/// Uniform-grid spatial hash for broad-phase 2D overlap queries.
#[derive(Clone, Debug)]
pub struct SpatialHash {
    cell_size: i32,
    buckets_w: u32,
    buckets_h: u32,
    buckets: Vec<Vec<usize>>,
    entries: Vec<Option<Entry>>,
}

impl SpatialHash {
    pub fn new(world_width: i32, world_height: i32, cell_size: i32) -> Self {
        let cell_size = cell_size.max(1);
        let buckets_w = ((world_width.max(1) + cell_size - 1) / cell_size) as u32;
        let buckets_h = ((world_height.max(1) + cell_size - 1) / cell_size) as u32;
        let bucket_count = buckets_w as usize * buckets_h as usize;
        Self {
            cell_size,
            buckets_w,
            buckets_h,
            buckets: alloc::vec![Vec::new(); bucket_count],
            entries: Vec::new(),
        }
    }

    pub fn insert(&mut self, id: u32, aabb: Aabb2) {
        let _ = self.remove(id);
        let touched = self.buckets_for_aabb(aabb);
        let entry_index = self.entries.len();
        self.entries.push(Some(Entry {
            id,
            aabb,
            buckets: touched.clone(),
        }));

        for bucket in touched {
            self.buckets[bucket].push(entry_index);
        }
    }

    pub fn remove(&mut self, id: u32) -> bool {
        let idx = self.find_entry(id);
        let Some(entry_idx) = idx else {
            return false;
        };
        let entry = self.entries[entry_idx].take().unwrap();
        for bucket_idx in entry.buckets {
            let bucket = &mut self.buckets[bucket_idx];
            if let Some(pos) = bucket.iter().position(|&i| i == entry_idx) {
                bucket.swap_remove(pos);
            }
        }
        true
    }

    pub fn update(&mut self, id: u32, aabb: Aabb2) -> bool {
        if !self.remove(id) {
            return false;
        }
        self.insert(id, aabb);
        true
    }

    pub fn query_aabb(&self, query: Aabb2) -> Vec<u32> {
        let mut candidates = Vec::<usize>::new();
        for bucket_idx in self.buckets_for_aabb(query) {
            for &entry_idx in &self.buckets[bucket_idx] {
                if !candidates.contains(&entry_idx) {
                    candidates.push(entry_idx);
                }
            }
        }

        let mut result = Vec::new();
        for idx in candidates {
            if let Some(entry) = &self.entries[idx] {
                if entry.aabb.intersects(&query) {
                    result.push(entry.id);
                }
            }
        }
        result
    }

    pub fn query_point(&self, point: Coord2) -> Vec<u32> {
        let bucket = self.bucket_index(point);
        let mut result = Vec::new();
        for &entry_idx in &self.buckets[bucket] {
            if let Some(entry) = &self.entries[entry_idx] {
                if entry.aabb.contains(point) {
                    result.push(entry.id);
                }
            }
        }
        result
    }

    fn find_entry(&self, id: u32) -> Option<usize> {
        self.entries
            .iter()
            .position(|e| e.as_ref().is_some_and(|entry| entry.id == id))
    }

    fn bucket_index(&self, coord: Coord2) -> usize {
        let x = (coord.x.div_euclid(self.cell_size)).clamp(0, self.buckets_w as i32 - 1) as usize;
        let y = (coord.y.div_euclid(self.cell_size)).clamp(0, self.buckets_h as i32 - 1) as usize;
        y * self.buckets_w as usize + x
    }

    fn buckets_for_aabb(&self, aabb: Aabb2) -> Vec<usize> {
        let mut out = Vec::new();
        let min_x = (aabb.min.x.div_euclid(self.cell_size)).clamp(0, self.buckets_w as i32 - 1);
        let max_x = (aabb.max.x.div_euclid(self.cell_size)).clamp(0, self.buckets_w as i32 - 1);
        let min_y = (aabb.min.y.div_euclid(self.cell_size)).clamp(0, self.buckets_h as i32 - 1);
        let max_y = (aabb.max.y.div_euclid(self.cell_size)).clamp(0, self.buckets_h as i32 - 1);

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                out.push(y as usize * self.buckets_w as usize + x as usize);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spatial_hash_insert_query() {
        let mut hash = SpatialHash::new(100, 100, 10);
        hash.insert(1, Aabb2::new(Coord2::new(5, 5), Coord2::new(15, 15)));
        let hits = hash.query_aabb(Aabb2::new(Coord2::new(8, 8), Coord2::new(12, 12)));
        assert_eq!(hits, alloc::vec![1]);
    }

    #[test]
    fn spatial_hash_no_false_positives() {
        let mut hash = SpatialHash::new(100, 100, 10);
        hash.insert(1, Aabb2::new(Coord2::new(5, 5), Coord2::new(15, 15)));
        let hits = hash.query_aabb(Aabb2::new(Coord2::new(30, 30), Coord2::new(40, 40)));
        assert!(hits.is_empty());
    }

    #[test]
    fn spatial_hash_remove() {
        let mut hash = SpatialHash::new(100, 100, 10);
        hash.insert(1, Aabb2::new(Coord2::new(0, 0), Coord2::new(10, 10)));
        assert!(hash.remove(1));
        assert!(hash.query_point(Coord2::new(5, 5)).is_empty());
    }

    #[test]
    fn spatial_hash_update() {
        let mut hash = SpatialHash::new(100, 100, 10);
        hash.insert(1, Aabb2::new(Coord2::new(0, 0), Coord2::new(10, 10)));
        assert!(hash.update(1, Aabb2::new(Coord2::new(20, 20), Coord2::new(30, 30))));
        assert!(hash.query_point(Coord2::new(5, 5)).is_empty());
        assert_eq!(hash.query_point(Coord2::new(25, 25)), alloc::vec![1]);
    }
}
