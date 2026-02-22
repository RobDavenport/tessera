# tessera Implementation Plan

## What This Is

A `no_std` Rust library for spatial grids, pathfinding, and procedural level generation. Provides the runtime data structures that bridge procedural content from `wavfc` and `grammex` into playable game levels with pathfinding, field-of-view, and spatial queries.

**Key design principles:**
- Generic `SpatialGrid` trait unifies 2D, 3D, and hex grids for algorithm reuse
- Integer costs (10 = cardinal, 14 = diagonal) for deterministic pathfinding — no floating-point
- No HashMap in pathfinding — all A* state uses `Vec` indexed by flat grid position
- Generation functions return data (rooms, corridors), not grids — caller stamps results onto their own grid
- Bridge module accepts primitive data (`&[u32]`, tuples) to avoid hard dependencies on sibling crates

**Integration with the Nethercore library ecosystem:**
```text
wavfc output ──→ grid_from_wfc() ──→ Grid<T>
grammex graph ──→ layout_from_graph() ──→ rooms + corridors
Grid passability ──→ grid_to_flow_field() ──→ navex FlowField
Grid walls ──→ walls_to_aabbs() ──→ navex obstacle avoidance
cogwise blackboard ←── spatial queries via tessera
```

**Reference implementations:** See sibling repos `wavfc/` (WFC tile generation), `grammex/` (graph grammar rewriting), `navex/` (steering behaviors that consume flow fields).

## Hard Rules

- `#![no_std]` with `extern crate alloc` — no std dependency in core library
- `rand_core` and `libm` are the ONLY dependencies
- All tests: `cargo test --target x86_64-pc-windows-msvc`
- WASM check: `cargo build --target wasm32-unknown-unknown --release`
- Deterministic: same grid + same start/goal = same path
- No `HashMap` in pathfinding or grid storage — use `Vec` indexed by flat grid index
- Integer costs for movement: 10 = cardinal, 14 = diagonal (approximates 10 × √2 ≈ 14.14)
- All coordinate types use `i32` (signed — negative coords are valid for hex and relative offsets)

---

## Phase 1: Coordinates + Directions

### Step 1: Coord2 (coord.rs)

```rust
/// 2D integer coordinate for grid positions.
///
/// Uses signed `i32` to support negative offsets and relative positions.
/// Row-major indexing: `index = y * width + x`.
///
/// # Example
/// ```
/// use tessera::Coord2;
///
/// let a = Coord2::new(3, 4);
/// let b = Coord2::new(1, 2);
/// let diff = a - b;  // Coord2(2, 2)
/// assert_eq!(a.manhattan_distance(b), 4);
/// ```
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Coord2 {
    pub x: i32,
    pub y: i32,
}

impl Coord2 {
    pub const ZERO: Coord2 = Coord2 { x: 0, y: 0 };

    pub fn new(x: i32, y: i32) -> Self { Coord2 { x, y } }

    /// Manhattan distance: |dx| + |dy|. Used as A* heuristic for 4-directional movement.
    pub fn manhattan_distance(self, other: Coord2) -> u32 {
        ((self.x - other.x).abs() + (self.y - other.y).abs()) as u32
    }

    /// Chebyshev distance: max(|dx|, |dy|). Used for 8-directional movement.
    pub fn chebyshev_distance(self, other: Coord2) -> u32 {
        let dx = (self.x - other.x).abs() as u32;
        let dy = (self.y - other.y).abs() as u32;
        dx.max(dy)
    }

    /// Octile distance: max(dx,dy) * 10 + min(dx,dy) * 4.
    /// Optimal A* heuristic for 8-directional grids with cost 10/14.
    pub fn octile_distance(self, other: Coord2) -> u32 {
        let dx = (self.x - other.x).abs() as u32;
        let dy = (self.y - other.y).abs() as u32;
        let (big, small) = if dx > dy { (dx, dy) } else { (dy, dx) };
        big * 10 + small * 4  // 14 - 10 = 4 extra for each diagonal
    }

    /// Convert to flat index for a grid of given width.
    /// Returns None if out of bounds.
    pub fn to_index(self, width: u32) -> Option<usize> {
        if self.x < 0 || self.y < 0 || self.x >= width as i32 {
            return None;
        }
        Some((self.y as usize) * (width as usize) + (self.x as usize))
    }

    /// Convert from flat index to coordinate.
    pub fn from_index(index: usize, width: u32) -> Self {
        Coord2 {
            x: (index % width as usize) as i32,
            y: (index / width as usize) as i32,
        }
    }
}

// Implement Add, Sub, Neg, Mul<i32> (scalar) for Coord2
impl core::ops::Add for Coord2 { /* ... */ }
impl core::ops::Sub for Coord2 { /* ... */ }
impl core::ops::Neg for Coord2 { /* ... */ }
```

### Step 2: Coord3 (coord.rs)

```rust
/// 3D integer coordinate for volumetric grids.
///
/// Flat index: `index = z * (width * height) + y * width + x`.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Coord3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl Coord3 {
    pub const ZERO: Coord3 = Coord3 { x: 0, y: 0, z: 0 };
    pub fn new(x: i32, y: i32, z: i32) -> Self { /* ... */ }
    pub fn manhattan_distance(self, other: Coord3) -> u32 { /* ... */ }

    pub fn to_index(self, width: u32, height: u32) -> Option<usize> {
        if self.x < 0 || self.y < 0 || self.z < 0
            || self.x >= width as i32 || self.y >= height as i32 {
            return None;
        }
        Some(
            (self.z as usize) * (width as usize * height as usize)
            + (self.y as usize) * (width as usize)
            + (self.x as usize)
        )
    }

    pub fn from_index(index: usize, width: u32, height: u32) -> Self { /* ... */ }
}
```

### Step 3: HexCoord (hex.rs)

```rust
/// Hexagonal coordinate using axial (q, r) system.
///
/// The third axis `s` is implicit: `q + r + s = 0`, so `s = -q - r`.
/// Uses flat-top hexagons. Neighbors are the 6 adjacent hexes.
///
/// # Coordinate System
/// ```text
///       +r
///      / \
///  -q /   \ +q
///     \   /
///      \ /
///       -r
/// ```
///
/// # Example
/// ```
/// use tessera::HexCoord;
///
/// let origin = HexCoord::new(0, 0);
/// let neighbor = HexCoord::new(1, 0);
/// assert_eq!(origin.hex_distance(neighbor), 1);
/// ```
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct HexCoord {
    pub q: i32,
    pub r: i32,
}

impl HexCoord {
    pub const ZERO: HexCoord = HexCoord { q: 0, r: 0 };

    pub fn new(q: i32, r: i32) -> Self { HexCoord { q, r } }

    /// The implicit third axis.
    pub fn s(self) -> i32 { -self.q - self.r }

    /// Hex distance (number of steps between two hexes).
    /// Formula: max(|dq|, |dr|, |ds|)
    pub fn hex_distance(self, other: HexCoord) -> u32 {
        let dq = (self.q - other.q).abs();
        let dr = (self.r - other.r).abs();
        let ds = (self.s() - other.s()).abs();
        dq.max(dr).max(ds) as u32
    }

    /// The 6 neighbor offsets in axial coordinates.
    pub const NEIGHBOR_OFFSETS: [HexCoord; 6] = [
        HexCoord { q: 1, r: 0 },
        HexCoord { q: 1, r: -1 },
        HexCoord { q: 0, r: -1 },
        HexCoord { q: -1, r: 0 },
        HexCoord { q: -1, r: 1 },
        HexCoord { q: 0, r: 1 },
    ];

    /// Get the 6 neighbors of this hex.
    pub fn neighbors(self) -> [HexCoord; 6] {
        let mut result = [HexCoord::ZERO; 6];
        for (i, offset) in Self::NEIGHBOR_OFFSETS.iter().enumerate() {
            result[i] = HexCoord::new(self.q + offset.q, self.r + offset.r);
        }
        result
    }

    /// Get all hexes at exactly `radius` steps away (a ring).
    /// Returns hexes in order around the ring.
    pub fn ring(self, radius: u32) -> Vec<HexCoord> { /* ... */ }

    /// Get all hexes within `radius` steps (a filled circle/spiral).
    pub fn spiral(self, radius: u32) -> Vec<HexCoord> { /* ... */ }
}
```

### Step 4: Directions (neighborhood.rs)

```rust
/// 4-directional movement (Von Neumann neighborhood).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dir4 {
    North, East, South, West,
}

impl Dir4 {
    pub const ALL: [Dir4; 4] = [Dir4::North, Dir4::East, Dir4::South, Dir4::West];

    /// Offset as Coord2.
    pub fn offset(self) -> Coord2 {
        match self {
            Dir4::North => Coord2::new(0, -1),
            Dir4::East  => Coord2::new(1, 0),
            Dir4::South => Coord2::new(0, 1),
            Dir4::West  => Coord2::new(-1, 0),
        }
    }

    pub fn opposite(self) -> Dir4 { /* ... */ }
    pub fn cw(self) -> Dir4 { /* ... */ }
    pub fn ccw(self) -> Dir4 { /* ... */ }
}

/// 8-directional movement (Moore neighborhood).
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dir8 {
    N, NE, E, SE, S, SW, W, NW,
}

impl Dir8 {
    pub const ALL: [Dir8; 8] = [Dir8::N, Dir8::NE, Dir8::E, Dir8::SE,
                                 Dir8::S, Dir8::SW, Dir8::W, Dir8::NW];

    pub fn offset(self) -> Coord2 {
        match self {
            Dir8::N  => Coord2::new(0, -1),
            Dir8::NE => Coord2::new(1, -1),
            Dir8::E  => Coord2::new(1, 0),
            Dir8::SE => Coord2::new(1, 1),
            Dir8::S  => Coord2::new(0, 1),
            Dir8::SW => Coord2::new(-1, 1),
            Dir8::W  => Coord2::new(-1, 0),
            Dir8::NW => Coord2::new(-1, -1),
        }
    }

    pub fn is_diagonal(self) -> bool {
        matches!(self, Dir8::NE | Dir8::SE | Dir8::SW | Dir8::NW)
    }

    /// Movement cost: 10 for cardinal, 14 for diagonal.
    /// 14 ≈ 10 × √2 ≈ 14.14, using integers for determinism.
    pub fn cost(self) -> u32 {
        if self.is_diagonal() { 14 } else { 10 }
    }

    pub fn opposite(self) -> Dir8 { /* ... */ }
}

/// 6-directional movement for hex grids.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum Dir6 {
    E, NE, NW, W, SW, SE,
}

impl Dir6 {
    pub const ALL: [Dir6; 6] = [Dir6::E, Dir6::NE, Dir6::NW, Dir6::W, Dir6::SW, Dir6::SE];

    pub fn offset(self) -> HexCoord {
        HexCoord::NEIGHBOR_OFFSETS[self as usize]
    }

    pub fn opposite(self) -> Dir6 { /* ... */ }
}

/// Neighborhood type for generic algorithms.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Neighborhood {
    /// 4 cardinal directions (Von Neumann).
    VonNeumann,
    /// 8 directions including diagonals (Moore).
    Moore,
    /// 6 hex directions.
    Hex,
}
```

**Tests for Phase 1:**
- `coord2_add_sub` — arithmetic operations
- `coord2_manhattan` — Coord2(0,0) to Coord2(3,4) = 7
- `coord2_chebyshev` — Coord2(0,0) to Coord2(3,4) = 4
- `coord2_octile` — Coord2(0,0) to Coord2(3,4) = 46 (3*10 + min(3,4-3)*4... verify)
- `coord2_index_roundtrip` — to_index then from_index returns same coord
- `coord3_index_roundtrip` — 3D index conversion
- `hex_distance` — adjacent = 1, two away = 2
- `hex_ring_count` — ring(r) has 6*r hexes (except r=0 which has 1)
- `hex_spiral_count` — spiral(r) has 3*r*(r+1)+1 hexes
- `dir4_opposite` — North ↔ South, East ↔ West
- `dir8_diagonal_cost` — diagonal = 14, cardinal = 10
- `dir6_offset` — each direction produces correct HexCoord offset

---

## Phase 2: Grid Storage

### Step 5: SpatialGrid Trait (grid_trait.rs)

```rust
/// Trait unifying all grid types for generic algorithms.
///
/// Algorithms like A*, flood fill, and FOV are generic over this trait.
/// Concrete types: `Grid<T>`, `Grid3<T>`, `HexGrid<T>`.
///
/// Grid cells are accessed by flat index (usize) internally. The trait
/// provides coordinate ↔ index conversion.
pub trait SpatialGrid {
    /// The cell data type.
    type Cell: Clone;
    /// The coordinate type (Coord2 for 2D grids, HexCoord for hex).
    type Coord: Copy + Eq;

    /// Grid width (for 2D grids).
    fn width(&self) -> u32;
    /// Grid height (for 2D grids).
    fn height(&self) -> u32;
    /// Total number of cells.
    fn len(&self) -> usize;
    /// Whether the grid is empty.
    fn is_empty(&self) -> bool { self.len() == 0 }

    /// Check if a coordinate is within bounds.
    fn in_bounds(&self, coord: Self::Coord) -> bool;

    /// Get cell by coordinate. Returns None if out of bounds.
    fn get(&self, coord: Self::Coord) -> Option<&Self::Cell>;
    /// Get mutable cell reference. Returns None if out of bounds.
    fn get_mut(&mut self, coord: Self::Coord) -> Option<&mut Self::Cell>;
    /// Set cell value. No-op if out of bounds.
    fn set(&mut self, coord: Self::Coord, value: Self::Cell);

    /// Convert flat index to coordinate.
    fn index_to_coord(&self, index: usize) -> Self::Coord;
    /// Convert coordinate to flat index. Returns None if out of bounds.
    fn coord_to_index(&self, coord: Self::Coord) -> Option<usize>;

    /// Get neighbor coordinates (respecting bounds).
    fn neighbors(&self, coord: Self::Coord) -> Vec<Self::Coord>;

    /// Get the movement cost between two adjacent cells.
    /// Returns None if movement is blocked.
    fn move_cost(&self, from: Self::Coord, to: Self::Coord) -> Option<u32>;
}
```

### Step 6: Grid<T> (grid.rs)

```rust
/// 2D rectangular grid backed by a flat Vec.
///
/// Row-major storage: `cells[y * width + x]`.
///
/// # Example
/// ```
/// use tessera::{Grid, Coord2};
///
/// let mut grid = Grid::new_filled(10, 10, false);
/// grid.set(Coord2::new(5, 5), true);
/// assert_eq!(grid.get(Coord2::new(5, 5)), Some(&true));
/// ```
#[derive(Clone, Debug)]
pub struct Grid<T> {
    cells: Vec<T>,
    width: u32,
    height: u32,
}

impl<T: Clone> Grid<T> {
    /// Create a grid filled with a uniform value.
    pub fn new_filled(width: u32, height: u32, fill: T) -> Self { /* ... */ }

    /// Create a grid from a function: `f(x, y) -> T`.
    pub fn from_fn(width: u32, height: u32, f: impl FnMut(i32, i32) -> T) -> Self { /* ... */ }

    /// Create from a flat vec (row-major).
    pub fn from_vec(width: u32, height: u32, cells: Vec<T>) -> Self { /* ... */ }

    /// Get a row slice.
    pub fn row(&self, y: u32) -> &[T] { /* ... */ }

    /// Get a mutable row slice.
    pub fn row_mut(&mut self, y: u32) -> &mut [T] { /* ... */ }

    /// Iterate all cells with coordinates.
    pub fn iter_coords(&self) -> impl Iterator<Item = (Coord2, &T)> { /* ... */ }

    /// Raw cell access by flat index.
    pub fn cells(&self) -> &[T] { &self.cells }

    /// Mutable raw cell access.
    pub fn cells_mut(&mut self) -> &mut [T] { &mut self.cells }
}

impl<T: Clone> SpatialGrid for Grid<T> {
    type Cell = T;
    type Coord = Coord2;
    // ... implement all trait methods
}
```

### Step 7: Grid3<T> (grid3.rs)

```rust
/// 3D volumetric grid backed by a flat Vec.
///
/// Storage: `cells[z * (width * height) + y * width + x]`.
#[derive(Clone, Debug)]
pub struct Grid3<T> {
    cells: Vec<T>,
    width: u32,
    height: u32,
    depth: u32,
}

impl<T: Clone> Grid3<T> {
    pub fn new_filled(width: u32, height: u32, depth: u32, fill: T) -> Self { /* ... */ }
    pub fn from_fn(w: u32, h: u32, d: u32, f: impl FnMut(i32, i32, i32) -> T) -> Self { /* ... */ }
    pub fn get_3d(&self, coord: Coord3) -> Option<&T> { /* ... */ }
    pub fn set_3d(&mut self, coord: Coord3, value: T) { /* ... */ }
    pub fn depth(&self) -> u32 { self.depth }
    pub fn slice(&self, z: u32) -> &[T] { /* ... */ }
}
```

### Step 8: HexGrid<T> (hex.rs, continued)

```rust
/// Hexagonal grid using axial coordinates.
///
/// Stores hexes within a given radius from the origin.
/// Total cells for radius `r`: `3*r*(r+1) + 1`.
///
/// Internal storage maps axial (q, r) to a flat index using offset conversion.
#[derive(Clone, Debug)]
pub struct HexGrid<T> {
    cells: Vec<T>,
    radius: u32,
}

impl<T: Clone> HexGrid<T> {
    pub fn new_filled(radius: u32, fill: T) -> Self { /* ... */ }

    /// The grid radius. Hexes with hex_distance(origin) <= radius are valid.
    pub fn radius(&self) -> u32 { self.radius }

    /// Total cell count: 3*r*(r+1) + 1.
    pub fn cell_count(radius: u32) -> usize {
        let r = radius as usize;
        3 * r * (r + 1) + 1
    }

    /// Convert axial (q,r) to flat index.
    /// Uses offset mapping: shift q and r to be non-negative.
    fn axial_to_index(&self, coord: HexCoord) -> Option<usize> { /* ... */ }

    /// Convert flat index to axial (q,r).
    fn index_to_axial(&self, index: usize) -> HexCoord { /* ... */ }
}

impl<T: Clone> SpatialGrid for HexGrid<T> {
    type Cell = T;
    type Coord = HexCoord;
    // Note: width()/height() return the bounding box dimensions
    // ... implement all trait methods
}
```

**Tests for Phase 2:**
- `grid_new_filled` — all cells have fill value
- `grid_from_fn` — f(x,y) produces expected pattern
- `grid_set_get` — set at coord, get back same value
- `grid_out_of_bounds` — get on invalid coord returns None
- `grid_row_slice` — row() returns correct width
- `grid_neighbors_corner` — corner has 2 (Dir4) or 3 (Dir8) neighbors
- `grid_neighbors_center` — center has 4 or 8 neighbors
- `grid3_3d_indexing` — z layers don't overlap
- `hexgrid_cell_count` — radius 2 = 19 cells
- `hexgrid_neighbor_count` — interior hex has 6 neighbors
- `hexgrid_boundary` — edge hexes have < 6 neighbors

---

## Phase 3: A* Pathfinding

### Step 9: A* State (pathfind.rs)

```rust
/// Result of a pathfinding search.
#[derive(Clone, Debug)]
pub struct PathResult<C> {
    /// The path from start to goal, inclusive of both endpoints.
    pub path: Vec<C>,
    /// Total movement cost along the path.
    pub cost: u32,
    /// Number of nodes expanded during the search (for performance analysis).
    pub nodes_expanded: u32,
}

/// Internal state for A* search, pre-allocated and reusable.
/// All storage is Vec-indexed by flat grid index — no HashMap.
struct AStarState {
    /// g-score: best known cost from start to each cell.
    g_scores: Vec<u32>,
    /// Parent pointer: flat index of the cell we came from.
    came_from: Vec<u32>,
    /// Cell status: 0 = unvisited, 1 = open, 2 = closed.
    status: Vec<u8>,
    /// Binary min-heap of (f-score, flat_index).
    open_list: Vec<(u32, u32)>,
}

const UNVISITED: u8 = 0;
const OPEN: u8 = 1;
const CLOSED: u8 = 2;
```

### Step 10: A* Algorithm (pathfind.rs)

**Algorithm pseudocode:**

```text
astar(grid, start, goal, is_passable):
    n = grid.len()
    g_scores = [u32::MAX; n]
    came_from = [u32::MAX; n]
    status = [UNVISITED; n]

    start_idx = grid.coord_to_index(start)
    goal_idx = grid.coord_to_index(goal)

    g_scores[start_idx] = 0
    open_list.push((heuristic(start, goal), start_idx))
    status[start_idx] = OPEN
    nodes_expanded = 0

    while let Some((_, current_idx)) = open_list.pop_min():
        if status[current_idx] == CLOSED:
            continue  // stale entry
        status[current_idx] = CLOSED
        nodes_expanded += 1

        if current_idx == goal_idx:
            return reconstruct_path(came_from, goal_idx)

        current_coord = grid.index_to_coord(current_idx)
        for neighbor_coord in grid.neighbors(current_coord):
            neighbor_idx = grid.coord_to_index(neighbor_coord)
            if status[neighbor_idx] == CLOSED: continue
            if !is_passable(neighbor_coord): continue

            move_cost = grid.move_cost(current_coord, neighbor_coord)?
            tentative_g = g_scores[current_idx] + move_cost

            if tentative_g < g_scores[neighbor_idx]:
                g_scores[neighbor_idx] = tentative_g
                came_from[neighbor_idx] = current_idx
                f = tentative_g + heuristic(neighbor_coord, goal)
                open_list.push((f, neighbor_idx))
                status[neighbor_idx] = OPEN

    return None  // no path found
```

```rust
/// Find the shortest path between two coordinates on any SpatialGrid.
///
/// Uses A* with the heuristic appropriate for the grid type:
/// - Grid with 4-dir: Manhattan distance × 10
/// - Grid with 8-dir: Octile distance
/// - HexGrid: Hex distance × 10
///
/// # Arguments
/// - `grid`: The spatial grid to search
/// - `start`: Starting coordinate
/// - `goal`: Target coordinate
/// - `is_passable`: Closure that returns true if a cell is walkable
///
/// # Returns
/// `Some(PathResult)` with the shortest path and cost, or `None` if
/// no path exists.
///
/// # Example
/// ```
/// use tessera::{Grid, Coord2, astar};
///
/// let grid = Grid::new_filled(20, 20, true); // all passable
/// let result = astar(&grid, Coord2::new(0, 0), Coord2::new(10, 10), |c| {
///     *grid.get(c).unwrap_or(&false)
/// });
/// assert!(result.is_some());
/// ```
pub fn astar<G: SpatialGrid<Coord = Coord2>>(
    grid: &G,
    start: Coord2,
    goal: Coord2,
    is_passable: impl Fn(Coord2) -> bool,
) -> Option<PathResult<Coord2>> { /* ... */ }
```

**Binary Min-Heap Implementation:**

Your job: implement a simple binary min-heap (or use `alloc::collections::BinaryHeap` with a `Reverse` wrapper). The heap stores `(f_score, flat_index)` pairs and pops the lowest f-score first.

```rust
/// Reconstruct the path by following came_from pointers from goal to start.
fn reconstruct_path(came_from: &[u32], goal_idx: u32, grid_width: u32) -> Vec<Coord2> {
    let mut path = Vec::new();
    let mut current = goal_idx;
    while current != u32::MAX {
        path.push(Coord2::from_index(current as usize, grid_width));
        current = came_from[current as usize];
    }
    path.reverse();
    path
}
```

**Tests for Phase 3:**
- `astar_straight_line` — path on empty grid is direct
- `astar_around_wall` — path goes around an obstacle
- `astar_no_path` — returns None when goal is walled off
- `astar_start_is_goal` — path is [start]
- `astar_diagonal` — 8-dir path uses diagonals when cheaper
- `astar_cost_accuracy` — total cost matches expected movement cost
- `astar_nodes_expanded` — straight-line path expands few nodes

---

## Phase 4: JPS + Dijkstra + Flow Fields

### Step 11: Jump Point Search (jps.rs)

JPS is a Grid-specific optimization of A* that prunes unnecessary nodes from the open list. Instead of adding every neighbor, it "jumps" in each direction until it finds a forced neighbor or hits a wall.

**JPS forced neighbor detection (cardinal directions):**

```text
Jumping East (+x):
    If cell at (x, y-1) is blocked AND (x+1, y-1) is not:
        (x+1, y-1) is a forced neighbor → this is a jump point.
    If cell at (x, y+1) is blocked AND (x+1, y+1) is not:
        (x+1, y+1) is a forced neighbor → this is a jump point.

Jumping North (-y):
    If cell at (x-1, y) is blocked AND (x-1, y-1) is not:
        forced neighbor.
    If cell at (x+1, y) is blocked AND (x+1, y-1) is not:
        forced neighbor.

Jumping diagonally (NE = +x, -y):
    1. Check for forced neighbors diagonally
    2. Also jump cardinally East and North from this point
    3. If either cardinal jump finds something, this is a jump point
```

```rust
/// Jump Point Search for Grid<T>. Not generic over SpatialGrid because
/// JPS relies on the regular grid structure for jump/forced-neighbor logic.
///
/// ~10x faster than A* on open grids with few obstacles.
///
/// # Arguments
/// Same as `astar` — interchangeable for Grid pathfinding.
pub fn jps(
    grid: &Grid<bool>,
    start: Coord2,
    goal: Coord2,
) -> Option<PathResult<Coord2>> { /* ... */ }

/// Jump in direction `dir` from `current` until hitting a jump point,
/// the goal, or a wall.
fn jump(
    grid: &Grid<bool>,
    current: Coord2,
    dir: Dir8,
    goal: Coord2,
) -> Option<Coord2> { /* ... */ }

/// Check if position has forced neighbors when approached from direction.
fn has_forced_neighbor(grid: &Grid<bool>, pos: Coord2, dir: Dir8) -> bool { /* ... */ }
```

### Step 12: Dijkstra Map (dijkstra.rs)

```rust
/// Multi-goal distance field built via BFS.
///
/// Unlike single-goal A*, a DijkstraMap computes distances from ALL cells
/// to ANY of the goal cells. This gives you:
/// - `distance_at(coord)`: how far is this cell from the nearest goal?
/// - `downhill(coord)`: which direction moves toward the nearest goal?
/// - `uphill(coord)`: which direction moves away from the nearest goal?
///
/// # Use Cases
/// - Fleeing: move uphill (away from danger)
/// - Chasing: move downhill (toward target)
/// - Heat maps: visualize distance fields
/// - Flow fields: convert to FlowFieldGrid for navex integration
///
/// # Example
/// ```
/// use tessera::{Grid, Coord2, DijkstraMap};
///
/// let grid = Grid::new_filled(20, 20, true);
/// let goals = vec![Coord2::new(10, 10)];
/// let dmap = DijkstraMap::build(&grid, &goals, |c| *grid.get(c).unwrap());
/// let dist = dmap.distance_at(Coord2::new(0, 0));
/// ```
pub struct DijkstraMap {
    distances: Vec<u32>,
    width: u32,
    height: u32,
}

impl DijkstraMap {
    /// Build from one or more goal cells using BFS.
    ///
    /// Algorithm:
    /// ```text
    /// distances = [u32::MAX; grid.len()]
    /// queue = VecDeque
    /// for each goal:
    ///     distances[goal] = 0
    ///     queue.push_back(goal)
    /// while let Some(current) = queue.pop_front():
    ///     for neighbor in grid.neighbors(current):
    ///         if !is_passable(neighbor): continue
    ///         cost = distances[current] + move_cost(current, neighbor)
    ///         if cost < distances[neighbor]:
    ///             distances[neighbor] = cost
    ///             queue.push_back(neighbor)
    /// ```
    pub fn build(
        grid: &Grid<bool>,
        goals: &[Coord2],
        is_passable: impl Fn(Coord2) -> bool,
    ) -> Self { /* ... */ }

    /// Distance from coord to nearest goal. u32::MAX if unreachable.
    pub fn distance_at(&self, coord: Coord2) -> u32 { /* ... */ }

    /// Direction toward the nearest goal (follow the gradient downhill).
    pub fn downhill(&self, coord: Coord2) -> Option<Dir8> { /* ... */ }

    /// Direction away from the nearest goal (flee uphill).
    pub fn uphill(&self, coord: Coord2) -> Option<Dir8> { /* ... */ }
}
```

### Step 13: Flow Field (flow.rs)

```rust
/// A precomputed direction field — every cell knows which direction
/// to step toward the nearest goal.
///
/// Built from a DijkstraMap. For integration with navex, export
/// via `bridge::grid_to_flow_field()`.
pub struct FlowFieldGrid {
    directions: Vec<Option<Dir8>>,
    width: u32,
    height: u32,
}

impl FlowFieldGrid {
    /// Build from a DijkstraMap by computing downhill direction at each cell.
    pub fn from_dijkstra(dmap: &DijkstraMap) -> Self { /* ... */ }

    /// Get the direction to step from this cell, or None if at a goal/unreachable.
    pub fn direction_at(&self, coord: Coord2) -> Option<Dir8> { /* ... */ }

    /// Follow the flow field from start, collecting the path.
    /// Stops at a goal or after max_steps.
    pub fn trace(&self, start: Coord2, max_steps: usize) -> Vec<Coord2> { /* ... */ }
}
```

**Tests for Phase 4:**
- `jps_matches_astar` — JPS and A* produce same-cost paths
- `jps_faster_than_astar` — JPS expands fewer nodes on open grid
- `dijkstra_single_goal` — distances radiate outward
- `dijkstra_multi_goal` — distance is to nearest goal
- `dijkstra_downhill` — following downhill reaches a goal
- `dijkstra_uphill` — following uphill moves away from goal
- `flow_field_all_cells_reachable` — all passable cells have a direction
- `flow_field_trace` — tracing from any cell reaches a goal

---

## Phase 5: Vision + Flood Fill

### Step 14: Line of Sight (los.rs)

```rust
/// Bresenham line algorithm: enumerate all cells on the line from a to b.
///
/// ```text
/// Bresenham's line:
///     dx = |x1 - x0|, dy = |y1 - y0|
///     sx = sign(x1 - x0), sy = sign(y1 - y0)
///     err = dx - dy
///     loop:
///         yield (x, y)
///         if (x, y) == (x1, y1): break
///         e2 = 2 * err
///         if e2 > -dy: err -= dy; x += sx
///         if e2 < dx: err += dx; y += sy
/// ```
pub fn bresenham(a: Coord2, b: Coord2) -> Vec<Coord2> { /* ... */ }

/// Check line of sight between two coordinates.
/// Returns true if no opaque cell blocks the line.
pub fn line_of_sight(
    a: Coord2,
    b: Coord2,
    is_opaque: impl Fn(Coord2) -> bool,
) -> bool { /* ... */ }
```

### Step 15: Field of View — Recursive Shadowcasting (los.rs)

```rust
/// Result of a field-of-view calculation.
pub struct FovResult {
    /// Bit per cell: true = visible.
    visible: Vec<bool>,
    width: u32,
    height: u32,
}

impl FovResult {
    pub fn is_visible(&self, coord: Coord2) -> bool { /* ... */ }
    pub fn visible_coords(&self) -> impl Iterator<Item = Coord2> + '_ { /* ... */ }
    pub fn count(&self) -> usize { /* ... */ }
}

/// Compute field of view using recursive shadowcasting.
///
/// Divides the circle into 8 octants and processes each independently.
/// Walls cast shadows that block vision to cells behind them.
///
/// # Algorithm (per octant)
/// ```text
/// for each octant (0..8):
///     cast_light(origin, radius, row=1, start_slope=1.0, end_slope=0.0)
///
/// cast_light(origin, radius, row, start_slope, end_slope):
///     if start_slope < end_slope: return
///     for y in row..=radius:
///         blocked = false
///         for x from round(y * end_slope) to round(y * start_slope):
///             // Transform (x, y) by octant to get real coordinates
///             actual = octant_transform(origin, x, y)
///             if not in_bounds(actual): continue
///             if is_opaque(actual):
///                 if not blocked:
///                     // Start of shadow — recurse with narrowed slopes
///                     cast_light(origin, radius, y+1, start_slope, left_slope(x,y))
///                     blocked = true
///                 next_start_slope = right_slope(x, y)
///             else:
///                 if blocked:
///                     start_slope = next_start_slope
///                     blocked = false
///                 mark_visible(actual)
///         if blocked: break  // entire row was blocked
/// ```
pub fn shadowcast(
    origin: Coord2,
    radius: u32,
    is_opaque: impl Fn(Coord2) -> bool,
    width: u32,
    height: u32,
) -> FovResult { /* ... */ }
```

### Step 16: Flood Fill (flood.rs)

```rust
/// BFS flood fill from a starting cell.
///
/// Returns all connected cells matching the predicate.
pub struct FloodResult {
    pub filled: Vec<bool>,
    pub count: usize,
    pub width: u32,
}

impl FloodResult {
    pub fn contains(&self, coord: Coord2) -> bool { /* ... */ }
}

/// Flood fill using BFS.
///
/// ```text
/// queue = [start]
/// visited[start] = true
/// count = 0
/// while queue not empty:
///     current = queue.pop_front()
///     count += 1
///     for neighbor in grid.neighbors(current):
///         if visited[neighbor]: continue
///         if !predicate(neighbor): continue
///         visited[neighbor] = true
///         queue.push_back(neighbor)
/// ```
pub fn flood_fill(
    grid: &Grid<bool>,
    start: Coord2,
    predicate: impl Fn(Coord2) -> bool,
) -> FloodResult { /* ... */ }

/// Check if two coordinates are connected (reachable from each other).
pub fn is_connected(
    grid: &Grid<bool>,
    a: Coord2,
    b: Coord2,
    predicate: impl Fn(Coord2) -> bool,
) -> bool { /* ... */ }
```

**Tests for Phase 5:**
- `bresenham_horizontal` — line from (0,0) to (5,0) has 6 cells
- `bresenham_diagonal` — line from (0,0) to (3,3) has 4 cells
- `bresenham_steep` — steep line covers expected cells
- `los_clear` — no obstacles = line of sight
- `los_blocked` — wall between two points blocks LOS
- `fov_open_room` — all cells within radius visible in empty grid
- `fov_wall_shadow` — cells behind a wall are not visible
- `fov_symmetry` — if A can see B, B can see A (within range)
- `flood_fill_room` — fills connected open cells
- `flood_fill_walls_block` — doesn't cross walls
- `is_connected_yes` — two points in same room
- `is_connected_no` — two points in different rooms

---

## Phase 6: Spatial Hash

### Step 17: Aabb2 (spatial_hash.rs)

```rust
/// Axis-aligned bounding box in 2D.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Aabb2 {
    pub min: Coord2,
    pub max: Coord2,
}

impl Aabb2 {
    pub fn new(min: Coord2, max: Coord2) -> Self { /* ... */ }
    pub fn from_center_half(center: Coord2, half_w: i32, half_h: i32) -> Self { /* ... */ }
    pub fn width(&self) -> u32 { (self.max.x - self.min.x) as u32 }
    pub fn height(&self) -> u32 { (self.max.y - self.min.y) as u32 }
    pub fn center(&self) -> Coord2 { /* ... */ }
    pub fn contains(&self, point: Coord2) -> bool { /* ... */ }
    pub fn intersects(&self, other: &Aabb2) -> bool { /* ... */ }
    pub fn union(&self, other: &Aabb2) -> Aabb2 { /* ... */ }
    pub fn expand(&self, amount: i32) -> Aabb2 { /* ... */ }
}
```

### Step 18: SpatialHash (spatial_hash.rs)

```rust
use alloc::collections::BTreeMap;

/// Broad-phase spatial lookup using a grid of buckets.
///
/// Entities are stored in buckets based on which grid cells their AABB
/// overlaps. Queries return all entities whose buckets overlap the
/// query region.
///
/// # Example
/// ```
/// use tessera::{SpatialHash, Aabb2, Coord2};
///
/// let mut hash = SpatialHash::new(16); // 16-unit cell size
/// hash.insert(0, Aabb2::new(Coord2::new(0, 0), Coord2::new(10, 10)));
/// hash.insert(1, Aabb2::new(Coord2::new(100, 100), Coord2::new(110, 110)));
///
/// let query = Aabb2::new(Coord2::new(5, 5), Coord2::new(15, 15));
/// let hits = hash.query(&query);
/// assert!(hits.contains(&0));
/// assert!(!hits.contains(&1));
/// ```
pub struct SpatialHash {
    cell_size: u32,
    cells: BTreeMap<(i32, i32), Vec<u32>>,
    entity_bounds: BTreeMap<u32, Aabb2>,
}

impl SpatialHash {
    pub fn new(cell_size: u32) -> Self { /* ... */ }
    pub fn insert(&mut self, id: u32, aabb: Aabb2) { /* ... */ }
    pub fn remove(&mut self, id: u32) { /* ... */ }
    pub fn update(&mut self, id: u32, new_aabb: Aabb2) { /* ... */ }
    pub fn query(&self, aabb: &Aabb2) -> Vec<u32> { /* ... */ }
    pub fn query_point(&self, point: Coord2) -> Vec<u32> { /* ... */ }
    pub fn clear(&mut self) { /* ... */ }
    pub fn len(&self) -> usize { /* ... */ }
}
```

**Tests for Phase 6:**
- `aabb_contains` — point inside/outside
- `aabb_intersects` — overlapping/non-overlapping boxes
- `aabb_union` — union of two boxes
- `spatial_hash_insert_query` — insert and retrieve
- `spatial_hash_no_false_positives` — far-away entities not returned
- `spatial_hash_remove` — removed entity not in query results
- `spatial_hash_update` — moved entity appears in new location

---

## Phase 7: Level Generation

### Step 19: BSP Dungeon (bsp.rs)

```rust
/// A room placed by BSP generation.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Room {
    pub x: i32,
    pub y: i32,
    pub w: u32,
    pub h: u32,
}

impl Room {
    pub fn center(&self) -> Coord2 { /* ... */ }
    pub fn intersects(&self, other: &Room) -> bool { /* ... */ }
    pub fn contains(&self, point: Coord2) -> bool { /* ... */ }
    pub fn corners(&self) -> [Coord2; 4] { /* ... */ }
    pub fn expand(&self, amount: i32) -> Room { /* ... */ }
}

/// A corridor connecting two rooms.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Corridor {
    pub from: Coord2,
    pub to: Coord2,
    pub width: u32,
}

/// Configuration for BSP dungeon generation.
#[derive(Clone, Debug)]
pub struct BspConfig {
    pub width: u32,
    pub height: u32,
    pub min_room_size: u32,
    pub max_room_size: u32,
    pub min_ratio: f32,
    pub corridor_width: u32,
    pub max_depth: u32,
}

/// Result of BSP generation — caller stamps these onto their grid.
#[derive(Clone, Debug)]
pub struct BspResult {
    pub rooms: Vec<Room>,
    pub corridors: Vec<Corridor>,
}

/// Generate a dungeon using Binary Space Partitioning.
///
/// Algorithm:
/// ```text
/// split(rect, depth, rng):
///     if depth >= max_depth OR rect too small:
///         place room within rect (with margins)
///         return
///     choose split direction:
///         if rect.w > 1.25 * rect.h: vertical
///         elif rect.h > 1.25 * rect.w: horizontal
///         else: random
///     split_pos = random in [min_size .. dimension - min_size]
///     split(left_half, depth+1, rng)
///     split(right_half, depth+1, rng)
///     connect(left_room.center, right_room.center)  // L-shaped corridor
/// ```
pub fn bsp_generate(config: &BspConfig, rng: &mut impl rand_core::RngCore) -> BspResult { /* ... */ }
```

### Step 20: Cellular Automata (cellular.rs)

```rust
/// Configuration for cellular automata cave generation.
#[derive(Clone, Debug)]
pub struct CellularConfig {
    pub width: u32,
    pub height: u32,
    /// Chance each cell starts as floor (0-100).
    pub fill_chance: u32,
    /// A floor becomes wall if wall_neighbors >= birth_limit.
    pub birth_limit: u32,
    /// A wall stays wall if wall_neighbors >= death_limit.
    pub death_limit: u32,
    /// Number of smoothing iterations.
    pub iterations: u32,
}

/// Generate a cave using cellular automata.
///
/// Algorithm:
/// ```text
/// 1. Fill grid randomly (fill_chance% floor, rest wall)
/// 2. For each iteration:
///     for each cell:
///         walls = count walls in Moore neighborhood (8 neighbors)
///         if cell is wall:
///             next[cell] = walls >= death_limit  (stay wall if crowded)
///         else:
///             next[cell] = walls >= birth_limit  (become wall if crowded)
/// ```
///
/// Returns Grid<bool> where true = floor, false = wall.
pub fn cellular_generate(
    config: &CellularConfig,
    rng: &mut impl rand_core::RngCore,
) -> Grid<bool> { /* ... */ }
```

### Step 21: Drunkard's Walk (drunkard.rs)

```rust
/// Configuration for drunkard's walk cave generation.
#[derive(Clone, Debug)]
pub struct DrunkardConfig {
    pub width: u32,
    pub height: u32,
    /// Maximum steps per walker.
    pub max_steps: u32,
    /// Number of walkers.
    pub num_walkers: u32,
    /// Chance to turn (0-100). Higher = more winding.
    pub turn_chance: u32,
    /// Stop when this fraction of cells are floor (0.0 - 1.0).
    pub target_floor_ratio: f32,
}

/// Generate organic caves using random walkers.
///
/// Each walker starts at the center, picks a random direction,
/// walks forward carving floors. Has a `turn_chance` to change direction.
pub fn drunkard_generate(
    config: &DrunkardConfig,
    rng: &mut impl rand_core::RngCore,
) -> Grid<bool> { /* ... */ }
```

### Step 22: Poisson Disk Sampling (poisson.rs)

```rust
/// Configuration for Poisson disk point generation.
#[derive(Clone, Debug)]
pub struct PoissonConfig {
    pub width: f32,
    pub height: f32,
    /// Minimum distance between any two points.
    pub min_distance: f32,
    /// Attempts per active point before removing from active list.
    pub max_attempts: u32,
}

/// Generate well-spaced random points using Bridson's algorithm.
///
/// Algorithm:
/// ```text
/// cell_size = min_distance / sqrt(2)
/// background_grid = Grid of Option<usize>  (spatial acceleration)
/// active_list = [random first point]
/// points = [first point]
///
/// while active_list not empty:
///     pick random point from active_list
///     for k in 0..max_attempts:
///         candidate = random point in annulus [min_distance, 2*min_distance]
///         if no existing point within min_distance of candidate:
///             accept: add to points + active_list + background_grid
///             break
///     if no candidate accepted:
///         remove point from active_list
/// ```
pub fn poisson_generate(
    config: &PoissonConfig,
    rng: &mut impl rand_core::RngCore,
) -> Vec<(f32, f32)> { /* ... */ }
```

### Step 23: Prefab (prefab.rs)

```rust
/// A hand-designed room template that can be stamped onto a grid.
///
/// Supports rotation and flipping for variety.
#[derive(Clone, Debug)]
pub struct Prefab<T: Clone> {
    pub cells: Grid<T>,
    /// Anchor point for placement (usually center or door position).
    pub anchor: Coord2,
}

impl<T: Clone> Prefab<T> {
    pub fn new(cells: Grid<T>, anchor: Coord2) -> Self { /* ... */ }

    /// Rotate 90° clockwise.
    pub fn rotate_cw(&self) -> Self { /* ... */ }

    /// Rotate 90° counter-clockwise.
    pub fn rotate_ccw(&self) -> Self { /* ... */ }

    /// Flip horizontally.
    pub fn flip_h(&self) -> Self { /* ... */ }

    /// Flip vertically.
    pub fn flip_v(&self) -> Self { /* ... */ }

    /// Stamp this prefab onto a target grid at the given position.
    /// Position is where the anchor maps to.
    pub fn stamp_onto(&self, target: &mut Grid<T>, position: Coord2) { /* ... */ }

    /// Check if stamping at position would go out of bounds.
    pub fn fits(&self, target: &Grid<T>, position: Coord2) -> bool { /* ... */ }
}
```

**Tests for Phase 7:**
- `bsp_rooms_no_overlap` — no two rooms intersect
- `bsp_all_connected` — all rooms reachable via flood fill
- `bsp_respects_min_size` — no room smaller than min_room_size
- `cellular_floor_ratio` — floor count is reasonable after iterations
- `cellular_largest_cave` — largest connected region > 50% of floors
- `drunkard_target_ratio` — achieves target floor ratio ± 5%
- `drunkard_connected` — starting point is in largest cave
- `poisson_min_distance` — no two points closer than min_distance
- `poisson_coverage` — points distributed across the area
- `prefab_rotate_cw_4x` — 4 rotations return to original
- `prefab_stamp` — cells appear at correct position on target
- `prefab_fits` — returns false if would go out of bounds

---

## Phase 8: Bridge Module

### Step 24: Bridge Functions (bridge.rs)

```rust
/// Convert WFC tile output into a Grid.
///
/// Takes flat array of tile IDs from wavfc collapse and wraps it
/// as a Grid<u32>. Does not depend on wavfc — accepts raw data.
pub fn grid_from_wfc(width: u32, height: u32, tile_ids: &[u32]) -> Grid<u32> { /* ... */ }

/// Convert graph layout into rooms and corridors.
///
/// Takes nodes as (x, y, width, height) tuples and edges as (from, to) pairs.
/// Produces a BspResult compatible with room/corridor stamping.
/// Does not depend on grammex — accepts raw data.
pub fn layout_from_graph(
    nodes: &[(i32, i32, u32, u32)],
    edges: &[(usize, usize)],
) -> BspResult { /* ... */ }

/// Convert a passability grid into a flow field.
///
/// Returns (directions, width, height) as flat arrays suitable for
/// passing to navex via FFI or direct memory sharing.
pub fn grid_to_flow_field(
    grid: &Grid<bool>,
    goals: &[Coord2],
) -> FlowFieldGrid { /* ... */ }

/// Convert wall cells into merged AABBs using greedy meshing.
///
/// Scans the grid for contiguous wall regions and merges them into
/// larger rectangles. Results can be passed to navex as obstacles.
///
/// Greedy meshing algorithm:
/// ```text
/// visited = Grid<bool>
/// aabbs = []
/// for y in 0..height:
///     for x in 0..width:
///         if !is_wall(x,y) or visited(x,y): continue
///         // Extend right as far as possible
///         end_x = x
///         while end_x+1 < width && is_wall(end_x+1, y) && !visited(end_x+1, y):
///             end_x += 1
///         // Extend down as far as possible (all cells in row must be wall)
///         end_y = y
///         while end_y+1 < height:
///             all_wall = true
///             for cx in x..=end_x:
///                 if !is_wall(cx, end_y+1) or visited(cx, end_y+1):
///                     all_wall = false; break
///             if !all_wall: break
///             end_y += 1
///         // Mark all cells visited, emit AABB
///         for cy in y..=end_y:
///             for cx in x..=end_x:
///                 visited.set(cx, cy, true)
///         aabbs.push(Aabb2::new(Coord2(x, y), Coord2(end_x, end_y)))
/// ```
pub fn walls_to_aabbs(
    grid: &Grid<bool>,
    is_wall: impl Fn(Coord2) -> bool,
) -> Vec<Aabb2> { /* ... */ }
```

**Tests for Phase 8:**
- `grid_from_wfc_roundtrip` — tile IDs preserved
- `layout_from_graph_creates_corridors` — edges become corridors
- `flow_field_from_grid` — all reachable cells have direction
- `walls_to_aabbs_merges` — 4 adjacent wall cells → 1 AABB
- `walls_to_aabbs_separate` — disconnected walls → separate AABBs
- `walls_to_aabbs_l_shape` — L-shaped wall → 2 AABBs (can't merge into rectangle)

---

## Phase 9: Infrastructure + lib.rs

### Step 25: Observer (observer.rs)

```rust
/// Observe pathfinding progress for visualization/debugging.
pub trait PathfindObserver {
    fn on_expand(&mut self, _coord: Coord2, _g: u32, _f: u32) {}
    fn on_path_found(&mut self, _path: &[Coord2], _cost: u32) {}
    fn on_no_path(&mut self) {}
}

/// Observe level generation progress.
pub trait GenerationObserver {
    fn on_room_placed(&mut self, _room: &Room) {}
    fn on_corridor_carved(&mut self, _from: Coord2, _to: Coord2) {}
    fn on_iteration(&mut self, _iteration: u32) {}
}

pub struct NoOpPathfindObserver;
impl PathfindObserver for NoOpPathfindObserver {}

pub struct NoOpGenerationObserver;
impl GenerationObserver for NoOpGenerationObserver {}
```

### Step 26: Error (error.rs)

```rust
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TesseraError {
    OutOfBounds { coord: Coord2, width: u32, height: u32 },
    GridSizeMismatch { expected: usize, got: usize },
    InvalidConfig(& 'static str),
    EmptyGoals,
}
```

### Step 27: lib.rs Re-exports

```rust
#![no_std]
extern crate alloc;

pub mod coord;
pub mod neighborhood;
pub mod grid_trait;
pub mod grid;
pub mod grid3;
pub mod hex;
pub mod pathfind;
pub mod jps;
pub mod dijkstra;
pub mod flow;
pub mod los;
pub mod flood;
pub mod spatial_hash;
pub mod query;
pub mod bsp;
pub mod cellular;
pub mod drunkard;
pub mod poisson;
pub mod prefab;
pub mod bridge;
pub mod config;
pub mod observer;
pub mod error;

pub use coord::{Coord2, Coord3};
pub use hex::HexCoord;
pub use neighborhood::{Dir4, Dir8, Dir6, Neighborhood};
pub use grid_trait::SpatialGrid;
pub use grid::Grid;
pub use grid3::Grid3;
pub use hex::HexGrid;
pub use pathfind::{astar, PathResult};
pub use jps::jps;
pub use dijkstra::DijkstraMap;
pub use flow::FlowFieldGrid;
pub use los::{bresenham, line_of_sight, shadowcast, FovResult};
pub use flood::{flood_fill, is_connected, FloodResult};
pub use spatial_hash::{SpatialHash, Aabb2};
pub use bsp::{bsp_generate, Room, Corridor, BspConfig, BspResult};
pub use cellular::{cellular_generate, CellularConfig};
pub use drunkard::{drunkard_generate, DrunkardConfig};
pub use poisson::{poisson_generate, PoissonConfig};
pub use prefab::Prefab;
```

### Full Test List

```text
# coord.rs (12 tests)
- coord2_add, coord2_sub, coord2_neg
- coord2_manhattan, coord2_chebyshev, coord2_octile
- coord2_index_roundtrip, coord2_out_of_bounds
- coord3_index_roundtrip
- hex_distance_adjacent, hex_ring_count, hex_spiral_count

# grid.rs (8 tests)
- grid_new_filled, grid_from_fn, grid_set_get
- grid_out_of_bounds, grid_row_slice
- grid_neighbors_corner, grid_neighbors_center, grid_iter_coords

# grid3.rs (3 tests)
- grid3_3d_indexing, grid3_layers_independent, grid3_out_of_bounds

# hex.rs (4 tests)
- hexgrid_cell_count, hexgrid_neighbor_count
- hexgrid_boundary, hexgrid_get_set

# pathfind.rs (7 tests)
- astar_straight_line, astar_around_wall, astar_no_path
- astar_start_is_goal, astar_diagonal, astar_cost_accuracy
- astar_nodes_expanded

# jps.rs (3 tests)
- jps_matches_astar, jps_faster, jps_no_path

# dijkstra.rs (4 tests)
- dijkstra_single_goal, dijkstra_multi_goal
- dijkstra_downhill, dijkstra_uphill

# flow.rs (2 tests)
- flow_field_all_reachable, flow_field_trace

# los.rs (6 tests)
- bresenham_horizontal, bresenham_diagonal, bresenham_steep
- los_clear, los_blocked, fov_wall_shadow

# flood.rs (3 tests)
- flood_fill_room, flood_fill_walls_block, is_connected

# spatial_hash.rs (4 tests)
- spatial_hash_insert_query, spatial_hash_no_false_positives
- spatial_hash_remove, spatial_hash_update

# bsp.rs (3 tests)
- bsp_rooms_no_overlap, bsp_all_connected, bsp_min_size

# cellular.rs (2 tests)
- cellular_floor_ratio, cellular_largest_cave

# drunkard.rs (2 tests)
- drunkard_target_ratio, drunkard_connected

# poisson.rs (2 tests)
- poisson_min_distance, poisson_coverage

# prefab.rs (3 tests)
- prefab_rotate_4x, prefab_stamp, prefab_fits

# bridge.rs (4 tests)
- grid_from_wfc_roundtrip, flow_field_from_grid
- walls_to_aabbs_merges, walls_to_aabbs_separate
```

---

## Phase 10: WASM Demo

The demo should have 4 tabs, each showcasing a different feature.

#### Tab 1: Pathfinding
- 40×30 grid rendered as tiles
- Click to toggle walls
- Shift-click to set start, Ctrl-click to set goal
- Show A* path (blue) and JPS path (green) side by side
- Highlight expanded nodes (light shade) to visualize search efficiency
- Display cost and nodes-expanded counts

#### Tab 2: Field of View
- Same grid but with an FOV origin point
- Click to place walls
- Drag the origin point to see shadowcast FOV update in real-time
- Visible cells are bright, shadowed cells are dark
- Show Bresenham lines from origin to mouse cursor

#### Tab 3: Dungeon Generator
- Toggle between BSP, Cellular Automata, and Drunkard's Walk
- Sliders for all generation parameters
- "Regenerate" button with random seed
- Overlay pathfinding from start to goal
- Show room outlines for BSP, connected regions for cellular
- Display generation stats (room count, floor %, connected %)

#### Tab 4: Hex Grid
- Hex grid rendered as flat-top hexagons
- Click to toggle walls
- Hex pathfinding with highlighted path
- Hex FOV from a movable origin
- Hex flood fill with colored regions
- Toggle between distance display and direction display

**WASM bindings pattern** (in `demo-wasm/src/lib.rs`):
- Export `GridWorld` struct wrapping Grid + pathfinding state
- Export `tick()`, `render()`, `toggle_wall()`, `set_start()`, `set_goal()` functions
- Return flat arrays of render data to JS for canvas 2D drawing

**JS pattern** (in `demo-wasm/www/main.js`):
- Canvas 2D rendering: rectangles for square grid, hexagon paths for hex
- Mouse handling for wall placement and point selection
- Tab switching and parameter sliders
- Color coding: floor=white, wall=black, path=blue, FOV=yellow, expanded=grey

---

## Algorithm References

### A* Pathfinding

```text
OPEN = priority queue (min f-score)
CLOSED = set
g[start] = 0
f[start] = h(start, goal)
OPEN.push(start)

while OPEN not empty:
    current = OPEN.pop()
    if current == goal: return reconstruct_path()
    CLOSED.add(current)

    for neighbor in grid.neighbors(current):
        if neighbor in CLOSED: skip
        if not passable(neighbor): skip

        tentative_g = g[current] + cost(current, neighbor)
        if tentative_g < g[neighbor]:
            g[neighbor] = tentative_g
            f[neighbor] = tentative_g + h(neighbor, goal)
            parent[neighbor] = current
            OPEN.push(neighbor)

Heuristics:
    4-dir: Manhattan × 10
    8-dir: Octile distance (max(dx,dy)*10 + min(dx,dy)*4)
    Hex: Hex distance × 10
```

### Jump Point Search

```text
JPS optimization for regular grids:

1. Instead of adding all neighbors to OPEN, "jump" in each direction
2. A jump point is found when:
   a. We reach the goal
   b. We find a forced neighbor (wall creates detour opportunity)
   c. (Diagonal only) A cardinal jump from here finds a jump point

Forced neighbor detection (jumping East):
    Wall at (x, y-1) AND open at (x+1, y-1) → forced neighbor NE
    Wall at (x, y+1) AND open at (x+1, y+1) → forced neighbor SE

Cardinal jump (East):
    for x' = x+1, x+2, ...:
        if wall(x', y): return None (hit wall)
        if (x', y) == goal: return (x', y)
        if has_forced_neighbor(x', y, East): return (x', y)

Diagonal jump (NE):
    for each step NE from (x, y):
        check horizontal (East) jump → if found, this is jump point
        check vertical (North) jump → if found, this is jump point
        if has_forced_neighbor here: this is jump point
```

### Recursive Shadowcasting

```text
8 octants, each processed with a transformation:
    Octant 0: ( x,  y) → ( x,  y)
    Octant 1: ( y,  x) → ( x,  y)
    Octant 2: ( y, -x) → ( x,  y)
    ... (reflections/rotations)

cast_light(origin, radius, row, start_slope, end_slope, transform):
    Processes one octant. Slopes define the visible arc:
        start_slope = left edge of visible arc
        end_slope = right edge of visible arc

    When a wall is encountered:
        1. The visible arc is narrowed (start_slope adjusted)
        2. A recursive call handles the portion before the wall

    When transitioning from wall back to floor:
        The start_slope is updated to the right edge of the wall

Slope calculation:
    left_slope(x, y) = (x - 0.5) / (y + 0.5)
    right_slope(x, y) = (x + 0.5) / (y - 0.5)
    (Using floats for slopes; can also use integer fractions)
```

### BSP Generation

```text
1. Start with full map rectangle
2. Recursively split:
    - If rectangle is large enough:
        Choose split direction (prefer splitting the longer dimension)
        Choose split position (random within valid range)
        Split into two child rectangles
        Recurse on both children
    - If too small to split:
        Place a room within the rectangle (random size within limits)
3. Connect rooms:
    - For each split, connect the nearest rooms in left/right children
    - L-shaped corridors: horizontal then vertical (or vice versa)
```

### Cellular Automata

```text
Standard B5678/S45678 variant for cave generation:

Initial: random fill (typically 45-55% wall)

Each iteration:
    For each cell:
        count = number of walls in 8 neighbors (Moore neighborhood)
        If cell is wall:
            If count >= death_limit (typically 3-4): stay wall
            Else: become floor
        If cell is floor:
            If count >= birth_limit (typically 4-5): become wall
            Else: stay floor

4-5 iterations typically produces good cave shapes.
```

### Poisson Disk Sampling (Bridson's Algorithm)

```text
Properties:
    - All points are at least min_distance apart
    - Distribution is roughly uniform (no clumps or gaps)
    - O(n) time complexity

Algorithm:
    cell_size = min_distance / sqrt(2)
    grid = 2D array of cells (each cell fits at most 1 point)

    1. Place first point randomly
    2. Add to active list
    3. While active list not empty:
        a. Pick random point P from active list
        b. For k attempts (typically 30):
            Generate random candidate in annulus [r, 2r] around P
            Check grid cells within r of candidate
            If no existing point within r: accept candidate
        c. If no candidate accepted: remove P from active list
```

### Greedy Wall Meshing

```text
For converting grid walls to minimal AABB set:

1. Scan left-to-right, top-to-bottom
2. For each unvisited wall cell:
    a. Extend right: find longest horizontal run of unvisited walls
    b. Extend down: try to extend the rectangle downward
       (all cells in each new row must be unvisited walls)
    c. Mark all cells in the rectangle as visited
    d. Emit AABB for the rectangle

This is a greedy approach — not globally optimal but produces
reasonably few AABBs in O(n) time.
```

---

## Verification Checklist

```bash
# In tessera/
cargo test --target x86_64-pc-windows-msvc
cargo build --target wasm32-unknown-unknown --release
cargo clippy --target x86_64-pc-windows-msvc

# WASM demo
cd demo-wasm
wasm-pack build --target web --release
# Serve demo-wasm/www/ with a local HTTP server and test in browser
```
