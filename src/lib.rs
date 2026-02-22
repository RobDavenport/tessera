#![no_std]

extern crate alloc;

pub mod bridge;
pub mod bsp;
pub mod cellular;
pub mod config;
pub mod coord;
pub mod dijkstra;
pub mod drunkard;
pub mod error;
pub mod flood;
pub mod flow;
pub mod grid;
pub mod grid3;
pub mod grid_trait;
pub mod hex;
pub mod jps;
pub mod los;
pub mod neighborhood;
pub mod observer;
pub mod pathfind;
pub mod poisson;
pub mod prefab;
pub mod query;
pub mod spatial_hash;

pub use bridge::{grid_from_wfc, grid_to_flow_field, layout_from_graph, walls_to_aabbs};
pub use bsp::{bsp_generate, BspConfig, BspResult, Corridor, Room};
pub use cellular::{cellular_generate, CellularConfig};
pub use coord::{Coord2, Coord3};
pub use dijkstra::DijkstraMap;
pub use drunkard::{drunkard_generate, DrunkardConfig};
pub use flow::FlowFieldGrid;
pub use flood::{flood_fill, is_connected, FloodResult};
pub use grid::Grid;
pub use grid3::Grid3;
pub use grid_trait::SpatialGrid;
pub use hex::{HexCoord, HexGrid};
pub use jps::jps;
pub use los::{bresenham, line_of_sight, shadowcast, FovResult};
pub use neighborhood::{Dir4, Dir6, Dir8, Neighborhood};
pub use pathfind::{astar, PathResult};
pub use poisson::{poisson_generate, PoissonConfig};
pub use prefab::Prefab;
pub use spatial_hash::{Aabb2, SpatialHash};
