//! RoadNetworkLayer: Generates A* roads from hub to all destinations.
//! Deterministic (1 candidate).
//!
//! Roads are placed incrementally: destinations are sorted by distance from the
//! hub (nearest first), and each path is computed with already-placed road tiles
//! costing zero. This causes later paths to merge onto existing roads, forming
//! a shared trunk that branches near each destination -- minimizing total road
//! tiles while producing equivalent-length routes.

use crate::layer::*;
use crate::location::*;
use crate::plan::*;
use crate::terrain::*;
use fnv::FnvHashSet;
use pathfinding::directed::astar::astar;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Generates A* road network from hub to all key destinations.
/// Deterministic (1 candidate).
pub struct RoadNetworkLayer;

impl PlacementLayer for RoadNetworkLayer {
    fn name(&self) -> &str {
        "road_network"
    }

    fn candidate_count(
        &self,
        _state: &PlacementState,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(1)
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index > 0 {
            return None;
        }

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let mut new_state = state.clone();
        let mut road_tiles: FnvHashSet<Location> = FnvHashSet::default();
        let mut road_edges: Vec<(Location, Location)> = Vec::new();

        // Collect all route destinations (internal infrastructure only).
        // Roads to adjacent rooms are handled at runtime, not during planning.
        let mut destinations: Vec<Location> = Vec::new();

        // Source containers
        destinations.extend(state.get_landmark_set("source_containers"));

        // Controller container
        if let Some(cc) = state.get_landmark("controller_container") {
            destinations.push(cc);
        }

        // Mineral container
        if let Some(mc) = state.get_landmark("mineral_container") {
            destinations.push(mc);
        }

        // Sort destinations by distance from hub (nearest first). This builds
        // a natural trunk road outward from the hub; farther destinations
        // branch off the existing trunk rather than creating parallel roads.
        destinations.sort_by_key(|d| hub.distance_to(*d));

        let occupied = &state.occupied;

        // Route to each destination incrementally. Already-placed road tiles
        // have zero movement cost, so A* strongly prefers reusing them.
        for dest in &destinations {
            if let Some((path, _cost)) = find_path(hub, *dest, terrain, occupied, &road_tiles) {
                let mut prev = None;
                for loc in &path {
                    road_tiles.insert(*loc);
                    if let Some(p) = prev {
                        road_edges.push((p, *loc));
                    }
                    prev = Some(*loc);
                }
            }
        }

        // Place road structures
        for &road_loc in &road_tiles {
            let already_has_structure = new_state
                .structures
                .get(&road_loc)
                .map(|items| {
                    items
                        .iter()
                        .any(|i| i.structure_type != StructureType::Road)
                })
                .unwrap_or(false);
            if !already_has_structure {
                new_state
                    .structures
                    .entry(road_loc)
                    .or_default()
                    .push(RoomItem::new(StructureType::Road, 1));
            }
        }

        // Store road edges in landmark set (as pairs -- flatten to a list)
        for (a, b) in &road_edges {
            new_state.add_to_landmark_set("road_edges_a", *a);
            new_state.add_to_landmark_set("road_edges_b", *b);
        }

        Some(Ok(new_state))
    }
}

/// A* pathfinding between two locations.
///
/// Tiles that already have roads cost nothing to traverse, encouraging path
/// reuse and minimizing total road count. Non-road tiles cost 1 (plains) or
/// 3 (swamp), with an additional penalty for tiles occupied by non-road
/// structures.
fn find_path(
    start: Location,
    goal: Location,
    terrain: &FastRoomTerrain,
    occupied: &FnvHashSet<Location>,
    existing_roads: &FnvHashSet<Location>,
) -> Option<(Vec<Location>, u32)> {
    let start_pos = (start.x() as i16, start.y() as i16);
    let goal_pos = (goal.x() as i16, goal.y() as i16);

    let result = astar(
        &start_pos,
        |&(x, y)| {
            NEIGHBORS_8
                .iter()
                .filter_map(move |&(dx, dy)| {
                    let nx = x + dx as i16;
                    let ny = y + dy as i16;
                    if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                        return None;
                    }
                    let ux = nx as u8;
                    let uy = ny as u8;
                    if terrain.is_wall(ux, uy) {
                        return None;
                    }
                    let loc = Location::from_coords(ux as u32, uy as u32);

                    // Already-placed roads are free to traverse, strongly
                    // encouraging path merging onto existing roads.
                    if existing_roads.contains(&loc) {
                        return Some(((nx, ny), 0u32));
                    }

                    let base_cost = if terrain.is_swamp(ux, uy) { 3u32 } else { 1u32 };
                    let structure_penalty = if occupied.contains(&loc) { 5u32 } else { 0u32 };
                    Some(((nx, ny), base_cost + structure_penalty))
                })
                .collect::<Vec<_>>()
        },
        |&(x, y)| {
            let dx = (x - goal_pos.0).unsigned_abs() as u32;
            let dy = (y - goal_pos.1).unsigned_abs() as u32;
            dx.max(dy)
        },
        |&(x, y)| x == goal_pos.0 && y == goal_pos.1,
    );

    result.map(|(path, cost)| {
        let locs: Vec<Location> = path
            .into_iter()
            .map(|(x, y)| Location::from_coords(x as u32, y as u32))
            .collect();
        (locs, cost)
    })
}
