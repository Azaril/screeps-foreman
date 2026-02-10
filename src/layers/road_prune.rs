//! RoadPruneLayer: Removes extraneous road tiles after the road network has
//! been generated.  Prunes unreachable roads and iteratively removes dead-end
//! road stubs that don't serve any building.
//!
//! Runs after RoadNetworkLayer and before ReachabilityLayer so the final road
//! layout is validated by the reachability check.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;
use log::*;
use std::collections::VecDeque;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Prunes dead-end and unreachable road tiles.
/// Deterministic (1 candidate).
pub struct RoadPruneLayer;

impl PlacementLayer for RoadPruneLayer {
    fn name(&self) -> &str {
        "road_prune"
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

        // --- Step 1: BFS from hub to find reachable tiles ----------------
        let reachable = walkable_bfs(hub, terrain, &new_state);

        // --- Step 2: Remove unreachable roads ----------------------------
        let all_road_locs: Vec<Location> = new_state
            .structures
            .iter()
            .filter(|(_, items)| items.iter().any(|i| i.structure_type == StructureType::Road))
            .map(|(loc, _)| *loc)
            .collect();

        let mut removed = 0u32;

        for &loc in &all_road_locs {
            let idx = loc.y() as usize * 50 + loc.x() as usize;
            if reachable[idx] == u32::MAX {
                remove_road(&mut new_state, loc);
                removed += 1;
            }
        }

        // --- Step 3: Iteratively prune dead-end roads --------------------
        // A dead-end road has 0 or 1 neighboring road tiles and is not
        // adjacent to any non-road structure.  Removing it may cascade:
        // its neighbor may become a new dead-end.
        loop {
            let mut pruned_this_pass = 0u32;

            let road_locs: Vec<Location> = new_state
                .structures
                .iter()
                .filter(|(_, items)| {
                    items.iter().any(|i| i.structure_type == StructureType::Road)
                })
                .map(|(loc, _)| *loc)
                .collect();

            for &loc in &road_locs {
                // Skip if adjacent to any non-road structure (serves a building).
                if is_adjacent_to_structure(&new_state, loc) {
                    continue;
                }

                // Count neighboring road tiles.
                let road_neighbors = count_road_neighbors(&new_state, loc);

                if road_neighbors <= 1 {
                    remove_road(&mut new_state, loc);
                    pruned_this_pass += 1;
                }
            }

            if pruned_this_pass == 0 {
                break;
            }

            removed += pruned_this_pass;
        }

        debug!("RoadPruneLayer: removed {} extraneous road tiles", removed);

        Some(Ok(new_state))
    }
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// BFS from hub over walkable tiles.
///
/// A tile is walkable if it's not a wall and either:
///   - not occupied by a non-road structure, OR
///   - has a road on it (roads make occupied tiles walkable).
///
/// This matches the reachability layer's walkability definition.
fn walkable_bfs(hub: Location, terrain: &FastRoomTerrain, state: &PlacementState) -> Vec<u32> {
    let mut dist = vec![u32::MAX; 50 * 50];
    let mut queue: VecDeque<(u8, u8, u32)> = VecDeque::new();

    let hx = hub.x();
    let hy = hub.y();
    dist[hy as usize * 50 + hx as usize] = 0;
    queue.push_back((hx, hy, 0));

    while let Some((x, y, d)) = queue.pop_front() {
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = x as i16 + dx as i16;
            let ny = y as i16 + dy as i16;
            if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                continue;
            }
            let ux = nx as u8;
            let uy = ny as u8;
            let idx = uy as usize * 50 + ux as usize;
            if dist[idx] != u32::MAX {
                continue;
            }
            if terrain.is_wall(ux, uy) {
                continue;
            }
            let loc = Location::from_coords(ux as u32, uy as u32);
            let is_walkable = !state.occupied.contains(&loc) || has_road_at(state, loc);
            if !is_walkable {
                continue;
            }
            dist[idx] = d + 1;
            queue.push_back((ux, uy, d + 1));
        }
    }

    dist
}

/// Check if a location has a road placed on it.
fn has_road_at(state: &PlacementState, loc: Location) -> bool {
    state
        .structures
        .get(&loc)
        .map(|items| items.iter().any(|i| i.structure_type == StructureType::Road))
        .unwrap_or(false)
}

/// Check if a location has any neighboring tile (Chebyshev distance 1) that
/// contains a non-road structure.
fn is_adjacent_to_structure(state: &PlacementState, loc: Location) -> bool {
    let x = loc.x();
    let y = loc.y();
    for &(dx, dy) in &NEIGHBORS_8 {
        let nx = x as i16 + dx as i16;
        let ny = y as i16 + dy as i16;
        if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
            continue;
        }
        let nloc = Location::from_coords(nx as u32, ny as u32);
        if let Some(items) = state.structures.get(&nloc) {
            if items
                .iter()
                .any(|i| i.structure_type != StructureType::Road)
            {
                return true;
            }
        }
    }
    false
}

/// Count the number of neighboring tiles (Chebyshev distance 1) that have a
/// road placed on them.
fn count_road_neighbors(state: &PlacementState, loc: Location) -> u8 {
    let x = loc.x();
    let y = loc.y();
    let mut count = 0u8;
    for &(dx, dy) in &NEIGHBORS_8 {
        let nx = x as i16 + dx as i16;
        let ny = y as i16 + dy as i16;
        if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
            continue;
        }
        let nloc = Location::from_coords(nx as u32, ny as u32);
        if let Some(items) = state.structures.get(&nloc) {
            if items.iter().any(|i| i.structure_type == StructureType::Road) {
                count += 1;
            }
        }
    }
    count
}

/// Remove the road structure from a location.  If the road was the only
/// structure at that location, remove the entry entirely.
fn remove_road(state: &mut PlacementState, loc: Location) {
    if let Some(items) = state.structures.get_mut(&loc) {
        items.retain(|i| i.structure_type != StructureType::Road);
        if items.is_empty() {
            state.structures.remove(&loc);
        }
    }
}
