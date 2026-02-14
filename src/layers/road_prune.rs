//! RoadPruneLayer: Removes extraneous road tiles after the road network has
//! been generated.  Prunes unreachable roads, iteratively removes dead-end
//! road stubs that don't serve any building, and eliminates redundant parallel
//! roads that don't improve path length to any structure.
//!
//! Runs after RoadNetworkLayer and before ReachabilityLayer so the final road
//! layout is validated by the reachability check.

use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::plan::*;
use crate::terrain::*;
use log::*;
use std::cmp::Reverse;
use std::collections::VecDeque;

use screeps::constants::StructureType;

/// Prunes dead-end, unreachable, and redundant road tiles.
/// Deterministic (1 candidate).
pub struct RoadPruneLayer;

impl PlacementLayer for RoadPruneLayer {
    fn name(&self) -> &str {
        "road_prune"
    }

    fn candidate_count(
        &self,
        _state: &PlacementState,
        _analysis: &AnalysisOutput,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(1)
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        _analysis: &AnalysisOutput,
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
            .filter(|(_, items)| {
                items
                    .iter()
                    .any(|i| i.structure_type == StructureType::Road)
            })
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
                    items
                        .iter()
                        .any(|i| i.structure_type == StructureType::Road)
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

        // --- Step 4: Redundant path elimination --------------------------
        // Remove roads that are not needed for connectivity or path length.
        // Process farthest from hub first so outer redundant paths are
        // removed before inner ones.
        removed += prune_redundant_roads(hub, terrain, &mut new_state);

        debug!("RoadPruneLayer: removed {} extraneous road tiles", removed);

        Some(Ok(new_state))
    }
}

// ---------------------------------------------------------------------------
// Redundant path elimination
// ---------------------------------------------------------------------------

/// Remove road tiles whose removal does not:
///   1. Disconnect any remaining road tile from the hub, or
///   2. Increase the road-distance from hub to any interactable structure.
///
/// This preserves long-distance trunk roads even if they are not adjacent to
/// any structure, because removing a trunk segment would disconnect the roads
/// beyond it.
///
/// Roads are processed in order of decreasing distance from hub so that outer
/// redundant branches are removed before inner ones.
fn prune_redundant_roads(
    hub: Location,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
) -> u32 {
    let mut removed = 0u32;

    // Compute baseline road-only BFS distances from hub.
    let baseline_dist = road_bfs(hub, terrain, state);

    // Collect all interactable structures and their baseline best-adjacent-road
    // distances.  These are the structures we must not degrade access to.
    let structure_locs: Vec<Location> = state
        .structures
        .iter()
        .filter(|(_, items)| {
            items.iter().any(|i| {
                !matches!(
                    i.structure_type,
                    StructureType::Road | StructureType::Wall | StructureType::Rampart
                )
            })
        })
        .map(|(loc, _)| *loc)
        .collect();

    let baseline_structure_dists: Vec<(Location, u32)> = structure_locs
        .iter()
        .filter_map(|&loc| best_adjacent_road_dist(loc, &baseline_dist).map(|dist| (loc, dist)))
        .collect();

    // Total number of road tiles (reachable or not). Used to detect when
    // removing a road disconnects other roads: if the number of unreachable
    // roads increases, the removed road was a bridge.
    let total_roads = count_total_roads(state);
    let baseline_reachable = count_reachable_roads(&baseline_dist, state);
    let mut expected_unreachable = total_roads - baseline_reachable;

    // Collect candidate road tiles sorted by decreasing distance from hub.
    let mut road_candidates: Vec<(Location, u32)> = state
        .structures
        .iter()
        .filter(|(_, items)| {
            items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
        })
        .map(|(loc, _)| {
            let idx = loc.y() as usize * 50 + loc.x() as usize;
            (*loc, baseline_dist[idx])
        })
        .collect();

    // Sort farthest first.
    road_candidates.sort_by_key(|r| Reverse(r.1));

    for (road_loc, _) in road_candidates {
        // Skip roads adjacent to non-road structures (they directly serve
        // buildings and should not be removed).
        if is_adjacent_to_structure(state, road_loc) {
            continue;
        }

        // Save the road's original RCL before removing so we can restore it.
        let original_rcl = state.structures.get(&road_loc).and_then(|items| {
            items
                .iter()
                .find(|i| i.structure_type == StructureType::Road)
                .and_then(|i| i.required_rcl)
        });

        // Tentatively remove the road.
        remove_road(state, road_loc);

        // Recompute road BFS from hub.
        let new_dist = road_bfs(hub, terrain, state);

        // Safety check 1: removing this road must not increase the number
        // of unreachable roads. If it does, this road was a bridge connecting
        // other roads to the hub network.
        let new_total = count_total_roads(state);
        let new_reachable = count_reachable_roads(&new_dist, state);
        let new_unreachable = new_total - new_reachable;
        let ok_connectivity = new_unreachable <= expected_unreachable;

        // Safety check 2: no structure's best adjacent road distance should
        // increase (preserves path length).
        let ok_distance = baseline_structure_dists.iter().all(|&(sloc, baseline_d)| {
            let new_d = best_adjacent_road_dist(sloc, &new_dist).unwrap_or(u32::MAX);
            new_d <= baseline_d
        });

        if ok_connectivity && ok_distance {
            // Road is truly redundant -- keep it removed.
            removed += 1;
            // The removed road is gone from total; update expected_unreachable
            // for the next iteration.
            expected_unreachable = new_unreachable;
        } else {
            // Removal would break connectivity or degrade path length.
            // Restore the road with its original RCL.
            let restored = match original_rcl {
                Some(rcl) => RoomItem::new(StructureType::Road, rcl),
                None => RoomItem::new_auto_rcl(StructureType::Road),
            };
            state.structures.entry(road_loc).or_default().push(restored);
        }
    }

    removed
}

// ---------------------------------------------------------------------------
// Road BFS and counting helpers
// ---------------------------------------------------------------------------

/// BFS over road tiles only, returning distances from hub.
/// Non-road tiles are impassable (distance = u32::MAX).
/// The hub itself is distance 0 even if it's not a road.
fn road_bfs(hub: Location, _terrain: &FastRoomTerrain, state: &PlacementState) -> Vec<u32> {
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
            let loc = Location::from_coords(ux as u32, uy as u32);
            // Only traverse road tiles (and the hub seed).
            let is_road = state
                .structures
                .get(&loc)
                .map(|items| {
                    items
                        .iter()
                        .any(|i| i.structure_type == StructureType::Road)
                })
                .unwrap_or(false);
            if !is_road {
                continue;
            }
            dist[idx] = d + 1;
            queue.push_back((ux, uy, d + 1));
        }
    }

    dist
}

/// Count road tiles that are reachable from the hub (distance != u32::MAX).
fn count_reachable_roads(road_dist: &[u32], state: &PlacementState) -> u32 {
    let mut count = 0u32;
    for (loc, items) in &state.structures {
        if items
            .iter()
            .any(|i| i.structure_type == StructureType::Road)
        {
            let idx = loc.y() as usize * 50 + loc.x() as usize;
            if road_dist[idx] != u32::MAX {
                count += 1;
            }
        }
    }
    count
}

/// Count total road tiles in the state (reachable or not).
fn count_total_roads(state: &PlacementState) -> u32 {
    let mut count = 0u32;
    for items in state.structures.values() {
        if items
            .iter()
            .any(|i| i.structure_type == StructureType::Road)
        {
            count += 1;
        }
    }
    count
}

/// Return the minimum road-BFS distance among all adjacent road tiles of a
/// given location.  Returns None if no adjacent road tile is reachable.
fn best_adjacent_road_dist(loc: Location, road_dist: &[u32]) -> Option<u32> {
    let x = loc.x();
    let y = loc.y();
    let mut best = None;
    for &(dx, dy) in &NEIGHBORS_8 {
        let nx = x as i16 + dx as i16;
        let ny = y as i16 + dy as i16;
        if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
            continue;
        }
        let idx = ny as usize * 50 + nx as usize;
        let d = road_dist[idx];
        if d != u32::MAX {
            best = Some(best.map_or(d, |b: u32| b.min(d)));
        }
    }
    best
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
        .map(|items| {
            items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
        })
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
            if items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
            {
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
