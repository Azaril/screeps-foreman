//! ReachabilityLayer: Validates that all non-road structures are reachable from the hub.
//! Deterministic (1 candidate). Rejects the plan if any structure is unreachable
//! or requires an unreasonably long path (detour detection).
//!
//! Additionally validates that every walkable tile adjacent to a spawn is reachable
//! from the hub. This prevents newly spawned creeps from being placed in dead-end
//! pockets that are disconnected from the rest of the base.

use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;
use fnv::{FnvHashMap, FnvHashSet};
use log::*;
use std::collections::VecDeque;

use screeps::constants::StructureType;

/// Maximum allowed ratio of path distance to Chebyshev distance before a
/// structure is considered to have an unreasonable detour.
/// path_distance <= chebyshev_distance * DETOUR_RATIO + DETOUR_THRESHOLD
const DETOUR_RATIO: u32 = 2;

/// Additive threshold for detour detection (tiles).
const DETOUR_THRESHOLD: u32 = 8;

/// Validation-only layer that BFS flood-fills from the hub and verifies
/// every non-road structure has at least one adjacent walkable tile that
/// is reachable within a reasonable path distance. Rejects the plan if
/// any structure is unreachable or requires an unreasonably long detour.
///
/// For spawns specifically, **every** walkable adjacent tile must be
/// hub-reachable. The game can place a newly spawned creep on any open
/// adjacent tile; if even one such tile is a dead-end pocket disconnected
/// from the hub, the creep will be stuck.
pub struct ReachabilityLayer;

impl PlacementLayer for ReachabilityLayer {
    fn name(&self) -> &str {
        "reachability"
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
        analysis: &AnalysisOutput,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index > 0 {
            return None;
        }

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        // BFS flood-fill from hub across walkable tiles, tracking distances.
        // A tile is walkable if:
        // - Not a wall
        // - Not occupied by a non-road structure, OR has a road on it
        let mut reachable: FnvHashMap<Location, u32> = FnvHashMap::default();
        let mut queue: VecDeque<(Location, u32)> = VecDeque::new();

        reachable.insert(hub, 0);
        queue.push_back((hub, 0));

        while let Some((loc, dist)) = queue.pop_front() {
            let x = loc.x();
            let y = loc.y();

            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = x as i16 + dx as i16;
                let ny = y as i16 + dy as i16;
                if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                    continue;
                }
                let ux = nx as u8;
                let uy = ny as u8;
                let nloc = Location::from_coords(ux as u32, uy as u32);

                if reachable.contains_key(&nloc) {
                    continue;
                }
                if terrain.is_wall(ux, uy) {
                    continue;
                }

                // A tile is walkable if it's not occupied by a non-road structure.
                // Tiles with roads (even if also occupied) are still walkable for
                // pathfinding purposes -- creeps can walk on roads.
                let is_walkable = !state.occupied.contains(&nloc) || has_road(state, nloc);

                if is_walkable {
                    let next_dist = dist + 1;
                    reachable.insert(nloc, next_dist);
                    queue.push_back((nloc, next_dist));
                }
            }
        }

        // Collect mineral-adjacent locations so we can skip them in the
        // reachability check. Mineral infrastructure (extractor + container)
        // may be placed in areas unreachable from the hub if the mineral is
        // walled off; this is acceptable since mineral extraction is optional.
        let mut mineral_area: FnvHashSet<Location> = FnvHashSet::default();
        for (mineral_loc, _, _) in &analysis.mineral_distances {
            mineral_area.insert(*mineral_loc);
            let mx = mineral_loc.x();
            let my = mineral_loc.y();
            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = mx as i16 + dx as i16;
                let ny = my as i16 + dy as i16;
                if (0..50).contains(&nx) && (0..50).contains(&ny) {
                    mineral_area.insert(Location::from_coords(nx as u32, ny as u32));
                }
            }
        }

        // Check that every non-road structure has at least one adjacent
        // reachable tile (a creep can stand next to it to interact) and
        // that the path distance is not unreasonably long.
        // Skip structures in the mineral area (optional infrastructure).
        for (loc, items) in &state.structures {
            let has_non_road = items
                .iter()
                .any(|i| i.structure_type != StructureType::Road);
            if !has_non_road {
                continue;
            }

            // Skip mineral-area structures
            if mineral_area.contains(loc) {
                continue;
            }

            // Find the nearest adjacent reachable tile and its distance
            let x = loc.x();
            let y = loc.y();
            let mut best_path_dist: Option<u32> = None;

            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = x as i16 + dx as i16;
                let ny = y as i16 + dy as i16;
                if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                    continue;
                }
                let nloc = Location::from_coords(nx as u32, ny as u32);
                if let Some(&dist) = reachable.get(&nloc) {
                    best_path_dist = Some(match best_path_dist {
                        Some(current) => current.min(dist),
                        None => dist,
                    });
                }
            }

            match best_path_dist {
                None => {
                    // No adjacent reachable tile -- structure is unreachable
                    trace!(
                        "Reachability: unreachable structure at ({}, {}), reachable_tiles={}",
                        x,
                        y,
                        reachable.len()
                    );
                    return Some(Err(()));
                }
                Some(path_dist) => {
                    // Check for unreasonable detours
                    let chebyshev = hub.distance_to(*loc) as u32;
                    let max_allowed = chebyshev * DETOUR_RATIO + DETOUR_THRESHOLD;
                    if path_dist > max_allowed {
                        trace!(
                            "Reachability: excessive detour to ({}, {}): path_dist={}, chebyshev={}, max_allowed={}",
                            x, y, path_dist, chebyshev, max_allowed
                        );
                        return Some(Err(()));
                    }
                }
            }
        }

        // Spawn-specific check: every walkable tile adjacent to a spawn must
        // be reachable from the hub. The game can place a newly spawned creep
        // on any open adjacent tile, so unreachable pockets next to spawns
        // would trap creeps.
        for (loc, items) in &state.structures {
            let is_spawn = items
                .iter()
                .any(|i| i.structure_type == StructureType::Spawn);
            if !is_spawn {
                continue;
            }

            let x = loc.x();
            let y = loc.y();

            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = x as i16 + dx as i16;
                let ny = y as i16 + dy as i16;
                if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                    continue;
                }
                let ux = nx as u8;
                let uy = ny as u8;
                let nloc = Location::from_coords(ux as u32, uy as u32);

                // Skip walls -- creeps cannot spawn onto walls
                if terrain.is_wall(ux, uy) {
                    continue;
                }

                // Skip tiles occupied by non-road structures (not walkable)
                if state.occupied.contains(&nloc) && !has_road(state, nloc) {
                    continue;
                }

                // This tile is walkable and adjacent to a spawn -- it must be
                // reachable from the hub, otherwise a creep could spawn here
                // and be stuck.
                if !reachable.contains_key(&nloc) {
                    trace!(
                        "Reachability: spawn at ({}, {}) has unreachable adjacent walkable tile ({}, {})",
                        x, y, ux, uy
                    );
                    return Some(Err(()));
                }
            }
        }

        // All structures are reachable within reasonable distances
        Some(Ok(state.clone()))
    }
}

/// Check if a location has a road placed on it.
fn has_road(state: &PlacementState, loc: Location) -> bool {
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
