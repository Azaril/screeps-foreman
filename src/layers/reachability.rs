//! ReachabilityLayer: Validates that all non-road structures are reachable from the hub.
//! Deterministic (1 candidate). Rejects the plan if any structure is unreachable.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;
use fnv::FnvHashSet;
use log::*;
use std::collections::VecDeque;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Validation-only layer that BFS flood-fills from the hub and verifies
/// every non-road structure has at least one adjacent walkable tile that
/// is reachable. Rejects the plan if any structure is unreachable.
pub struct ReachabilityLayer;

impl PlacementLayer for ReachabilityLayer {
    fn name(&self) -> &str {
        "reachability"
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

        // BFS flood-fill from hub across walkable tiles.
        // A tile is walkable if:
        // - Not a wall
        // - Not occupied by a non-road structure, OR has a road on it
        let mut reachable: FnvHashSet<Location> = FnvHashSet::default();
        let mut queue: VecDeque<Location> = VecDeque::new();

        reachable.insert(hub);
        queue.push_back(hub);

        while let Some(loc) = queue.pop_front() {
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

                if reachable.contains(&nloc) {
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
                    reachable.insert(nloc);
                    queue.push_back(nloc);
                }
            }
        }

        // Collect mineral-adjacent locations so we can skip them in the
        // reachability check. Mineral infrastructure (extractor + container)
        // may be placed in areas unreachable from the hub if the mineral is
        // walled off; this is acceptable since mineral extraction is optional.
        let mut mineral_area: FnvHashSet<Location> = FnvHashSet::default();
        for (mineral_loc, _, _) in &state.analysis.mineral_distances {
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
        // reachable tile (a creep can stand next to it to interact).
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

            // The structure itself might be reachable (e.g. roads under it),
            // but what matters is that a creep can stand adjacent to it.
            let x = loc.x();
            let y = loc.y();
            let mut serviceable = false;

            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = x as i16 + dx as i16;
                let ny = y as i16 + dy as i16;
                if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                    continue;
                }
                let nloc = Location::from_coords(nx as u32, ny as u32);
                if reachable.contains(&nloc) {
                    serviceable = true;
                    break;
                }
            }

            if !serviceable {
                trace!(
                    "Reachability: unreachable structure at ({}, {}), reachable_tiles={}",
                    x, y, reachable.len()
                );
                return Some(Err(()));
            }
        }

        // All structures are reachable -- pass through unchanged
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
