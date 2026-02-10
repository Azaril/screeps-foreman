//! RclAssignmentLayer: Post-processing layer that assigns RCL values to all structures.
//! Deterministic (1 candidate). Runs after all structures are placed.
//!
//! Uses flood-fill distances from the hub (accounting for terrain walls) to
//! determine build priority ordering rather than Chebyshev distance.

use crate::constants::min_rcl_for_nth;
use crate::layer::*;
use crate::location::*;
use crate::terrain::*;
use fnv::FnvHashMap;

use screeps::constants::StructureType;

/// Assigns RCL values to all structures based on the Screeps API limits
/// and strategic priority (flood-fill distance to hub).
/// Deterministic (1 candidate).
pub struct RclAssignmentLayer;

impl PlacementLayer for RclAssignmentLayer {
    fn name(&self) -> &str {
        "rcl_assignment"
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

        // Compute hub flood-fill distances for accurate ordering
        let hub_dists = new_state.hub_distances(terrain).cloned();

        // Collect all non-road, non-container structures grouped by type.
        // Each entry is (location, structure_type, distance_to_hub).
        let mut by_type: FnvHashMap<StructureType, Vec<(Location, u32)>> = FnvHashMap::default();

        for (loc, items) in &state.structures {
            for item in items {
                match item.structure_type {
                    // Roads and containers have no per-RCL limits
                    StructureType::Road | StructureType::Container => continue,
                    // Ramparts and walls are handled separately
                    StructureType::Rampart | StructureType::Wall => continue,
                    st => {
                        // Use flood-fill distance if available, fall back to Chebyshev
                        let dist = hub_dists
                            .as_ref()
                            .and_then(|d| *d.get(loc.x() as usize, loc.y() as usize))
                            .unwrap_or_else(|| hub.distance_to(*loc) as u32);
                        by_type.entry(st).or_default().push((*loc, dist));
                    }
                }
            }
        }

        // For each structure type, sort by strategic priority and assign RCL.
        // Priority: closer to hub (by flood-fill) = lower RCL (built earlier).
        for (st, entries) in &mut by_type {
            // Sort by distance to hub (closest first)
            entries.sort_by_key(|e| e.1);

            for (i, (loc, _)) in entries.iter().enumerate() {
                let nth = (i + 1) as u32;
                let rcl = min_rcl_for_nth(*st, nth);

                // Update the structure's RCL in the new state
                if let Some(items) = new_state.structures.get_mut(loc) {
                    for item in items.iter_mut() {
                        if item.structure_type == *st {
                            item.required_rcl = rcl;
                        }
                    }
                }
            }
        }

        // Assign RCL to ramparts/walls: all get RCL 2 (earliest available)
        for (loc, items) in &mut new_state.structures {
            for item in items.iter_mut() {
                if item.structure_type == StructureType::Rampart
                    || item.structure_type == StructureType::Wall
                {
                    item.required_rcl = 2;
                    let _ = loc; // suppress unused warning
                }
            }
        }

        // Assign RCL to roads: inherit the max RCL of adjacent non-road structures.
        // Roads that serve high-RCL structures should be built at that RCL.
        let road_locs: Vec<Location> = new_state
            .structures
            .iter()
            .filter(|(_, items)| {
                items.iter().any(|i| i.structure_type == StructureType::Road)
                    && !items
                        .iter()
                        .any(|i| i.structure_type != StructureType::Road)
            })
            .map(|(loc, _)| *loc)
            .collect();

        for road_loc in &road_locs {
            let mut max_adjacent_rcl = 1u8; // Roads default to RCL 1

            // Check all 8 neighbors for non-road structures
            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = road_loc.x() as i16 + dx as i16;
                let ny = road_loc.y() as i16 + dy as i16;
                if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                    continue;
                }
                let nloc = Location::from_coords(nx as u32, ny as u32);
                if let Some(items) = new_state.structures.get(&nloc) {
                    for item in items {
                        if item.structure_type != StructureType::Road
                            && item.required_rcl > max_adjacent_rcl
                        {
                            max_adjacent_rcl = item.required_rcl;
                        }
                    }
                }
            }

            // Update road RCL
            if let Some(items) = new_state.structures.get_mut(road_loc) {
                for item in items.iter_mut() {
                    if item.structure_type == StructureType::Road {
                        item.required_rcl = max_adjacent_rcl;
                    }
                }
            }
        }

        // Assign RCL to containers: RCL 1 (available from the start)
        for items in new_state.structures.values_mut() {
            for item in items.iter_mut() {
                if item.structure_type == StructureType::Container {
                    item.required_rcl = 1;
                }
            }
        }

        Some(Ok(new_state))
    }
}
