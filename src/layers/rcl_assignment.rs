//! RclAssignmentLayer: Post-processing layer that assigns RCL values to all structures.
//! Deterministic (1 candidate). Runs after all structures are placed.
//!
//! Uses flood-fill distances from the hub (accounting for terrain walls) to
//! determine build priority ordering rather than Chebyshev distance.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
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
        // Structures that already have an explicit RCL are left unchanged.
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
                            item.required_rcl = Some(rcl);
                        }
                    }
                }
            }
        }

        // Assign RCL to ramparts/walls: set to RCL 2 (earliest available)
        // unless the layer already pinned a specific RCL.
        for (loc, items) in &mut new_state.structures {
            for item in items.iter_mut() {
                if item.structure_type == StructureType::Rampart
                    || item.structure_type == StructureType::Wall
                {
                    if item.required_rcl.is_none() {
                        item.required_rcl = Some(2);
                    }
                    let _ = loc; // suppress unused warning
                }
            }
        }

        // Assign RCL to roads.
        //
        // Roads without an explicit RCL (None) inherit the minimum RCL of
        // adjacent non-road structures so they are available as soon as the
        // earliest adjacent structure needs them. If no adjacent structures
        // exist, they default to RCL 1.
        //
        // Roads that already have an explicit RCL (set by the layer that
        // placed them, e.g. RoadNetworkLayer with a pinned RCL) keep the
        // *minimum* of their current value and the adjacent-structure value.
        // This ensures that if a road serves both a high-RCL building and a
        // low-RCL road-network route, it is built at the earlier RCL.
        let road_locs: Vec<Location> = new_state
            .structures
            .iter()
            .filter(|(_, items)| {
                items
                    .iter()
                    .any(|i| i.structure_type == StructureType::Road)
                    && !items
                        .iter()
                        .any(|i| i.structure_type != StructureType::Road)
            })
            .map(|(loc, _)| *loc)
            .collect();

        for road_loc in &road_locs {
            let mut min_adjacent_rcl: Option<u8> = None;

            // Check all 8 neighbors for non-road structures
            for &(dx, dy) in &NEIGHBORS_8 {
                let nloc = match road_loc.checked_add(dx, dy) {
                    Some(l) => l,
                    None => continue,
                };
                if let Some(items) = new_state.structures.get(&nloc) {
                    for item in items {
                        if item.structure_type != StructureType::Road {
                            let adj_rcl = item.required_rcl();
                            min_adjacent_rcl =
                                Some(min_adjacent_rcl.map_or(adj_rcl, |cur| cur.min(adj_rcl)));
                        }
                    }
                }
            }

            // Update road RCL
            if let Some(items) = new_state.structures.get_mut(road_loc) {
                for item in items.iter_mut() {
                    if item.structure_type == StructureType::Road {
                        match (item.required_rcl, min_adjacent_rcl) {
                            // Road has no explicit RCL and no adjacent structures: default to 1
                            (None, None) => {
                                item.required_rcl = Some(1);
                            }
                            // Road has no explicit RCL but has adjacent structures: use adjacent
                            (None, Some(adj)) => {
                                item.required_rcl = Some(adj);
                            }
                            // Road has explicit RCL but no adjacent structures: keep explicit
                            (Some(_), None) => {}
                            // Road has explicit RCL and adjacent structures: use minimum
                            (Some(existing), Some(adj)) => {
                                item.required_rcl = Some(existing.min(adj));
                            }
                        }
                    }
                }
            }
        }

        // Assign RCL to containers: default to RCL 1 (available from the start)
        // unless the layer already pinned a specific RCL.
        for items in new_state.structures.values_mut() {
            for item in items.iter_mut() {
                if item.structure_type == StructureType::Container && item.required_rcl.is_none() {
                    item.required_rcl = Some(1);
                }
            }
        }

        Some(Ok(new_state))
    }
}
