//! SourceInfraLayer: Places source containers and links.
//!
//! Container placement is optimized for harvest throughput: the primary sort
//! key is the number of "harvest positions" -- tiles adjacent to both the
//! source and the container where a creep can simultaneously harvest and
//! transfer. Ties are broken by flood-fill distance to hub for shorter haul
//! routes.
//!
//! Link placement is optimized to minimize distance to hub for faster energy
//! transfer.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Places source containers and links near each source.
/// Deterministic (1 candidate) -- picks the best container position per source.
pub struct SourceInfraLayer;

impl PlacementLayer for SourceInfraLayer {
    fn name(&self) -> &str {
        "source_infra"
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

        let _hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let mut new_state = state.clone();
        let source_count = analysis.source_distances.len();
        let mut containers_placed = 0usize;

        // Pre-compute hub flood-fill distances for accurate pathing distance
        let hub_dists = new_state.hub_distances(terrain).cloned();

        // Place containers and links for each source
        for (source_loc, _, _) in &analysis.source_distances {
            // Collect candidate container positions adjacent to source
            let mut candidates: Vec<(Location, u8, u32)> = Vec::new(); // (loc, harvest_positions, hub_dist)

            for &(dx, dy) in &NEIGHBORS_8 {
                if let Some(nloc) = source_loc.checked_add(dx, dy) {
                    if terrain.is_wall_at(nloc)
                        || new_state.has_any_structure(nloc)
                        || new_state.is_excluded(nloc)
                    {
                        continue;
                    }

                    // Count harvest positions: tiles adjacent to BOTH the source
                    // and this container candidate where a creep could stand.
                    let harvest_positions =
                        count_harvest_positions(*source_loc, nloc, terrain, &new_state);

                    // Use flood-fill distance to hub if available, fall back to Chebyshev
                    let hub_dist = hub_dists
                        .as_ref()
                        .and_then(|d| *d.get(nloc.x() as usize, nloc.y() as usize))
                        .unwrap_or_else(|| {
                            let hub = state.get_landmark("hub").unwrap();
                            hub.distance_to(nloc) as u32
                        });

                    candidates.push((nloc, harvest_positions, hub_dist));
                }
            }

            // Sort by: most harvest positions first, then closest to hub
            candidates.sort_by(|a, b| {
                b.1.cmp(&a.1) // more harvest positions first
                    .then_with(|| a.2.cmp(&b.2)) // then closer to hub
            });

            if let Some(&(container_loc, _, _)) = candidates.first() {
                new_state.place_structure(container_loc, StructureType::Container, 0);
                new_state.add_to_landmark_set("source_containers", container_loc);
                containers_placed += 1;

                // Place link adjacent to container, preferring position closest to hub
                let mut link_candidates: Vec<(Location, u32)> = Vec::new();

                for &(dx, dy) in &NEIGHBORS_8 {
                    if let Some(nloc) = container_loc.checked_add(dx, dy) {
                        if !nloc.is_interior() {
                            continue;
                        }
                        if terrain.is_wall_at(nloc)
                            || new_state.has_any_structure(nloc)
                            || new_state.is_excluded(nloc)
                        {
                            continue;
                        }

                        let hub_dist = hub_dists
                            .as_ref()
                            .and_then(|d| *d.get(nloc.x() as usize, nloc.y() as usize))
                            .unwrap_or(u32::MAX);

                        link_candidates.push((nloc, hub_dist));
                    }
                }

                // Sort by distance to hub (closest first)
                link_candidates.sort_by_key(|c| c.1);

                if let Some(&(link_loc, _)) = link_candidates.first() {
                    new_state.place_structure(link_loc, StructureType::Link, 0);
                }
            }
        }

        // Reject if not all sources got containers
        if containers_placed < source_count {
            return Some(Err(()));
        }

        Some(Ok(new_state))
    }
}

/// Count the number of tiles that are adjacent to both the source and the
/// container candidate, where a creep could stand (not wall, not occupied).
/// These are "harvest positions" -- a creep standing there can simultaneously
/// harvest the source and transfer to the container.
fn count_harvest_positions(
    source: Location,
    container: Location,
    terrain: &FastRoomTerrain,
    state: &PlacementState,
) -> u8 {
    let mut count = 0u8;
    for &(dx, dy) in &NEIGHBORS_8 {
        if let Some(nloc) = source.checked_add(dx, dy) {
            // Must be walkable and not occupied
            if terrain.is_wall_at(nloc) || state.is_occupied(nloc) {
                continue;
            }

            // Must also be adjacent to the container (Chebyshev distance 1)
            let dist = nloc.distance_to(container);
            if dist == 1 {
                count += 1;
            }
        }
    }
    count
}
