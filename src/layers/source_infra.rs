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
            let sx = source_loc.x();
            let sy = source_loc.y();

            // Collect candidate container positions adjacent to source
            let mut candidates: Vec<(u8, u8, u8, u32)> = Vec::new(); // (x, y, harvest_positions, hub_dist)

            for &(dx, dy) in &NEIGHBORS_8 {
                let x = sx as i16 + dx as i16;
                let y = sy as i16 + dy as i16;
                if !(0..50).contains(&x) || !(0..50).contains(&y) {
                    continue;
                }
                let ux = x as u8;
                let uy = y as u8;
                if terrain.is_wall(ux, uy)
                    || new_state.has_any_structure(ux, uy)
                    || new_state.is_excluded(ux, uy)
                {
                    continue;
                }

                // Count harvest positions: tiles adjacent to BOTH the source
                // and this container candidate where a creep could stand.
                let harvest_positions =
                    count_harvest_positions(sx, sy, ux, uy, terrain, &new_state);

                // Use flood-fill distance to hub if available, fall back to Chebyshev
                let hub_dist = hub_dists
                    .as_ref()
                    .and_then(|d| *d.get(ux as usize, uy as usize))
                    .unwrap_or_else(|| {
                        let hub = state.get_landmark("hub").unwrap();
                        hub.distance_to(Location::from_coords(ux as u32, uy as u32)) as u32
                    });

                candidates.push((ux, uy, harvest_positions, hub_dist));
            }

            // Sort by: most harvest positions first, then closest to hub
            candidates.sort_by(|a, b| {
                b.2.cmp(&a.2) // more harvest positions first
                    .then_with(|| a.3.cmp(&b.3)) // then closer to hub
            });

            if let Some(&(cx, cy, _, _)) = candidates.first() {
                new_state.place_structure(cx, cy, StructureType::Container, 0);
                let container_loc = Location::from_coords(cx as u32, cy as u32);
                new_state.add_to_landmark_set("source_containers", container_loc);
                containers_placed += 1;

                // Place link adjacent to container, preferring position closest to hub
                let mut link_candidates: Vec<(u8, u8, u32)> = Vec::new();

                for &(dx, dy) in &NEIGHBORS_8 {
                    let lx = cx as i16 + dx as i16;
                    let ly = cy as i16 + dy as i16;
                    if !(1..49).contains(&lx) || !(1..49).contains(&ly) {
                        continue;
                    }
                    let ux = lx as u8;
                    let uy = ly as u8;
                    if terrain.is_wall(ux, uy)
                        || new_state.has_any_structure(ux, uy)
                        || new_state.is_excluded(ux, uy)
                    {
                        continue;
                    }

                    let hub_dist = hub_dists
                        .as_ref()
                        .and_then(|d| *d.get(ux as usize, uy as usize))
                        .unwrap_or(u32::MAX);

                    link_candidates.push((ux, uy, hub_dist));
                }

                // Sort by distance to hub (closest first)
                link_candidates.sort_by_key(|c| c.2);

                if let Some(&(lx, ly, _)) = link_candidates.first() {
                    new_state.place_structure(lx, ly, StructureType::Link, 0);
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
    source_x: u8,
    source_y: u8,
    container_x: u8,
    container_y: u8,
    terrain: &FastRoomTerrain,
    state: &PlacementState,
) -> u8 {
    let mut count = 0u8;
    for &(dx, dy) in &NEIGHBORS_8 {
        let nx = source_x as i16 + dx as i16;
        let ny = source_y as i16 + dy as i16;
        if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
            continue;
        }
        let ux = nx as u8;
        let uy = ny as u8;

        // Must be walkable and not occupied
        if terrain.is_wall(ux, uy) || state.is_occupied(ux, uy) {
            continue;
        }

        // Must also be adjacent to the container (Chebyshev distance 1)
        let cdx = (ux as i16 - container_x as i16).abs();
        let cdy = (uy as i16 - container_y as i16).abs();
        if cdx <= 1 && cdy <= 1 && !(cdx == 0 && cdy == 0) {
            count += 1;
        }
    }
    count
}
