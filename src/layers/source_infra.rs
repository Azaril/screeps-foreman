//! SourceInfraLayer: Places source containers and links.
//! Low-medium branching -- container position choices per source.

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

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let mut new_state = state.clone();
        let source_count = analysis.source_distances.len();
        let mut containers_placed = 0usize;

        // Place containers and links for each source
        for (source_loc, _, _) in &analysis.source_distances {
            let sx = source_loc.x();
            let sy = source_loc.y();

            // Find the adjacent tile closest to hub for the container
            let mut best = None;
            let mut best_dist = u32::MAX;

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
                let dist_to_hub =
                    hub.distance_to(Location::from_coords(ux as u32, uy as u32)) as u32;
                if dist_to_hub < best_dist {
                    best_dist = dist_to_hub;
                    best = Some((ux, uy));
                }
            }

            if let Some((cx, cy)) = best {
                new_state.place_structure(cx, cy, StructureType::Container, 0);
                let container_loc = Location::from_coords(cx as u32, cy as u32);
                new_state.add_to_landmark_set("source_containers", container_loc);
                containers_placed += 1;

                // Place link adjacent to container
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
                    new_state.place_structure(ux, uy, StructureType::Link, 0);
                    break;
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
