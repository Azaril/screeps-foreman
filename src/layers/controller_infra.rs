//! ControllerInfraLayer: Places controller container, link, and upgrade area.
//! Low branching -- deterministic placement.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Places controller container, link, and identifies upgrade area.
/// Deterministic (1 candidate).
pub struct ControllerInfraLayer;

impl PlacementLayer for ControllerInfraLayer {
    fn name(&self) -> &str {
        "controller_infra"
    }

    fn is_applicable(&self, _state: &PlacementState, analysis: &AnalysisOutput) -> bool {
        !analysis.controller_distances.is_empty()
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

        let mut new_state = state.clone();

        let (ctrl_loc, _, _) = match analysis.controller_distances.first() {
            Some(entry) => entry,
            None => return Some(Err(())),
        };

        let cx = ctrl_loc.x() as i8;
        let cy = ctrl_loc.y() as i8;

        // Find upgrade area tiles (within range 3 of controller, passable, not occupied)
        let mut candidates: Vec<(Location, u32)> = Vec::new();
        for dy in -3..=3i8 {
            for dx in -3..=3i8 {
                let x = cx as i16 + dx as i16;
                let y = cy as i16 + dy as i16;
                if !xy_is_interior(x, y) {
                    continue;
                }
                let loc = Location::from_xy(x as u8, y as u8);
                let dist = dx.unsigned_abs().max(dy.unsigned_abs()) as u32;
                if dist > 3 || dist == 0 {
                    continue;
                }
                if terrain.is_wall_at(loc)
                    || new_state.has_any_structure(loc)
                    || new_state.is_excluded(loc)
                {
                    continue;
                }
                let storage_dist = analysis
                    .source_distances
                    .first()
                    .and_then(|(_, dm, _)| *dm.get(loc.x() as usize, loc.y() as usize))
                    .unwrap_or(50);
                candidates.push((loc, storage_dist));
            }
        }
        candidates.sort_by_key(|c| c.1);

        // Reject if no upgrade area tiles found
        if candidates.is_empty() {
            return Some(Err(()));
        }

        // Take up to 4 upgrade positions
        for (loc, _) in candidates.iter().take(4) {
            new_state.add_to_landmark_set("upgrade_area", *loc);
        }

        // Place container in upgrade area
        let upgrade_area = new_state.get_landmark_set("upgrade_area").to_vec();
        let container_loc = match upgrade_area.first() {
            Some(&loc) if !new_state.has_any_structure(loc) && !new_state.is_excluded(loc) => loc,
            _ => return Some(Err(())),
        };

        new_state.place_structure(container_loc, StructureType::Container, 0);
        new_state.set_landmark("controller_container", container_loc);

        // Place link adjacent to container
        let mut link_placed = false;
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
                new_state.place_structure(nloc, StructureType::Link, 0);
                link_placed = true;
                break;
            }
        }

        // Reject if no link could be placed
        if !link_placed {
            return Some(Err(()));
        }

        Some(Ok(new_state))
    }
}
