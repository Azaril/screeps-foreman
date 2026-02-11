//! ControllerInfraLayer: Places controller container, link, and upgrade area.
//! Low branching -- deterministic placement.

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
        let mut candidates: Vec<(u8, u8, u32)> = Vec::new();
        for dy in -3..=3i8 {
            for dx in -3..=3i8 {
                let x = cx as i16 + dx as i16;
                let y = cy as i16 + dy as i16;
                if !(1..49).contains(&x) || !(1..49).contains(&y) {
                    continue;
                }
                let ux = x as u8;
                let uy = y as u8;
                let dist = dx.unsigned_abs().max(dy.unsigned_abs()) as u32;
                if dist > 3 || dist == 0 {
                    continue;
                }
                if terrain.is_wall(ux, uy)
                    || new_state.has_any_structure(ux, uy)
                    || new_state.is_excluded(ux, uy)
                {
                    continue;
                }
                let storage_dist = analysis
                    .source_distances
                    .first()
                    .and_then(|(_, dm, _)| *dm.get(ux as usize, uy as usize))
                    .unwrap_or(50);
                candidates.push((ux, uy, storage_dist));
            }
        }
        candidates.sort_by_key(|c| c.2);

        // Reject if no upgrade area tiles found
        if candidates.is_empty() {
            return Some(Err(()));
        }

        // Take up to 4 upgrade positions
        for (x, y, _) in candidates.iter().take(4) {
            new_state
                .add_to_landmark_set("upgrade_area", Location::from_coords(*x as u32, *y as u32));
        }

        // Place container in upgrade area
        let upgrade_area = new_state.get_landmark_set("upgrade_area").to_vec();
        let container_loc = match upgrade_area.first() {
            Some(&loc)
                if !new_state.has_any_structure(loc.x(), loc.y())
                    && !new_state.is_excluded(loc.x(), loc.y()) =>
            {
                loc
            }
            _ => return Some(Err(())),
        };

        new_state.place_structure(
            container_loc.x(),
            container_loc.y(),
            StructureType::Container,
            0,
        );
        new_state.set_landmark("controller_container", container_loc);

        // Place link adjacent to container
        let mut link_placed = false;
        for &(dx, dy) in &NEIGHBORS_8 {
            let lx = container_loc.x() as i16 + dx as i16;
            let ly = container_loc.y() as i16 + dy as i16;
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
            link_placed = true;
            break;
        }

        // Reject if no link could be placed
        if !link_placed {
            return Some(Err(()));
        }

        Some(Ok(new_state))
    }
}
