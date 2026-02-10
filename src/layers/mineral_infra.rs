//! MineralInfraLayer: Places extractor and mineral container.
//! Low branching -- deterministic placement.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Places extractor on the mineral and a container adjacent to it.
/// Deterministic (1 candidate).
pub struct MineralInfraLayer;

impl PlacementLayer for MineralInfraLayer {
    fn name(&self) -> &str {
        "mineral_infra"
    }

    fn is_applicable(&self, state: &PlacementState) -> bool {
        !state.analysis.mineral_distances.is_empty()
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

        let mut new_state = state.clone();

        for (mineral_loc, _, _) in &state.analysis.mineral_distances {
            let mx = mineral_loc.x();
            let my = mineral_loc.y();

            // Place extractor on the mineral
            new_state.place_structure(mx, my, StructureType::Extractor, 6);
            new_state.set_landmark("extractor", *mineral_loc);

            // Place container adjacent to mineral
            for &(dx, dy) in &NEIGHBORS_8 {
                let x = mx as i16 + dx as i16;
                let y = my as i16 + dy as i16;
                if !(1..49).contains(&x) || !(1..49).contains(&y) {
                    continue;
                }
                let ux = x as u8;
                let uy = y as u8;
                if terrain.is_wall(ux, uy) || new_state.has_any_structure(ux, uy) {
                    continue;
                }
                new_state.place_structure(ux, uy, StructureType::Container, 6);
                new_state.set_landmark(
                    "mineral_container",
                    Location::from_coords(ux as u32, uy as u32),
                );
                break;
            }
        }

        Some(Ok(new_state))
    }
}
