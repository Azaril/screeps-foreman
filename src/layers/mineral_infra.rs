//! MineralInfraLayer: Places extractor and mineral container.
//! Low branching -- deterministic placement.

use crate::constants::*;
use crate::layer::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Places extractor on the mineral and a container adjacent to it.
/// Deterministic (1 candidate).
pub struct MineralInfraLayer;

impl PlacementLayer for MineralInfraLayer {
    fn name(&self) -> &str {
        "mineral_infra"
    }

    fn is_applicable(&self, _state: &PlacementState, analysis: &AnalysisOutput) -> bool {
        !analysis.mineral_distances.is_empty()
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

        for (mineral_loc, _, _) in &analysis.mineral_distances {
            // Place extractor on the mineral
            new_state.place_structure(*mineral_loc, StructureType::Extractor, 6);
            new_state.set_landmark("extractor", *mineral_loc);

            // Place container adjacent to mineral
            for &(dx, dy) in &NEIGHBORS_8 {
                if let Some(nloc) = mineral_loc.checked_add(dx, dy) {
                    if !nloc.is_interior() {
                        continue;
                    }
                    if terrain.is_wall_at(nloc)
                        || new_state.has_any_structure(nloc)
                        || new_state.is_excluded(nloc)
                    {
                        continue;
                    }
                    new_state.place_structure(nloc, StructureType::Container, 6);
                    new_state.set_landmark("mineral_container", nloc);
                    break;
                }
            }
        }

        Some(Ok(new_state))
    }
}
