//! UpkeepScoreLayer: Scores the upkeep cost of the plan.
//! Placed after DefenseLayer.

use crate::layer::*;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Scores the plan's upkeep cost. Lower upkeep = higher score.
/// Considers rampart count, container count, and swamp road count.
pub struct UpkeepScoreLayer;

impl PlacementLayer for UpkeepScoreLayer {
    fn name(&self) -> &str {
        "upkeep_score"
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

        let ramparts = state.get_landmark_set("ramparts");
        let rampart_count = ramparts.len() as f32;

        let container_count = state
            .structures
            .values()
            .flat_map(|items| items.iter())
            .filter(|i| i.structure_type == StructureType::Container)
            .count() as f32;

        let swamp_road_count = state
            .structures
            .iter()
            .filter(|(loc, items)| {
                items
                    .iter()
                    .any(|i| i.structure_type == StructureType::Road)
                    && terrain.is_swamp(loc.x(), loc.y())
            })
            .count() as f32;

        let max_upkeep = 200.0;
        let upkeep_score = (1.0
            - (rampart_count + container_count * 2.0 + swamp_road_count * 3.0) / max_upkeep)
            .max(0.0);

        let mut new_state = state.clone();
        new_state.push_score("upkeep_cost", upkeep_score, 0.5);
        Some(Ok(new_state))
    }
}
