//! TowerCoverageScoreLayer: Scores minimum tower damage across rampart perimeter.
//! Placed after DefenseLayer.

use crate::layer::*;
use crate::stamps::tower::tower_damage_at_range;
use crate::terrain::*;

/// Scores the minimum total tower damage at any rampart position.
/// Higher damage = better tower coverage = higher score.
pub struct TowerCoverageScoreLayer;

impl PlacementLayer for TowerCoverageScoreLayer {
    fn name(&self) -> &str {
        "tower_coverage_score"
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
        _terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index > 0 {
            return None;
        }

        let ramparts = state.get_landmark_set("ramparts");
        let towers = state.get_landmark_set("towers");

        let tower_coverage = if !ramparts.is_empty() && !towers.is_empty() {
            let mut min_total_damage = u32::MAX;
            for rampart in ramparts {
                let mut total_damage = 0u32;
                for tower in towers {
                    let range = rampart.distance_to(*tower) as u32;
                    total_damage += tower_damage_at_range(range);
                }
                if total_damage < min_total_damage {
                    min_total_damage = total_damage;
                }
            }
            (min_total_damage as f32 / 3600.0).min(1.0)
        } else {
            0.0
        };

        let mut new_state = state.clone();
        new_state.push_score("tower_coverage", tower_coverage, 1.0);
        Some(Ok(new_state))
    }
}
