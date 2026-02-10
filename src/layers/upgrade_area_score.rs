//! UpgradeAreaScoreLayer: Scores the quality of the upgrade area.
//! Placed after ControllerInfraLayer.

use crate::layer::*;
use crate::terrain::*;

/// Scores the upgrade area quality based on the number of upgrade positions found.
/// Up to 4 positions is ideal.
pub struct UpgradeAreaScoreLayer;

impl PlacementLayer for UpgradeAreaScoreLayer {
    fn name(&self) -> &str {
        "upgrade_area_score"
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

        let upgrade_area = state.get_landmark_set("upgrade_area");
        let upgrade_quality = (upgrade_area.len() as f32 / 4.0).min(1.0);

        let mut new_state = state.clone();
        new_state.push_score("upgrade_area_quality", upgrade_quality, 0.5);
        Some(Ok(new_state))
    }
}
