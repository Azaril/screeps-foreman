//! UpgradeAreaScoreLayer: Scores the quality of the upgrade area.
//! Placed after ControllerInfraLayer.

use crate::layer::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

/// Scores the upgrade area on work-slot count and link-era overlap so
/// the search prefers hub/layout branches whose controller area parks
/// more zero-movement upgraders (ADR 0009 D2).
///
/// `slots / 8` because 8 is the open-terrain maximum (9 coverage tiles
/// minus the reserved hauler approach); `overlap / 4` because ~4
/// link-covered slots saturate realistic link throughput pre-RCL8.
pub struct UpgradeAreaScoreLayer;

impl PlacementLayer for UpgradeAreaScoreLayer {
    fn name(&self) -> &str {
        "upgrade_area_score"
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
        _analysis: &AnalysisOutput,
        _terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index > 0 {
            return None;
        }

        let slots = state.get_landmark_set("upgrade_area");
        let slot_quality = (slots.len() as f32 / 8.0).min(1.0);
        let overlap = state
            .get_landmark("controller_link")
            .map(|link| slots.iter().filter(|&&s| s.distance_to(link) <= 1).count())
            .unwrap_or(0);
        let overlap_quality = (overlap as f32 / 4.0).min(1.0);
        let upgrade_quality = 0.7 * slot_quality + 0.3 * overlap_quality;

        let mut new_state = state.clone();
        new_state.push_score("upgrade_area_quality", upgrade_quality, 0.5);
        Some(Ok(new_state))
    }
}
