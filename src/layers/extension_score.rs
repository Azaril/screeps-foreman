//! ExtensionScoreLayer: Scores extension placement efficiency.
//! Placed after ExtensionLayer.

use crate::layer::*;
use crate::terrain::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Scores how efficiently extensions are placed relative to the hub.
/// Closer extensions = higher score (less travel time for filling).
pub struct ExtensionScoreLayer;

impl PlacementLayer for ExtensionScoreLayer {
    fn name(&self) -> &str {
        "extension_score"
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

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let extension_locs: Vec<_> = state
            .structures
            .iter()
            .filter(|(_, items)| {
                items
                    .iter()
                    .any(|i| i.structure_type == StructureType::Extension)
            })
            .map(|(loc, _)| *loc)
            .collect();

        let ext_efficiency = if !extension_locs.is_empty() {
            let avg_dist: f32 = extension_locs
                .iter()
                .map(|loc| hub.distance_to(*loc) as f32)
                .sum::<f32>()
                / extension_locs.len() as f32;
            (1.0 - avg_dist / 25.0).max(0.0)
        } else {
            0.0
        };

        let mut new_state = state.clone();
        new_state.push_score("extension_efficiency", ext_efficiency, 1.0);
        Some(Ok(new_state))
    }
}
