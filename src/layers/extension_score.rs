//! ExtensionScoreLayer: Scores extension placement efficiency.
//! Placed after ExtensionLayer.
//!
//! Uses flood-fill distances from the hub (accounting for terrain walls)
//! rather than Chebyshev distance for more accurate travel time estimates.

use crate::layer::*;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Scores how efficiently extensions are placed relative to the hub.
/// Closer extensions = higher score (less travel time for filling).
/// Uses flood-fill distance to account for terrain obstacles.
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
        terrain: &FastRoomTerrain,
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

        let mut new_state = state.clone();

        let ext_efficiency = if !extension_locs.is_empty() {
            // Use flood-fill distances from hub for accurate path distances
            let avg_dist: f32 = if let Some(hub_dists) = new_state.hub_distances(terrain) {
                extension_locs
                    .iter()
                    .map(|loc| {
                        hub_dists
                            .get(loc.x() as usize, loc.y() as usize)
                            .unwrap_or(25) as f32
                    })
                    .sum::<f32>()
                    / extension_locs.len() as f32
            } else {
                // Fallback to Chebyshev if hub distances unavailable
                extension_locs
                    .iter()
                    .map(|loc| hub.distance_to(*loc) as f32)
                    .sum::<f32>()
                    / extension_locs.len() as f32
            };
            (1.0 - avg_dist / 25.0).max(0.0)
        } else {
            0.0
        };

        new_state.push_score("extension_efficiency", ext_efficiency, 1.0);
        Some(Ok(new_state))
    }
}
