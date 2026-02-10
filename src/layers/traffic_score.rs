//! TrafficScoreLayer: Placeholder scoring layer for traffic congestion.
//! Placed at the end of the stack.

use crate::layer::*;
use crate::terrain::*;

/// Placeholder scoring layer for traffic congestion analysis.
/// Currently returns a fixed score; can be expanded with actual
/// traffic simulation in the future.
pub struct TrafficScoreLayer;

impl PlacementLayer for TrafficScoreLayer {
    fn name(&self) -> &str {
        "traffic_score"
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

        let mut new_state = state.clone();
        new_state.push_score("traffic_congestion", 0.5, 0.2);
        Some(Ok(new_state))
    }
}
