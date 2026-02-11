//! AnchorLayer: Pure position selector for the hub.
//! High branching -- generates one candidate per valid (x, y) position.
//! Does NOT place any structures; that is the job of a subsequent StampLayer.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

/// Selects a hub position from valid anchor locations.
/// Positions are pre-sorted by distance transform (descending) so that
/// the search explores the most open areas first, enabling better pruning.
///
/// A `stride` parameter controls the minimum spacing between candidates
/// to avoid exploring nearly-identical positions. stride=1 means every
/// valid tile is a candidate; stride=2 means only every other tile in
/// each direction, etc.
pub struct AnchorLayer {
    pub min_dist_transform: u8,
    pub stride: u8,
}

impl Default for AnchorLayer {
    fn default() -> Self {
        AnchorLayer {
            min_dist_transform: 3,
            stride: 2,
        }
    }
}

impl PlacementLayer for AnchorLayer {
    fn name(&self) -> &str {
        "anchor"
    }

    fn candidate_count(
        &self,
        _state: &PlacementState,
        _analysis: &AnalysisOutput,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        None
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        analysis: &AnalysisOutput,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        let min_dt = self.min_dist_transform;
        let stride = self.stride.max(1);

        // Collect all valid positions with their distance transform values
        // on first call, then index into the sorted list.
        // Since layers are stateless, we rebuild each time -- but the
        // search engine caches the frame so this only runs once per
        // candidate index.
        let mut positions: Vec<(u8, u8, u8)> = Vec::new();

        let border = ROOM_BUILD_BORDER;
        let mut y = border;
        while y < ROOM_HEIGHT - border {
            let mut x = border;
            while x < ROOM_WIDTH - border {
                if !terrain.is_wall(x, y) {
                    let dt = *analysis.dist_transform.get(x as usize, y as usize);
                    if dt >= min_dt {
                        positions.push((x, y, dt));
                    }
                }
                x += stride;
            }
            y += stride;
        }

        // Sort by distance transform descending (most open areas first),
        // breaking ties by position for determinism.
        positions.sort_by(|a, b| b.2.cmp(&a.2).then(a.1.cmp(&b.1)).then(a.0.cmp(&b.0)));

        if index >= positions.len() {
            return None;
        }

        let (x, y, _dt) = positions[index];
        let mut new_state = state.clone();
        let hub_loc = Location::from_coords(x as u32, y as u32);
        new_state.set_landmark("hub", hub_loc);
        Some(Ok(new_state))
    }
}
