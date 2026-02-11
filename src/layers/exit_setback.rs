//! ExitSetbackLayer: Excludes tiles near room exits from placement.
//!
//! This layer marks tiles within a configurable Chebyshev distance of exit
//! tiles (passable border tiles) as excluded. Tiles that are already walls
//! are not added to the exclusion set since they are naturally blocked by
//! terrain.
//!
//! The layer is generic and can be reused for any border-distance enforcement
//! by adjusting the `distance` parameter.

use crate::constants::*;
use crate::layer::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

/// Excludes tiles within a given Chebyshev distance of room exit tiles.
///
/// Exit tiles are passable (non-wall) tiles on the room border (x=0, x=49,
/// y=0, y=49). The `distance` parameter controls how far the exclusion zone
/// extends inward from each exit tile. A distance of 1 excludes the exit
/// tile itself and all tiles directly adjacent to it.
///
/// Tiles that are walls are not added to the exclusion set since terrain
/// already blocks placement on those tiles.
pub struct ExitSetbackLayer {
    /// Chebyshev distance from exit tiles to exclude.
    pub distance: u32,
}

impl ExitSetbackLayer {
    /// Create a new ExitSetbackLayer with the given setback distance.
    pub fn new(distance: u32) -> Self {
        ExitSetbackLayer { distance }
    }
}

impl PlacementLayer for ExitSetbackLayer {
    fn name(&self) -> &str {
        "exit_setback"
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

        // Use the pre-computed exit distances from analysis to find all tiles
        // within the setback distance of any exit tile.
        for y in 0..ROOM_HEIGHT as usize {
            for x in 0..ROOM_WIDTH as usize {
                // Skip tiles that are already walls -- terrain blocks them naturally.
                if terrain.is_wall(x as u8, y as u8) {
                    continue;
                }

                if let Some(dist) = analysis.exit_distances.get(x, y) {
                    if *dist <= self.distance {
                        new_state.exclude_tile(x as u8, y as u8);
                    }
                }
            }
        }

        Some(Ok(new_state))
    }
}
