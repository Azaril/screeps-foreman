//! ExtensionScoreLayer: Scores extension placement efficiency.
//! Placed after ExtensionLayer.
//!
//! Scores two aspects:
//! 1. **Distance efficiency** -- average flood-fill distance from hub to extensions.
//!    Closer extensions = less travel time for filler creeps.
//! 2. **Fill-path efficiency** -- how many extensions a filler creep can fill per
//!    road stop. Extensions adjacent to roads that serve multiple extensions score
//!    higher, reflecting faster fill cycles.

use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Scores how efficiently extensions are placed relative to the hub.
/// Combines distance efficiency and fill-path efficiency.
pub struct ExtensionScoreLayer;

impl PlacementLayer for ExtensionScoreLayer {
    fn name(&self) -> &str {
        "extension_score"
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

        // 1. Distance efficiency (weight 1.0)
        let ext_efficiency = if !extension_locs.is_empty() {
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

        // 2. Fill-path efficiency (weight 0.5)
        // Compute the average number of extensions fillable per road stop.
        // A road tile that is adjacent to 3 extensions is better than one
        // adjacent to 1, because the filler creep fills more per stop.
        let fill_efficiency = if !extension_locs.is_empty() {
            let extension_set: fnv::FnvHashSet<Location> = extension_locs.iter().copied().collect();

            // Find all road tiles and count how many extensions each can fill
            let mut total_fill_capacity = 0u32;
            let mut road_count = 0u32;

            for (loc, items) in &state.structures {
                let is_road = items
                    .iter()
                    .any(|i| i.structure_type == StructureType::Road);
                if !is_road {
                    continue;
                }

                let mut ext_adjacent = 0u32;
                for &(dx, dy) in &NEIGHBORS_8 {
                    let nx = loc.x() as i16 + dx as i16;
                    let ny = loc.y() as i16 + dy as i16;
                    if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                        continue;
                    }
                    let nloc = Location::from_coords(nx as u32, ny as u32);
                    if extension_set.contains(&nloc) {
                        ext_adjacent += 1;
                    }
                }

                if ext_adjacent > 0 {
                    total_fill_capacity += ext_adjacent;
                    road_count += 1;
                }
            }

            if road_count > 0 {
                // Average extensions per fill-road. Ideal is ~3 (4x4 stamp corners).
                // Normalize: 3.0 ext/stop -> 1.0 score, 1.0 ext/stop -> 0.33
                let avg_fill = total_fill_capacity as f32 / road_count as f32;
                (avg_fill / 3.0).min(1.0)
            } else {
                0.0
            }
        } else {
            0.0
        };
        new_state.push_score("fill_efficiency", fill_efficiency, 0.5);

        Some(Ok(new_state))
    }
}
