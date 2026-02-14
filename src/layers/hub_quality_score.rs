//! HubQualityScoreLayer: Scores how many key structures are adjacent to the hub.
//! Placed after the hub stamp layer.
//!
//! The fast-filler hub design places Storage, Link, 2 Spawns, 1 Extension, and
//! Terminal adjacent to the filler creep. This layer scores how many of those
//! key structures are actually hub-adjacent (some may be missing due to terrain
//! constraints).

use crate::constants::*;
use crate::layer::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Scores how many key structures are adjacent to the hub position.
/// The fast-filler design targets: Storage, Link, 2x Spawn, Extension, Terminal.
/// Higher is better.
pub struct HubQualityScoreLayer;

impl PlacementLayer for HubQualityScoreLayer {
    fn name(&self) -> &str {
        "hub_quality_score"
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

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        // Key structure types for the fast-filler hub. Spawns are counted
        // separately since we want 2 of them adjacent.
        let key_types = [
            StructureType::Storage,
            StructureType::Terminal,
            StructureType::Link,
        ];

        let mut key_count = 0u32;
        let mut spawn_count = 0u32;
        let mut extension_count = 0u32;

        for &(dx, dy) in &NEIGHBORS_8 {
            if let Some(loc) = hub.checked_add(dx, dy) {
                if let Some(items) = state.structures.get(&loc) {
                    for item in items {
                        if key_types.contains(&item.structure_type) {
                            key_count += 1;
                        } else if item.structure_type == StructureType::Spawn {
                            spawn_count += 1;
                        } else if item.structure_type == StructureType::Extension {
                            extension_count += 1;
                        }
                    }
                }
            }
        }

        // Target: 3 key types + 2 spawns + 1 extension = 6 total
        let total = key_count + spawn_count.min(2) + extension_count.min(1);
        let hub_quality = (total as f32 / 6.0).min(1.0);

        let mut new_state = state.clone();
        new_state.push_score("hub_quality", hub_quality, 1.5);
        Some(Ok(new_state))
    }
}
