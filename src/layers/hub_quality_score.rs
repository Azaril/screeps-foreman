//! HubQualityScoreLayer: Scores how many key structures are adjacent to the hub.
//! Placed after the hub stamp layer.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Scores how many key structures (Storage, Terminal, Link, Factory, PowerSpawn)
/// are adjacent to the hub position. Higher is better.
pub struct HubQualityScoreLayer;

impl PlacementLayer for HubQualityScoreLayer {
    fn name(&self) -> &str {
        "hub_quality_score"
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

        let key_types = [
            StructureType::Storage,
            StructureType::Terminal,
            StructureType::Link,
            StructureType::Factory,
            StructureType::PowerSpawn,
        ];

        let mut adjacent_count = 0;
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = hub.x() as i16 + dx as i16;
            let ny = hub.y() as i16 + dy as i16;
            if (0..50).contains(&nx) && (0..50).contains(&ny) {
                let loc = Location::from_coords(nx as u32, ny as u32);
                if let Some(items) = state.structures.get(&loc) {
                    for item in items {
                        if key_types.contains(&item.structure_type) {
                            adjacent_count += 1;
                        }
                    }
                }
            }
        }

        let hub_quality = (adjacent_count as f32 / key_types.len() as f32).min(1.0);

        let mut new_state = state.clone();
        new_state.push_score("hub_quality", hub_quality, 1.5);
        Some(Ok(new_state))
    }
}
