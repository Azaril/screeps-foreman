//! UtilityLayer: Places observer and nuker.
//! Low branching -- deterministic placement.

use crate::layer::*;
use crate::terrain::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Places observer (and nuker if not already placed by hub stamp).
/// Deterministic (1 candidate).
pub struct UtilityLayer;

impl PlacementLayer for UtilityLayer {
    fn name(&self) -> &str {
        "utility"
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

        let ax = hub.x() as i16;
        let ay = hub.y() as i16;
        let mut new_state = state.clone();

        // Place observer at a moderate distance from hub
        'observer: for radius in 3..=10i16 {
            for dy in -radius..=radius {
                for dx in -radius..=radius {
                    if dx.abs().max(dy.abs()) != radius {
                        continue;
                    }
                    let x = ax + dx;
                    let y = ay + dy;
                    if (2..48).contains(&x) && (2..48).contains(&y) {
                        let ux = x as u8;
                        let uy = y as u8;
                        if !terrain.is_wall(ux, uy) && !new_state.is_occupied(ux, uy) {
                            new_state.place_structure(ux, uy, StructureType::Observer, 8);
                            break 'observer;
                        }
                    }
                }
            }
        }

        Some(Ok(new_state))
    }
}
