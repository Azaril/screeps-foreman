//! UtilityLayer: Places observer, factory, power spawn, and nuker.
//! Low branching -- deterministic placement.
//!
//! Factory and PowerSpawn are placed near the hub (within 2-3 tiles) so they
//! remain accessible to the filler creep with minimal movement. Nuker and
//! Observer are placed at moderate distance since they are rarely interacted
//! with.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Places observer, factory, power spawn, and nuker near the hub.
/// Factory and PowerSpawn prefer positions within 2-3 tiles of hub for
/// easy filler access. Observer and Nuker are placed farther out.
/// Deterministic (1 candidate).
pub struct UtilityLayer;

impl PlacementLayer for UtilityLayer {
    fn name(&self) -> &str {
        "utility"
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

        let mut new_state = state.clone();

        // Place factory near hub (within 2-3 tiles) -- needs frequent filler access
        if !has_structure_type(&new_state, StructureType::Factory) {
            place_near_hub(
                hub,
                2,
                4,
                terrain,
                &mut new_state,
                StructureType::Factory,
                7,
            );
        }

        // Place power spawn near hub (within 2-3 tiles) -- needs frequent filler access
        if !has_structure_type(&new_state, StructureType::PowerSpawn) {
            place_near_hub(
                hub,
                2,
                4,
                terrain,
                &mut new_state,
                StructureType::PowerSpawn,
                8,
            );
        }

        // Place nuker at moderate distance from hub
        if !has_structure_type(&new_state, StructureType::Nuker) {
            place_near_hub(hub, 2, 6, terrain, &mut new_state, StructureType::Nuker, 8);
        }

        // Place observer at a moderate distance from hub
        if !has_structure_type(&new_state, StructureType::Observer) {
            place_near_hub(
                hub,
                3,
                10,
                terrain,
                &mut new_state,
                StructureType::Observer,
                8,
            );
        }

        Some(Ok(new_state))
    }
}

/// Check if a structure type already exists in the placement state.
fn has_structure_type(state: &PlacementState, st: StructureType) -> bool {
    state
        .structures
        .values()
        .any(|items| items.iter().any(|i| i.structure_type == st))
}

/// Place a structure at the nearest available position within a distance range
/// from the hub. Searches outward from `min_radius` to `max_radius`.
#[allow(clippy::too_many_arguments)]
fn place_near_hub(
    hub: Location,
    min_radius: i16,
    max_radius: i16,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
    structure_type: StructureType,
    rcl: u8,
) {
    let ax = hub.x() as i16;
    let ay = hub.y() as i16;
    let border = ROOM_BUILD_BORDER as i16;
    for radius in min_radius..=max_radius {
        for dy in -radius..=radius {
            for dx in -radius..=radius {
                if dx.abs().max(dy.abs()) != radius {
                    continue;
                }
                let x = ax + dx;
                let y = ay + dy;
                if (border..ROOM_WIDTH as i16 - border).contains(&x)
                    && (border..ROOM_HEIGHT as i16 - border).contains(&y)
                {
                    let loc = Location::from_xy(x as u8, y as u8);
                    if !terrain.is_wall_at(loc)
                        && !state.has_any_structure(loc)
                        && !state.is_excluded(loc)
                        && !has_road_at(state, loc)
                    {
                        state.place_structure(loc, structure_type, rcl);
                        return;
                    }
                }
            }
        }
    }
}

/// Check if a tile has a road placed on it.
fn has_road_at(state: &PlacementState, loc: Location) -> bool {
    state
        .structures
        .get(&loc)
        .map(|items| {
            items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
        })
        .unwrap_or(false)
}
