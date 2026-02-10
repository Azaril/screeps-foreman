//! SpawnLayer: Places 2 additional spawns near the hub.
//! Deterministic (1 candidate) -- picks the closest valid positions.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Places additional spawns near the hub (up to 3 total).
/// Deterministic: picks the closest unoccupied positions to the hub.
pub struct SpawnLayer;

impl PlacementLayer for SpawnLayer {
    fn name(&self) -> &str {
        "spawn"
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

        let hub = state.get_landmark("hub")?;
        let existing_spawns = state.get_landmark_set("spawns").len();
        let needed = 3usize.saturating_sub(existing_spawns);

        if needed == 0 {
            return Some(Ok(state.clone()));
        }

        let ax = hub.x() as i16;
        let ay = hub.y() as i16;

        // Collect candidate positions sorted by distance from hub
        let mut candidates: Vec<(u8, u8, u32)> = Vec::new();
        for radius in 2..=8i16 {
            for dy in -radius..=radius {
                for dx in -radius..=radius {
                    if dx.abs().max(dy.abs()) != radius {
                        continue;
                    }
                    let x = ax + dx;
                    let y = ay + dy;
                    if !(2..48).contains(&x) || !(2..48).contains(&y) {
                        continue;
                    }
                    let ux = x as u8;
                    let uy = y as u8;
                    if terrain.is_wall(ux, uy) || state.is_occupied(ux, uy) {
                        continue;
                    }
                    let dist = (dx.unsigned_abs() as u32).max(dy.unsigned_abs() as u32);
                    candidates.push((ux, uy, dist));
                }
            }
        }
        candidates.sort_by_key(|c| c.2);

        let mut new_state = state.clone();
        let spawn_rcls = [7u8, 8];
        let mut placed = 0usize;

        for (x, y, _) in &candidates {
            if placed >= needed {
                break;
            }
            if new_state.is_occupied(*x, *y) {
                continue;
            }
            let rcl = spawn_rcls[placed.min(spawn_rcls.len() - 1)];
            new_state.place_structure(*x, *y, StructureType::Spawn, rcl);
            new_state.add_to_landmark_set(
                "spawns",
                Location::from_coords(*x as u32, *y as u32),
            );
            placed += 1;
        }

        Some(Ok(new_state))
    }
}
