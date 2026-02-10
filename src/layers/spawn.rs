//! SpawnLayer: Places 2 additional spawns near the hub.
//! Deterministic (1 candidate) -- picks positions with good spacing.
//!
//! Spawns prefer positions with more open adjacent tiles to reduce congestion
//! during spawning. A minimum of 2 open neighbors is required, and spawns are
//! spaced apart from each other. Uses flood-fill distances from the hub for
//! accurate path-distance ordering.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Minimum number of open (walkable, non-occupied) adjacent tiles required
/// around a spawn candidate.
const MIN_OPEN_NEIGHBORS: u8 = 2;

/// Minimum Chebyshev distance between spawns to prevent congestion.
const MIN_SPAWN_SPACING: u8 = 2;

/// Places additional spawns near the hub (up to 3 total).
/// Prefers positions with more open adjacent tiles and enforces spacing
/// between spawns to prevent congestion.
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

        let mut new_state = state.clone();

        // Pre-compute hub flood-fill distances for accurate ordering
        let hub_dists = new_state.hub_distances(terrain).cloned();

        let ax = hub.x() as i16;
        let ay = hub.y() as i16;

        // Collect candidate positions with open-neighbor count and flood-fill distance
        let mut candidates: Vec<(u8, u8, u32, u8)> = Vec::new(); // (x, y, dist, open_neighbors)
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
                    if terrain.is_wall(ux, uy) || state.is_occupied(ux, uy) || has_road(state, ux, uy) {
                        continue;
                    }

                    // Count open adjacent tiles (not wall, not occupied by non-road structure)
                    let open = count_open_neighbors(ux, uy, terrain, state);

                    // Require minimum open neighbors for spawn accessibility
                    if open < MIN_OPEN_NEIGHBORS {
                        continue;
                    }

                    // Use flood-fill distance if available, fall back to Chebyshev
                    let dist = hub_dists
                        .as_ref()
                        .and_then(|d| *d.get(ux as usize, uy as usize))
                        .unwrap_or_else(|| (dx.unsigned_abs() as u32).max(dy.unsigned_abs() as u32));

                    candidates.push((ux, uy, dist, open));
                }
            }
        }

        // Sort by: more open neighbors first, then closer to hub (by flood-fill distance)
        candidates.sort_by(|a, b| {
            b.3.cmp(&a.3) // more open neighbors first
                .then_with(|| a.2.cmp(&b.2)) // then closer to hub
        });

        let spawn_rcls = [7u8, 8];
        let mut placed = 0usize;
        let mut placed_spawn_positions: Vec<(u8, u8)> = Vec::new();

        // Include existing spawn positions for spacing checks
        for spawn_loc in state.get_landmark_set("spawns") {
            placed_spawn_positions.push((spawn_loc.x(), spawn_loc.y()));
        }

        for &(x, y, _, _) in &candidates {
            if placed >= needed {
                break;
            }
            if new_state.has_any_structure(x, y) {
                continue;
            }

            // Check spacing from already-placed spawns
            let too_close = placed_spawn_positions.iter().any(|&(sx, sy)| {
                let dx = (x as i16 - sx as i16).unsigned_abs() as u8;
                let dy = (y as i16 - sy as i16).unsigned_abs() as u8;
                dx.max(dy) < MIN_SPAWN_SPACING
            });
            if too_close {
                continue;
            }

            let rcl = spawn_rcls[placed.min(spawn_rcls.len() - 1)];
            new_state.place_structure(x, y, StructureType::Spawn, rcl);
            new_state.add_to_landmark_set(
                "spawns",
                Location::from_coords(x as u32, y as u32),
            );
            placed_spawn_positions.push((x, y));
            placed += 1;
        }

        Some(Ok(new_state))
    }
}

/// Check if a tile already has a road placed on it.
fn has_road(state: &PlacementState, x: u8, y: u8) -> bool {
    let loc = Location::from_coords(x as u32, y as u32);
    state
        .structures
        .get(&loc)
        .map(|items| items.iter().any(|i| i.structure_type == StructureType::Road))
        .unwrap_or(false)
}

/// Count the number of open (walkable, not occupied by non-road structures)
/// adjacent tiles around a position.
fn count_open_neighbors(x: u8, y: u8, terrain: &FastRoomTerrain, state: &PlacementState) -> u8 {
    let mut count = 0u8;
    for &(dx, dy) in &NEIGHBORS_8 {
        let nx = x as i16 + dx as i16;
        let ny = y as i16 + dy as i16;
        if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
            continue;
        }
        let ux = nx as u8;
        let uy = ny as u8;
        if !terrain.is_wall(ux, uy) && !state.is_occupied(ux, uy) {
            count += 1;
        }
    }
    count
}
