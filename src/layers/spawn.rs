//! SpawnLayer: Places additional spawns near the hub (up to 3 total).
//! Deterministic (1 candidate) -- picks positions with good spacing.
//!
//! The hub stamp places 2 spawns; this layer places the remaining 1.
//! Spawns prefer positions with more open adjacent tiles to reduce congestion
//! during spawning. A minimum of 2 open neighbors is required, and spawns are
//! spaced apart from each other. Uses flood-fill distances from the hub for
//! accurate path-distance ordering.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
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
        let mut candidates: Vec<(Location, u32, u8)> = Vec::new(); // (loc, dist, open_neighbors)
        for radius in 2..=8i16 {
            for dy in -radius..=radius {
                for dx in -radius..=radius {
                    if dx.abs().max(dy.abs()) != radius {
                        continue;
                    }
                    let x = ax + dx;
                    let y = ay + dy;
                    let border = ROOM_BUILD_BORDER as i16;
                    if !(border..ROOM_WIDTH as i16 - border).contains(&x)
                        || !(border..ROOM_HEIGHT as i16 - border).contains(&y)
                    {
                        continue;
                    }
                    let loc = Location::from_xy(x as u8, y as u8);
                    if terrain.is_wall_at(loc) || state.is_occupied(loc) || has_road(state, loc) {
                        continue;
                    }

                    // Count open adjacent tiles (not wall, not occupied by non-road structure)
                    let open = count_open_neighbors(loc, terrain, state);

                    // Require minimum open neighbors for spawn accessibility
                    if open < MIN_OPEN_NEIGHBORS {
                        continue;
                    }

                    // Use flood-fill distance if available, fall back to Chebyshev
                    let dist = hub_dists
                        .as_ref()
                        .and_then(|d| *d.get(loc.x() as usize, loc.y() as usize))
                        .unwrap_or_else(|| {
                            (dx.unsigned_abs() as u32).max(dy.unsigned_abs() as u32)
                        });

                    candidates.push((loc, dist, open));
                }
            }
        }

        // Sort by: more open neighbors first, then closer to hub (by flood-fill distance)
        candidates.sort_by(|a, b| {
            b.2.cmp(&a.2) // more open neighbors first
                .then_with(|| a.1.cmp(&b.1)) // then closer to hub
        });

        let spawn_rcls = [7u8, 8];
        let mut placed = 0usize;
        let mut placed_spawn_positions: Vec<Location> = Vec::new();

        // Include existing spawn positions for spacing checks
        for spawn_loc in state.get_landmark_set("spawns") {
            placed_spawn_positions.push(*spawn_loc);
        }

        for &(loc, _, _) in &candidates {
            if placed >= needed {
                break;
            }
            if new_state.has_any_structure(loc) || new_state.is_excluded(loc) {
                continue;
            }

            // Check spacing from already-placed spawns
            let too_close = placed_spawn_positions
                .iter()
                .any(|&sloc| loc.distance_to(sloc) < MIN_SPAWN_SPACING);
            if too_close {
                continue;
            }

            let rcl = spawn_rcls[placed.min(spawn_rcls.len() - 1)];
            new_state.place_structure(loc, StructureType::Spawn, rcl);
            new_state.add_to_landmark_set("spawns", loc);
            placed_spawn_positions.push(loc);
            placed += 1;
        }

        Some(Ok(new_state))
    }
}

/// Check if a tile already has a road placed on it.
fn has_road(state: &PlacementState, loc: Location) -> bool {
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

/// Count the number of open (walkable, not occupied by non-road structures)
/// adjacent tiles around a position.
fn count_open_neighbors(loc: Location, terrain: &FastRoomTerrain, state: &PlacementState) -> u8 {
    let mut count = 0u8;
    for &(dx, dy) in &NEIGHBORS_8 {
        if let Some(nloc) = loc.checked_add(dx, dy) {
            if !terrain.is_wall_at(nloc) && !state.is_occupied(nloc) {
                count += 1;
            }
        }
    }
    count
}
