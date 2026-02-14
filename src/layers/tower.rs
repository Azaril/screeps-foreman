//! TowerLayer: Places 6 towers to maximize minimum damage across attack positions
//! while preferring positions closer to the hub (storage) for faster refueling.
//!
//! Uses greedy placement followed by iterative swap refinement to escape local
//! optima. The search radius is wide enough (10 tiles) to allow towers to spread
//! toward the defensive perimeter for better coverage.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::stamps::tower::tower_damage_at_range;
use crate::terrain::*;

use screeps::constants::StructureType;

/// Weight for hub proximity in the composite tower score.
/// Moderate value balances coverage vs. refueling distance.
const HUB_PROXIMITY_WEIGHT: f32 = 15.0;

/// Maximum search radius for tower candidates (Chebyshev distance from hub).
/// Wider radius allows towers to spread toward ramparts for better coverage.
const MAX_TOWER_RADIUS: i16 = 10;

/// Number of iterative refinement passes after greedy placement.
/// Each pass tries swapping each tower to all candidate positions.
const REFINEMENT_PASSES: usize = 3;

/// Places 6 towers using greedy placement followed by iterative swap refinement.
/// Deterministic (1 candidate) -- does not expand the search tree.
pub struct TowerLayer;

impl PlacementLayer for TowerLayer {
    fn name(&self) -> &str {
        "tower"
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

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let ax = hub.x() as i16;
        let ay = hub.y() as i16;

        let new_state = state.clone();

        // Collect perimeter tiles (exit tiles + tiles at distance ~8-12 from anchor)
        let mut perimeter_tiles: Vec<Location> = Vec::new();
        for exit in &analysis.exits.all {
            perimeter_tiles.push(*exit);
        }
        for dy in -12..=12i16 {
            for dx in -12..=12i16 {
                let dist = dx.abs().max(dy.abs());
                if !(8..=12).contains(&dist) {
                    continue;
                }
                let x = ax + dx;
                let y = ay + dy;
                if xy_in_bounds(x, y) && !terrain.is_wall(x as u8, y as u8) {
                    perimeter_tiles.push(Location::from_xy(x as u8, y as u8));
                }
            }
        }

        if perimeter_tiles.is_empty() {
            // Fallback: place towers at fixed offsets
            let mut new_state = new_state;
            let offsets: [(i8, i8); 6] = [(-2, -1), (-1, -2), (2, -1), (-1, 2), (2, 1), (1, 2)];
            for (i, &(dx, dy)) in offsets.iter().enumerate() {
                if let Some(loc) = hub.checked_add(dx, dy) {
                    let rcl = match i {
                        0 => 3,
                        1 => 5,
                        2 => 7,
                        _ => 8,
                    };
                    if !terrain.is_wall(loc.x(), loc.y())
                        && !new_state.is_occupied(loc)
                        && !has_road(&new_state, loc)
                    {
                        new_state.place_structure(loc, StructureType::Tower, rcl);
                        new_state.add_to_landmark_set("towers", loc);
                    }
                }
            }
            return Some(Ok(new_state));
        }

        // Generate candidate tower positions near hub
        let tower_candidates: Vec<Location> = {
            let mut candidates = Vec::new();
            for dy in -MAX_TOWER_RADIUS..=MAX_TOWER_RADIUS {
                for dx in -MAX_TOWER_RADIUS..=MAX_TOWER_RADIUS {
                    let x = ax + dx;
                    let y = ay + dy;
                    let border = ROOM_BUILD_BORDER as i16;
                    if (border..ROOM_WIDTH as i16 - border).contains(&x)
                        && (border..ROOM_HEIGHT as i16 - border).contains(&y)
                    {
                        let loc = Location::from_xy(x as u8, y as u8);
                        if !terrain.is_wall(loc.x(), loc.y())
                            && !state.is_occupied(loc)
                            && !has_road(state, loc)
                        {
                            candidates.push(loc);
                        }
                    }
                }
            }
            candidates
        };

        // Maximum possible hub distance for normalization
        let max_hub_dist = MAX_TOWER_RADIUS as f32;

        // --- Phase 1: Greedy placement ---
        let mut tower_positions: Vec<Location> = Vec::new();
        let tower_rcls = [3u8, 5, 7, 8, 8, 8];

        for _rcl in &tower_rcls {
            let mut best_pos = None;
            let mut best_score = f32::NEG_INFINITY;

            for &candidate in &tower_candidates {
                if tower_positions.contains(&candidate) {
                    continue;
                }

                let score = evaluate_tower_set_with_candidate(
                    &tower_positions,
                    candidate,
                    &perimeter_tiles,
                    hub,
                    max_hub_dist,
                );

                if score > best_score {
                    best_score = score;
                    best_pos = Some(candidate);
                }
            }

            if let Some(pos) = best_pos {
                tower_positions.push(pos);
            }
        }

        // --- Phase 2: Iterative swap refinement ---
        // Try swapping each tower to every candidate position; keep improvements.
        for _pass in 0..REFINEMENT_PASSES {
            let mut improved = false;
            for ti in 0..tower_positions.len() {
                let current_score =
                    evaluate_tower_set(&tower_positions, &perimeter_tiles, hub, max_hub_dist);

                let mut best_swap = None;
                let mut best_swap_score = current_score;

                for &candidate in &tower_candidates {
                    if tower_positions.contains(&candidate) {
                        continue;
                    }

                    let mut test_positions = tower_positions.clone();
                    test_positions[ti] = candidate;

                    let score =
                        evaluate_tower_set(&test_positions, &perimeter_tiles, hub, max_hub_dist);

                    if score > best_swap_score {
                        best_swap_score = score;
                        best_swap = Some(candidate);
                    }
                }

                if let Some(new_pos) = best_swap {
                    tower_positions[ti] = new_pos;
                    improved = true;
                }
            }
            if !improved {
                break;
            }
        }

        // --- Place towers in state ---
        let mut new_state = new_state;
        for (i, &loc) in tower_positions.iter().enumerate() {
            let rcl = tower_rcls[i.min(tower_rcls.len() - 1)];
            new_state.place_structure(loc, StructureType::Tower, rcl);
            new_state.add_to_landmark_set("towers", loc);
        }

        Some(Ok(new_state))
    }
}

/// Evaluate a complete tower set: returns composite score of min_damage + proximity.
fn evaluate_tower_set(
    tower_positions: &[Location],
    perimeter_tiles: &[Location],
    hub: Location,
    max_hub_dist: f32,
) -> f32 {
    if tower_positions.is_empty() || perimeter_tiles.is_empty() {
        return 0.0;
    }

    let mut min_damage = u32::MAX;
    for &perimeter in perimeter_tiles {
        let mut total_damage = 0u32;
        for &tower in tower_positions {
            let range = tower.distance_to(perimeter) as u32;
            total_damage += tower_damage_at_range(range);
        }
        if total_damage < min_damage {
            min_damage = total_damage;
        }
    }

    // Average hub proximity across all towers
    let avg_proximity: f32 = tower_positions
        .iter()
        .map(|&tower| {
            let hub_dist = tower.distance_to(hub) as f32;
            1.0 - (hub_dist / max_hub_dist).min(1.0)
        })
        .sum::<f32>()
        / tower_positions.len() as f32;

    min_damage as f32 + HUB_PROXIMITY_WEIGHT * avg_proximity
}

/// Evaluate a tower set with one additional candidate tower.
fn evaluate_tower_set_with_candidate(
    existing: &[Location],
    candidate: Location,
    perimeter_tiles: &[Location],
    hub: Location,
    max_hub_dist: f32,
) -> f32 {
    let mut min_damage = u32::MAX;
    for &perimeter in perimeter_tiles {
        let mut total_damage = 0u32;
        for &tower in existing {
            let range = tower.distance_to(perimeter) as u32;
            total_damage += tower_damage_at_range(range);
        }
        let range = candidate.distance_to(perimeter) as u32;
        total_damage += tower_damage_at_range(range);

        if total_damage < min_damage {
            min_damage = total_damage;
        }
    }

    // Proximity for the candidate tower
    let hub_dist = candidate.distance_to(hub) as f32;
    let proximity = 1.0 - (hub_dist / max_hub_dist).min(1.0);
    min_damage as f32 + HUB_PROXIMITY_WEIGHT * proximity
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
