//! TowerLayer: Places 6 towers to maximize minimum damage across attack positions
//! while preferring positions closer to the hub (storage) for faster refueling.
//!
//! Uses greedy placement followed by iterative swap refinement to escape local
//! optima. The search radius is wide enough (10 tiles) to allow towers to spread
//! toward the defensive perimeter for better coverage.

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
        let mut perimeter_tiles: Vec<(u8, u8)> = Vec::new();
        for exit in &analysis.exits.all {
            perimeter_tiles.push((exit.x(), exit.y()));
        }
        for dy in -12..=12i16 {
            for dx in -12..=12i16 {
                let dist = dx.abs().max(dy.abs());
                if !(8..=12).contains(&dist) {
                    continue;
                }
                let x = ax + dx;
                let y = ay + dy;
                if (0..50).contains(&x)
                    && (0..50).contains(&y)
                    && !terrain.is_wall(x as u8, y as u8)
                {
                    perimeter_tiles.push((x as u8, y as u8));
                }
            }
        }

        if perimeter_tiles.is_empty() {
            // Fallback: place towers at fixed offsets
            let mut new_state = new_state;
            let offsets = [(-2, -1), (-1, -2), (2, -1), (-1, 2), (2, 1), (1, 2)];
            for (i, (dx, dy)) in offsets.iter().enumerate() {
                let x = (ax + dx) as u8;
                let y = (ay + dy) as u8;
                let rcl = match i {
                    0 => 3,
                    1 => 5,
                    2 => 7,
                    _ => 8,
                };
                if !terrain.is_wall(x, y)
                    && !new_state.is_occupied(x, y)
                    && !has_road(&new_state, x, y)
                {
                    new_state.place_structure(x, y, StructureType::Tower, rcl);
                    new_state
                        .add_to_landmark_set("towers", Location::from_coords(x as u32, y as u32));
                }
            }
            return Some(Ok(new_state));
        }

        // Generate candidate tower positions near hub
        let tower_candidates: Vec<(u8, u8)> = {
            let mut candidates = Vec::new();
            for dy in -MAX_TOWER_RADIUS..=MAX_TOWER_RADIUS {
                for dx in -MAX_TOWER_RADIUS..=MAX_TOWER_RADIUS {
                    let x = ax + dx;
                    let y = ay + dy;
                    if (2..48).contains(&x) && (2..48).contains(&y) {
                        let ux = x as u8;
                        let uy = y as u8;
                        if !terrain.is_wall(ux, uy)
                            && !state.is_occupied(ux, uy)
                            && !has_road(state, ux, uy)
                        {
                            candidates.push((ux, uy));
                        }
                    }
                }
            }
            candidates
        };

        // Maximum possible hub distance for normalization
        let max_hub_dist = MAX_TOWER_RADIUS as f32;

        // --- Phase 1: Greedy placement ---
        let mut tower_positions: Vec<(u8, u8)> = Vec::new();
        let tower_rcls = [3u8, 5, 7, 8, 8, 8];

        for _rcl in &tower_rcls {
            let mut best_pos = None;
            let mut best_score = f32::NEG_INFINITY;

            for &(tx, ty) in &tower_candidates {
                if tower_positions.contains(&(tx, ty)) {
                    continue;
                }

                let score = evaluate_tower_set_with_candidate(
                    &tower_positions,
                    tx,
                    ty,
                    &perimeter_tiles,
                    hub,
                    max_hub_dist,
                );

                if score > best_score {
                    best_score = score;
                    best_pos = Some((tx, ty));
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

                for &(cx, cy) in &tower_candidates {
                    if tower_positions.contains(&(cx, cy)) {
                        continue;
                    }

                    let mut test_positions = tower_positions.clone();
                    test_positions[ti] = (cx, cy);

                    let score =
                        evaluate_tower_set(&test_positions, &perimeter_tiles, hub, max_hub_dist);

                    if score > best_swap_score {
                        best_swap_score = score;
                        best_swap = Some((cx, cy));
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
        for (i, &(tx, ty)) in tower_positions.iter().enumerate() {
            let rcl = tower_rcls[i.min(tower_rcls.len() - 1)];
            new_state.place_structure(tx, ty, StructureType::Tower, rcl);
            new_state.add_to_landmark_set("towers", Location::from_coords(tx as u32, ty as u32));
        }

        Some(Ok(new_state))
    }
}

/// Evaluate a complete tower set: returns composite score of min_damage + proximity.
fn evaluate_tower_set(
    tower_positions: &[(u8, u8)],
    perimeter_tiles: &[(u8, u8)],
    hub: Location,
    max_hub_dist: f32,
) -> f32 {
    if tower_positions.is_empty() || perimeter_tiles.is_empty() {
        return 0.0;
    }

    let mut min_damage = u32::MAX;
    for &(px, py) in perimeter_tiles {
        let mut total_damage = 0u32;
        for &(tx, ty) in tower_positions {
            let range = chebyshev_dist(tx, ty, px, py);
            total_damage += tower_damage_at_range(range);
        }
        if total_damage < min_damage {
            min_damage = total_damage;
        }
    }

    // Average hub proximity across all towers
    let avg_proximity: f32 = tower_positions
        .iter()
        .map(|&(tx, ty)| {
            let hub_dist = chebyshev_dist(tx, ty, hub.x(), hub.y()) as f32;
            1.0 - (hub_dist / max_hub_dist).min(1.0)
        })
        .sum::<f32>()
        / tower_positions.len() as f32;

    min_damage as f32 + HUB_PROXIMITY_WEIGHT * avg_proximity
}

/// Evaluate a tower set with one additional candidate tower.
fn evaluate_tower_set_with_candidate(
    existing: &[(u8, u8)],
    tx: u8,
    ty: u8,
    perimeter_tiles: &[(u8, u8)],
    hub: Location,
    max_hub_dist: f32,
) -> f32 {
    let mut min_damage = u32::MAX;
    for &(px, py) in perimeter_tiles {
        let mut total_damage = 0u32;
        for &(etx, ety) in existing {
            let range = chebyshev_dist(etx, ety, px, py);
            total_damage += tower_damage_at_range(range);
        }
        let range = chebyshev_dist(tx, ty, px, py);
        total_damage += tower_damage_at_range(range);

        if total_damage < min_damage {
            min_damage = total_damage;
        }
    }

    // Proximity for the candidate tower
    let hub_dist = chebyshev_dist(tx, ty, hub.x(), hub.y()) as f32;
    let proximity = 1.0 - (hub_dist / max_hub_dist).min(1.0);
    min_damage as f32 + HUB_PROXIMITY_WEIGHT * proximity
}

/// Check if a tile already has a road placed on it.
fn has_road(state: &PlacementState, x: u8, y: u8) -> bool {
    let loc = Location::from_coords(x as u32, y as u32);
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

fn chebyshev_dist(x1: u8, y1: u8, x2: u8, y2: u8) -> u32 {
    let dx = (x1 as i32 - x2 as i32).unsigned_abs();
    let dy = (y1 as i32 - y2 as i32).unsigned_abs();
    dx.max(dy)
}
