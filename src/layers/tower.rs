//! TowerLayer: Places 6 towers to maximize minimum damage across attack positions.
//! Uses greedy placement -- deterministic given the current state.

use crate::layer::*;
use crate::location::*;
use crate::stamps::tower::tower_damage_at_range;
use crate::terrain::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Places 6 towers using greedy placement to maximize minimum damage.
/// Deterministic (1 candidate) -- does not expand the search tree.
pub struct TowerLayer;

impl PlacementLayer for TowerLayer {
    fn name(&self) -> &str {
        "tower"
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

        // Collect perimeter tiles (exit tiles + tiles at distance ~8-12 from anchor)
        let mut perimeter_tiles: Vec<(u8, u8)> = Vec::new();
        for exit in &state.analysis.exits.all {
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
                if !terrain.is_wall(x, y) && !new_state.is_occupied(x, y) {
                    new_state.place_structure(x, y, StructureType::Tower, rcl);
                    new_state.add_to_landmark_set(
                        "towers",
                        Location::from_coords(x as u32, y as u32),
                    );
                }
            }
            return Some(Ok(new_state));
        }

        // Greedy tower placement
        let mut tower_positions: Vec<(u8, u8)> = Vec::new();

        // Generate candidate tower positions near hub
        let mut tower_candidates: Vec<(u8, u8)> = Vec::new();
        for dy in -6..=6i16 {
            for dx in -6..=6i16 {
                let x = ax + dx;
                let y = ay + dy;
                if (2..48).contains(&x) && (2..48).contains(&y) {
                    let ux = x as u8;
                    let uy = y as u8;
                    if !terrain.is_wall(ux, uy) && !new_state.is_occupied(ux, uy) {
                        tower_candidates.push((ux, uy));
                    }
                }
            }
        }

        let tower_rcls = [3u8, 5, 7, 8, 8, 8];

        for &rcl in &tower_rcls {
            let mut best_pos = None;
            let mut best_min_damage = 0u32;

            for &(tx, ty) in &tower_candidates {
                if tower_positions.contains(&(tx, ty)) {
                    continue;
                }

                let mut min_damage = u32::MAX;
                for &(px, py) in &perimeter_tiles {
                    let mut total_damage = 0u32;
                    for &(etx, ety) in &tower_positions {
                        let range = chebyshev_dist(etx, ety, px, py);
                        total_damage += tower_damage_at_range(range);
                    }
                    let range = chebyshev_dist(tx, ty, px, py);
                    total_damage += tower_damage_at_range(range);

                    if total_damage < min_damage {
                        min_damage = total_damage;
                    }
                }

                if min_damage > best_min_damage {
                    best_min_damage = min_damage;
                    best_pos = Some((tx, ty));
                }
            }

            if let Some((tx, ty)) = best_pos {
                tower_positions.push((tx, ty));
                new_state.place_structure(tx, ty, StructureType::Tower, rcl);
                new_state.add_to_landmark_set(
                    "towers",
                    Location::from_coords(tx as u32, ty as u32),
                );
            }
        }

        Some(Ok(new_state))
    }
}

fn chebyshev_dist(x1: u8, y1: u8, x2: u8, y2: u8) -> u32 {
    let dx = (x1 as i32 - x2 as i32).unsigned_abs();
    let dy = (y1 as i32 - y2 as i32).unsigned_abs();
    dx.max(dy)
}
