//! AnchorScoreLayer: Scoring-only layer that evaluates the chosen anchor position.
//! Pushes ScoreEntry values for source distance, source balance, controller distance,
//! exit proximity, mineral distance, terrain openness, and defensibility.
//! Enables early pruning of bad anchors.

use crate::constants::*;
use crate::layer::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

/// Scoring-only layer that evaluates the anchor/hub position.
/// No structures are placed; only ScoreEntry values are pushed.
pub struct AnchorScoreLayer;

/// Radius for counting buildable tiles around the hub for the openness score.
const OPENNESS_RADIUS: i16 = 8;

/// Maximum expected buildable tiles within `OPENNESS_RADIUS` for normalization.
/// A fully open area has (2*8+1)^2 = 289 tiles; ~200 is a reasonable upper bound
/// after accounting for typical wall density.
const OPENNESS_MAX_TILES: f32 = 200.0;

impl PlacementLayer for AnchorScoreLayer {
    fn name(&self) -> &str {
        "anchor_score"
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

        let x = hub.x();
        let y = hub.y();

        let mut new_state = state.clone();

        // 1. Source distance score (weight 3.0)
        let mut source_score_sum = 0.0f32;
        let mut source_count = 0;
        let mut source_dists = Vec::new();
        for (_, dist_map, max_dist) in &analysis.source_distances {
            if *max_dist > 0 {
                if let Some(dist) = dist_map.get(x as usize, y as usize) {
                    let normalized = 1.0 - (*dist as f32 / *max_dist as f32);
                    source_score_sum += normalized;
                    source_count += 1;
                    source_dists.push(*dist as f32);
                }
            }
        }
        let source_distance = if source_count > 0 {
            source_score_sum / source_count as f32
        } else {
            0.0
        };
        new_state.push_score("source_distance", source_distance, 3.0);

        // 2. Source balance (weight 1.5)
        let source_balance = if source_dists.len() > 1 {
            let max_d = source_dists.iter().cloned().fold(0.0f32, f32::max);
            let min_d = source_dists.iter().cloned().fold(f32::INFINITY, f32::min);
            if max_d > 0.0 {
                1.0 - ((max_d - min_d) / max_d)
            } else {
                1.0
            }
        } else {
            1.0
        };
        new_state.push_score("source_balance", source_balance, 1.5);

        // 3. Controller distance (weight 1.0)
        let mut controller_distance = 0.0f32;
        for (_, dist_map, max_dist) in &analysis.controller_distances {
            if *max_dist > 0 {
                if let Some(dist) = dist_map.get(x as usize, y as usize) {
                    controller_distance = 1.0 - (*dist as f32 / *max_dist as f32);
                }
            }
        }
        new_state.push_score("controller_distance", controller_distance, 1.0);

        // 4. Exit proximity (weight 0.5)
        let exit_proximity = if analysis.exit_max_distance > 0 {
            if let Some(dist) = analysis.exit_distances.get(x as usize, y as usize) {
                let ideal_dist = 15.0f32;
                let actual = *dist as f32;
                let deviation = (actual - ideal_dist).abs() / ideal_dist;
                (1.0 - deviation).max(0.0)
            } else {
                0.0
            }
        } else {
            0.0
        };
        new_state.push_score("exit_proximity", exit_proximity, 0.5);

        // 5. Mineral distance (weight 0.3)
        let mut mineral_distance = 0.0f32;
        for (_, dist_map, max_dist) in &analysis.mineral_distances {
            if *max_dist > 0 {
                if let Some(dist) = dist_map.get(x as usize, y as usize) {
                    mineral_distance = 1.0 - (*dist as f32 / *max_dist as f32);
                }
            }
        }
        new_state.push_score("mineral_distance", mineral_distance, 0.3);

        // 6. Terrain openness (weight 1.0)
        // Count buildable (non-wall) tiles within OPENNESS_RADIUS of the hub.
        // More open terrain = more room for extensions near hub = better fill times.
        let terrain_openness = {
            let hx = x as i16;
            let hy = y as i16;
            let mut buildable_count = 0u32;
            for dy in -OPENNESS_RADIUS..=OPENNESS_RADIUS {
                for dx in -OPENNESS_RADIUS..=OPENNESS_RADIUS {
                    let tx = hx + dx;
                    let ty = hy + dy;
                    if tx >= ROOM_BUILD_BORDER as i16
                        && tx < (ROOM_WIDTH - ROOM_BUILD_BORDER) as i16
                        && ty >= ROOM_BUILD_BORDER as i16
                        && ty < (ROOM_HEIGHT - ROOM_BUILD_BORDER) as i16
                        && !terrain.is_wall(tx as u8, ty as u8)
                    {
                        buildable_count += 1;
                    }
                }
            }
            (buildable_count as f32 / OPENNESS_MAX_TILES).min(1.0)
        };
        new_state.push_score("terrain_openness", terrain_openness, 1.0);

        // 7. Defensibility (weight 1.0)
        // Approximate the min-cut size by summing the minimum distance-transform
        // values along the shortest path from each exit cluster to the hub area.
        // Lower distance-transform values along paths = narrower chokepoints =
        // fewer ramparts needed = higher defensibility score.
        let defensibility = {
            let hx = x as i16;
            let hy = y as i16;

            // Sample the minimum distance-transform value in each quadrant
            // between the hub and the room edges. Narrow passages (low dt values)
            // indicate natural chokepoints that reduce rampart count.
            let mut min_dt_sum = 0u32;
            let mut quadrant_count = 0u32;

            // Check 8 radial directions from hub toward edges
            let directions: [(i16, i16); 8] = [
                (0, -1),
                (1, -1),
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
            ];

            for &(ddx, ddy) in &directions {
                let mut min_dt = u8::MAX;
                let mut px = hx;
                let mut py = hy;

                // Walk from hub toward edge, tracking minimum distance transform
                for _step in 0..25 {
                    px += ddx;
                    py += ddy;
                    if !(0..50).contains(&px) || !(0..50).contains(&py) {
                        break;
                    }
                    if terrain.is_wall(px as u8, py as u8) {
                        // Wall = natural barrier, very defensible
                        min_dt = 0;
                        break;
                    }
                    let dt = *analysis.dist_transform.get(px as usize, py as usize);
                    if dt < min_dt {
                        min_dt = dt;
                    }
                }

                if min_dt < u8::MAX {
                    min_dt_sum += min_dt as u32;
                    quadrant_count += 1;
                }
            }

            if quadrant_count > 0 {
                // Lower average min-dt = narrower passages = better defensibility.
                // A fully open room has avg min-dt ~10-15; a well-choked room has ~2-4.
                let avg_min_dt = min_dt_sum as f32 / quadrant_count as f32;
                // Normalize: dt=0 (wall) -> 1.0, dt=10+ -> 0.0
                (1.0 - avg_min_dt / 10.0).max(0.0)
            } else {
                0.5 // neutral if no data
            }
        };
        new_state.push_score("defensibility", defensibility, 1.0);

        Some(Ok(new_state))
    }
}
