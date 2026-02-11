//! AnchorScoreLayer: Scoring-only layer that evaluates the chosen anchor position.
//! Pushes ScoreEntry values for source distance, source balance, controller distance,
//! exit proximity, openness, and mineral distance. Enables early pruning of bad anchors.

use crate::layer::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;

/// Scoring-only layer that evaluates the anchor/hub position.
/// No structures are placed; only ScoreEntry values are pushed.
pub struct AnchorScoreLayer;

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
        _terrain: &FastRoomTerrain,
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

        Some(Ok(new_state))
    }
}
