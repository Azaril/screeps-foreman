//! StampLayer: Generic layer that places a stamp at positions near a landmark.
//! Supports optional placements -- stamps with `required: false` entries will
//! have those entries skipped if they don't fit.

use crate::layer::*;
use crate::location::*;
use crate::stamps::Stamp;
use crate::terrain::*;

use screeps::constants::StructureType;

/// A generic layer that places one of several stamp variants at positions
/// relative to a named landmark.
pub struct StampLayer {
    /// Name for fingerprinting / debugging.
    pub layer_name: &'static str,
    /// Landmark to read as the anchor position.
    pub anchor_landmark: &'static str,
    /// Stamps to try (all variants including rotations).
    pub stamps: Vec<Stamp>,
    /// Max Chebyshev distance from anchor to try placing.
    /// 0 means place exactly at the anchor position.
    pub search_radius: u8,
    /// Map of StructureType -> landmark set name to populate on placement.
    pub landmark_mappings: Vec<(StructureType, &'static str)>,
}

impl StampLayer {
    /// The grid side length for the search area: `2 * radius + 1`.
    fn grid_side(&self) -> usize {
        self.search_radius as usize * 2 + 1
    }

    /// Total number of (stamp, offset) permutations.
    fn total_candidates(&self) -> usize {
        let side = self.grid_side();
        self.stamps.len() * side * side
    }

    /// Decompose a flat index into (stamp_index, dx_offset, dy_offset)
    /// where dx/dy are offsets from -radius to +radius.
    fn decompose_index(&self, index: usize) -> (usize, i16, i16) {
        let side = self.grid_side();
        let positions_per_stamp = side * side;
        let stamp_index = index / positions_per_stamp;
        let pos_index = index % positions_per_stamp;
        let dy = (pos_index / side) as i16 - self.search_radius as i16;
        let dx = (pos_index % side) as i16 - self.search_radius as i16;
        (stamp_index, dx, dy)
    }
}

impl PlacementLayer for StampLayer {
    fn name(&self) -> &str {
        self.layer_name
    }

    fn candidate_count(
        &self,
        _state: &PlacementState,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(self.total_candidates())
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index >= self.total_candidates() {
            return None;
        }

        let anchor = match state.get_landmark(self.anchor_landmark) {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let (stamp_index, dx, dy) = self.decompose_index(index);
        let stamp = &self.stamps[stamp_index];

        let x = anchor.x() as i16 + dx;
        let y = anchor.y() as i16 + dy;

        if !(2..48).contains(&x) || !(2..48).contains(&y) {
            return Some(Err(()));
        }

        let ux = x as u8;
        let uy = y as u8;

        // Check if required placements fit on terrain
        if !stamp.fits_at(ux, uy, terrain) {
            return Some(Err(()));
        }

        // Get filtered placements (required + optional that fit)
        let placements = stamp.place_at_filtered(ux, uy, terrain, &state.structures);

        // Check no overlap with existing structures (any type, including roads)
        let has_overlap = placements.iter().any(|(px, py, _st, _)| {
            state.has_any_structure(*px, *py)
        });
        if has_overlap {
            return Some(Err(()));
        }

        let mut new_state = state.clone();
        for (px, py, st, rcl) in &placements {
            new_state.place_structure(*px, *py, *st, *rcl);

            // Populate landmark sets based on mappings
            for (mapped_type, landmark_name) in &self.landmark_mappings {
                if st == mapped_type {
                    new_state.add_to_landmark_set(
                        *landmark_name,
                        Location::from_coords(*px as u32, *py as u32),
                    );
                }
            }
        }

        Some(Ok(new_state))
    }
}

/// Create a StampLayer that places the hub stamp exactly at the "hub" landmark.
pub fn hub_stamp_layer() -> StampLayer {
    use crate::stamps::hub::hub_stamps;

    let stamps = hub_stamps();
    debug_assert!(
        stamps.iter().all(|s| s.validate()),
        "hub stamps failed validation"
    );

    StampLayer {
        layer_name: "hub_stamp",
        anchor_landmark: "hub",
        stamps,
        search_radius: 0,
        landmark_mappings: vec![(StructureType::Spawn, "spawns")],
    }
}

/// Create a StampLayer that places the lab stamp near the "hub" landmark.
pub fn lab_stamp_layer() -> GreedyStampLayer {
    use crate::stamps::lab::{lab_stamps, validate_lab_stamp};

    let stamps: Vec<_> = lab_stamps()
        .into_iter()
        .filter(validate_lab_stamp)
        .collect();
    debug_assert!(
        stamps.iter().all(|s| s.validate()),
        "lab stamps failed validation"
    );

    GreedyStampLayer {
        layer_name: "lab_stamp",
        anchor_landmark: "hub",
        stamps,
        search_radius: 8,
        landmark_mappings: vec![(StructureType::Lab, "labs")],
    }
}

/// A deterministic stamp layer that picks the single best placement
/// (closest to anchor, first stamp variant that fits). Produces exactly
/// 1 candidate instead of branching over all valid positions.
pub struct GreedyStampLayer {
    pub layer_name: &'static str,
    pub anchor_landmark: &'static str,
    pub stamps: Vec<Stamp>,
    pub search_radius: u8,
    pub landmark_mappings: Vec<(StructureType, &'static str)>,
}

impl PlacementLayer for GreedyStampLayer {
    fn name(&self) -> &str {
        self.layer_name
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

        let anchor = match state.get_landmark(self.anchor_landmark) {
            Some(loc) => loc,
            None => return Some(Err(())),
        };
        let ax = anchor.x() as i16;
        let ay = anchor.y() as i16;

        // Iterate by increasing distance from anchor (spiral outward).
        // For each distance, try all stamps and all offsets at that distance.
        // Return the first valid placement found (closest to anchor).
        for dist in 0..=self.search_radius as i16 {
            for dy in -dist..=dist {
                for dx in -dist..=dist {
                    if dx.abs().max(dy.abs()) != dist {
                        continue;
                    }
                    let x = ax + dx;
                    let y = ay + dy;
                    if !(2..48).contains(&x) || !(2..48).contains(&y) {
                        continue;
                    }
                    let ux = x as u8;
                    let uy = y as u8;

                    for stamp in &self.stamps {
                        if !stamp.fits_at(ux, uy, terrain) {
                            continue;
                        }

                        let placements =
                            stamp.place_at_filtered(ux, uy, terrain, &state.structures);

                        let has_overlap = placements.iter().any(|(px, py, _st, _)| {
                            state.has_any_structure(*px, *py)
                        });
                        if has_overlap {
                            continue;
                        }

                        // Found a valid placement -- apply it
                        let mut new_state = state.clone();
                        for (px, py, st, rcl) in &placements {
                            new_state.place_structure(*px, *py, *st, *rcl);
                            for (mapped_type, landmark_name) in &self.landmark_mappings {
                                if st == mapped_type {
                                    new_state.add_to_landmark_set(
                                        *landmark_name,
                                        Location::from_coords(*px as u32, *py as u32),
                                    );
                                }
                            }
                        }
                        return Some(Ok(new_state));
                    }
                }
            }
        }

        // No valid placement found
        Some(Err(()))
    }
}
