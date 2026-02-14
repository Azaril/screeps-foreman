//! StampLayer: Generic layer that places a stamp at positions near a landmark.
//! Supports optional placements -- stamps with `required: false` entries will
//! have those entries skipped if they don't fit.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
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
        _analysis: &AnalysisOutput,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(self.total_candidates())
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        _analysis: &AnalysisOutput,
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

        let border = ROOM_BUILD_BORDER as i16;
        if !(border..ROOM_WIDTH as i16 - border).contains(&x)
            || !(border..ROOM_HEIGHT as i16 - border).contains(&y)
        {
            return Some(Err(()));
        }

        let ux = x as u8;
        let uy = y as u8;

        // Check if required placements fit on terrain and exclusions
        if !stamp.fits_at(ux, uy, terrain, &state.excluded) {
            return Some(Err(()));
        }

        // Get filtered placements (required + optional that fit)
        let placements =
            stamp.place_at_filtered(ux, uy, terrain, &state.structures, &state.excluded);

        // Check no overlap with existing structures (any type, including roads)
        let has_overlap = placements.iter().any(|(px, py, _st, _)| {
            let loc = Location::from_xy(*px, *py);
            state.has_any_structure(loc)
        });
        if has_overlap {
            return Some(Err(()));
        }

        let mut new_state = state.clone();
        for (px, py, st, rcl) in &placements {
            let loc = Location::from_xy(*px, *py);
            match rcl {
                Some(r) => new_state.place_structure(loc, *st, *r),
                None => new_state.place_structure_auto_rcl(loc, *st),
            }

            // Populate landmark sets based on mappings
            for (mapped_type, landmark_name) in &self.landmark_mappings {
                if st == mapped_type {
                    new_state.add_to_landmark_set(*landmark_name, loc);
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
        landmark_mappings: vec![
            (StructureType::Spawn, "spawns"),
            (StructureType::Extension, "hub_extensions"),
        ],
    }
}

/// Create a StampLayer that places the lab stamp near the "hub" landmark.
///
/// Uses `ScoredStampLayer` which collects all valid placements within the
/// search radius and picks the one with the best composite score (distance
/// to hub + road adjacency + open staging area).
pub fn lab_stamp_layer() -> ScoredStampLayer {
    use crate::stamps::lab::{lab_stamps, validate_lab_stamp};

    let stamps: Vec<_> = lab_stamps()
        .into_iter()
        .filter(validate_lab_stamp)
        .collect();
    debug_assert!(
        stamps.iter().all(|s| s.validate()),
        "lab stamps failed validation"
    );

    ScoredStampLayer {
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
                    let border = ROOM_BUILD_BORDER as i16;
                    if !(border..ROOM_WIDTH as i16 - border).contains(&x)
                        || !(border..ROOM_HEIGHT as i16 - border).contains(&y)
                    {
                        continue;
                    }
                    let ux = x as u8;
                    let uy = y as u8;

                    for stamp in &self.stamps {
                        if !stamp.fits_at(ux, uy, terrain, &state.excluded) {
                            continue;
                        }

                        let placements = stamp.place_at_filtered(
                            ux,
                            uy,
                            terrain,
                            &state.structures,
                            &state.excluded,
                        );

                        let has_overlap = placements.iter().any(|(px, py, _st, _)| {
                            let loc = Location::from_xy(*px, *py);
                            state.has_any_structure(loc)
                        });
                        if has_overlap {
                            continue;
                        }

                        // Found a valid placement -- apply it
                        let mut new_state = state.clone();
                        for (px, py, st, rcl) in &placements {
                            let loc = Location::from_xy(*px, *py);
                            match rcl {
                                Some(r) => new_state.place_structure(loc, *st, *r),
                                None => new_state.place_structure_auto_rcl(loc, *st),
                            }
                            for (mapped_type, landmark_name) in &self.landmark_mappings {
                                if st == mapped_type {
                                    new_state.add_to_landmark_set(*landmark_name, loc);
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

/// A deterministic stamp layer that evaluates all valid placements within the
/// search radius and picks the one with the best composite score. Unlike
/// `GreedyStampLayer` (which takes the first valid placement closest to anchor),
/// this layer considers road adjacency and open staging area around the stamp.
///
/// Score components:
/// 1. **Distance** (weight 3.0): Chebyshev distance from anchor, normalized.
///    Closer is better.
/// 2. **Road adjacency** (weight 2.0): Number of non-road structure tiles in
///    the stamp that have an adjacent road tile (from existing roads or stamp
///    roads). More road-adjacent structures = better creep access.
/// 3. **Open staging area** (weight 1.0): Number of open (walkable, non-occupied)
///    tiles adjacent to the stamp footprint. More open space = less congestion
///    for creeps loading/unloading.
pub struct ScoredStampLayer {
    pub layer_name: &'static str,
    pub anchor_landmark: &'static str,
    pub stamps: Vec<Stamp>,
    pub search_radius: u8,
    pub landmark_mappings: Vec<(StructureType, &'static str)>,
}

impl PlacementLayer for ScoredStampLayer {
    fn name(&self) -> &str {
        self.layer_name
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

        let anchor = match state.get_landmark(self.anchor_landmark) {
            Some(loc) => loc,
            None => return Some(Err(())),
        };
        let ax = anchor.x() as i16;
        let ay = anchor.y() as i16;

        type PlacementList = Vec<(u8, u8, StructureType, Option<u8>)>;

        // Collect all valid placements with their scores.
        let mut best_score = f32::NEG_INFINITY;
        let mut best_placements: Option<PlacementList> = None;

        for dist in 0..=self.search_radius as i16 {
            for dy in -dist..=dist {
                for dx in -dist..=dist {
                    if dx.abs().max(dy.abs()) != dist {
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
                    let ux = x as u8;
                    let uy = y as u8;

                    for stamp in &self.stamps {
                        if !stamp.fits_at(ux, uy, terrain, &state.excluded) {
                            continue;
                        }

                        let placements = stamp.place_at_filtered(
                            ux,
                            uy,
                            terrain,
                            &state.structures,
                            &state.excluded,
                        );

                        let has_overlap = placements.iter().any(|(px, py, _st, _)| {
                            let loc = Location::from_xy(*px, *py);
                            state.has_any_structure(loc)
                        });
                        if has_overlap {
                            continue;
                        }

                        // Score this placement
                        let score = score_placement(
                            &placements,
                            dist,
                            self.search_radius as i16,
                            state,
                            terrain,
                        );

                        if score > best_score {
                            best_score = score;
                            best_placements = Some(placements);
                        }
                    }
                }
            }
        }

        // Apply the best placement
        if let Some(placements) = best_placements {
            let mut new_state = state.clone();
            for (px, py, st, rcl) in &placements {
                let loc = Location::from_xy(*px, *py);
                match rcl {
                    Some(r) => new_state.place_structure(loc, *st, *r),
                    None => new_state.place_structure_auto_rcl(loc, *st),
                }
                for (mapped_type, landmark_name) in &self.landmark_mappings {
                    if st == mapped_type {
                        new_state.add_to_landmark_set(*landmark_name, loc);
                    }
                }
            }
            Some(Ok(new_state))
        } else {
            Some(Err(()))
        }
    }
}

/// Score a stamp placement based on distance, road adjacency, and open staging area.
fn score_placement(
    placements: &[(u8, u8, StructureType, Option<u8>)],
    dist: i16,
    max_dist: i16,
    state: &PlacementState,
    terrain: &FastRoomTerrain,
) -> f32 {
    // 1. Distance score: closer to anchor is better
    let distance_score = if max_dist > 0 {
        1.0 - (dist as f32 / max_dist as f32)
    } else {
        1.0
    };

    // Build sets of road tiles and non-road structure tiles from this stamp
    let mut stamp_roads: fnv::FnvHashSet<Location> = fnv::FnvHashSet::default();
    let mut stamp_structures: Vec<Location> = Vec::new();
    let mut stamp_footprint: fnv::FnvHashSet<Location> = fnv::FnvHashSet::default();

    for &(px, py, st, _) in placements {
        let loc = Location::from_xy(px, py);
        stamp_footprint.insert(loc);
        if st == StructureType::Road {
            stamp_roads.insert(loc);
        } else {
            stamp_structures.push(loc);
        }
    }

    // 2. Road adjacency: count non-road structures adjacent to a road
    let mut road_adjacent_count = 0u32;
    for &sloc in &stamp_structures {
        let has_adjacent_road = NEIGHBORS_8.iter().any(|&(ddx, ddy)| {
            let nloc = match sloc.checked_add(ddx, ddy) {
                Some(l) => l,
                None => return false,
            };
            // Check stamp roads or existing roads in state
            if stamp_roads.contains(&nloc) {
                return true;
            }
            state
                .structures
                .get(&nloc)
                .map(|items| {
                    items
                        .iter()
                        .any(|i| i.structure_type == StructureType::Road)
                })
                .unwrap_or(false)
        });
        if has_adjacent_road {
            road_adjacent_count += 1;
        }
    }
    let road_adjacency_score = if !stamp_structures.is_empty() {
        road_adjacent_count as f32 / stamp_structures.len() as f32
    } else {
        0.0
    };

    // 3. Open staging area: count open tiles adjacent to the stamp footprint
    let mut open_count = 0u32;
    let mut checked: fnv::FnvHashSet<Location> = fnv::FnvHashSet::default();
    for &floc in &stamp_footprint {
        for &(ddx, ddy) in &NEIGHBORS_8 {
            let nloc = match floc.checked_add(ddx, ddy) {
                Some(l) if l.is_interior() => l,
                _ => continue,
            };
            if stamp_footprint.contains(&nloc) {
                continue;
            }
            if !checked.insert(nloc) {
                continue;
            }
            if !terrain.is_wall_at(nloc) && !state.is_occupied(nloc) {
                open_count += 1;
            }
        }
    }
    // Normalize: ~20 open tiles around a lab cluster is good
    let open_score = (open_count as f32 / 20.0).min(1.0);

    // Composite score
    distance_score * 3.0 + road_adjacency_score * 2.0 + open_score * 1.0
}
