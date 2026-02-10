//! Core types for the search tree architecture.
//!
//! `PlacementState` is the evolving plan state passed through the search tree.
//! `PlacementLayer` is the trait that each layer in the search tree implements.
//! `ScoreEntry` records a weighted score from a layer for pruning and final scoring.

use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::plan::*;
use crate::terrain::*;
use fnv::{FnvHashMap, FnvHashSet};
use serde::{Deserialize, Serialize};
use std::sync::Arc;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// A weighted score entry pushed by layers during candidate generation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ScoreEntry {
    pub name: String,
    pub score: f32,
    pub weight: f32,
}

/// The evolving plan state passed through the search tree.
/// Each layer reads this and produces modified versions (one per candidate).
#[derive(Clone, Serialize, Deserialize)]
pub struct PlacementState {
    /// Pre-computed terrain analysis data.
    pub analysis: AnalysisOutput,
    /// Structure placements by location.
    pub structures: FnvHashMap<Location, Vec<RoomItem>>,
    /// Tiles occupied by non-road structures (for collision detection).
    pub occupied: FnvHashSet<Location>,
    /// Named locations for cross-layer reference (e.g. "hub", "storage").
    pub landmarks: FnvHashMap<String, Location>,
    /// Named location sets (e.g. "towers", "spawns", "upgrade_area").
    pub landmark_sets: FnvHashMap<String, Vec<Location>>,
    /// Accumulated weighted scores from layers that have run so far.
    pub scores: Vec<ScoreEntry>,
    /// Cached hub flood-fill distances (terrain-only, ignoring structures).
    /// Computed lazily after the hub landmark is set. Shared via Arc to
    /// avoid expensive clones during search tree expansion.
    #[serde(skip)]
    hub_flood_fill: Option<Arc<RoomDataArray<Option<u32>>>>,
}

impl PlacementState {
    /// Create a new PlacementState from analysis output.
    pub fn from_analysis(analysis: AnalysisOutput) -> Self {
        PlacementState {
            analysis,
            structures: FnvHashMap::default(),
            occupied: FnvHashSet::default(),
            landmarks: FnvHashMap::default(),
            landmark_sets: FnvHashMap::default(),
            scores: Vec::new(),
            hub_flood_fill: None,
        }
    }

    /// Cumulative weighted score so far.
    pub fn cumulative_score(&self) -> f32 {
        let total_weight: f32 = self.scores.iter().map(|s| s.weight).sum();
        if total_weight > 0.0 {
            self.scores.iter().map(|s| s.score * s.weight).sum::<f32>() / total_weight
        } else {
            0.0
        }
    }

    /// Push a score entry.
    pub fn push_score(&mut self, name: impl Into<String>, score: f32, weight: f32) {
        self.scores.push(ScoreEntry {
            name: name.into(),
            score,
            weight,
        });
    }

    /// Place a structure at the given coordinates.
    pub fn place_structure(&mut self, x: u8, y: u8, structure_type: StructureType, rcl: u8) {
        let loc = Location::from_coords(x as u32, y as u32);
        self.structures
            .entry(loc)
            .or_default()
            .push(RoomItem::new(structure_type, rcl));
        if structure_type != StructureType::Road {
            self.occupied.insert(loc);
        }
    }

    /// Check if a tile is occupied by a non-road structure.
    pub fn is_occupied(&self, x: u8, y: u8) -> bool {
        self.occupied
            .contains(&Location::from_coords(x as u32, y as u32))
    }

    /// Check if a tile has any structure placed on it (including roads).
    pub fn has_any_structure(&self, x: u8, y: u8) -> bool {
        self.structures
            .contains_key(&Location::from_coords(x as u32, y as u32))
    }

    /// Set a named landmark location.
    pub fn set_landmark(&mut self, name: impl Into<String>, loc: Location) {
        self.landmarks.insert(name.into(), loc);
    }

    /// Get a named landmark location.
    pub fn get_landmark(&self, name: &str) -> Option<Location> {
        self.landmarks.get(name).copied()
    }

    /// Add a location to a named landmark set.
    pub fn add_to_landmark_set(&mut self, name: impl Into<String>, loc: Location) {
        self.landmark_sets
            .entry(name.into())
            .or_default()
            .push(loc);
    }

    /// Get a named landmark set.
    pub fn get_landmark_set(&self, name: &str) -> &[Location] {
        self.landmark_sets
            .get(name)
            .map(|v| v.as_slice())
            .unwrap_or(&[])
    }

    /// Get the hub flood-fill distance map, computing it if not yet cached.
    /// Returns `None` if the hub landmark has not been set yet.
    /// The flood-fill uses terrain-only distances (ignores placed structures)
    /// so it remains valid as structures are added to the plan.
    pub fn hub_distances(&mut self, terrain: &FastRoomTerrain) -> Option<&RoomDataArray<Option<u32>>> {
        if self.hub_flood_fill.is_none() {
            if let Some(hub) = self.get_landmark("hub") {
                let (dist_map, _max) = flood_fill_distance(terrain, &[hub]);
                self.hub_flood_fill = Some(Arc::new(dist_map));
            }
        }
        self.hub_flood_fill.as_ref().map(|arc| arc.as_ref())
    }

    /// Get the hub flood-fill distance for a specific location.
    /// Returns `None` if hub is not set or the location is unreachable.
    pub fn hub_distance_to(&mut self, terrain: &FastRoomTerrain, loc: Location) -> Option<u32> {
        self.hub_distances(terrain)
            .and_then(|dists| *dists.get(loc.x() as usize, loc.y() as usize))
    }

    /// Structure-aware hub distance map.
    ///
    /// Unlike `hub_distances` (which only respects terrain walls), this method
    /// accounts for placed structures:
    ///   - **Passable:** roads, containers, ramparts (walkable in Screeps)
    ///   - **Blocked:** everything else (extensions, spawns, towers, walls, etc.)
    ///
    /// This gives actual pathing distance around the built layout.
    ///
    /// **Not cached** -- the result depends on the current structure layout
    /// which changes as layers run.  Compute on-demand where needed.
    pub fn hub_pathing_distances(
        &self,
        terrain: &FastRoomTerrain,
    ) -> Option<RoomDataArray<Option<u32>>> {
        let hub = self.get_landmark("hub")?;
        let structures = &self.structures;
        let (dist_map, _) = flood_fill_distance_with_obstacles(terrain, &[hub], |x, y| {
            let loc = Location::from_coords(x as u32, y as u32);
            match structures.get(&loc) {
                None => true, // no structure = passable
                Some(items) => items.iter().all(|i| {
                    matches!(
                        i.structure_type,
                        StructureType::Road | StructureType::Container | StructureType::Rampart
                    )
                }),
            }
        });
        Some(dist_map)
    }

    /// Build a PlanScore from the accumulated ScoreEntry values.
    pub fn to_plan_score(&self) -> PlanScore {
        let mut score = PlanScore::default();
        let mut total_weighted = 0.0f32;
        let mut total_weight = 0.0f32;

        for entry in &self.scores {
            total_weighted += entry.score * entry.weight;
            total_weight += entry.weight;

            match entry.name.as_str() {
                "source_distance" => score.source_distance = entry.score,
                "source_balance" => score.source_balance = entry.score,
                "controller_distance" => score.controller_distance = entry.score,
                "mineral_distance" => score.mineral_distance = entry.score,
                "exit_proximity" => score.exit_proximity = entry.score,
                "upkeep_cost" => score.upkeep_cost = entry.score,
                "tower_coverage" => score.tower_coverage = entry.score,
                "extension_efficiency" => score.extension_efficiency = entry.score,
                "hub_quality" => score.hub_quality = entry.score,
                "upgrade_area_quality" => score.upgrade_area_quality = entry.score,
                "traffic_congestion" => score.traffic_congestion = entry.score,
                _ => {}
            }
        }

        score.total = if total_weight > 0.0 {
            total_weighted / total_weight
        } else {
            0.0
        };

        score
    }
}

/// A placement layer generates candidate placements for the search tree.
///
/// Layers are stateless -- all mutable state lives in `PlacementState`.
/// Layers don't need to be serializable; only the search state does.
pub trait PlacementLayer {
    /// Human-readable name for debugging/profiling and fingerprinting.
    fn name(&self) -> &str;

    /// Return the number of candidates this layer will produce for the
    /// given state, or None if the count is expensive to compute.
    fn candidate_count(
        &self,
        _state: &PlacementState,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        None
    }

    /// Generate the candidate at the given index.
    ///
    /// Returns `None` if index is out of range (signals end of candidates).
    /// Returns `Some(Ok(state))` for a valid candidate with placements/scores applied.
    /// Returns `Some(Err(()))` to reject this candidate as structurally invalid.
    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>>;

    /// Quick check before expanding candidates.
    /// Return false to skip this layer (state passes through unchanged).
    fn is_applicable(&self, _state: &PlacementState) -> bool {
        true
    }
}
