use super::{CpuBudget, PhaseResult};
use crate::location::*;
use crate::room_data::*;
use crate::terrain::*;
use serde::{Deserialize, Serialize};

/// Output of the analysis phase -- pre-computed terrain data for subsequent phases.
///
/// Distance arrays use compact u16 serialization (2 bytes/element instead of
/// bincode's default 5 bytes for `Option<u32>`), roughly halving the
/// serialized size of this struct.
#[derive(Clone, Serialize, Deserialize)]
pub struct AnalysisOutput {
    /// Distance transform: for each tile, Chebyshev distance to nearest wall/edge.
    pub dist_transform: RoomDataArray<u8>,
    /// Flood-fill distances from each source to all reachable tiles.
    #[serde(with = "crate::terrain::compact_distance_vec_serde")]
    pub source_distances: Vec<(Location, RoomDataArray<Option<u32>>, u32)>,
    /// Flood-fill distances from each controller to all reachable tiles.
    #[serde(with = "crate::terrain::compact_distance_vec_serde")]
    pub controller_distances: Vec<(Location, RoomDataArray<Option<u32>>, u32)>,
    /// Flood-fill distances from each mineral to all reachable tiles.
    #[serde(with = "crate::terrain::compact_distance_vec_serde")]
    pub mineral_distances: Vec<(Location, RoomDataArray<Option<u32>>, u32)>,
    /// Passable exit tiles grouped by direction.
    pub exits: ExitData,
    /// Flood-fill distance from all exits combined.
    #[serde(with = "crate::terrain::compact_distance_serde")]
    pub exit_distances: RoomDataArray<Option<u32>>,
    pub exit_max_distance: u32,
}

/// Exit tiles grouped by room side.
#[derive(Clone, Serialize, Deserialize)]
pub struct ExitData {
    pub top: Vec<Location>,
    pub right: Vec<Location>,
    pub bottom: Vec<Location>,
    pub left: Vec<Location>,
    pub all: Vec<Location>,
}

/// Phase 1: Analyze terrain, compute distance maps.
#[derive(Clone, Serialize, Deserialize)]
pub struct AnalysisPhase {
    /// Which sub-step we're on (for multi-tick support).
    step: AnalysisStep,
    partial: PartialAnalysis,
}

#[derive(Clone, Serialize, Deserialize)]
enum AnalysisStep {
    DistanceTransform,
    SourceDistances,
    ControllerDistances,
    MineralDistances,
    ExitDistances,
    Complete,
}

#[derive(Clone, Serialize, Deserialize)]
struct PartialAnalysis {
    dist_transform: Option<RoomDataArray<u8>>,
    source_distances: Vec<(Location, RoomDataArray<Option<u32>>, u32)>,
    controller_distances: Vec<(Location, RoomDataArray<Option<u32>>, u32)>,
    mineral_distances: Vec<(Location, RoomDataArray<Option<u32>>, u32)>,
    exits: Option<ExitData>,
    exit_distances: Option<RoomDataArray<Option<u32>>>,
    exit_max_distance: u32,
    /// Track which source index we've computed so far (for incremental).
    source_index: usize,
    controller_index: usize,
    mineral_index: usize,
}

impl Default for AnalysisPhase {
    fn default() -> Self {
        Self::new()
    }
}

impl AnalysisPhase {
    pub fn new() -> Self {
        AnalysisPhase {
            step: AnalysisStep::DistanceTransform,
            partial: PartialAnalysis {
                dist_transform: None,
                source_distances: Vec::new(),
                controller_distances: Vec::new(),
                mineral_distances: Vec::new(),
                exits: None,
                exit_distances: None,
                exit_max_distance: 0,
                source_index: 0,
                controller_index: 0,
                mineral_index: 0,
            },
        }
    }

    pub fn tick(
        &mut self,
        data_source: &dyn PlannerRoomDataSource,
        budget: &CpuBudget,
    ) -> PhaseResult<AnalysisOutput> {
        let terrain = data_source.get_terrain();

        loop {
            match &self.step {
                AnalysisStep::DistanceTransform => {
                    self.partial.dist_transform = Some(distance_transform(terrain));

                    // Compute exits
                    let all_exits = terrain.get_exits();
                    let mut top = Vec::new();
                    let mut right = Vec::new();
                    let mut bottom = Vec::new();
                    let mut left = Vec::new();
                    for exit in &all_exits {
                        if exit.y() == 0 {
                            top.push(*exit);
                        }
                        if exit.x() == 49 {
                            right.push(*exit);
                        }
                        if exit.y() == 49 {
                            bottom.push(*exit);
                        }
                        if exit.x() == 0 {
                            left.push(*exit);
                        }
                    }
                    self.partial.exits = Some(ExitData {
                        top,
                        right,
                        bottom,
                        left,
                        all: all_exits,
                    });

                    self.step = AnalysisStep::SourceDistances;
                    if !budget.has_budget() {
                        return PhaseResult::Running;
                    }
                }
                AnalysisStep::SourceDistances => {
                    let sources = data_source.get_sources();
                    while self.partial.source_index < sources.len() {
                        let source = sources[self.partial.source_index];
                        if let Some(loc) = source.as_location() {
                            let (dist_map, max_dist) = flood_fill_distance(terrain, &[loc]);
                            self.partial
                                .source_distances
                                .push((loc, dist_map, max_dist));
                        }
                        self.partial.source_index += 1;
                        if !budget.has_budget() {
                            return PhaseResult::Running;
                        }
                    }
                    self.step = AnalysisStep::ControllerDistances;
                    if !budget.has_budget() {
                        return PhaseResult::Running;
                    }
                }
                AnalysisStep::ControllerDistances => {
                    let controllers = data_source.get_controllers();
                    while self.partial.controller_index < controllers.len() {
                        let ctrl = controllers[self.partial.controller_index];
                        if let Some(loc) = ctrl.as_location() {
                            let (dist_map, max_dist) = flood_fill_distance(terrain, &[loc]);
                            self.partial
                                .controller_distances
                                .push((loc, dist_map, max_dist));
                        }
                        self.partial.controller_index += 1;
                        if !budget.has_budget() {
                            return PhaseResult::Running;
                        }
                    }
                    self.step = AnalysisStep::MineralDistances;
                    if !budget.has_budget() {
                        return PhaseResult::Running;
                    }
                }
                AnalysisStep::MineralDistances => {
                    let minerals = data_source.get_minerals();
                    while self.partial.mineral_index < minerals.len() {
                        let mineral = minerals[self.partial.mineral_index];
                        if let Some(loc) = mineral.as_location() {
                            let (dist_map, max_dist) = flood_fill_distance(terrain, &[loc]);
                            self.partial
                                .mineral_distances
                                .push((loc, dist_map, max_dist));
                        }
                        self.partial.mineral_index += 1;
                        if !budget.has_budget() {
                            return PhaseResult::Running;
                        }
                    }
                    self.step = AnalysisStep::ExitDistances;
                    if !budget.has_budget() {
                        return PhaseResult::Running;
                    }
                }
                AnalysisStep::ExitDistances => {
                    let exits = self.partial.exits.as_ref().unwrap();
                    let (exit_dist, exit_max) = flood_fill_distance(terrain, &exits.all);
                    self.partial.exit_distances = Some(exit_dist);
                    self.partial.exit_max_distance = exit_max;
                    self.step = AnalysisStep::Complete;
                }
                AnalysisStep::Complete => {
                    let output = AnalysisOutput {
                        dist_transform: self.partial.dist_transform.take().unwrap(),
                        source_distances: std::mem::take(&mut self.partial.source_distances),
                        controller_distances: std::mem::take(
                            &mut self.partial.controller_distances,
                        ),
                        mineral_distances: std::mem::take(&mut self.partial.mineral_distances),
                        exits: self.partial.exits.take().unwrap(),
                        exit_distances: self.partial.exit_distances.take().unwrap(),
                        exit_max_distance: self.partial.exit_max_distance,
                    };
                    return PhaseResult::Complete(output);
                }
            }
        }
    }
}
