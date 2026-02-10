//! Public API for the room planner.
//!
//! The `PlannerBuilder` provides a fluent, append-only API for configuring the
//! planning process. Layers are added in order and the builder produces a
//! `PlanningState` that can be ticked incrementally or run to completion.

use crate::layer::PlacementLayer;
use crate::layers::default_layers;
use crate::pipeline::{CpuBudget, PlanningState};
use crate::room_data::PlannerRoomDataSource;
use crate::search::compute_fingerprint;

// Re-export key types for convenience
pub use crate::pipeline::PlanningState as PlanState;
pub use crate::plan::{
    AllowAllFilter, BuildPriority, ExistingStructure, ExecutionFilter, Plan, PlanOperation,
    RclSubstitution,
};

#[cfg(not(feature = "shim"))]
pub use crate::plan::{execute_operations, snapshot_structures};
pub use crate::room_data::PlanLocation;
pub use crate::terrain::{FastRoomTerrain, TerrainFlags};

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Result of a planning tick.
pub enum PlanResult {
    /// Planning is still in progress.
    Running(PlanningState),
    /// Planning completed successfully.
    Complete(Plan),
    /// Planning failed.
    Failed(String),
}

/// Append-only builder for configuring the planning process.
pub struct PlannerBuilder {
    layers: Vec<Box<dyn PlacementLayer>>,
    prune_margin: f32,
}

impl PlannerBuilder {
    /// Start with an empty layer stack.
    pub fn new() -> Self {
        PlannerBuilder {
            layers: Vec::new(),
            prune_margin: 0.1,
        }
    }

    /// Append a layer to the end of the stack.
    pub fn add_layer(mut self, layer: Box<dyn PlacementLayer>) -> Self {
        self.layers.push(layer);
        self
    }

    /// Set the pruning margin (default: 0.1).
    /// Candidates whose cumulative score is more than this amount below
    /// the best complete score are pruned.
    pub fn prune_margin(mut self, margin: f32) -> Self {
        self.prune_margin = margin;
        self
    }

    /// Compute a fingerprint (hash of layer names) for cross-tick validation.
    pub fn fingerprint(&self) -> u64 {
        compute_fingerprint(&self.layers)
    }

    /// Build the search engine and start planning (begins with Analysis phase).
    pub fn build(self) -> PlanningState {
        PlanningState::Analysis {
            phase: crate::pipeline::analysis::AnalysisPhase::new(),
            layers: self.layers,
            prune_margin: self.prune_margin,
        }
    }

    /// Provide layers to a resumed PlanningState for the current tick.
    /// Validates the fingerprint matches; returns Err if mismatched
    /// (caller should restart planning).
    #[allow(clippy::result_large_err)]
    pub fn resume(self, state: PlanningState) -> Result<PlanningState, PlanningState> {
        match state {
            PlanningState::Analysis { phase, .. } => {
                // Analysis phase doesn't need layers yet; just update them
                Ok(PlanningState::Analysis {
                    phase,
                    layers: self.layers,
                    prune_margin: self.prune_margin,
                })
            }
            PlanningState::Searching(mut engine) => {
                if engine.inject_layers(self.layers) {
                    Ok(PlanningState::Searching(engine))
                } else {
                    // Fingerprint mismatch -- signal restart
                    Err(PlanningState::Searching(engine))
                }
            }
            PlanningState::Finalizing(phase) => {
                // Finalizing doesn't need layers
                Ok(PlanningState::Finalizing(phase))
            }
            other => Ok(other),
        }
    }
}

impl Default for PlannerBuilder {
    /// Returns a builder pre-loaded with the default 13-layer stack.
    fn default() -> Self {
        let mut builder = PlannerBuilder::new();
        for layer in default_layers() {
            builder.layers.push(layer);
        }
        builder
    }
}

/// Start a new planning pipeline with default layers.
pub fn start_planning() -> PlanningState {
    PlannerBuilder::default().build()
}

/// Run one tick of planning with the given CPU budget.
pub fn tick_planning(
    state: PlanningState,
    data_source: &dyn PlannerRoomDataSource,
    budget: &CpuBudget,
) -> PlanResult {
    let new_state = crate::pipeline::tick_pipeline(state, data_source, budget);
    match new_state {
        PlanningState::Complete(plan) => PlanResult::Complete(plan),
        PlanningState::Failed(msg) => PlanResult::Failed(msg),
        other => PlanResult::Running(other),
    }
}

/// Run planning to completion (for offline/bench use).
pub fn plan_room(data_source: &dyn PlannerRoomDataSource) -> Result<Plan, String> {
    let budget = CpuBudget::unlimited();
    let mut state = start_planning();

    loop {
        state = crate::pipeline::tick_pipeline(state, data_source, &budget);
        match state {
            PlanningState::Complete(plan) => return Ok(plan),
            PlanningState::Failed(msg) => return Err(msg),
            _ => continue,
        }
    }
}

/// Get the build priority for a structure type at a given RCL.
pub fn get_build_priority(structure_type: StructureType, rcl: u32) -> BuildPriority {
    match structure_type {
        StructureType::Spawn => BuildPriority::Critical,
        StructureType::Extension => {
            if rcl <= 2 {
                BuildPriority::Critical
            } else {
                BuildPriority::Medium
            }
        }
        StructureType::Storage => BuildPriority::Critical,
        StructureType::Container => BuildPriority::High,
        StructureType::Tower => BuildPriority::Critical,
        StructureType::Terminal => BuildPriority::High,
        StructureType::Link => BuildPriority::High,
        StructureType::Lab => BuildPriority::Medium,
        StructureType::Extractor => BuildPriority::Medium,
        StructureType::Factory => BuildPriority::Medium,
        StructureType::Observer => BuildPriority::Low,
        StructureType::PowerSpawn => BuildPriority::Medium,
        StructureType::Nuker => BuildPriority::Low,
        StructureType::Wall => BuildPriority::Low,
        StructureType::Rampart => BuildPriority::Low,
        StructureType::Road => BuildPriority::VeryLow,
        _ => BuildPriority::Medium,
    }
}

/// Run planning with a time limit (for bench use).
pub fn plan_room_with_timeout<F>(
    data_source: &dyn PlannerRoomDataSource,
    should_continue: F,
) -> Result<Option<Plan>, String>
where
    F: Fn() -> bool + 'static,
{
    let budget = CpuBudget::new(should_continue);
    let mut state = start_planning();

    loop {
        state = crate::pipeline::tick_pipeline(state, data_source, &budget);
        match state {
            PlanningState::Complete(plan) => return Ok(Some(plan)),
            PlanningState::Failed(msg) => return Err(msg),
            _ => {
                if !budget.has_budget() {
                    return Ok(None);
                }
            }
        }
    }
}
