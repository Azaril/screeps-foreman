pub mod analysis;
pub mod finalize;

use crate::layer::*;
use crate::plan::*;
use crate::room_data::*;
use crate::search::*;
use serde::{Deserialize, Serialize};

/// CPU budget for incremental planning.
pub struct CpuBudget {
    /// Function that returns true if the planner should continue working.
    should_continue: Box<dyn Fn() -> bool>,
}

impl CpuBudget {
    pub fn new<F: Fn() -> bool + 'static>(should_continue: F) -> Self {
        CpuBudget {
            should_continue: Box::new(should_continue),
        }
    }

    /// Returns true if there is budget remaining to continue work.
    pub fn has_budget(&self) -> bool {
        (self.should_continue)()
    }

    /// Unlimited budget (for offline/bench use).
    pub fn unlimited() -> Self {
        CpuBudget {
            should_continue: Box::new(|| true),
        }
    }
}

/// Result of a single tick of phase work.
pub enum PhaseResult<T> {
    /// Phase needs more ticks to complete.
    Running,
    /// Phase is complete with output.
    Complete(T),
}

/// The overall planning pipeline state.
/// Simplified to 3 active phases: Analysis, Searching, Finalizing.
#[derive(Serialize, Deserialize)]
pub enum PlanningState {
    /// Fixed pre-computation (distance transforms, flood fills).
    Analysis {
        phase: analysis::AnalysisPhase,
        /// Layers are carried through but not serialized -- they are
        /// re-injected by the caller on resume.
        #[serde(skip)]
        layers: Vec<Box<dyn PlacementLayer>>,
        #[serde(default = "default_prune_margin")]
        prune_margin: f32,
    },
    /// Tree search over placement layers.
    Searching(SearchEngine),
    /// Finalization (build order, rich output from the best plan).
    Finalizing(finalize::FinalizePhase),
    /// Terminal states.
    Complete(Plan),
    Failed(String),
}

fn default_prune_margin() -> f32 {
    0.1
}

/// Run one tick of the planning pipeline.
pub fn tick_pipeline(
    state: PlanningState,
    data_source: &dyn PlannerRoomDataSource,
    budget: &CpuBudget,
) -> PlanningState {
    match state {
        PlanningState::Analysis {
            mut phase,
            layers,
            prune_margin,
        } => {
            match phase.tick(data_source, budget) {
                PhaseResult::Running => PlanningState::Analysis {
                    phase,
                    layers,
                    prune_margin,
                },
                PhaseResult::Complete(output) => {
                    // Transition to Searching -- analysis is stored once on the engine
                    let initial_state = PlacementState::new();
                    let engine = SearchEngine::new(layers, output, initial_state, prune_margin);
                    PlanningState::Searching(engine)
                }
            }
        }
        PlanningState::Searching(mut engine) => {
            let terrain = data_source.get_terrain();
            match engine.step(terrain, budget) {
                SearchResult::Running => PlanningState::Searching(engine),
                SearchResult::Complete => match engine.take_best_plan() {
                    Some((best_state, _analysis)) => {
                        let next = finalize::FinalizePhase::from_placement_state(best_state);
                        PlanningState::Finalizing(next)
                    }
                    None => PlanningState::Failed("No valid plan found".to_string()),
                },
            }
        }
        PlanningState::Finalizing(mut phase) => match phase.tick(data_source, budget) {
            PhaseResult::Running => PlanningState::Finalizing(phase),
            PhaseResult::Complete(plan) => PlanningState::Complete(plan),
        },
        // Terminal states
        s @ PlanningState::Complete(_) | s @ PlanningState::Failed(_) => s,
    }
}

/// Check if the pipeline has reached a terminal state.
pub fn is_complete(state: &PlanningState) -> bool {
    matches!(state, PlanningState::Complete(_) | PlanningState::Failed(_))
}

/// Extract the completed plan, if any.
pub fn get_plan(state: PlanningState) -> Option<Plan> {
    match state {
        PlanningState::Complete(plan) => Some(plan),
        _ => None,
    }
}
