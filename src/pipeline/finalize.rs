//! Finalization phase: converts the best PlacementState into a rich Plan
//! with build order, score, substitutions, and metadata.

use crate::layer::PlacementState;
use crate::location::*;
use crate::plan::*;
use crate::planner::get_build_priority;
use crate::room_data::*;
use super::{CpuBudget, PhaseResult};
use serde::{Deserialize, Serialize};

use screeps::constants::StructureType;

/// Phase: Score the plan, assign RCL build order, produce final Plan.
#[derive(Clone, Serialize, Deserialize)]
pub struct FinalizePhase {
    placement_state: PlacementState,
    step: FinalizeStep,
}

#[derive(Clone, Serialize, Deserialize)]
enum FinalizeStep {
    BuildOrder,
    Complete,
}

impl FinalizePhase {
    /// Create from the best PlacementState found by the search engine.
    pub fn from_placement_state(state: PlacementState) -> Self {
        FinalizePhase {
            placement_state: state,
            step: FinalizeStep::BuildOrder,
        }
    }

    pub fn tick(
        &mut self,
        _data_source: &dyn PlannerRoomDataSource,
        _budget: &CpuBudget,
    ) -> PhaseResult<Plan> {
        loop {
            match &self.step {
                FinalizeStep::BuildOrder => {
                    self.step = FinalizeStep::Complete;
                }
                FinalizeStep::Complete => {
                    let plan = self.build_plan();
                    return PhaseResult::Complete(plan);
                }
            }
        }
    }

    fn build_plan(&self) -> Plan {
        let score = self.placement_state.to_plan_score();
        let build_order = self.compute_build_order();
        let substitutions = self.compute_substitutions();

        let hub_position = self
            .placement_state
            .get_landmark("hub")
            .unwrap_or_else(|| Location::from_coords(25, 25));

        let upgrade_area = self
            .placement_state
            .get_landmark_set("upgrade_area")
            .to_vec();

        // Reconstruct road edges from landmark sets
        let road_edges_a = self.placement_state.get_landmark_set("road_edges_a");
        let road_edges_b = self.placement_state.get_landmark_set("road_edges_b");
        let road_network: Vec<(Location, Location)> = road_edges_a
            .iter()
            .zip(road_edges_b.iter())
            .map(|(a, b)| (*a, *b))
            .collect();

        Plan {
            structures: self.placement_state.structures.clone(),
            hub_position,
            upgrade_area,
            road_network,
            build_order,
            score,
            substitutions,
        }
    }

    fn compute_build_order(&self) -> Vec<BuildStep> {
        let mut steps: Vec<BuildStep> = Vec::new();

        for (loc, items) in &self.placement_state.structures {
            for item in items {
                let priority = get_build_priority(item.structure_type, item.required_rcl as u32);
                steps.push(BuildStep {
                    structure_type: item.structure_type,
                    location: *loc,
                    required_rcl: item.required_rcl,
                    priority,
                });
            }
        }

        let hub = self
            .placement_state
            .get_landmark("hub")
            .unwrap_or_else(|| Location::from_coords(25, 25));

        steps.sort_by(|a, b| {
            a.required_rcl
                .cmp(&b.required_rcl)
                .then_with(|| b.priority.cmp(&a.priority))
                .then_with(|| {
                    let da = hub.distance_to(a.location);
                    let db = hub.distance_to(b.location);
                    da.cmp(&db)
                })
        });

        steps
    }

    /// Generate RCL-dependent substitutions for structures that have cheaper
    /// stand-ins at lower RCLs.
    ///
    /// Currently generates:
    /// - Storage (RCL 4) → Container (RCL 1): A container is placed at the
    ///   storage location from RCL 1 until RCL 4, when it is replaced by
    ///   the actual storage structure.
    fn compute_substitutions(&self) -> Vec<RclSubstitution> {
        let mut substitutions = Vec::new();

        for (loc, items) in &self.placement_state.structures {
            for item in items {
                if let Some(sub) = substitution_for(item.structure_type, item.required_rcl, *loc) {
                    substitutions.push(sub);
                }
            }
        }

        // Sort substitutions by active_from_rcl so they are processed in order
        substitutions.sort_by_key(|s| s.active_from_rcl);

        substitutions
    }
}

/// Return an RCL substitution for a structure type, if one applies.
///
/// This is the central place to define which structures have temporary
/// stand-ins at lower RCLs. Add new substitution rules here as needed.
fn substitution_for(
    structure_type: StructureType,
    required_rcl: u8,
    location: Location,
) -> Option<RclSubstitution> {
    match structure_type {
        // Storage (available at RCL 4): use a container from RCL 1 until
        // storage becomes available. The container provides early resource
        // storage at the hub position.
        StructureType::Storage => Some(RclSubstitution {
            location,
            substitute: StructureType::Container,
            target: StructureType::Storage,
            active_from_rcl: 1,
            replaced_at_rcl: required_rcl,
            priority: BuildPriority::High,
        }),
        _ => None,
    }
}

