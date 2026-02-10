use crate::location::*;
use crate::visual::*;
use fnv::FnvHashMap;
use serde::{Deserialize, Serialize};

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// A single structure placement in the room plan.
#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct RoomItem {
    #[serde(rename = "s")]
    pub structure_type: StructureType,
    #[serde(rename = "r")]
    pub required_rcl: u8,
}

impl RoomItem {
    pub fn new(structure_type: StructureType, required_rcl: u8) -> Self {
        RoomItem {
            structure_type,
            required_rcl,
        }
    }

    pub fn structure_type(&self) -> StructureType {
        self.structure_type
    }

    pub fn required_rcl(&self) -> u8 {
        self.required_rcl
    }
}

/// Build priority for construction ordering.
#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Ord, PartialOrd, Serialize, Deserialize)]
pub enum BuildPriority {
    VeryLow = 0,
    Low = 1,
    Medium = 2,
    High = 3,
    Critical = 4,
}

/// A single step in the build order.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BuildStep {
    pub structure_type: StructureType,
    pub location: Location,
    pub required_rcl: u8,
    pub priority: BuildPriority,
}

/// An RCL-dependent substitution: at a given location, before the target
/// structure's `required_rcl` is reached, a cheaper substitute structure
/// should be placed instead.
///
/// When the room reaches `active_from_rcl` (inclusive) but is below
/// `replaced_at_rcl` (exclusive), the `substitute` structure is placed.
/// Once the room reaches `replaced_at_rcl`, the substitute is destroyed
/// and the target structure is placed.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RclSubstitution {
    /// Location of the substitution.
    pub location: Location,
    /// The structure type to place as a temporary stand-in.
    pub substitute: StructureType,
    /// The final structure type this substitution stands in for.
    pub target: StructureType,
    /// Minimum RCL at which the substitute should be placed (inclusive).
    pub active_from_rcl: u8,
    /// RCL at which the target structure becomes available and the
    /// substitute should be replaced (same as the target's `required_rcl`).
    pub replaced_at_rcl: u8,
    /// Build priority for the substitute.
    pub priority: BuildPriority,
}

/// Sub-scores for plan quality analysis.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PlanScore {
    pub total: f32,
    pub source_distance: f32,
    pub source_balance: f32,
    pub controller_distance: f32,
    pub mineral_distance: f32,
    pub exit_proximity: f32,
    pub upkeep_cost: f32,
    pub tower_coverage: f32,
    pub extension_efficiency: f32,
    pub hub_quality: f32,
    pub upgrade_area_quality: f32,
    pub traffic_congestion: f32,
}

// ---------------------------------------------------------------------------
// Execution filter trait (caller-provided policy)
// ---------------------------------------------------------------------------

/// Caller-provided filter that decides whether a build step should be
/// placed during plan execution.
///
/// Implement this in your game crate where you have access to the live
/// game state (e.g. to check adjacent structures for roads, or to defer
/// walls/ramparts at low RCL).
pub trait ExecutionFilter {
    /// Return `true` if the given build step should be placed right now.
    fn should_place(&self, step: &BuildStep) -> bool;
}

/// A no-op filter that allows every build step.
pub struct AllowAllFilter;

impl ExecutionFilter for AllowAllFilter {
    fn should_place(&self, _step: &BuildStep) -> bool {
        true
    }
}

// ---------------------------------------------------------------------------
// Plan operations (pure data, no game API)
// ---------------------------------------------------------------------------

/// An operation that the plan wants the caller to perform.
///
/// Returned by [`Plan::get_build_operations`] and
/// [`Plan::get_cleanup_operations`]. The caller is responsible for
/// executing these against the game API (or any other backend).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanOperation {
    /// Create a construction site at the given location.
    CreateSite {
        location: Location,
        structure_type: StructureType,
    },
    /// Destroy the structure at the given location.
    /// `safe_only` indicates the structure should only be destroyed if it
    /// has no stored resources (e.g. containers with energy).
    DestroyStructure {
        location: Location,
        structure_type: StructureType,
        safe_only: bool,
    },
}

// ---------------------------------------------------------------------------
// Existing structure snapshot (input for cleanup)
// ---------------------------------------------------------------------------

/// A lightweight description of an existing structure in the room,
/// used as input to [`Plan::get_cleanup_operations`].
///
/// This avoids requiring game-API types in the plan logic.
#[derive(Clone, Debug)]
pub struct ExistingStructure {
    pub location: Location,
    pub structure_type: StructureType,
    /// Whether the structure currently holds resources in its store.
    pub has_store: bool,
}

// ---------------------------------------------------------------------------
// Plan
// ---------------------------------------------------------------------------

/// The complete room plan with rich metadata.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Plan {
    /// Structure placements by location.
    pub structures: FnvHashMap<Location, Vec<RoomItem>>,
    /// Hub/filler creep position.
    pub hub_position: Location,
    /// Controller upgrade area tiles.
    pub upgrade_area: Vec<Location>,
    /// Road network edges (for hauling route optimization).
    pub road_network: Vec<(Location, Location)>,
    /// Priority-ordered build sequence.
    pub build_order: Vec<BuildStep>,
    /// Overall plan score and sub-scores.
    pub score: PlanScore,
    /// RCL-dependent substitutions: temporary structures that stand in for
    /// planned structures until the required RCL is reached.
    /// Defaults to empty for backward compatibility with old serialized plans.
    #[serde(default)]
    pub substitutions: Vec<RclSubstitution>,
}

impl Plan {
    /// Render the plan using a visualizer.
    pub fn visualize<V: RoomVisualizer>(&self, visualizer: &mut V) {
        for (location, items) in &self.structures {
            for item in items {
                visualizer.render(*location, item.structure_type);
            }
        }
    }

    /// Get all locations of a specific structure type.
    pub fn get_locations(&self, structure_type: StructureType) -> Vec<Location> {
        self.structures
            .iter()
            .filter(|(_, items)| items.iter().any(|i| i.structure_type == structure_type))
            .map(|(loc, _)| *loc)
            .collect()
    }

    /// Check if the plan has all mandatory structures for RCL 8.
    pub fn is_complete(&self) -> bool {
        let count = |st: StructureType| -> usize {
            self.structures
                .values()
                .flat_map(|items| items.iter())
                .filter(|i| i.structure_type == st)
                .count()
        };

        count(StructureType::Spawn) >= 3
            && count(StructureType::Extension) >= 60
            && count(StructureType::Storage) >= 1
            && count(StructureType::Terminal) >= 1
            && count(StructureType::Lab) >= 10
            && count(StructureType::Factory) >= 1
            && count(StructureType::Observer) >= 1
            && count(StructureType::PowerSpawn) >= 1
            && count(StructureType::Nuker) >= 1
            && count(StructureType::Tower) >= 6
    }

    /// Compute the list of build and destroy operations for this tick.
    ///
    /// This is the primary API for plan execution. It returns a list of
    /// [`PlanOperation`] values that the caller should apply to the game.
    /// The plan itself never touches the game API.
    ///
    /// # Arguments
    /// * `room_level` - Current room controller level.
    /// * `max_placements` - Maximum number of `CreateSite` operations to emit.
    /// * `filter` - Caller-provided policy for which build steps to place.
    pub fn get_build_operations(
        &self,
        room_level: u8,
        max_placements: u32,
        filter: &dyn ExecutionFilter,
    ) -> Vec<PlanOperation> {
        let mut ops = Vec::new();
        let mut current_placements: u32 = 0;

        // Phase 1: Handle substitution transitions.
        // When the room has reached a substitution's replaced_at_rcl, emit
        // a destroy for the substitute so the target structure can be placed.
        for sub in &self.substitutions {
            if room_level >= sub.replaced_at_rcl {
                ops.push(PlanOperation::DestroyStructure {
                    location: sub.location,
                    structure_type: sub.substitute,
                    safe_only: true,
                });
            }
        }

        // Phase 2: Place structures from the build order.
        for step in &self.build_order {
            if current_placements >= max_placements {
                break;
            }
            if room_level >= step.required_rcl && filter.should_place(step) {
                ops.push(PlanOperation::CreateSite {
                    location: step.location,
                    structure_type: step.structure_type,
                });
                current_placements += 1;
            }
        }

        // Phase 3: Place active substitutions.
        for sub in &self.substitutions {
            if current_placements >= max_placements {
                break;
            }
            if room_level >= sub.active_from_rcl && room_level < sub.replaced_at_rcl {
                ops.push(PlanOperation::CreateSite {
                    location: sub.location,
                    structure_type: sub.substitute,
                });
                current_placements += 1;
            }
        }

        ops
    }

    /// Compute destroy operations for structures that don't belong in the
    /// plan at the current RCL.
    ///
    /// The caller provides a snapshot of existing structures as
    /// [`ExistingStructure`] values. The plan returns
    /// [`PlanOperation::DestroyStructure`] for each structure that is
    /// neither in the plan nor an active substitution.
    ///
    /// # Arguments
    /// * `existing` - Snapshot of structures currently in the room.
    /// * `room_level` - Current room controller level.
    pub fn get_cleanup_operations(
        &self,
        existing: &[ExistingStructure],
        room_level: u8,
    ) -> Vec<PlanOperation> {
        let mut invalid = Vec::new();
        let mut has_valid_spawn = false;
        let mut has_invalid_spawn = false;

        for s in existing {
            let matches_plan = self
                .structures
                .get(&s.location)
                .iter()
                .flat_map(|v| *v)
                .any(|r| r.structure_type() == s.structure_type);

            let matches_substitution = self.substitutions.iter().any(|sub| {
                sub.location == s.location
                    && sub.substitute == s.structure_type
                    && room_level >= sub.active_from_rcl
                    && room_level < sub.replaced_at_rcl
            });

            if matches_plan || matches_substitution {
                if s.structure_type == StructureType::Spawn {
                    has_valid_spawn = true;
                }
            } else {
                if s.structure_type == StructureType::Spawn {
                    has_invalid_spawn = true;
                }
                invalid.push(s);
            }
        }

        let mut ops = Vec::new();
        for s in invalid {
            // Don't destroy the last spawn
            let can_destroy = if s.structure_type == StructureType::Spawn {
                has_valid_spawn
            } else {
                true
            };

            if can_destroy && !s.has_store {
                ops.push(PlanOperation::DestroyStructure {
                    location: s.location,
                    structure_type: s.structure_type,
                    safe_only: false,
                });
            }
        }

        // Suppress the unused variable warning when there are no invalid spawns
        let _ = has_invalid_spawn;

        ops
    }
}

// ---------------------------------------------------------------------------
// Default game-API implementation (drop-in convenience)
// ---------------------------------------------------------------------------

/// Execute a list of plan operations against the game API.
///
/// This is the default implementation provided for convenience. It
/// creates construction sites and destroys structures as directed by the
/// operations list.
///
/// Returns the number of construction sites successfully created.
#[cfg(not(feature = "shim"))]
pub fn execute_operations(room: &Room, operations: &[PlanOperation]) -> u32 {
    let room_name = room.name();
    let mut sites_created = 0u32;

    for op in operations {
        match op {
            PlanOperation::CreateSite {
                location,
                structure_type,
            } => {
                if room
                    .create_construction_site(location.x(), location.y(), *structure_type, None)
                    .is_ok()
                {
                    sites_created += 1;
                }
            }
            PlanOperation::DestroyStructure {
                location,
                structure_type,
                safe_only,
            } => {
                let pos = RoomPosition::new(location.x(), location.y(), room_name);
                let structures = room.look_for_at(look::STRUCTURES, &pos);
                for structure in &structures {
                    if structure.structure_type() == *structure_type {
                        let should_destroy = if *safe_only {
                            structure
                                .as_has_store()
                                .map(|s| s.store().get_used_capacity(None) == 0)
                                .unwrap_or(true)
                        } else {
                            true
                        };
                        if should_destroy {
                            let _ = structure.destroy();
                        }
                    }
                }
            }
        }
    }

    sites_created
}

/// Build an [`ExistingStructure`] snapshot from the game's structure list.
///
/// This is the default implementation provided for convenience. It
/// converts the game's `StructureObject` slice into the lightweight
/// representation that [`Plan::get_cleanup_operations`] expects.
#[cfg(not(feature = "shim"))]
pub fn snapshot_structures(structures: &[StructureObject]) -> Vec<ExistingStructure> {
    structures
        .iter()
        .map(|s| {
            let pos = s.pos();
            ExistingStructure {
                location: Location::from_coords(pos.x().u8() as u32, pos.y().u8() as u32),
                structure_type: s.structure_type(),
                has_store: s
                    .as_has_store()
                    .map(|store| store.store().get_used_capacity(None) > 0)
                    .unwrap_or(false),
            }
        })
        .collect()
}
