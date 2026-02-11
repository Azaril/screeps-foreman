use crate::location::*;
use crate::visual::*;
use fnv::FnvHashMap;
use screeps::constants::StructureType;
use serde::{Deserialize, Serialize};

/// A single structure placement in the room plan.
///
/// `required_rcl` is optional during the planning phase: layers may set it to
/// `None` to indicate that the RCL should be resolved automatically by the
/// [`RclAssignmentLayer`] (or defaulted to 1 during finalization). Layers that
/// want to pin a specific RCL (e.g. road-network roads at RCL 2) set
/// `Some(rcl)`.
///
/// **Serialization compatibility:** Old serialized plans store `required_rcl`
/// as a bare `u8`. The custom `deserialize_rcl` helper treats `0` as `None`
/// (RCL 0 is not a valid game level) so old data round-trips correctly.
#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct RoomItem {
    #[serde(rename = "s")]
    pub structure_type: StructureType,
    #[serde(
        rename = "r",
        serialize_with = "serialize_rcl",
        deserialize_with = "deserialize_rcl"
    )]
    pub required_rcl: Option<u8>,
}

/// Serialize `Option<u8>` as a plain `u8` (None → 0).
fn serialize_rcl<S: serde::Serializer>(val: &Option<u8>, s: S) -> Result<S::Ok, S::Error> {
    s.serialize_u8(val.unwrap_or(0))
}

/// Deserialize a plain `u8` into `Option<u8>` (0 → None).
fn deserialize_rcl<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Option<u8>, D::Error> {
    let v = u8::deserialize(d)?;
    Ok(if v == 0 { None } else { Some(v) })
}

impl RoomItem {
    /// Create a new `RoomItem` with an explicit RCL.
    pub fn new(structure_type: StructureType, required_rcl: u8) -> Self {
        RoomItem {
            structure_type,
            required_rcl: Some(required_rcl),
        }
    }

    /// Create a new `RoomItem` with no RCL set (to be resolved later).
    pub fn new_auto_rcl(structure_type: StructureType) -> Self {
        RoomItem {
            structure_type,
            required_rcl: None,
        }
    }

    pub fn structure_type(&self) -> StructureType {
        self.structure_type
    }

    /// Return the required RCL, defaulting to 1 if not set.
    pub fn required_rcl(&self) -> u8 {
        self.required_rcl.unwrap_or(1)
    }

    /// Return the raw optional RCL value.
    pub fn required_rcl_opt(&self) -> Option<u8> {
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

/// Caller-provided stateful filter that decides whether a build step
/// should be placed during plan execution.
///
/// Implement this in your game crate where you have access to the live
/// game state (e.g. to check adjacent structures for roads, to defer
/// walls/ramparts at low RCL, or to cap the number of in-flight
/// construction sites).
///
/// The filter is called in two phases for each approved placement:
/// 1. [`should_place`](ExecutionFilter::should_place) — decide whether
///    the build step should be placed right now.
/// 2. [`added_placement`](ExecutionFilter::added_placement) — called
///    after the placement is committed, so the filter can update its
///    internal state (e.g. increment a construction-site counter).
pub trait ExecutionFilter {
    /// Return `true` if the given build step should be placed right now.
    fn should_place(&self, step: &BuildStep) -> bool;

    /// Called after a placement has been committed to the operations list.
    ///
    /// Use this to update any internal bookkeeping (e.g. increment a
    /// counter of in-flight construction sites).
    fn added_placement(&mut self, step: &BuildStep);
}

/// A no-op filter that allows every build step.
pub struct AllowAllFilter;

impl ExecutionFilter for AllowAllFilter {
    fn should_place(&self, _step: &BuildStep) -> bool {
        true
    }

    fn added_placement(&mut self, _step: &BuildStep) {}
}

/// Caller-provided stateful filter that decides whether a misplaced
/// structure should be removed during plan cleanup.
///
/// Implement this in your game crate where you have access to the live
/// game state (e.g. to prevent removing the last spawn in a room).
///
/// The filter is called in two phases for each approved removal:
/// 1. [`should_remove`](CleanupFilter::should_remove) — decide whether
///    the structure may be removed.
/// 2. [`added_removal`](CleanupFilter::added_removal) — called after the
///    removal is committed, so the filter can update its internal state
///    (e.g. decrement a remaining-spawn counter).
pub trait CleanupFilter {
    /// Return `true` if the given structure should be removed.
    ///
    /// Called for each structure that the plan considers invalid (not
    /// matching the plan or an active substitution). The implementation
    /// can veto removal by returning `false`.
    fn should_remove(&self, structure: &ExistingStructure) -> bool;

    /// Called after a removal has been committed to the operations list.
    ///
    /// Use this to update any internal bookkeeping (e.g. decrement a
    /// counter of remaining structures of a given type).
    fn added_removal(&mut self, structure: &ExistingStructure);
}

/// A no-op cleanup filter that allows every removal.
pub struct AllowAllCleanupFilter;

impl CleanupFilter for AllowAllCleanupFilter {
    fn should_remove(&self, _structure: &ExistingStructure) -> bool {
        true
    }

    fn added_removal(&mut self, _structure: &ExistingStructure) {}
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
    /// The `filter` controls which build steps are placed and receives
    /// a callback after each placement so it can track state (e.g. cap
    /// the number of in-flight construction sites).
    ///
    /// # Arguments
    /// * `room_level` - Current room controller level.
    /// * `filter` - Caller-provided policy for which build steps to place.
    pub fn get_build_operations(
        &self,
        room_level: u8,
        filter: &mut dyn ExecutionFilter,
    ) -> Vec<PlanOperation> {
        let mut ops = Vec::new();

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
            if room_level >= step.required_rcl && filter.should_place(step) {
                filter.added_placement(step);
                ops.push(PlanOperation::CreateSite {
                    location: step.location,
                    structure_type: step.structure_type,
                });
            }
        }

        // Phase 3: Place active substitutions.
        for sub in &self.substitutions {
            let step = BuildStep {
                structure_type: sub.substitute,
                location: sub.location,
                required_rcl: sub.active_from_rcl,
                priority: sub.priority,
            };
            if room_level >= sub.active_from_rcl
                && room_level < sub.replaced_at_rcl
                && filter.should_place(&step)
            {
                filter.added_placement(&step);
                ops.push(PlanOperation::CreateSite {
                    location: sub.location,
                    structure_type: sub.substitute,
                });
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
    /// The `filter` parameter lets the caller veto individual removals
    /// based on live game state (e.g. to protect the last spawn in a
    /// room). Use [`AllowAllCleanupFilter`] to allow all removals.
    ///
    /// # Arguments
    /// * `existing` - Snapshot of structures currently in the room.
    /// * `room_level` - Current room controller level.
    /// * `filter` - Caller-provided policy for which structures may be removed.
    pub fn get_cleanup_operations(
        &self,
        existing: &[ExistingStructure],
        room_level: u8,
        filter: &mut dyn CleanupFilter,
    ) -> Vec<PlanOperation> {
        let mut invalid = Vec::new();

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

            if !matches_plan && !matches_substitution {
                invalid.push(s);
            }
        }

        let mut ops = Vec::new();
        for s in invalid {
            if !s.has_store && filter.should_remove(s) {
                filter.added_removal(s);
                ops.push(PlanOperation::DestroyStructure {
                    location: s.location,
                    structure_type: s.structure_type,
                    safe_only: false,
                });
            }
        }

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
#[cfg(feature = "screeps")]
pub fn execute_operations(room: &screeps::Room, operations: &[PlanOperation]) -> u32 {
    use screeps::StructureProperties;
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
                let pos = screeps::RoomPosition::new(location.x(), location.y(), room_name);
                let structures = room.look_for_at(screeps::look::STRUCTURES, &pos);
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
#[cfg(feature = "screeps")]
pub fn snapshot_structures(structures: &[screeps::StructureObject]) -> Vec<ExistingStructure> {
    structures
        .iter()
        .map(|s| {
            let pos = screeps::HasPosition::pos(s);
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
