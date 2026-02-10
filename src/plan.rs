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

    /// Execute the plan by creating construction sites (in-game only).
    #[cfg(not(feature = "shim"))]
    pub fn execute(&self, room: &Room, max_placements: u32) {
        let room_name = room.name();
        let room_level = room.controller().map(|c| c.level()).unwrap_or(0);
        let mut current_placements = 0;

        // Use build_order if available, otherwise fall back to structures map
        if !self.build_order.is_empty() {
            for step in &self.build_order {
                if current_placements >= max_placements {
                    return;
                }
                if room_level >= step.required_rcl {
                    if step.structure_type == StructureType::Storage {
                        // Destroy any container at the storage location first
                        let structures = room.look_for_at(
                            look::STRUCTURES,
                            &RoomPosition::new(step.location.x(), step.location.y(), room_name),
                        );
                        for structure in &structures {
                            if let StructureObject::StructureContainer(container) = structure {
                                let _ = container.destroy();
                            }
                        }
                    }
                    if room
                        .create_construction_site(
                            step.location.x(),
                            step.location.y(),
                            step.structure_type,
                            None,
                        )
                        .is_ok()
                    {
                        current_placements += 1;
                    }
                } else if step.structure_type == StructureType::Storage {
                    // Place a container as a temporary stand-in for storage
                    if room
                        .create_construction_site(
                            step.location.x(),
                            step.location.y(),
                            StructureType::Container,
                            None,
                        )
                        .is_ok()
                    {
                        current_placements += 1;
                    }
                }
            }
        }
    }

    /// Remove structures not in the plan (in-game only).
    #[cfg(not(feature = "shim"))]
    pub fn cleanup(&self, structures: &[StructureObject]) {
        let mut invalid_structures = Vec::new();
        let mut valid_structures = Vec::new();

        for structure in structures {
            let structure_pos = structure.pos();
            let structure_type = structure.structure_type();

            let is_valid = self
                .structures
                .get(&Location::from_coords(
                    structure_pos.x().u8() as u32,
                    structure_pos.y().u8() as u32,
                ))
                .iter()
                .flat_map(|v| *v)
                .any(|r| {
                    r.structure_type() == structure_type
                        || (r.structure_type() == StructureType::Storage
                            && structure_type == StructureType::Container)
                });

            if is_valid {
                valid_structures.push(structure);
            } else {
                invalid_structures.push(structure);
            }
        }

        let has_valid_spawn = valid_structures
            .iter()
            .any(|s| s.structure_type() == StructureType::Spawn);

        for structure in invalid_structures {
            let can_destroy = match structure.structure_type() {
                StructureType::Spawn => has_valid_spawn,
                _ => true,
            };

            let has_store = structure
                .as_has_store()
                .map(|s| s.store().get_used_capacity(None) > 0)
                .unwrap_or(false);

            if can_destroy && !has_store {
                let _ = structure.destroy();
            }
        }
    }
}
