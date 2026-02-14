pub use screeps_common::plan_location::*;

use crate::terrain::*;

/// Trait for providing room data to the planner.
/// Implementations exist for both in-game (screeps API) and offline (bench) use.
pub trait PlannerRoomDataSource {
    fn get_terrain(&self) -> &FastRoomTerrain;
    fn get_controllers(&self) -> &[PlanLocation];
    fn get_sources(&self) -> &[PlanLocation];
    fn get_minerals(&self) -> &[PlanLocation];
}
