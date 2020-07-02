use super::planner::*;
use super::*;

pub fn has_mandatory_buildings(state: &PlannerState, context: &mut NodeContext) -> bool {
    state.get_count(StructureType::Spawn) >= 3
        && state.get_count(StructureType::Extension) >= 60
        && state.get_count(StructureType::Storage) >= 1
        && state.get_count(StructureType::Terminal) >= 1
        && state.get_count(StructureType::Lab) >= 10
        && state.get_count(StructureType::Factory) >= 1
        && state.get_count(StructureType::Observer) >= 1
        && state.get_count(StructureType::PowerSpawn) >= 1
        && state.get_count(StructureType::Nuker) >= 1
        && state.get_count(StructureType::Tower) >= 6
        && (state.get_count(StructureType::Extractor) as usize) == context.minerals().len()
}
