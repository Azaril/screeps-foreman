use super::*;

use screeps::constants::StructureType;

/// Hub stamp: the filler creep stands at (0,0), surrounded by key structures.
/// Layout (relative to hub position):
///
/// ```text
///       Link
///  Spawn  HUB  Terminal
///       Storage
///  PSpawn      Factory
/// ```
///
/// The hub position is where a single filler creep stands to service
/// storage, terminal, link, factory, and power spawn without moving.
pub fn hub_stamps() -> Vec<Stamp> {
    let base = Stamp {
        name: "hub",
        placements: vec![
            // Core structures adjacent to hub (required)
            sp(StructureType::Storage, 0, 1, 4),
            sp(StructureType::Terminal, 1, 0, 6),
            sp(StructureType::Link, 0, -1, 5),
            sp(StructureType::Spawn, -1, 0, 1),
            sp(StructureType::PowerSpawn, -1, 1, 8),
            sp(StructureType::Factory, 1, 1, 7),
            // Nuker nearby (required)
            sp(StructureType::Nuker, 1, -1, 8),
            // Hub center road (required -- filler stands here)
            sp_auto(StructureType::Road, 0, 0),
            // Peripheral roads (optional -- best-effort in constrained terrain)
            sp_opt_auto(StructureType::Road, -1, -1),
            sp_opt_auto(StructureType::Road, 1, -1),
            sp_opt_auto(StructureType::Road, -1, 1),
            sp_opt_auto(StructureType::Road, 2, 0),
            sp_opt_auto(StructureType::Road, 2, 1),
            sp_opt_auto(StructureType::Road, 0, 2),
            sp_opt_auto(StructureType::Road, -1, 2),
            sp_opt_auto(StructureType::Road, -2, 1),
            sp_opt_auto(StructureType::Road, -2, 0),
            sp_opt_auto(StructureType::Road, 0, -2),
            sp_opt_auto(StructureType::Road, 1, -2),
            sp_opt_auto(StructureType::Road, 2, -1),
        ],
        min_radius: 3,
    };

    base.all_rotations()
}
