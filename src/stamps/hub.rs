use super::*;

use screeps::constants::StructureType;

/// Hub stamp: "fast-filler" design. The filler creep stands at (0,0),
/// surrounded by 2 spawns, 2 extensions, storage, and link -- all within
/// transfer range 1. This lets a single stationary filler creep service
/// spawns and extensions without moving, dramatically improving spawn
/// throughput at RCL 7-8.
///
/// Layout (relative to hub position):
///
/// ```text
///        Link
///  Spawn1 HUB  Spawn2
///        Storage
///  Ext1        Terminal
/// ```
///
/// Factory, PowerSpawn, and Nuker are placed by the UtilityLayer at
/// near-hub positions (within 2-3 tiles) rather than occupying prime
/// hub-adjacent slots.
pub fn hub_stamps() -> Vec<Stamp> {
    let base = Stamp {
        name: "hub",
        placements: vec![
            // Core structures adjacent to hub (required) -- fast-filler layout
            sp(StructureType::Storage, 0, 1, 4),
            sp(StructureType::Link, 0, -1, 5),
            sp(StructureType::Spawn, -1, 0, 1),
            sp(StructureType::Spawn, 1, 0, 7),
            sp(StructureType::Extension, -1, 1, 2),
            sp(StructureType::Terminal, 1, 1, 6),
            // Hub center road (required -- filler stands here)
            sp_auto(StructureType::Road, 0, 0),
            // Peripheral roads (optional -- best-effort in constrained terrain)
            sp_opt_auto(StructureType::Road, -1, -1),
            sp_opt_auto(StructureType::Road, 1, -1),
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
