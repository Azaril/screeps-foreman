pub const ROOM_WIDTH: u8 = 50;
pub const ROOM_HEIGHT: u8 = 50;
pub const ROOM_BUILD_BORDER: u8 = 2;

use screeps::constants::StructureType;

/// Maximum number of a given structure type allowed at a given RCL.
/// Returns 0 if the structure is not available at that RCL.
/// Based on the Screeps API: <https://docs.screeps.com/control.html>
///
/// Note: Container and Road have no per-RCL limits (unlimited at all RCLs).
/// Rampart and Wall share the same limits.
pub fn max_structures_at_rcl(structure_type: StructureType, rcl: u8) -> u32 {
    match structure_type {
        StructureType::Spawn => match rcl {
            0 => 0,
            1..=6 => 1,
            7 => 2,
            _ => 3,
        },
        StructureType::Extension => match rcl {
            0 | 1 => 0,
            2 => 5,
            3 => 10,
            4 => 20,
            5 => 30,
            6 => 40,
            7 => 50,
            _ => 60,
        },
        StructureType::Link => match rcl {
            0..=4 => 0,
            5 => 2,
            6 => 3,
            7 => 4,
            _ => 6,
        },
        StructureType::Storage => match rcl {
            0..=3 => 0,
            _ => 1,
        },
        StructureType::Tower => match rcl {
            0..=2 => 0,
            3..=4 => 1,
            5..=6 => 2,
            7 => 3,
            _ => 6,
        },
        StructureType::Observer => match rcl {
            0..=7 => 0,
            _ => 1,
        },
        StructureType::PowerSpawn => match rcl {
            0..=7 => 0,
            _ => 1,
        },
        StructureType::Extractor => match rcl {
            0..=5 => 0,
            _ => 1,
        },
        StructureType::Lab => match rcl {
            0..=5 => 0,
            6 => 3,
            7 => 6,
            _ => 10,
        },
        StructureType::Terminal => match rcl {
            0..=5 => 0,
            _ => 1,
        },
        StructureType::Nuker => match rcl {
            0..=7 => 0,
            _ => 1,
        },
        StructureType::Factory => match rcl {
            0..=6 => 0,
            _ => 1,
        },
        StructureType::Rampart | StructureType::Wall => match rcl {
            0 | 1 => 0,
            _ => 2500, // Effectively unlimited
        },
        StructureType::Road | StructureType::Container => 2500, // Effectively unlimited
        _ => 0,
    }
}

/// Return the minimum RCL at which the Nth structure of a given type can be built.
/// `count` is 1-based (the 1st structure, the 2nd structure, etc.).
/// Returns 0 if the structure type has no RCL limits (roads, containers).
/// Returns 9 (impossible) if the count exceeds the maximum for that type.
pub fn min_rcl_for_nth(structure_type: StructureType, count: u32) -> u8 {
    if count == 0 {
        return 0;
    }
    for rcl in 1..=8u8 {
        if max_structures_at_rcl(structure_type, rcl) >= count {
            return rcl;
        }
    }
    9 // Not achievable
}
