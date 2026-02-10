use super::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Extension cross stamp: 5 extensions in a plus pattern with surrounding roads.
/// Extensions are on checkerboard-compatible positions.
/// Center extension is required; outer extensions and roads are optional.
pub fn extension_cross_stamp() -> Stamp {
    Stamp {
        name: "extension_cross",
        placements: vec![
            // Center extension (required)
            sp(StructureType::Extension, 0, 0, 2),
            // Outer extensions (optional -- may be blocked in constrained rooms)
            sp_opt(StructureType::Extension, 0, 2, 2),
            sp_opt(StructureType::Extension, 2, 0, 2),
            sp_opt(StructureType::Extension, 0, -2, 2),
            sp_opt(StructureType::Extension, -2, 0, 2),
            // Roads between extensions (optional)
            sp_opt(StructureType::Road, -1, -1, 1),
            sp_opt(StructureType::Road, 1, -1, 1),
            sp_opt(StructureType::Road, -1, 1, 1),
            sp_opt(StructureType::Road, 1, 1, 1),
            sp_opt(StructureType::Road, 0, -1, 1),
            sp_opt(StructureType::Road, 0, 1, 1),
            sp_opt(StructureType::Road, -1, 0, 1),
            sp_opt(StructureType::Road, 1, 0, 1),
        ],
        min_radius: 3,
    }
}
