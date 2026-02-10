use super::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Generate the extension stamp hierarchy from largest to smallest.
///
/// Each stamp is an NxN block with corners replaced by roads.  The corner
/// roads provide diagonal access so a creep standing on a corner can fill
/// interior extensions that would otherwise be unreachable from the external
/// border.  All extensions in every stamp are within Chebyshev distance 1 of
/// at least one road tile (corner or border).
///
/// Stamps are returned largest-first: 4x4, 3x3, 2x2.
/// The caller should try them in order and fall back to smaller sizes.
pub fn extension_stamps() -> Vec<ExtensionStampDef> {
    vec![
        extension_stamp_4x4(),
        extension_stamp_3x3(),
        extension_stamp_2x2(),
    ]
}

/// Metadata for an extension stamp used by the placement layer.
pub struct ExtensionStampDef {
    pub stamp: Stamp,
    /// Minimum number of extensions that must be placed for this stamp to be
    /// worth keeping.  If fewer extensions survive the reachability check,
    /// the stamp is rolled back and the next smaller size is tried.
    pub min_extensions: u8,
}

/// 4x4 minus corners: 12 extensions + 4 corner roads + border roads.
///
/// ```text
/// r r r r r r
/// r R E E R r       R = corner road (required)
/// r E E E E r       E = extension (optional)
/// r E E E E r       r = border road (optional)
/// r R E E R r
/// r r r r r r
/// ```
///
/// Corner roads at (0,0), (3,0), (0,3), (3,3) provide diagonal access
/// to interior tiles (1,1), (2,1), (1,2), (2,2).
fn extension_stamp_4x4() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_4x4",
        placements: vec![
            // Corner roads (required — structural, enable interior access)
            sp(StructureType::Road, 0, 0, 1),
            sp(StructureType::Road, 3, 0, 1),
            sp(StructureType::Road, 0, 3, 1),
            sp(StructureType::Road, 3, 3, 1),
            // Extensions — top row (y=0)
            sp_opt(StructureType::Extension, 1, 0, 2),
            sp_opt(StructureType::Extension, 2, 0, 2),
            // Extensions — row y=1
            sp_opt(StructureType::Extension, 0, 1, 2),
            sp_opt(StructureType::Extension, 1, 1, 2),
            sp_opt(StructureType::Extension, 2, 1, 2),
            sp_opt(StructureType::Extension, 3, 1, 2),
            // Extensions — row y=2
            sp_opt(StructureType::Extension, 0, 2, 2),
            sp_opt(StructureType::Extension, 1, 2, 2),
            sp_opt(StructureType::Extension, 2, 2, 2),
            sp_opt(StructureType::Extension, 3, 2, 2),
            // Extensions — bottom row (y=3)
            sp_opt(StructureType::Extension, 1, 3, 2),
            sp_opt(StructureType::Extension, 2, 3, 2),
            // Border roads (optional — walkability around the cluster)
            // Top border (y=-1)
            sp_opt(StructureType::Road, -1, -1, 1),
            sp_opt(StructureType::Road, 0, -1, 1),
            sp_opt(StructureType::Road, 1, -1, 1),
            sp_opt(StructureType::Road, 2, -1, 1),
            sp_opt(StructureType::Road, 3, -1, 1),
            sp_opt(StructureType::Road, 4, -1, 1),
            // Bottom border (y=4)
            sp_opt(StructureType::Road, -1, 4, 1),
            sp_opt(StructureType::Road, 0, 4, 1),
            sp_opt(StructureType::Road, 1, 4, 1),
            sp_opt(StructureType::Road, 2, 4, 1),
            sp_opt(StructureType::Road, 3, 4, 1),
            sp_opt(StructureType::Road, 4, 4, 1),
            // Left border (x=-1, y=0..3)
            sp_opt(StructureType::Road, -1, 0, 1),
            sp_opt(StructureType::Road, -1, 1, 1),
            sp_opt(StructureType::Road, -1, 2, 1),
            sp_opt(StructureType::Road, -1, 3, 1),
            // Right border (x=4, y=0..3)
            sp_opt(StructureType::Road, 4, 0, 1),
            sp_opt(StructureType::Road, 4, 1, 1),
            sp_opt(StructureType::Road, 4, 2, 1),
            sp_opt(StructureType::Road, 4, 3, 1),
        ],
        min_radius: 5,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 8,
    }
}

/// 3x3 minus corners: 5 extensions + 4 corner roads + border roads.
///
/// ```text
/// r r r r r
/// r R E R r       R = corner road (required)
/// r E E E r       E = extension (optional)
/// r R E R r       r = border road (optional)
/// r r r r r
/// ```
///
/// All 4 corner roads reach center (1,1) diagonally.
fn extension_stamp_3x3() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_3x3",
        placements: vec![
            // Corner roads (required)
            sp(StructureType::Road, 0, 0, 1),
            sp(StructureType::Road, 2, 0, 1),
            sp(StructureType::Road, 0, 2, 1),
            sp(StructureType::Road, 2, 2, 1),
            // Extensions — cardinal + center
            sp_opt(StructureType::Extension, 1, 0, 2), // top
            sp_opt(StructureType::Extension, 0, 1, 2), // left
            sp_opt(StructureType::Extension, 1, 1, 2), // center
            sp_opt(StructureType::Extension, 2, 1, 2), // right
            sp_opt(StructureType::Extension, 1, 2, 2), // bottom
            // Border roads (optional)
            // Top border (y=-1)
            sp_opt(StructureType::Road, -1, -1, 1),
            sp_opt(StructureType::Road, 0, -1, 1),
            sp_opt(StructureType::Road, 1, -1, 1),
            sp_opt(StructureType::Road, 2, -1, 1),
            sp_opt(StructureType::Road, 3, -1, 1),
            // Bottom border (y=3)
            sp_opt(StructureType::Road, -1, 3, 1),
            sp_opt(StructureType::Road, 0, 3, 1),
            sp_opt(StructureType::Road, 1, 3, 1),
            sp_opt(StructureType::Road, 2, 3, 1),
            sp_opt(StructureType::Road, 3, 3, 1),
            // Left border (x=-1, y=0..2)
            sp_opt(StructureType::Road, -1, 0, 1),
            sp_opt(StructureType::Road, -1, 1, 1),
            sp_opt(StructureType::Road, -1, 2, 1),
            // Right border (x=3, y=0..2)
            sp_opt(StructureType::Road, 3, 0, 1),
            sp_opt(StructureType::Road, 3, 1, 1),
            sp_opt(StructureType::Road, 3, 2, 1),
        ],
        min_radius: 4,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 3,
    }
}

/// 2x2 block: 4 extensions + border roads.
///
/// ```text
/// r r r r
/// r E E r       E = extension (optional)
/// r E E r       r = border road (optional)
/// r r r r
/// ```
///
/// All tiles adjacent to external border — no corners to remove.
fn extension_stamp_2x2() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_2x2",
        placements: vec![
            // Extensions
            sp_opt(StructureType::Extension, 0, 0, 2),
            sp_opt(StructureType::Extension, 1, 0, 2),
            sp_opt(StructureType::Extension, 0, 1, 2),
            sp_opt(StructureType::Extension, 1, 1, 2),
            // Border roads (optional)
            // Top border (y=-1)
            sp_opt(StructureType::Road, -1, -1, 1),
            sp_opt(StructureType::Road, 0, -1, 1),
            sp_opt(StructureType::Road, 1, -1, 1),
            sp_opt(StructureType::Road, 2, -1, 1),
            // Bottom border (y=2)
            sp_opt(StructureType::Road, -1, 2, 1),
            sp_opt(StructureType::Road, 0, 2, 1),
            sp_opt(StructureType::Road, 1, 2, 1),
            sp_opt(StructureType::Road, 2, 2, 1),
            // Left border (x=-1, y=0..1)
            sp_opt(StructureType::Road, -1, 0, 1),
            sp_opt(StructureType::Road, -1, 1, 1),
            // Right border (x=2, y=0..1)
            sp_opt(StructureType::Road, 2, 0, 1),
            sp_opt(StructureType::Road, 2, 1, 1),
        ],
        min_radius: 3,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 2,
    }
}
