use super::*;

use screeps::constants::StructureType;

/// Generate the extension stamp hierarchy from largest to smallest.
///
/// Each stamp guarantees every extension is within Chebyshev distance 1 of
/// at least one road tile (the fill-range constraint). Stamps are tried
/// largest-first: 4x4, 5x3 corridor, 3x3, 2x2.
///
/// The 5x3 corridor stamp has connected internal roads (a straight line),
/// allowing a filler creep to walk through filling extensions without
/// backtracking -- useful in constrained terrain where 4x4 blocks don't fit.
pub fn extension_stamps() -> Vec<ExtensionStampDef> {
    vec![
        extension_stamp_4x4(),
        extension_stamp_5x3_corridor(),
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

/// 4x4 block: 12 extensions + 4 corner roads + border roads.
///
/// ```text
///     r r    
///   R E E R  
/// r E E E E r
/// r E E E E r
///   R E E R  
///     r r    
/// ```
///
/// R = corner road (required), E = extension (optional), r = border road (optional).
/// The shape is a 4x4 block with corners replaced by roads.  Creeps on
/// corner or border roads can reach all adjacent extensions diagonally.
fn extension_stamp_4x4() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_4x4",
        placements: vec![
            // Corner roads (required)
            sp_auto(StructureType::Road, 0, 0),
            sp_auto(StructureType::Road, 3, 0),
            sp_auto(StructureType::Road, 0, 3),
            sp_auto(StructureType::Road, 3, 3),
            // Extensions — row y=0 (between corner roads)
            sp_opt_auto(StructureType::Extension, 1, 0),
            sp_opt_auto(StructureType::Extension, 2, 0),
            // Extensions — row y=1 (full width)
            sp_opt_auto(StructureType::Extension, 0, 1),
            sp_opt_auto(StructureType::Extension, 1, 1),
            sp_opt_auto(StructureType::Extension, 2, 1),
            sp_opt_auto(StructureType::Extension, 3, 1),
            // Extensions — row y=2 (full width)
            sp_opt_auto(StructureType::Extension, 0, 2),
            sp_opt_auto(StructureType::Extension, 1, 2),
            sp_opt_auto(StructureType::Extension, 2, 2),
            sp_opt_auto(StructureType::Extension, 3, 2),
            // Extensions — row y=3 (between corner roads)
            sp_opt_auto(StructureType::Extension, 1, 3),
            sp_opt_auto(StructureType::Extension, 2, 3),
            // Border roads (optional — walkability around the cluster)
            // Top border (y=-1), inner positions only
            sp_opt_auto(StructureType::Road, 1, -1),
            sp_opt_auto(StructureType::Road, 2, -1),
            // Bottom border (y=4), inner positions only
            sp_opt_auto(StructureType::Road, 1, 4),
            sp_opt_auto(StructureType::Road, 2, 4),
            // Left border (x=-1, y=1..2)
            sp_opt_auto(StructureType::Road, -1, 1),
            sp_opt_auto(StructureType::Road, -1, 2),
            // Right border (x=4, y=1..2)
            sp_opt_auto(StructureType::Road, 4, 1),
            sp_opt_auto(StructureType::Road, 4, 2),
        ],
        min_radius: 5,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 8,
    }
}

/// 5x3 corridor: 10 extensions flanking a connected 5-road line.
///
/// ```text
///     r  r  r  r  r
///   r E  E  E  E  E  r
///   r R  R  R  R  R  r
///   r E  E  E  E  E  r
///     r  r  r  r  r
/// ```
///
/// R = connected road line (required), E = extension (optional),
/// r = border road (optional).
///
/// The 5 internal roads form a connected horizontal path. A filler creep
/// walks left-to-right along the road, filling 2 extensions per stop (one
/// above, one below). This gives 10 extensions with zero-backtrack fill.
///
/// Fill analysis: 5 stops x 2 ext/stop = 10 extensions, linear path.
fn extension_stamp_5x3_corridor() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_5x3_corridor",
        placements: vec![
            // Connected road line (required) — row y=1
            sp_auto(StructureType::Road, 0, 1),
            sp_auto(StructureType::Road, 1, 1),
            sp_auto(StructureType::Road, 2, 1),
            sp_auto(StructureType::Road, 3, 1),
            sp_auto(StructureType::Road, 4, 1),
            // Extensions — row y=0 (above road line)
            sp_opt_auto(StructureType::Extension, 0, 0),
            sp_opt_auto(StructureType::Extension, 1, 0),
            sp_opt_auto(StructureType::Extension, 2, 0),
            sp_opt_auto(StructureType::Extension, 3, 0),
            sp_opt_auto(StructureType::Extension, 4, 0),
            // Extensions — row y=2 (below road line)
            sp_opt_auto(StructureType::Extension, 0, 2),
            sp_opt_auto(StructureType::Extension, 1, 2),
            sp_opt_auto(StructureType::Extension, 2, 2),
            sp_opt_auto(StructureType::Extension, 3, 2),
            sp_opt_auto(StructureType::Extension, 4, 2),
            // Border roads (optional — walkability around the cluster)
            // Top border (y=-1)
            sp_opt_auto(StructureType::Road, 0, -1),
            sp_opt_auto(StructureType::Road, 1, -1),
            sp_opt_auto(StructureType::Road, 2, -1),
            sp_opt_auto(StructureType::Road, 3, -1),
            sp_opt_auto(StructureType::Road, 4, -1),
            // Bottom border (y=3)
            sp_opt_auto(StructureType::Road, 0, 3),
            sp_opt_auto(StructureType::Road, 1, 3),
            sp_opt_auto(StructureType::Road, 2, 3),
            sp_opt_auto(StructureType::Road, 3, 3),
            sp_opt_auto(StructureType::Road, 4, 3),
            // Left border (x=-1, y=0..2)
            sp_opt_auto(StructureType::Road, -1, 0),
            sp_opt_auto(StructureType::Road, -1, 1),
            sp_opt_auto(StructureType::Road, -1, 2),
            // Right border (x=5, y=0..2)
            sp_opt_auto(StructureType::Road, 5, 0),
            sp_opt_auto(StructureType::Road, 5, 1),
            sp_opt_auto(StructureType::Road, 5, 2),
        ],
        min_radius: 5,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 6,
    }
}

/// 3x3 block: 5 extensions + 4 corner roads + border roads.
///
/// ```text
///     r
///   r E r
/// r E E E r
///   r E r
///     r  
/// ```
///
/// R (corner roads at (0,0),(2,0),(0,2),(2,2)) shown as `r` since they
/// double as border access.  E = extension (optional), r = border road
/// (optional).  The cross-shaped extensions are all reachable from the
/// surrounding roads.
fn extension_stamp_3x3() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_3x3",
        placements: vec![
            // Corner roads (required)
            sp_auto(StructureType::Road, 0, 0),
            sp_auto(StructureType::Road, 2, 0),
            sp_auto(StructureType::Road, 0, 2),
            sp_auto(StructureType::Road, 2, 2),
            // Extensions — cross/plus shape
            sp_opt_auto(StructureType::Extension, 1, 0), // top
            sp_opt_auto(StructureType::Extension, 0, 1), // left
            sp_opt_auto(StructureType::Extension, 1, 1), // center
            sp_opt_auto(StructureType::Extension, 2, 1), // right
            sp_opt_auto(StructureType::Extension, 1, 2), // bottom
            // Border roads (optional)
            // Top border (y=-1), center only
            sp_opt_auto(StructureType::Road, 1, -1),
            // Bottom border (y=3), center only
            sp_opt_auto(StructureType::Road, 1, 3),
            // Left border (x=-1, y=1 only)
            sp_opt_auto(StructureType::Road, -1, 1),
            // Right border (x=3, y=1 only)
            sp_opt_auto(StructureType::Road, 3, 1),
        ],
        min_radius: 4,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 3,
    }
}

/// 2x2 block: 4 extensions + border roads (no corner roads).
///
/// ```text
///   r r
/// r E E r       E = extension (optional)
/// r E E r       r = border road (optional)
///   r r
/// ```
///
/// Creeps on any border road can reach all adjacent extensions via
/// diagonal movement, so corner roads are not needed.
fn extension_stamp_2x2() -> ExtensionStampDef {
    let stamp = Stamp {
        name: "ext_2x2",
        placements: vec![
            // Extensions
            sp_opt_auto(StructureType::Extension, 0, 0),
            sp_opt_auto(StructureType::Extension, 1, 0),
            sp_opt_auto(StructureType::Extension, 0, 1),
            sp_opt_auto(StructureType::Extension, 1, 1),
            // Border roads (optional), cardinal positions only
            // Top border (y=-1)
            sp_opt_auto(StructureType::Road, 0, -1),
            sp_opt_auto(StructureType::Road, 1, -1),
            // Bottom border (y=2)
            sp_opt_auto(StructureType::Road, 0, 2),
            sp_opt_auto(StructureType::Road, 1, 2),
            // Left border (x=-1, y=0..1)
            sp_opt_auto(StructureType::Road, -1, 0),
            sp_opt_auto(StructureType::Road, -1, 1),
            // Right border (x=2, y=0..1)
            sp_opt_auto(StructureType::Road, 2, 0),
            sp_opt_auto(StructureType::Road, 2, 1),
        ],
        min_radius: 3,
    };
    ExtensionStampDef {
        stamp,
        min_extensions: 2,
    }
}
