use super::*;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Lab stamps: 10 labs arranged so that all 8 reaction labs are within range 2
/// of both source labs.
///
/// Compact layout:
/// ```text
///  L L L
///  L S L
///    R
///  L S L
///  L L L
/// ```
/// Where S = source lab, L = reaction lab, R = road.
/// The two source labs are at (0,0) and (0,2). All reaction labs are in the
/// intersection of range-2 Chebyshev balls around both sources:
/// x in [-2, 2], y in [0, 2].
pub fn lab_stamps() -> Vec<Stamp> {
    let compact = Stamp {
        name: "lab_compact",
        placements: vec![
            // Source lab 1
            sp(StructureType::Lab, 0, 0, 6),
            // Source lab 2
            sp(StructureType::Lab, 0, 2, 6),
            // Reaction labs (all within range 2 of both sources)
            sp(StructureType::Lab, -1, 0, 6),
            sp(StructureType::Lab, 1, 0, 6),
            sp(StructureType::Lab, -1, 1, 7),
            sp(StructureType::Lab, 1, 1, 7),
            sp(StructureType::Lab, -1, 2, 7),
            sp(StructureType::Lab, 1, 2, 7),
            sp(StructureType::Lab, -2, 1, 8),
            sp(StructureType::Lab, 2, 1, 8),
            // Road through the middle for access (optional)
            sp_opt(StructureType::Road, 0, 1, 1),
        ],
        min_radius: 3,
    };

    let diamond = Stamp {
        name: "lab_diamond",
        placements: vec![
            // Source lab 1
            sp(StructureType::Lab, 0, 0, 6),
            // Source lab 2
            sp(StructureType::Lab, 2, 0, 6),
            // Reaction labs (all within range 2 of both sources)
            sp(StructureType::Lab, 1, -1, 6),
            sp(StructureType::Lab, 1, 1, 7),
            sp(StructureType::Lab, 0, -1, 7),
            sp(StructureType::Lab, 2, -1, 7),
            sp(StructureType::Lab, 0, 1, 7),
            sp(StructureType::Lab, 2, 1, 8),
            sp(StructureType::Lab, 1, -2, 8),
            sp(StructureType::Lab, 1, 2, 8),
            // Road (optional)
            sp_opt(StructureType::Road, 1, 0, 1),
        ],
        min_radius: 3,
    };

    let mut stamps = Vec::new();
    stamps.extend(compact.all_rotations());
    stamps.extend(diamond.all_rotations());
    stamps
}

/// Validate that all 8 reaction labs are within range 2 of both source labs.
pub fn validate_lab_stamp(stamp: &Stamp) -> bool {
    let labs: Vec<_> = stamp
        .placements
        .iter()
        .filter(|p| p.structure_type == StructureType::Lab)
        .collect();

    if labs.len() != 10 {
        return false;
    }

    // First two labs are source labs
    let source1 = (labs[0].dx, labs[0].dy);
    let source2 = (labs[1].dx, labs[1].dy);

    // All remaining labs must be within range 2 of both sources
    labs[2..].iter().all(|lab| {
        let d1 = (lab.dx - source1.0).abs().max((lab.dy - source1.1).abs());
        let d2 = (lab.dx - source2.0).abs().max((lab.dy - source2.1).abs());
        d1 <= 2 && d2 <= 2
    })
}
