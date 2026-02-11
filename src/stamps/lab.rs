use super::*;

use screeps::constants::StructureType;

/// Lab stamp: two clusters of 5 labs mirrored diagonally, separated by a
/// diagonal road corridor. 10 labs total with 3 required roads and optional
/// perimeter roads for road-network integration.
///
/// Each reaction requires 3 labs within range 2: two reagent sources and one
/// output. The first two labs listed are source candidates (within range 2 of
/// every other lab), satisfying the `get_labs` filter in the labs mission.
/// Maximizing the number of lab pairs within range 2 enables future chain
/// reactions where output labs become sources for the next tier.
///
/// ```text
///      x: -1  0  1  2
/// y:-1            L  L
/// y: 0         R  S  L
/// y: 1      L  S  R  L
/// y: 2      L  L        
/// ```
///
/// S = source lab, L = reaction lab, R = required road.
/// Optional perimeter roads (not shown) surround the layout.
/// The 4 rotations cover all orientations.
pub fn lab_stamps() -> Vec<Stamp> {
    let stamp = Stamp {
        name: "lab_cluster",
        placements: vec![
            // Source lab 1 (within range 2 of all other labs)
            sp(StructureType::Lab, 1, 0, 6),
            // Source lab 2 (within range 2 of all other labs)
            sp(StructureType::Lab, 0, 1, 6),
            // Upper-right cluster (reaction labs)
            sp(StructureType::Lab, 1, -1, 6),
            sp(StructureType::Lab, 2, -1, 6),
            sp(StructureType::Lab, 2, 0, 7),
            sp(StructureType::Lab, 2, 1, 7),
            // Lower-left cluster (reaction labs)
            sp(StructureType::Lab, -1, 1, 7),
            sp(StructureType::Lab, -1, 2, 8),
            sp(StructureType::Lab, 0, 2, 8),
            sp(StructureType::Lab, 1, 2, 8),
            // Required diagonal road corridor
            sp_auto(StructureType::Road, 0, 0),
            sp_auto(StructureType::Road, 1, 1),
            sp_auto(StructureType::Road, 2, 2),
            // Optional perimeter roads for creep access
            sp_opt_auto(StructureType::Road, 0, -1),
            sp_opt_auto(StructureType::Road, 1, -2),
            sp_opt_auto(StructureType::Road, 2, -2),
            sp_opt_auto(StructureType::Road, 3, -1),
            sp_opt_auto(StructureType::Road, 3, 0),
            sp_opt_auto(StructureType::Road, 3, 1),
            sp_opt_auto(StructureType::Road, 3, 2),
            sp_opt_auto(StructureType::Road, 2, 3),
            sp_opt_auto(StructureType::Road, 1, 3),
            sp_opt_auto(StructureType::Road, 0, 3),
            sp_opt_auto(StructureType::Road, -1, 3),
            sp_opt_auto(StructureType::Road, -2, 2),
            sp_opt_auto(StructureType::Road, -2, 1),
            sp_opt_auto(StructureType::Road, -1, 0),
        ],
        min_radius: 2,
    };

    stamp.all_rotations()
}

/// Validate that all 8 reaction labs are within Chebyshev distance 2 of both
/// source labs. The first two lab placements are treated as sources; the
/// remaining 8 are reaction/output labs. Each output lab must be within range 2
/// of both sources so that `output.runReaction(source1, source2)` succeeds.
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
