pub mod extension;
pub mod hub;
pub mod lab;
pub mod tower;

use fnv::FnvHashMap;

use screeps::constants::StructureType;

use crate::constants::*;
use crate::location::Location;

/// A placement within a stamp: structure type at a relative offset from the stamp anchor.
#[derive(Clone, Debug)]
pub struct StampPlacement {
    pub structure_type: StructureType,
    pub dx: i8,
    pub dy: i8,
    /// The RCL at which this structure should be built. `None` means the RCL
    /// will be resolved automatically by the `RclAssignmentLayer` (e.g. roads
    /// inherit the RCL of adjacent buildings so they aren't placed too early).
    pub required_rcl: Option<u8>,
    /// If true, this placement must fit for the stamp to be considered valid.
    /// If false, the placement is best-effort and will be skipped if blocked.
    pub required: bool,
}

/// A stamp is a fixed arrangement of structures relative to an anchor point.
/// Multiple variants can exist for each logical group (different orientations, shapes).
#[derive(Clone, Debug)]
pub struct Stamp {
    pub name: &'static str,
    pub placements: Vec<StampPlacement>,
    /// Minimum distance transform value required at the anchor for this stamp to fit.
    pub min_radius: u8,
}

impl Stamp {
    /// Get all placements with absolute coordinates given an anchor position.
    /// Returns all placements that are within room bounds (does not check terrain or occupancy).
    pub fn place_at(&self, anchor_x: u8, anchor_y: u8) -> Vec<(u8, u8, StructureType, Option<u8>)> {
        self.placements
            .iter()
            .filter_map(|p| {
                let x = anchor_x as i16 + p.dx as i16;
                let y = anchor_y as i16 + p.dy as i16;
                if xy_in_bounds(x, y) {
                    Some((x as u8, y as u8, p.structure_type, p.required_rcl))
                } else {
                    None
                }
            })
            .collect()
    }

    /// Get all placements that actually fit, filtering by terrain, occupancy, and exclusions.
    /// Required placements that don't fit are still included (caller should use `fits_at`
    /// first to verify the stamp is valid). Optional placements that don't fit are omitted.
    ///
    /// A tile is considered blocked if it has ANY existing structure (including roads)
    /// or is in the excluded set. This prevents roads from being placed on non-road
    /// structures and vice versa, and respects exclusion zones set by earlier layers.
    pub fn place_at_filtered(
        &self,
        anchor_x: u8,
        anchor_y: u8,
        terrain: &crate::terrain::FastRoomTerrain,
        structures: &FnvHashMap<Location, Vec<crate::plan::RoomItem>>,
        excluded: &fnv::FnvHashSet<Location>,
    ) -> Vec<(u8, u8, StructureType, Option<u8>)> {
        self.placements
            .iter()
            .filter_map(|p| {
                let x = anchor_x as i16 + p.dx as i16;
                let y = anchor_y as i16 + p.dy as i16;
                if !xy_in_bounds(x, y) {
                    return None;
                }
                let ux = x as u8;
                let uy = y as u8;

                // Check if this placement fits
                let loc = Location::from_xy(ux, uy);
                let is_excluded = excluded.contains(&loc);
                let fits = if !xy_is_interior(x, y) {
                    // Edge tiles: only roads allowed, and no existing structures or exclusions
                    p.structure_type == StructureType::Road
                        && !structures.contains_key(&loc)
                        && !is_excluded
                } else {
                    !terrain.is_wall(ux, uy) && !structures.contains_key(&loc) && !is_excluded
                };

                if fits {
                    Some((ux, uy, p.structure_type, p.required_rcl))
                } else if p.required {
                    // Required placement doesn't fit -- include it anyway so caller
                    // can detect the issue (they should have called fits_at first).
                    Some((ux, uy, p.structure_type, p.required_rcl))
                } else {
                    // Optional placement doesn't fit -- skip it
                    None
                }
            })
            .collect()
    }

    /// Check if the stamp fits at the given anchor without overlapping walls or excluded tiles.
    /// Only checks required placements. Optional placements that don't fit are ignored.
    pub fn fits_at(
        &self,
        anchor_x: u8,
        anchor_y: u8,
        terrain: &crate::terrain::FastRoomTerrain,
        excluded: &fnv::FnvHashSet<Location>,
    ) -> bool {
        self.placements.iter().all(|p| {
            if !p.required {
                return true; // Optional placements don't affect fit
            }
            let x = anchor_x as i16 + p.dx as i16;
            let y = anchor_y as i16 + p.dy as i16;
            if !xy_is_interior(x, y) {
                // Roads can be on edge, but structures cannot be in build border
                p.structure_type == StructureType::Road
                    && xy_in_bounds(x, y)
                    && !excluded.contains(&Location::from_xy(x as u8, y as u8))
            } else {
                !terrain.is_wall(x as u8, y as u8)
                    && !excluded.contains(&Location::from_xy(x as u8, y as u8))
            }
        })
    }

    /// Validate that the stamp meets basic structural requirements:
    /// - At least one required placement exists.
    /// - No duplicate (dx, dy) positions for non-road structures.
    /// - All required placements are within min_radius of the origin.
    pub fn validate(&self) -> bool {
        // Must have at least one required placement
        let has_required = self.placements.iter().any(|p| p.required);
        if !has_required {
            return false;
        }

        // Check no duplicate (dx, dy) for non-road structures
        let mut seen = std::collections::HashSet::new();
        for p in &self.placements {
            if p.structure_type != StructureType::Road && !seen.insert((p.dx, p.dy)) {
                return false;
            }
        }

        // Check all required placements are within min_radius of origin
        for p in &self.placements {
            if p.required {
                let dist = p.dx.unsigned_abs().max(p.dy.unsigned_abs());
                if dist > self.min_radius {
                    return false;
                }
            }
        }

        true
    }

    /// Rotate the stamp 90 degrees clockwise.
    pub fn rotated_cw(&self) -> Stamp {
        Stamp {
            name: self.name,
            placements: self
                .placements
                .iter()
                .map(|p| StampPlacement {
                    structure_type: p.structure_type,
                    dx: -p.dy,
                    dy: p.dx,
                    required_rcl: p.required_rcl,
                    required: p.required,
                })
                .collect(),
            min_radius: self.min_radius,
        }
    }

    /// Get all 4 rotations of this stamp.
    pub fn all_rotations(&self) -> Vec<Stamp> {
        let r0 = self.clone();
        let r1 = r0.rotated_cw();
        let r2 = r1.rotated_cw();
        let r3 = r2.rotated_cw();
        vec![r0, r1, r2, r3]
    }
}

/// Helper to create a required StampPlacement with an explicit RCL.
pub fn sp(structure_type: StructureType, dx: i8, dy: i8, rcl: u8) -> StampPlacement {
    StampPlacement {
        structure_type,
        dx,
        dy,
        required_rcl: Some(rcl),
        required: true,
    }
}

/// Helper to create an optional StampPlacement with an explicit RCL.
pub fn sp_opt(structure_type: StructureType, dx: i8, dy: i8, rcl: u8) -> StampPlacement {
    StampPlacement {
        structure_type,
        dx,
        dy,
        required_rcl: Some(rcl),
        required: false,
    }
}

/// Helper to create a required StampPlacement with automatic RCL assignment.
///
/// The RCL will be resolved by the `RclAssignmentLayer` based on adjacent
/// structures. Use this for roads that should inherit the RCL of the
/// buildings they serve.
pub fn sp_auto(structure_type: StructureType, dx: i8, dy: i8) -> StampPlacement {
    StampPlacement {
        structure_type,
        dx,
        dy,
        required_rcl: None,
        required: true,
    }
}

/// Helper to create an optional StampPlacement with automatic RCL assignment.
///
/// The RCL will be resolved by the `RclAssignmentLayer` based on adjacent
/// structures. Use this for roads that should inherit the RCL of the
/// buildings they serve.
pub fn sp_opt_auto(structure_type: StructureType, dx: i8, dy: i8) -> StampPlacement {
    StampPlacement {
        structure_type,
        dx,
        dy,
        required_rcl: None,
        required: false,
    }
}
