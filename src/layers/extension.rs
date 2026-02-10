//! ExtensionLayer: Places extensions using checkerboard flood-fill from hub.
//! Deterministic (1 candidate). Configurable target and minimum.

use crate::layer::*;
use crate::location::*;
use crate::terrain::*;
use fnv::FnvHashSet;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Places extensions via checkerboard flood-fill from hub.
/// Deterministic (1 candidate).
pub struct ExtensionLayer {
    /// Target number of extensions to place.
    pub target: u8,
    /// Minimum number of extensions required. If fewer are placed, the
    /// candidate is rejected. Defaults to `target` (all must be placed).
    pub min_required: u8,
}

impl Default for ExtensionLayer {
    fn default() -> Self {
        ExtensionLayer {
            target: 60,
            min_required: 60,
        }
    }
}

impl PlacementLayer for ExtensionLayer {
    fn name(&self) -> &str {
        "extension"
    }

    fn candidate_count(
        &self,
        _state: &PlacementState,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(1)
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index > 0 {
            return None;
        }

        let hub = match state.get_landmark("hub") {
            Some(loc) => loc,
            None => return Some(Err(())),
        };

        let mut new_state = state.clone();
        let target = self.target;
        let parity = (hub.x() as u16 + hub.y() as u16) % 2;

        // BFS from hub to find extension positions
        let mut visited: FnvHashSet<Location> = FnvHashSet::default();
        let mut queue = std::collections::VecDeque::new();
        queue.push_back((hub, 0u32));
        visited.insert(hub);

        let mut candidates: Vec<(Location, u32)> = Vec::new();

        while let Some((loc, dist)) = queue.pop_front() {
            let x = loc.x();
            let y = loc.y();

            // Check if this tile is a valid extension position
            let tile_parity = (x as u16 + y as u16) % 2;
            if tile_parity == parity
                && !new_state.is_occupied(x, y)
                && !terrain.is_wall(x, y)
                && (2..48).contains(&x)
                && (2..48).contains(&y)
                && dist > 1
            {
                candidates.push((loc, dist));
            }

            // Expand to cardinal neighbors
            for &(dx, dy) in &NEIGHBORS_4 {
                let nx = x as i16 + dx as i16;
                let ny = y as i16 + dy as i16;
                if (1..49).contains(&nx) && (1..49).contains(&ny) {
                    let nloc = Location::from_coords(nx as u32, ny as u32);
                    if !visited.contains(&nloc) && !terrain.is_wall(nx as u8, ny as u8) {
                        visited.insert(nloc);
                        queue.push_back((nloc, dist + 1));
                    }
                }
            }
        }

        // Sort by distance (closest first)
        candidates.sort_by_key(|c| c.1);

        let mut extensions_placed = 0u8;
        for (loc, _) in candidates {
            if extensions_placed >= target {
                break;
            }
            if new_state.is_occupied(loc.x(), loc.y()) {
                continue;
            }
            new_state.place_structure(loc.x(), loc.y(), StructureType::Extension, 0);
            extensions_placed += 1;
        }

        // Reject if fewer than minimum required extensions were placed
        if extensions_placed < self.min_required {
            return Some(Err(()));
        }

        Some(Ok(new_state))
    }
}
