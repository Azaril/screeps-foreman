//! ExtensionLayer: Places extensions using a multi-size stamp hierarchy.
//! Deterministic (1 candidate). Configurable target and minimum.
//!
//! Uses an incremental BFS from the hub to find the nearest viable placement
//! position, tries stamps largest-first (4x4 -> 3x3 -> 2x2) with anchor
//! offsets, then restarts the BFS after each successful placement so
//! subsequent stamps see accurate distances and road connectivity.
//!
//! After each stamp placement, a per-stamp verification ensures:
//!   1. All stamp roads are path-connected to the hub (via walkable BFS).
//!   2. Disconnected roads are connected by pathfinding a short road link.
//!   3. Every extension has at least one adjacent hub-reachable road.
//!
//! A 1x1 fallback fills any remaining quota with individual extensions.

use crate::layer::*;
use crate::location::*;
use crate::stamps::extension::{extension_stamps, ExtensionStampDef};
use crate::terrain::*;
use fnv::FnvHashSet;
use log::*;
use std::collections::VecDeque;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Places extensions via stamp hierarchy outward from the hub.
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

        let stamps = extension_stamps();
        let mut new_state = state.clone();
        let target = self.target;
        let mut extensions_placed = 0u8;

        // Precompute extension offsets for each stamp (used for anchor offsetting).
        let stamp_ext_offsets: Vec<Vec<(i8, i8)>> = stamps
            .iter()
            .map(|sd| {
                sd.stamp
                    .placements
                    .iter()
                    .filter(|p| p.structure_type == StructureType::Extension)
                    .map(|p| (p.dx, p.dy))
                    .collect()
            })
            .collect();

        // --- Main loop: incremental BFS, place stamp, restart ------------
        // Each iteration runs a BFS from the hub over walkable tiles.
        // The first unoccupied non-wall tile dequeued is the nearest
        // candidate.  We try stamps there; on success we restart the BFS
        // so the next search reflects the new roads and extensions.
        loop {
            if extensions_placed >= target {
                break;
            }

            let placed = search_and_place(
                hub,
                terrain,
                &mut new_state,
                &stamps,
                &stamp_ext_offsets,
                target - extensions_placed,
            );

            if placed == 0 {
                // BFS exhausted — no more viable positions.
                break;
            }

            extensions_placed += placed;
        }

        debug!(
            "ExtensionLayer: placed {} / {} extensions (min {})",
            extensions_placed, target, self.min_required
        );

        if extensions_placed < self.min_required {
            return Some(Err(()));
        }

        Some(Ok(new_state))
    }
}

// ---------------------------------------------------------------------------
// Core search: incremental BFS from hub, try stamps at each candidate
// ---------------------------------------------------------------------------

/// Run a BFS from the hub over walkable tiles.  At each candidate tile
/// (unoccupied, non-wall), try stamps largest-first with anchor offsets.
/// Returns the number of extensions placed (0 if BFS exhausted).
fn search_and_place(
    hub: Location,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
    stamps: &[ExtensionStampDef],
    stamp_ext_offsets: &[Vec<(i8, i8)>],
    remaining: u8,
) -> u8 {
    let mut visited = vec![false; 50 * 50];
    let mut queue: VecDeque<(u8, u8)> = VecDeque::new();

    let hx = hub.x();
    let hy = hub.y();

    // The hub tile may itself be occupied (e.g. if the hub landmark is on a
    // spawn rather than a road).  Seed the BFS from all road tiles adjacent
    // to (and including) the hub so we can escape the hub cluster.
    let seed_tiles: Vec<(u8, u8)> = {
        let mut seeds = vec![(hx, hy)];
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = hx as i16 + dx as i16;
            let ny = hy as i16 + dy as i16;
            if (0..50).contains(&nx) && (0..50).contains(&ny) {
                seeds.push((nx as u8, ny as u8));
            }
        }
        seeds
    };

    for &(sx, sy) in &seed_tiles {
        let idx = sy as usize * 50 + sx as usize;
        if visited[idx] {
            continue;
        }
        if terrain.is_wall(sx, sy) {
            continue;
        }
        if state.is_occupied(sx, sy) {
            continue;
        }
        visited[idx] = true;
        queue.push_back((sx, sy));
    }

    while let Some((x, y)) = queue.pop_front() {

        // If this tile is unoccupied, not a wall, and not already a road,
        // it's a candidate for stamp placement (or 1x1 fallback).
        if !state.is_occupied(x, y)
            && !terrain.is_wall(x, y)
            && !tile_has_road(state, x, y)
            && (1..49).contains(&x)
            && (1..49).contains(&y)
        {
            // Try stamps largest-first with anchor offsets.
            for (si, stamp_def) in stamps.iter().enumerate() {
                if let Some(count) = try_stamp_with_offsets(
                    stamp_def,
                    &stamp_ext_offsets[si],
                    x,
                    y,
                    terrain,
                    state,
                    hub,
                    remaining,
                ) {
                    return count;
                }
            }

            // 1x1 fallback: place a single extension if it has an adjacent
            // road reachable from the hub.
            if has_adjacent_hub_road(x, y, terrain, state, hub) {
                state.place_structure(x, y, StructureType::Extension, 0);
                return 1;
            }
        }

        // Expand BFS to walkable neighbors (roads + unoccupied non-wall).
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = x as i16 + dx as i16;
            let ny = y as i16 + dy as i16;
            if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                continue;
            }
            let ux = nx as u8;
            let uy = ny as u8;
            let idx = uy as usize * 50 + ux as usize;
            if visited[idx] {
                continue;
            }
            if terrain.is_wall(ux, uy) {
                continue;
            }
            // Walkable = not occupied by a non-road structure.
            if state.is_occupied(ux, uy) {
                continue;
            }
            visited[idx] = true;
            queue.push_back((ux, uy));
        }
    }

    // BFS exhausted.
    0
}

// ---------------------------------------------------------------------------
// Stamp placement with anchor offsets
// ---------------------------------------------------------------------------

/// Try to place a stamp such that at least one of its extension tiles lands
/// on `(cx, cy)`.  Tries all valid anchor offsets.  Returns `Some(count)` on
/// success, `None` if no offset works.
#[allow(clippy::too_many_arguments)]
fn try_stamp_with_offsets(
    stamp_def: &ExtensionStampDef,
    ext_offsets: &[(i8, i8)],
    cx: u8,
    cy: u8,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
    hub: Location,
    remaining: u8,
) -> Option<u8> {
    // Deduplicate candidate anchors.
    let mut tried_anchors: FnvHashSet<(u8, u8)> = FnvHashSet::default();

    for &(edx, edy) in ext_offsets {
        let ax = cx as i16 - edx as i16;
        let ay = cy as i16 - edy as i16;
        if !(0..50).contains(&ax) || !(0..50).contains(&ay) {
            continue;
        }
        let ax = ax as u8;
        let ay = ay as u8;
        if !tried_anchors.insert((ax, ay)) {
            continue;
        }

        if let Some(count) = try_place_stamp(stamp_def, ax, ay, terrain, state, hub, remaining) {
            return Some(count);
        }
    }

    None
}

/// Try to place a stamp at the given anchor.  Returns `Some(count)` with the
/// number of extensions placed if the stamp was accepted, or `None` if the
/// stamp should be skipped.
///
/// After tentative placement, verifies road connectivity to hub and extension
/// fillability.  Disconnected stamp roads are connected via short pathfound
/// road links.  Unfillable extensions are removed.
fn try_place_stamp(
    stamp_def: &ExtensionStampDef,
    anchor_x: u8,
    anchor_y: u8,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
    hub: Location,
    remaining: u8,
) -> Option<u8> {
    let stamp = &stamp_def.stamp;

    // Quick check: do required placements fit terrain?
    if !stamp.fits_at(anchor_x, anchor_y, terrain) {
        return None;
    }

    // Get filtered placements.
    let placements = stamp.place_at_filtered(anchor_x, anchor_y, terrain, &state.structures);

    // Separate into roads and extensions.
    let mut road_placements: Vec<(u8, u8)> = Vec::new();
    let mut ext_placements: Vec<(u8, u8)> = Vec::new();

    for &(px, py, st, _rcl) in &placements {
        if st == StructureType::Road && !state.has_any_structure(px, py) {
            road_placements.push((px, py));
        } else if st == StructureType::Extension && !state.has_any_structure(px, py) {
            ext_placements.push((px, py));
        }
    }

    // Limit extensions to remaining quota.
    if ext_placements.len() > remaining as usize {
        ext_placements.truncate(remaining as usize);
    }

    // Check minimum extension count before doing any work.
    if (ext_placements.len() as u8) < stamp_def.min_extensions {
        return None;
    }

    // --- Tentative placement ---
    let snapshot_structures = state.structures.clone();
    let snapshot_occupied = state.occupied.clone();

    // Place roads (dedup against existing).
    place_roads(&road_placements, state);

    // Place extensions.
    for &(ex, ey) in &ext_placements {
        state.place_structure(ex, ey, StructureType::Extension, 0);
    }

    // --- Per-stamp road connectivity verification ---
    let mut bfs = walkable_bfs(hub, terrain, state);

    // Find disconnected stamp roads and connect them.
    let disconnected_roads: Vec<(u8, u8)> = road_placements
        .iter()
        .copied()
        .filter(|&(rx, ry)| bfs[ry as usize * 50 + rx as usize] == u32::MAX)
        .collect();

    if !disconnected_roads.is_empty() {
        // Try to connect each disconnected road segment to the hub network.
        for &(rx, ry) in &disconnected_roads {
            if bfs[ry as usize * 50 + rx as usize] != u32::MAX {
                continue; // Already connected by a previous link.
            }
            if let Some(path) = pathfind_road_connection(rx, ry, terrain, state, &bfs) {
                place_roads(&path, state);
                // Recompute BFS after adding connecting roads.
                bfs = walkable_bfs(hub, terrain, state);
            }
        }
    }

    // --- Extension fillability check ---
    // Each extension must have at least one adjacent road with bfs dist != MAX.
    let mut fillable_exts: Vec<(u8, u8)> = Vec::new();
    let mut unfillable_exts: Vec<(u8, u8)> = Vec::new();

    for &(ex, ey) in &ext_placements {
        if has_adjacent_reachable_road(ex, ey, state, &bfs) {
            fillable_exts.push((ex, ey));
        } else {
            unfillable_exts.push((ex, ey));
        }
    }

    // Remove unfillable extensions from state.
    if !unfillable_exts.is_empty() {
        for &(ux, uy) in &unfillable_exts {
            let loc = Location::from_coords(ux as u32, uy as u32);
            if let Some(items) = state.structures.get_mut(&loc) {
                items.retain(|i| i.structure_type != StructureType::Extension);
                if items.is_empty() {
                    state.structures.remove(&loc);
                }
            }
            state.occupied.remove(&loc);
        }
    }

    let placed = fillable_exts.len() as u8;

    if placed < stamp_def.min_extensions {
        // Not enough fillable extensions — full rollback.
        state.structures = snapshot_structures;
        state.occupied = snapshot_occupied;
        return None;
    }

    Some(placed)
}

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Check if a tile already has a road placed on it.
fn tile_has_road(state: &PlacementState, x: u8, y: u8) -> bool {
    let loc = Location::from_coords(x as u32, y as u32);
    state
        .structures
        .get(&loc)
        .map(|items| items.iter().any(|i| i.structure_type == StructureType::Road))
        .unwrap_or(false)
}

/// Place road tiles, skipping tiles that already have any structure.
fn place_roads(roads: &[(u8, u8)], state: &mut PlacementState) {
    for &(rx, ry) in roads {
        if !state.has_any_structure(rx, ry) {
            state.place_structure(rx, ry, StructureType::Road, 0);
        }
    }
}

/// BFS from hub over walkable tiles (roads + unoccupied non-wall).
/// Returns a flat 50x50 distance array.
fn walkable_bfs(hub: Location, terrain: &FastRoomTerrain, state: &PlacementState) -> Vec<u32> {
    let mut dist = vec![u32::MAX; 50 * 50];
    let mut queue: VecDeque<(u8, u8, u32)> = VecDeque::new();

    let hx = hub.x();
    let hy = hub.y();
    dist[hy as usize * 50 + hx as usize] = 0;
    queue.push_back((hx, hy, 0));

    while let Some((x, y, d)) = queue.pop_front() {
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = x as i16 + dx as i16;
            let ny = y as i16 + dy as i16;
            if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                continue;
            }
            let ux = nx as u8;
            let uy = ny as u8;
            let idx = uy as usize * 50 + ux as usize;
            if dist[idx] != u32::MAX {
                continue;
            }
            if terrain.is_wall(ux, uy) {
                continue;
            }
            if state.is_occupied(ux, uy) {
                continue;
            }
            dist[idx] = d + 1;
            queue.push_back((ux, uy, d + 1));
        }
    }

    dist
}

/// Check whether tile (x, y) has at least one adjacent road tile that is
/// reachable from the hub (bfs dist != MAX).
fn has_adjacent_reachable_road(x: u8, y: u8, state: &PlacementState, bfs: &[u32]) -> bool {
    for &(dx, dy) in &NEIGHBORS_8 {
        let nx = x as i16 + dx as i16;
        let ny = y as i16 + dy as i16;
        if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
            continue;
        }
        let ux = nx as u8;
        let uy = ny as u8;
        // Must be a road tile.
        let loc = Location::from_coords(ux as u32, uy as u32);
        let is_road = state
            .structures
            .get(&loc)
            .map(|items| items.iter().any(|i| i.structure_type == StructureType::Road))
            .unwrap_or(false);
        if !is_road {
            continue;
        }
        // Must be reachable from hub.
        let idx = uy as usize * 50 + ux as usize;
        if bfs[idx] != u32::MAX {
            return true;
        }
    }
    false
}

/// Quick check: does tile (x, y) have an adjacent road that is path-connected
/// to the hub?  Used for 1x1 fallback placement.
fn has_adjacent_hub_road(
    x: u8,
    y: u8,
    terrain: &FastRoomTerrain,
    state: &PlacementState,
    hub: Location,
) -> bool {
    // Run a full walkable BFS to check — this is only called for 1x1
    // fallback after all stamps have been exhausted, so it's rare.
    let bfs = walkable_bfs(hub, terrain, state);
    has_adjacent_reachable_road(x, y, state, &bfs)
}

/// Pathfind a short road connection from a disconnected road tile to the
/// nearest tile reachable from the hub.  Returns the path as a list of
/// (x, y) positions to place roads on, or None if no connection is possible.
///
/// BFS from the disconnected tile through non-wall, non-occupied tiles until
/// we reach a tile with bfs[idx] != MAX (already connected to hub).
fn pathfind_road_connection(
    start_x: u8,
    start_y: u8,
    terrain: &FastRoomTerrain,
    state: &PlacementState,
    hub_bfs: &[u32],
) -> Option<Vec<(u8, u8)>> {
    let mut visited = vec![false; 50 * 50];
    let mut parent: Vec<Option<(u8, u8)>> = vec![None; 50 * 50];
    let mut queue: VecDeque<(u8, u8)> = VecDeque::new();

    let start_idx = start_y as usize * 50 + start_x as usize;
    visited[start_idx] = true;
    queue.push_back((start_x, start_y));

    let mut found: Option<(u8, u8)> = None;

    while let Some((x, y)) = queue.pop_front() {
        let idx = y as usize * 50 + x as usize;
        // If this tile is already reachable from hub, we found the connection.
        if hub_bfs[idx] != u32::MAX && (x != start_x || y != start_y) {
            found = Some((x, y));
            break;
        }

        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = x as i16 + dx as i16;
            let ny = y as i16 + dy as i16;
            if !(1..49).contains(&nx) || !(1..49).contains(&ny) {
                continue;
            }
            let ux = nx as u8;
            let uy = ny as u8;
            let nidx = uy as usize * 50 + ux as usize;
            if visited[nidx] {
                continue;
            }
            if terrain.is_wall(ux, uy) {
                continue;
            }
            if state.is_occupied(ux, uy) {
                continue;
            }
            visited[nidx] = true;
            parent[nidx] = Some((x, y));
            queue.push_back((ux, uy));
        }
    }

    // Reconstruct path (excluding start and end — start is already a road,
    // end is already reachable).
    let end = found?;
    let mut path = Vec::new();
    let mut cur = end;
    loop {
        if cur.0 == start_x && cur.1 == start_y {
            break;
        }
        path.push(cur);
        let idx = cur.1 as usize * 50 + cur.0 as usize;
        cur = parent[idx]?;
    }
    path.reverse();

    Some(path)
}
