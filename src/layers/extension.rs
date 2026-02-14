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

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::stamps::extension::{extension_stamps, ExtensionStampDef};
use crate::terrain::*;
use fnv::FnvHashSet;
use log::*;
use std::collections::VecDeque;

use screeps::constants::StructureType;

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
        _analysis: &AnalysisOutput,
        _terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(1)
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        _analysis: &AnalysisOutput,
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
    let mut visited = vec![false; ROOM_AREA];
    let mut queue: VecDeque<Location> = VecDeque::new();

    // The hub tile may itself be occupied (e.g. if the hub landmark is on a
    // spawn rather than a road).  Seed the BFS from all road tiles adjacent
    // to (and including) the hub so we can escape the hub cluster.
    let seed_tiles: Vec<Location> = {
        let mut seeds = vec![hub];
        for &(dx, dy) in &NEIGHBORS_8 {
            if let Some(nloc) = hub.checked_add(dx, dy) {
                seeds.push(nloc);
            }
        }
        seeds
    };

    for &seed in &seed_tiles {
        let idx = seed.to_index();
        if visited[idx] {
            continue;
        }
        if terrain.is_wall_at(seed) {
            continue;
        }
        if state.is_occupied(seed) {
            continue;
        }
        visited[idx] = true;
        queue.push_back(seed);
    }

    while let Some(loc) = queue.pop_front() {
        // If this tile is unoccupied, not a wall, and not already a road,
        // it's a candidate for stamp placement (or 1x1 fallback).
        if !state.is_occupied(loc)
            && !terrain.is_wall_at(loc)
            && !tile_has_road(state, loc)
            && loc.is_interior()
        {
            // Try stamps largest-first with anchor offsets.
            for (si, stamp_def) in stamps.iter().enumerate() {
                if let Some(count) = try_stamp_with_offsets(
                    stamp_def,
                    &stamp_ext_offsets[si],
                    loc,
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
            if has_adjacent_hub_road(loc, terrain, state, hub) {
                state.place_structure_auto_rcl(loc, StructureType::Extension);
                return 1;
            }
        }

        // Expand BFS to walkable neighbors (roads + unoccupied non-wall).
        for &(dx, dy) in &NEIGHBORS_8 {
            if let Some(nloc) = loc.checked_add(dx, dy) {
                let idx = nloc.to_index();
                if visited[idx] {
                    continue;
                }
                if terrain.is_wall_at(nloc) {
                    continue;
                }
                // Walkable = not occupied by a non-road structure.
                if state.is_occupied(nloc) {
                    continue;
                }
                visited[idx] = true;
                queue.push_back(nloc);
            }
        }
    }

    // BFS exhausted.
    0
}

// ---------------------------------------------------------------------------
// Stamp placement with anchor offsets
// ---------------------------------------------------------------------------

/// Try to place a stamp such that at least one of its extension tiles lands
/// on `center`.  Tries all valid anchor offsets.  Returns `Some(count)` on
/// success, `None` if no offset works.
#[allow(clippy::too_many_arguments)]
fn try_stamp_with_offsets(
    stamp_def: &ExtensionStampDef,
    ext_offsets: &[(i8, i8)],
    center: Location,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
    hub: Location,
    remaining: u8,
) -> Option<u8> {
    // Deduplicate candidate anchors.
    let mut tried_anchors: FnvHashSet<Location> = FnvHashSet::default();

    let cx = center.x();
    let cy = center.y();

    for &(edx, edy) in ext_offsets {
        let ax = cx as i16 - edx as i16;
        let ay = cy as i16 - edy as i16;
        if !xy_in_bounds(ax, ay) {
            continue;
        }
        let anchor = Location::from_xy(ax as u8, ay as u8);
        if !tried_anchors.insert(anchor) {
            continue;
        }

        if let Some(count) = try_place_stamp(stamp_def, anchor, terrain, state, hub, remaining) {
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
    anchor: Location,
    terrain: &FastRoomTerrain,
    state: &mut PlacementState,
    hub: Location,
    remaining: u8,
) -> Option<u8> {
    let stamp = &stamp_def.stamp;
    let anchor_x = anchor.x();
    let anchor_y = anchor.y();

    // Quick check: do required placements fit terrain and exclusions?
    if !stamp.fits_at(anchor_x, anchor_y, terrain, &state.excluded) {
        return None;
    }

    // Get filtered placements.
    let placements = stamp.place_at_filtered(
        anchor_x,
        anchor_y,
        terrain,
        &state.structures,
        &state.excluded,
    );

    // Separate into roads and extensions.
    let mut road_placements: Vec<Location> = Vec::new();
    let mut ext_placements: Vec<Location> = Vec::new();

    for &(px, py, st, _rcl) in &placements {
        let ploc = Location::from_xy(px, py);
        if st == StructureType::Road && !state.has_any_structure(ploc) {
            road_placements.push(ploc);
        } else if st == StructureType::Extension && !state.has_any_structure(ploc) {
            ext_placements.push(ploc);
        }
    }

    // Limit extensions to remaining quota, keeping those nearest to the hub
    // by structure-aware pathing distance.  This prevents holes near the hub
    // when the last stamp is truncated.
    if ext_placements.len() > remaining as usize {
        // Compute structure-aware distances so we sort by actual pathing
        // distance around placed structures, not straight-line Chebyshev.
        if let Some(hub_dists) = state.hub_pathing_distances(terrain) {
            ext_placements.sort_by_key(|&eloc| {
                (*hub_dists.get(eloc.x() as usize, eloc.y() as usize)).unwrap_or(u32::MAX)
            });
        } else {
            // Fallback: sort by Chebyshev distance to hub.
            ext_placements.sort_by_key(|&eloc| hub.distance_to(eloc) as u32);
        }
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
    for &eloc in &ext_placements {
        state.place_structure_auto_rcl(eloc, StructureType::Extension);
    }

    // --- Per-stamp road connectivity verification ---
    let mut bfs = walkable_bfs(hub, terrain, state);

    // Find disconnected stamp roads and connect them.
    let disconnected_roads: Vec<Location> = road_placements
        .iter()
        .copied()
        .filter(|&rloc| bfs[rloc.to_index()] == u32::MAX)
        .collect();

    if !disconnected_roads.is_empty() {
        // Try to connect each disconnected road segment to the hub network.
        for &rloc in &disconnected_roads {
            if bfs[rloc.to_index()] != u32::MAX {
                continue; // Already connected by a previous link.
            }
            if let Some(path) = pathfind_road_connection(rloc, terrain, state, &bfs) {
                place_roads(&path, state);
                // Recompute BFS after adding connecting roads.
                bfs = walkable_bfs(hub, terrain, state);
            }
        }
    }

    // --- Extension fillability check ---
    // Each extension must have at least one adjacent road with bfs dist != MAX.
    let mut fillable_exts: Vec<Location> = Vec::new();
    let mut unfillable_exts: Vec<Location> = Vec::new();

    for &eloc in &ext_placements {
        if has_adjacent_reachable_road(eloc, state, &bfs) {
            fillable_exts.push(eloc);
        } else {
            unfillable_exts.push(eloc);
        }
    }

    // Remove unfillable extensions from state.
    if !unfillable_exts.is_empty() {
        for &uloc in &unfillable_exts {
            if let Some(items) = state.structures.get_mut(&uloc) {
                items.retain(|i| i.structure_type != StructureType::Extension);
                if items.is_empty() {
                    state.structures.remove(&uloc);
                }
            }
            state.occupied.remove(&uloc);
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
fn tile_has_road(state: &PlacementState, loc: Location) -> bool {
    state
        .structures
        .get(&loc)
        .map(|items| {
            items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
        })
        .unwrap_or(false)
}

/// Place road tiles, skipping tiles that already have any structure or are excluded.
fn place_roads(roads: &[Location], state: &mut PlacementState) {
    for &rloc in roads {
        if !state.has_any_structure(rloc) && !state.is_excluded(rloc) {
            state.place_structure_auto_rcl(rloc, StructureType::Road);
        }
    }
}

/// BFS from hub over walkable tiles (roads + unoccupied non-wall).
/// Returns a flat 50x50 distance array.
fn walkable_bfs(hub: Location, terrain: &FastRoomTerrain, state: &PlacementState) -> Vec<u32> {
    let mut dist = vec![u32::MAX; ROOM_AREA];
    let mut queue: VecDeque<(Location, u32)> = VecDeque::new();

    dist[hub.to_index()] = 0;
    queue.push_back((hub, 0));

    while let Some((loc, d)) = queue.pop_front() {
        for &(dx, dy) in &NEIGHBORS_8 {
            if let Some(nloc) = loc.checked_add(dx, dy) {
                let idx = nloc.to_index();
                if dist[idx] != u32::MAX {
                    continue;
                }
                if terrain.is_wall_at(nloc) {
                    continue;
                }
                if state.is_occupied(nloc) {
                    continue;
                }
                dist[idx] = d + 1;
                queue.push_back((nloc, d + 1));
            }
        }
    }

    dist
}

/// Check whether a tile has at least one adjacent road tile that is
/// reachable from the hub (bfs dist != MAX).
fn has_adjacent_reachable_road(loc: Location, state: &PlacementState, bfs: &[u32]) -> bool {
    for &(dx, dy) in &NEIGHBORS_8 {
        if let Some(nloc) = loc.checked_add(dx, dy) {
            // Must be a road tile.
            let is_road = state
                .structures
                .get(&nloc)
                .map(|items| {
                    items
                        .iter()
                        .any(|i| i.structure_type == StructureType::Road)
                })
                .unwrap_or(false);
            if !is_road {
                continue;
            }
            // Must be reachable from hub.
            if bfs[nloc.to_index()] != u32::MAX {
                return true;
            }
        }
    }
    false
}

/// Quick check: does a tile have an adjacent road that is path-connected
/// to the hub?  Used for 1x1 fallback placement.
fn has_adjacent_hub_road(
    loc: Location,
    terrain: &FastRoomTerrain,
    state: &PlacementState,
    hub: Location,
) -> bool {
    // Run a full walkable BFS to check — this is only called for 1x1
    // fallback after all stamps have been exhausted, so it's rare.
    let bfs = walkable_bfs(hub, terrain, state);
    has_adjacent_reachable_road(loc, state, &bfs)
}

/// Pathfind a short road connection from a disconnected road tile to the
/// nearest tile reachable from the hub.  Returns the path as a list of
/// locations to place roads on, or None if no connection is possible.
///
/// BFS from the disconnected tile through non-wall, non-occupied tiles until
/// we reach a tile with bfs[idx] != MAX (already connected to hub).
fn pathfind_road_connection(
    start: Location,
    terrain: &FastRoomTerrain,
    state: &PlacementState,
    hub_bfs: &[u32],
) -> Option<Vec<Location>> {
    let mut visited = vec![false; ROOM_AREA];
    let mut parent: Vec<Option<Location>> = vec![None; ROOM_AREA];
    let mut queue: VecDeque<Location> = VecDeque::new();

    let start_idx = start.to_index();
    visited[start_idx] = true;
    queue.push_back(start);

    let mut found: Option<Location> = None;

    while let Some(loc) = queue.pop_front() {
        let idx = loc.to_index();
        // If this tile is already reachable from hub, we found the connection.
        if hub_bfs[idx] != u32::MAX && loc != start {
            found = Some(loc);
            break;
        }

        for &(dx, dy) in &NEIGHBORS_8 {
            // Use checked_add (bounds check) then filter to interior tiles.
            if let Some(nloc) = loc.checked_add(dx, dy) {
                if !nloc.is_interior() {
                    continue;
                }
                let nidx = nloc.to_index();
                if visited[nidx] {
                    continue;
                }
                if terrain.is_wall_at(nloc) {
                    continue;
                }
                if state.is_occupied(nloc) {
                    continue;
                }
                visited[nidx] = true;
                parent[nidx] = Some(loc);
                queue.push_back(nloc);
            }
        }
    }

    // Reconstruct path (excluding start and end — start is already a road,
    // end is already reachable).
    let end = found?;
    let mut path = Vec::new();
    let mut cur = end;
    loop {
        if cur == start {
            break;
        }
        path.push(cur);
        let idx = cur.to_index();
        cur = parent[idx]?;
    }
    path.reverse();

    Some(path)
}
