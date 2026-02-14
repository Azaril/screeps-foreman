//! DefenseLayer: Places ramparts and walls using a true max-flow min-cut algorithm.
//! Deterministic (1 candidate).
//!
//! The min-cut is computed on a node-split flow network:
//!   - Each passable tile becomes two graph nodes (in, out) connected by a
//!     capacity-1 edge. Cutting this edge means placing a rampart on that tile.
//!   - Adjacent passable tiles are connected out→in with infinite capacity.
//!   - A virtual source feeds all exit tiles (infinite capacity).
//!   - A virtual sink drains all protected tiles (infinite capacity).
//!
//! Distance enforcement is handled by expanding the protected region around
//! structures: valuable structures (spawns, towers, labs, hub) are expanded by
//! 3 tiles, the controller gets a 1-tile buffer to prevent `attackController`,
//! while mining infrastructure (source containers, mineral containers,
//! extractors) uses no buffer. This ensures the min-cut is naturally pushed
//! outward to maintain safe distances without a separate enforcement pass.
//!
//! Cut tiles are alternated between walls and ramparts using a checkerboard
//! pattern. Walls have higher hit points and decay slower, providing stronger
//! defense. Ramparts allow friendly creep movement, maintaining pathfinding
//! connectivity through the defensive line.

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::plan::*;
use crate::terrain::*;
use fnv::FnvHashSet;
use log::*;
use rs_graph::builder::Builder;
use rs_graph::maxflow::dinic;
use rs_graph::traits::*;
use rs_graph::Net;
use std::collections::VecDeque;

use screeps::constants::StructureType;

/// Infinite capacity sentinel for edges that should never be cut.
const INF_CAP: u32 = ROOM_AREA as u32 + 1;

/// Places ramparts via true max-flow min-cut.
/// Deterministic (1 candidate).
pub struct DefenseLayer;

impl PlacementLayer for DefenseLayer {
    fn name(&self) -> &str {
        "defense"
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
        analysis: &AnalysisOutput,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        if index > 0 {
            return None;
        }

        if state.get_landmark("hub").is_none() {
            return Some(Err(()));
        }

        let mut new_state = state.clone();

        // Build the set of protected tiles by expanding around structures with
        // per-type buffer distances. This replaces the old enforce_range3 pass.
        // If the required protected region overlaps exit tiles the plan is
        // rejected -- exit tiles are source nodes in the flow network and
        // cannot be on the sink side without making the min-cut degenerate.
        let protected = match build_protected_region(state, analysis, terrain) {
            Ok(p) => p,
            Err(()) => return Some(Err(())),
        };

        // Compute min-cut tiles
        let cut_locations = compute_min_cut(&protected, analysis, terrain);

        // Classify cut tiles into walls and ramparts. Walls are stronger but
        // block movement; ramparts allow friendly creeps through. We use a
        // checkerboard pattern so that every wall has adjacent rampart
        // neighbors, maintaining pathfinding connectivity.
        let cut_set: FnvHashSet<Location> = cut_locations.iter().copied().collect();
        let (walls, ramparts) = classify_wall_rampart(&cut_locations, &cut_set);

        // Place walls (blocking, higher HP, slower decay)
        for &loc in &walls {
            new_state
                .structures
                .entry(loc)
                .or_default()
                .push(RoomItem::new(StructureType::Wall, 2));
            new_state.occupied.insert(loc);
            new_state.add_to_landmark_set("ramparts", loc);
        }

        // Place ramparts (passable to friendly creeps)
        for &loc in &ramparts {
            new_state
                .structures
                .entry(loc)
                .or_default()
                .push(RoomItem::new(StructureType::Rampart, 2));
            new_state.add_to_landmark_set("ramparts", loc);
        }

        // Validate: flood-fill from exits stopping at cut tiles. Any
        // non-mineral structure reachable from exits without crossing the
        // wall is undefended -- reject the plan.
        if !validate_all_structures_defended(state, analysis, terrain, &cut_set) {
            return Some(Err(()));
        }

        Some(Ok(new_state))
    }
}

/// Expand a set of locations by `radius` tiles (Chebyshev distance), skipping
/// walls and out-of-bounds tiles.
fn expand_region(
    seeds: impl Iterator<Item = Location>,
    radius: u8,
    terrain: &FastRoomTerrain,
    out: &mut FnvHashSet<Location>,
) {
    let r = radius as i16;
    for loc in seeds {
        for dy in -r..=r {
            for dx in -r..=r {
                let x = loc.x() as i16 + dx;
                let y = loc.y() as i16 + dy;
                if xy_in_bounds(x, y) && !terrain.is_wall(x as u8, y as u8) {
                    out.insert(Location::from_xy(x as u8, y as u8));
                }
            }
        }
    }
}

/// Classify min-cut tiles into walls and ramparts.
///
/// Uses a checkerboard pattern `(x + y) % 2` as the initial assignment: even
/// parity → wall, odd parity → rampart. A wall candidate is downgraded to a
/// rampart if it has no adjacent cut-tile that is already a rampart, which
/// would leave no passable path through the defensive line at that point.
///
/// Returns `(walls, ramparts)`.
fn classify_wall_rampart(
    cut_locations: &[Location],
    cut_set: &FnvHashSet<Location>,
) -> (Vec<Location>, Vec<Location>) {
    // Initial assignment: checkerboard parity decides wall vs rampart.
    let mut is_wall: FnvHashSet<Location> = FnvHashSet::default();
    let mut is_rampart: FnvHashSet<Location> = FnvHashSet::default();

    for &loc in cut_locations {
        if (loc.x() ^ loc.y()) & 1 == 0 {
            is_wall.insert(loc);
        } else {
            is_rampart.insert(loc);
        }
    }

    // Safety pass: every wall must have at least one adjacent rampart in the
    // cut set. If not, downgrade it to a rampart so creeps can still path
    // through the defensive line.
    let walls_snapshot: Vec<Location> = is_wall.iter().copied().collect();
    for loc in walls_snapshot {
        let has_adjacent_rampart = NEIGHBORS_8.iter().any(|&(dx, dy)| {
            if let Some(nloc) = loc.checked_add(dx, dy) {
                cut_set.contains(&nloc) && is_rampart.contains(&nloc)
            } else {
                false
            }
        });

        if !has_adjacent_rampart {
            is_wall.remove(&loc);
            is_rampart.insert(loc);
        }
    }

    let walls: Vec<Location> = cut_locations
        .iter()
        .copied()
        .filter(|l| is_wall.contains(l))
        .collect();
    let ramparts: Vec<Location> = cut_locations
        .iter()
        .copied()
        .filter(|l| is_rampart.contains(l))
        .collect();

    (walls, ramparts)
}

/// Build the set of tiles that the min-cut must protect.
///
/// The protected region is built in layers:
/// 1. Valuable structures (spawns, towers, labs, hub) get a 3-tile buffer.
/// 2. Controller: 1-tile buffer to protect against `attackController`.
/// 3. All occupied structure tiles are protected with no extra buffer.
/// 4. Mining infrastructure (source/mineral containers) gets no buffer.
///
/// Returns `Err(())` if the required protected region includes exit tiles
/// (row/col 0 or 49). Exit tiles are source nodes in the flow network; if
/// they also appear on the sink side the min-cut becomes degenerate and the
/// plan cannot produce a valid defensive perimeter.
fn build_protected_region(
    state: &PlacementState,
    analysis: &AnalysisOutput,
    terrain: &FastRoomTerrain,
) -> Result<FnvHashSet<Location>, ()> {
    let mut protected: FnvHashSet<Location> = FnvHashSet::default();

    // Valuable structures: expand by 3 tiles. This covers spawns, towers,
    // labs, and the hub itself with enough buffer to keep ranged attackers
    // from hitting them without breaching the perimeter.
    let hub_loc = state.get_landmark("hub");
    let valuable = state
        .get_landmark_set("spawns")
        .iter()
        .chain(state.get_landmark_set("towers").iter())
        .chain(state.get_landmark_set("labs").iter())
        .chain(hub_loc.iter())
        .copied();
    expand_region(valuable, 3, terrain, &mut protected);

    // Controller: protect with a 1-tile buffer. The controller is a
    // high-value target -- enemies can use `attackController` to drain
    // downgrade ticks. A 1-tile buffer ensures the min-cut wall is pushed
    // out far enough that hostile creeps cannot reach an adjacent tile to
    // attack the controller without first breaching the perimeter.
    let controller_locs: Vec<Location> = analysis
        .controller_distances
        .iter()
        .map(|(loc, _, _)| *loc)
        .collect();
    expand_region(controller_locs.into_iter(), 1, terrain, &mut protected);

    // All occupied structure tiles are protected with no extra buffer --
    // this catches everything (extensions, roads, links, etc.) that should
    // simply be inside the wall.
    expand_region(state.occupied.iter().copied(), 0, terrain, &mut protected);

    // Mining infrastructure: no buffer (just the tile itself). These are
    // already covered by the occupied set above, but we list them explicitly
    // for clarity and in case they are ever removed from `occupied`.
    let extractor_loc = state.get_landmark("extractor");
    let mineral_container_loc = state.get_landmark("mineral_container");
    let mining = state
        .get_landmark_set("source_containers")
        .iter()
        .chain(extractor_loc.iter())
        .chain(mineral_container_loc.iter())
        .copied();
    expand_region(mining, 0, terrain, &mut protected);

    // Reject if the protected region overlaps exit tiles. Exit tiles are
    // source nodes in the flow network; placing them on the sink side too
    // makes the min-cut degenerate and the wall cannot protect those tiles.
    let has_exit_overlap = protected.iter().any(|loc| loc.is_border());
    if has_exit_overlap {
        trace!("Defense: protected region overlaps exit tiles, rejecting plan");
        return Err(());
    }

    Ok(protected)
}

/// Compute min-cut rampart positions using a true max-flow / min-cut algorithm.
///
/// Constructs a node-split directed graph and runs Dinic's algorithm to find
/// the minimum vertex cut separating room exits from the protected region.
fn compute_min_cut(
    protected: &FnvHashSet<Location>,
    analysis: &crate::pipeline::analysis::AnalysisOutput,
    terrain: &FastRoomTerrain,
) -> Vec<Location> {
    // --- 1. Identify passable tiles and assign indices ---

    // For each passable tile (x, y) we create two graph nodes:
    //   in-node  = tile_in[x][y]
    //   out-node = tile_out[x][y]
    // Connected by a directed edge in→out with capacity 1.

    let exits = &analysis.exits.all;
    let exit_set: FnvHashSet<Location> = exits.iter().copied().collect();

    // Map (x,y) → sequential tile index (only passable, non-wall tiles).
    let mut tile_index: [[Option<usize>; ROOM_WIDTH as usize]; ROOM_HEIGHT as usize] =
        [[None; ROOM_WIDTH as usize]; ROOM_HEIGHT as usize];
    let mut tile_coords: Vec<Location> = Vec::new();

    for y in 0..ROOM_HEIGHT {
        for x in 0..ROOM_WIDTH {
            if !terrain.is_wall(x, y) {
                let idx = tile_coords.len();
                tile_index[x as usize][y as usize] = Some(idx);
                tile_coords.push(Location::from_xy(x, y));
            }
        }
    }

    let num_tiles = tile_coords.len();
    if num_tiles == 0 {
        return Vec::new();
    }

    // --- 2. Build the flow network ---
    //
    // Node layout:
    //   0 .. num_tiles-1           : in-nodes  (tile_in[i]  = i)
    //   num_tiles .. 2*num_tiles-1 : out-nodes (tile_out[i] = num_tiles + i)
    //   2*num_tiles                : virtual source
    //   2*num_tiles + 1            : virtual sink
    //
    // Edges:
    //   in[i] → out[i]  capacity 1   (for each passable tile)
    //   out[i] → in[j]  capacity INF  (for each pair of adjacent passable tiles)
    //   source → in[i]  capacity INF  (for each exit tile i)
    //   out[i] → sink   capacity INF  (for each protected tile i)

    let num_nodes = 2 * num_tiles + 2;
    let source_idx = 2 * num_tiles;
    let sink_idx = 2 * num_tiles + 1;

    // Pre-count edges for capacity hint:
    //   num_tiles (in→out) + adjacency edges + exit edges + protected edges
    // Adjacency is at most 8 * num_tiles but we don't need an exact count.
    let edge_estimate = num_tiles + 8 * num_tiles + exit_set.len() + protected.len();

    let mut builder =
        <Net as rs_graph::builder::Buildable>::Builder::with_capacities(num_nodes, edge_estimate);
    let nodes: Vec<_> = (0..num_nodes).map(|_| builder.add_node()).collect();
    let mut capacities: Vec<u32> = Vec::with_capacity(edge_estimate);

    // Helper: add a directed edge and record its capacity.
    // rs-graph's Net is undirected internally but dinic treats it as directed
    // via the (u, v) convention; we add edges u→v.
    let add_edge = |b: &mut <Net as rs_graph::builder::Buildable>::Builder,
                    caps: &mut Vec<u32>,
                    u: usize,
                    v: usize,
                    cap: u32| {
        b.add_edge(nodes[u], nodes[v]);
        caps.push(cap);
    };

    // In→Out edges (capacity 1 per tile)
    for i in 0..num_tiles {
        add_edge(&mut builder, &mut capacities, i, num_tiles + i, 1);
    }

    // Adjacency edges: out[i] → in[j] for each neighbor j of tile i
    for (i, &loc) in tile_coords.iter().enumerate() {
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = loc.x() as i16 + dx as i16;
            let ny = loc.y() as i16 + dy as i16;
            if !xy_in_bounds(nx, ny) {
                continue;
            }
            if let Some(j) = tile_index[nx as usize][ny as usize] {
                add_edge(&mut builder, &mut capacities, num_tiles + i, j, INF_CAP);
            }
        }
    }

    // Source → in[i] for exit tiles
    for &eloc in &exit_set {
        if let Some(i) = tile_index[eloc.x() as usize][eloc.y() as usize] {
            add_edge(&mut builder, &mut capacities, source_idx, i, INF_CAP);
        }
    }

    // Out[i] → sink for protected tiles
    for &ploc in protected {
        if let Some(i) = tile_index[ploc.x() as usize][ploc.y() as usize] {
            add_edge(
                &mut builder,
                &mut capacities,
                num_tiles + i,
                sink_idx,
                INF_CAP,
            );
        }
    }

    let graph = builder.into_graph();
    let src = nodes[source_idx];
    let snk = nodes[sink_idx];

    // --- 3. Run Dinic's max-flow algorithm ---
    let (_value, _flow, mincut_nodes) = dinic(&graph, src, snk, |e| capacities[graph.edge_id(e)]);

    // --- 4. Extract rampart positions ---
    //
    // The min-cut returns the set of nodes on the source side. A tile's
    // internal edge (in→out) is cut when the in-node is in the cut set but
    // the out-node is not. Those tiles are where we place ramparts.
    //
    // Build a fast lookup of cut-side nodes.
    let cut_set: FnvHashSet<usize> = mincut_nodes.iter().map(|n| graph.node_id(*n)).collect();

    let mut ramparts: Vec<Location> = Vec::new();
    for i in 0..num_tiles {
        let in_node_id = graph.node_id(nodes[i]);
        let out_node_id = graph.node_id(nodes[num_tiles + i]);
        // In-node on source side, out-node on sink side → this tile is cut
        if cut_set.contains(&in_node_id) && !cut_set.contains(&out_node_id) {
            let loc = tile_coords[i];
            // Only place ramparts on buildable interior tiles
            if !loc.is_border() {
                ramparts.push(loc);
            }
        }
    }

    ramparts
}

/// Validate that all core structures are on the defended (interior) side of
/// the wall. Flood-fills from exit tiles, stopping at cut tiles and terrain
/// walls. If any structure tile is reachable from exits without crossing a
/// cut tile, the wall has a gap and the plan is invalid.
///
/// Source and mineral infrastructure (containers, links near sources/minerals)
/// is excluded from the check because these structures may legitimately sit
/// outside the wall when the source or mineral is near an exit.
///
/// Returns `true` if all structures are defended, `false` otherwise.
fn validate_all_structures_defended(
    state: &PlacementState,
    analysis: &AnalysisOutput,
    terrain: &FastRoomTerrain,
    cut_set: &FnvHashSet<Location>,
) -> bool {
    // Collect structure locations that must be defended.
    // Skip roads (they don't need wall protection) and remote-area
    // structures (mineral and source infrastructure may be outside the wall
    // when the source/mineral is near an exit).
    let mut must_defend: FnvHashSet<Location> = FnvHashSet::default();

    // Build mineral area to exclude (same logic as ReachabilityLayer).
    let mut remote_area: FnvHashSet<Location> = FnvHashSet::default();
    for (mineral_loc, _, _) in &analysis.mineral_distances {
        remote_area.insert(*mineral_loc);
        for &(dx, dy) in &NEIGHBORS_8 {
            if let Some(nloc) = mineral_loc.checked_add(dx, dy) {
                remote_area.insert(nloc);
            }
        }
    }

    // Build source area to exclude: source tiles and their immediate
    // neighbors may host containers and links that legitimately sit
    // outside the wall when the source is near an exit.
    for (source_loc, _, _) in &analysis.source_distances {
        remote_area.insert(*source_loc);
        for &(dx, dy) in &NEIGHBORS_8 {
            if let Some(nloc) = source_loc.checked_add(dx, dy) {
                remote_area.insert(nloc);
            }
        }
    }

    for (loc, items) in &state.structures {
        let has_non_road = items
            .iter()
            .any(|i| i.structure_type != StructureType::Road);
        if !has_non_road {
            continue;
        }
        if remote_area.contains(loc) {
            continue;
        }
        must_defend.insert(*loc);
    }

    if must_defend.is_empty() {
        return true;
    }

    // Flood-fill from all exit tiles, stopping at cut tiles and walls.
    // Any tile reachable by this fill is on the "outside" (exit side).
    let exits = &analysis.exits.all;
    let mut outside: FnvHashSet<Location> = FnvHashSet::default();
    let mut queue: VecDeque<Location> = VecDeque::new();

    for exit in exits {
        if !cut_set.contains(exit) {
            outside.insert(*exit);
            queue.push_back(*exit);
        }
    }

    while let Some(loc) = queue.pop_front() {
        for &(dx, dy) in &NEIGHBORS_8 {
            let nloc = match loc.checked_add(dx, dy) {
                Some(l) => l,
                None => continue,
            };

            if outside.contains(&nloc) {
                continue;
            }
            if terrain.is_wall(nloc.x(), nloc.y()) {
                continue;
            }
            // Cut tiles (walls/ramparts) block the flood-fill.
            if cut_set.contains(&nloc) {
                continue;
            }

            outside.insert(nloc);
            queue.push_back(nloc);
        }
    }

    // Check if any structure that must be defended is on the outside.
    for loc in &must_defend {
        if outside.contains(loc) {
            trace!(
                "Defense: structure at ({}, {}) is outside the wall (undefended)",
                loc.x(),
                loc.y()
            );
            return false;
        }
    }

    true
}
