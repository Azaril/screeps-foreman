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
//! 3 tiles, while mining infrastructure (source containers, mineral containers,
//! extractors) uses no buffer. This ensures the min-cut is naturally pushed
//! outward to maintain safe distances without a separate enforcement pass.
//!
//! Cut tiles are alternated between walls and ramparts using a checkerboard
//! pattern. Walls have higher hit points and decay slower, providing stronger
//! defense. Ramparts allow friendly creep movement, maintaining pathfinding
//! connectivity through the defensive line.

use crate::layer::*;
use crate::location::*;
use crate::plan::*;
use crate::terrain::*;
use fnv::FnvHashSet;
use rs_graph::builder::Builder;
use rs_graph::maxflow::dinic;
use rs_graph::traits::*;
use rs_graph::Net;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// Infinite capacity sentinel for edges that should never be cut.
const INF_CAP: u32 = 50 * 50 + 1;

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

        // Build the set of protected tiles by expanding around structures with
        // per-type buffer distances. This replaces the old enforce_range3 pass.
        let protected = build_protected_region(hub, state, terrain);

        // Compute min-cut tiles
        let cut_locations =
            compute_min_cut(&protected, &state.analysis, terrain);

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

        // Add perimeter roads on rampart tiles (walls are impassable, no road needed)
        let rampart_set: FnvHashSet<Location> = ramparts.iter().copied().collect();
        for &rampart_loc in &ramparts {
            for &(dx, dy) in &NEIGHBORS_8 {
                let nx = rampart_loc.x() as i16 + dx as i16;
                let ny = rampart_loc.y() as i16 + dy as i16;
                if !(1..49).contains(&nx) || !(1..49).contains(&ny) {
                    continue;
                }
                let loc = Location::from_coords(nx as u32, ny as u32);
                if !rampart_set.contains(&loc)
                    && !cut_set.contains(&loc)
                    && !terrain.is_wall(nx as u8, ny as u8)
                    && new_state.occupied.contains(&loc)
                {
                    let has_structure = new_state
                        .structures
                        .get(&loc)
                        .map(|items| {
                            items
                                .iter()
                                .any(|i| i.structure_type != StructureType::Road)
                        })
                        .unwrap_or(false);
                    if !has_structure {
                        new_state
                            .structures
                            .entry(rampart_loc)
                            .or_default()
                            .push(RoomItem::new(StructureType::Road, 1));
                        break;
                    }
                }
            }
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
    out: &mut FnvHashSet<(u8, u8)>,
) {
    let r = radius as i16;
    for loc in seeds {
        for dy in -r..=r {
            for dx in -r..=r {
                let x = loc.x() as i16 + dx;
                let y = loc.y() as i16 + dy;
                if (0..50).contains(&x)
                    && (0..50).contains(&y)
                    && !terrain.is_wall(x as u8, y as u8)
                {
                    out.insert((x as u8, y as u8));
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
            let nx = loc.x() as i16 + dx as i16;
            let ny = loc.y() as i16 + dy as i16;
            if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                return false;
            }
            let nloc = Location::from_coords(nx as u32, ny as u32);
            cut_set.contains(&nloc) && is_rampart.contains(&nloc)
        });

        if !has_adjacent_rampart {
            is_wall.remove(&loc);
            is_rampart.insert(loc);
        }
    }

    let walls: Vec<Location> = cut_locations.iter().copied().filter(|l| is_wall.contains(l)).collect();
    let ramparts: Vec<Location> = cut_locations.iter().copied().filter(|l| is_rampart.contains(l)).collect();

    (walls, ramparts)
}

/// Build the set of tiles that the min-cut must protect.
///
/// Valuable structures (spawns, towers, labs, hub) get a 3-tile buffer so that
/// ramparts end up at least 3 tiles away. Mining infrastructure (source
/// containers, mineral containers, extractors) gets no buffer -- they only need
/// to be inside the wall, not at a safe distance.
fn build_protected_region(
    hub: Location,
    state: &PlacementState,
    terrain: &FastRoomTerrain,
) -> FnvHashSet<(u8, u8)> {
    let mut protected: FnvHashSet<(u8, u8)> = FnvHashSet::default();

    // Valuable structures: expand by 3 tiles
    let hub_loc = state.get_landmark("hub");
    let valuable = state
        .get_landmark_set("spawns")
        .iter()
        .chain(state.get_landmark_set("towers").iter())
        .chain(state.get_landmark_set("labs").iter())
        .chain(hub_loc.iter())
        .copied();
    expand_region(valuable, 3, terrain, &mut protected);

    // Hub area: ensure a generous protected zone around the hub
    expand_region(std::iter::once(hub), 8, terrain, &mut protected);

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

    protected
}

/// Compute min-cut rampart positions using a true max-flow / min-cut algorithm.
///
/// Constructs a node-split directed graph and runs Dinic's algorithm to find
/// the minimum vertex cut separating room exits from the protected region.
fn compute_min_cut(
    protected: &FnvHashSet<(u8, u8)>,
    analysis: &crate::pipeline::analysis::AnalysisOutput,
    terrain: &FastRoomTerrain,
) -> Vec<Location> {
    // --- 1. Identify passable tiles and assign indices ---

    // For each passable tile (x, y) we create two graph nodes:
    //   in-node  = tile_in[x][y]
    //   out-node = tile_out[x][y]
    // Connected by a directed edge in→out with capacity 1.

    let exits = &analysis.exits.all;
    let exit_set: FnvHashSet<(u8, u8)> = exits.iter().map(|l| (l.x(), l.y())).collect();

    // Map (x,y) → sequential tile index (only passable, non-wall tiles).
    let mut tile_index: [[Option<usize>; 50]; 50] = [[None; 50]; 50];
    let mut tile_coords: Vec<(u8, u8)> = Vec::new();

    for y in 0u8..50 {
        for x in 0u8..50 {
            if !terrain.is_wall(x, y) {
                let idx = tile_coords.len();
                tile_index[x as usize][y as usize] = Some(idx);
                tile_coords.push((x, y));
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

    let mut builder = <Net as rs_graph::builder::Buildable>::Builder::with_capacities(
        num_nodes,
        edge_estimate,
    );
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
    for (i, &(x, y)) in tile_coords.iter().enumerate() {
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = x as i16 + dx as i16;
            let ny = y as i16 + dy as i16;
            if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                continue;
            }
            if let Some(j) = tile_index[nx as usize][ny as usize] {
                add_edge(
                    &mut builder,
                    &mut capacities,
                    num_tiles + i,
                    j,
                    INF_CAP,
                );
            }
        }
    }

    // Source → in[i] for exit tiles
    for &(ex, ey) in &exit_set {
        if let Some(i) = tile_index[ex as usize][ey as usize] {
            add_edge(&mut builder, &mut capacities, source_idx, i, INF_CAP);
        }
    }

    // Out[i] → sink for protected tiles
    for &(px, py) in protected {
        if let Some(i) = tile_index[px as usize][py as usize] {
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
    let (_value, _flow, mincut_nodes) =
        dinic(&graph, src, snk, |e| capacities[graph.edge_id(e)]);

    // --- 4. Extract rampart positions ---
    //
    // The min-cut returns the set of nodes on the source side. A tile's
    // internal edge (in→out) is cut when the in-node is in the cut set but
    // the out-node is not. Those tiles are where we place ramparts.
    //
    // Build a fast lookup of cut-side nodes.
    let cut_set: FnvHashSet<usize> = mincut_nodes
        .iter()
        .map(|n| graph.node_id(*n))
        .collect();

    let mut ramparts: Vec<Location> = Vec::new();
    for i in 0..num_tiles {
        let in_node_id = graph.node_id(nodes[i]);
        let out_node_id = graph.node_id(nodes[num_tiles + i]);
        // In-node on source side, out-node on sink side → this tile is cut
        if cut_set.contains(&in_node_id) && !cut_set.contains(&out_node_id) {
            let (x, y) = tile_coords[i];
            // Only place ramparts on buildable interior tiles
            if x > 0 && x < 49 && y > 0 && y < 49 {
                ramparts.push(Location::from_coords(x as u32, y as u32));
            }
        }
    }

    ramparts
}
