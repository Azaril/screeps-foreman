//! RoadNetworkLayer: Generates A* roads from hub to destinations.
//! Deterministic (1 candidate).
//!
//! Roads are placed incrementally: destinations are sorted by distance from the
//! hub (nearest first), and each path is computed with already-placed road tiles
//! costing zero. This causes later paths to merge onto existing roads, forming
//! a shared trunk that branches near each destination -- minimizing total road
//! tiles while producing equivalent-length routes.
//!
//! The layer is parameterized by a *destination generator* function so the same
//! routing algorithm can be reused for different sets of targets (e.g. early
//! infrastructure roads vs. full building coverage).

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::plan::*;
use crate::terrain::*;
use fnv::{FnvHashMap, FnvHashSet};
use log::*;
use pathfinding::directed::astar::astar;

use screeps::constants::StructureType;

/// A function that inspects the current placement state and returns the set of
/// locations the road network should route to from the hub.
pub type DestinationGenerator = fn(&PlacementState) -> Vec<Location>;

/// Generates an A* road network from hub to destinations provided by a
/// generator function.  Deterministic (1 candidate).
///
/// The optional `rcl` parameter pins all roads placed by this layer to a
/// specific RCL value. When `None`, roads are placed with auto-RCL and the
/// `RclAssignmentLayer` will resolve them later (typically inheriting the
/// minimum RCL of adjacent structures).
pub struct RoadNetworkLayer {
    layer_name: &'static str,
    destinations: DestinationGenerator,
    /// If set, all roads placed by this layer are pinned to this RCL.
    rcl: Option<u8>,
}

impl RoadNetworkLayer {
    /// Routes to source containers and controller container only.
    /// Intended to run *before* extensions so they fill around the road trunk.
    ///
    /// Roads are pinned to RCL 2 so the core infrastructure road network is
    /// available early, independent of adjacent building RCLs.
    pub fn infrastructure() -> Self {
        RoadNetworkLayer {
            layer_name: "road_network_infra",
            destinations: infrastructure_destinations,
            rcl: Some(2),
        }
    }

    /// Routes to all interactable buildings that lack an adjacent hub-connected
    /// road.  Excludes roads, walls, and ramparts.
    ///
    /// Roads are automatically assigned an RCL by the `RclAssignmentLayer`.
    pub fn all_buildings() -> Self {
        RoadNetworkLayer {
            layer_name: "road_network",
            destinations: all_buildings_destinations,
            rcl: None,
        }
    }

    /// Set the RCL that all roads placed by this layer should use.
    ///
    /// When set, roads are pinned to this value. The `RclAssignmentLayer`
    /// will keep the minimum of this value and the adjacent-structure RCL.
    pub fn with_rcl(mut self, rcl: u8) -> Self {
        self.rcl = Some(rcl);
        self
    }

    /// Set roads to use automatic RCL assignment (resolved by `RclAssignmentLayer`).
    pub fn with_auto_rcl(mut self) -> Self {
        self.rcl = None;
        self
    }
}

impl PlacementLayer for RoadNetworkLayer {
    fn name(&self) -> &str {
        self.layer_name
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

        let mut new_state = state.clone();
        let mut road_tiles: FnvHashSet<Location> = FnvHashSet::default();
        let mut road_edges: Vec<(Location, Location)> = Vec::new();

        // Track which road tiles existed before this layer ran, so we only
        // apply this layer's pinned RCL to roads that are actually part of
        // a path we computed (not pre-existing roads that A* merely traversed).
        let mut preexisting_roads: FnvHashSet<Location> = FnvHashSet::default();

        // Seed road_tiles from all existing road structures so the A*
        // algorithm reuses roads placed by earlier layers / instances.
        for (loc, items) in &state.structures {
            if items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
            {
                road_tiles.insert(*loc);
                preexisting_roads.insert(*loc);
            }
        }

        // Collect destinations from the generator.
        let mut destinations = (self.destinations)(state);

        // Sort destinations by distance from hub (nearest first). This builds
        // a natural trunk road outward from the hub; farther destinations
        // branch off the existing trunk rather than creating parallel roads.
        destinations.sort_by_key(|d| hub.distance_to(*d));

        // Track which tiles are part of paths computed by this layer.
        let mut path_tiles: FnvHashSet<Location> = FnvHashSet::default();

        // Route to each destination incrementally. Already-placed road tiles
        // have zero movement cost, so A* strongly prefers reusing them.
        for dest in &destinations {
            if let Some((path, _cost)) =
                find_path(hub, *dest, terrain, &state.structures, &road_tiles)
            {
                trace!(
                    "{}: path to ({},{}) has {} tiles",
                    self.layer_name,
                    dest.x(),
                    dest.y(),
                    path.len()
                );
                let mut prev = None;
                for loc in &path {
                    road_tiles.insert(*loc);
                    path_tiles.insert(*loc);
                    if let Some(p) = prev {
                        road_edges.push((p, *loc));
                    }
                    prev = Some(*loc);
                }
            } else {
                debug!(
                    "{}: no path to ({},{})",
                    self.layer_name,
                    dest.x(),
                    dest.y()
                );
            }
        }

        // Place road structures.
        //
        // Only apply this layer's pinned RCL to roads that are part of a
        // path we computed. Pre-existing roads that A* merely traversed
        // (cost 0) keep their original RCL — they belong to another layer
        // (e.g. stamp roads around labs) and should not inherit this layer's
        // early RCL.
        for &road_loc in &path_tiles {
            if let Some(items) = new_state.structures.get_mut(&road_loc) {
                let has_road = items
                    .iter()
                    .any(|i| i.structure_type == StructureType::Road);
                if has_road {
                    // Only merge RCL onto roads that this layer actually
                    // needs (newly placed roads, or pre-existing roads that
                    // are part of a computed path and don't yet have an RCL).
                    if !preexisting_roads.contains(&road_loc) {
                        // Road was placed by a previous iteration of this
                        // layer's path loop — set its RCL.
                        if let Some(layer_rcl) = self.rcl {
                            for item in items.iter_mut() {
                                if item.structure_type == StructureType::Road {
                                    item.required_rcl = Some(match item.required_rcl {
                                        Some(existing) => existing.min(layer_rcl),
                                        None => layer_rcl,
                                    });
                                }
                            }
                        }
                    }
                    // Pre-existing roads: leave their RCL unchanged.
                } else {
                    // Tile has structures but no road. Place a road if the
                    // tile only contains structures that coexist with roads
                    // (containers, ramparts). Skip tiles with buildings
                    // (extensions, spawns, towers, etc.) where a road would
                    // not make sense.
                    let all_coexist = items.iter().all(|i| {
                        matches!(
                            i.structure_type,
                            StructureType::Container | StructureType::Rampart
                        )
                    });
                    if all_coexist {
                        let road = match self.rcl {
                            Some(rcl) => RoomItem::new(StructureType::Road, rcl),
                            None => RoomItem::new_auto_rcl(StructureType::Road),
                        };
                        items.push(road);
                    }
                }
            } else {
                // No structures at this tile — place a new road.
                let road = match self.rcl {
                    Some(rcl) => RoomItem::new(StructureType::Road, rcl),
                    None => RoomItem::new_auto_rcl(StructureType::Road),
                };
                new_state.structures.entry(road_loc).or_default().push(road);
            }
        }

        // Store road edges in landmark set (as pairs -- flatten to a list)
        for (a, b) in &road_edges {
            new_state.add_to_landmark_set("road_edges_a", *a);
            new_state.add_to_landmark_set("road_edges_b", *b);
        }

        Some(Ok(new_state))
    }
}

// ---------------------------------------------------------------------------
// Destination generators
// ---------------------------------------------------------------------------

/// Infrastructure destinations: source containers + controller container.
fn infrastructure_destinations(state: &PlacementState) -> Vec<Location> {
    let mut dests = Vec::new();
    dests.extend(state.get_landmark_set("source_containers"));
    if let Some(cc) = state.get_landmark("controller_container") {
        dests.push(cc);
    }
    dests
}

/// All-buildings destinations: mineral container + every interactable building
/// that does not yet have an adjacent road connected to the hub.
///
/// Excludes roads, walls, and ramparts (non-interactable).
fn all_buildings_destinations(state: &PlacementState) -> Vec<Location> {
    let mut dests = Vec::new();

    // Mineral container (explicit landmark, may not be in structures yet)
    if let Some(mc) = state.get_landmark("mineral_container") {
        dests.push(mc);
    }

    // Collect existing road locations for adjacency checks.
    let road_set: FnvHashSet<Location> = state
        .structures
        .iter()
        .filter(|(_, items)| {
            items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
        })
        .map(|(loc, _)| *loc)
        .collect();

    // Find all interactable structures that lack an adjacent road.
    for (loc, items) in &state.structures {
        let has_interactable = items.iter().any(|i| {
            !matches!(
                i.structure_type,
                StructureType::Road | StructureType::Wall | StructureType::Rampart
            )
        });
        if !has_interactable {
            continue;
        }

        // Check if this structure already has an adjacent road tile.
        let has_adjacent_road = NEIGHBORS_8.iter().any(|&(dx, dy)| {
            loc.checked_add(dx, dy)
                .is_some_and(|nloc| road_set.contains(&nloc))
        });

        if !has_adjacent_road {
            dests.push(*loc);
        }
    }

    dests
}

// ---------------------------------------------------------------------------
// A* pathfinding (pub(crate) for reuse by other layers)
// ---------------------------------------------------------------------------

/// A* pathfinding between two locations.
///
/// Tiles that already have roads cost nothing to traverse, encouraging path
/// reuse and minimizing total road count.  Non-road tiles are costed as:
///   - Plains: 10 (scaled for tie-breaking resolution)
///   - Swamp:  60 (reflects ~5x higher maintenance cost of swamp roads)
///   - Container: +100 (walkable but adds fatigue; strongly discouraged)
///   - Other occupied structure: +500 (effectively blocked)
///
/// The swamp cost (60) is set high relative to plains (10) to aggressively
/// avoid swamp roads. In Screeps, roads on swamp decay at ~5x the rate of
/// roads on plains, making them significantly more expensive to maintain.
///
/// **Tie-breaking bias:** tiles adjacent to an existing road get a small
/// discount (-1), causing A* to deterministically prefer paths that stay
/// close to the existing road network when costs are otherwise equal.
pub(crate) fn find_path(
    start: Location,
    goal: Location,
    terrain: &FastRoomTerrain,
    structures: &FnvHashMap<Location, Vec<RoomItem>>,
    existing_roads: &FnvHashSet<Location>,
) -> Option<(Vec<Location>, u32)> {
    let start_pos = (start.x() as i16, start.y() as i16);
    let goal_pos = (goal.x() as i16, goal.y() as i16);

    let result = astar(
        &start_pos,
        |&(x, y)| {
            NEIGHBORS_8
                .iter()
                .filter_map(move |&(dx, dy)| {
                    let nx = x + dx as i16;
                    let ny = y + dy as i16;
                    if !xy_in_bounds(nx, ny) {
                        return None;
                    }
                    let ux = nx as u8;
                    let uy = ny as u8;
                    if terrain.is_wall(ux, uy) {
                        return None;
                    }
                    let loc = Location::from_xy(ux, uy);

                    // Already-placed roads are free to traverse, strongly
                    // encouraging path merging onto existing roads.
                    if existing_roads.contains(&loc) {
                        return Some(((nx, ny), 0u32));
                    }

                    // Base cost scaled by 10 for tie-breaking resolution.
                    // Swamp cost is 6x plains to reflect ~5x higher maintenance.
                    let base_cost = if terrain.is_swamp(ux, uy) {
                        60u32
                    } else {
                        10u32
                    };

                    // Structure penalty based on type.
                    let structure_penalty = if let Some(items) = structures.get(&loc) {
                        let has_container = items
                            .iter()
                            .any(|i| i.structure_type == StructureType::Container);
                        let has_other_occupied = items.iter().any(|i| {
                            !matches!(
                                i.structure_type,
                                StructureType::Road
                                    | StructureType::Container
                                    | StructureType::Rampart
                            )
                        });
                        if has_other_occupied {
                            // Impassable: roads cannot be placed on top of
                            // buildings (extensions, spawns, towers, etc.).
                            // Routing through them would create gaps in the
                            // road network since no road structure is placed.
                            return None;
                        } else if has_container {
                            // Walkable but adds fatigue; strongly discouraged
                            100u32
                        } else {
                            // Only roads/ramparts -- passable at normal cost
                            0u32
                        }
                    } else {
                        0u32
                    };

                    // Tie-breaking: small discount for tiles adjacent to an
                    // existing road.  This deterministically collapses
                    // equal-cost parallel paths onto the existing network.
                    let adjacency_discount = if NEIGHBORS_8.iter().any(|&(adx, ady)| {
                        loc.checked_add(adx, ady)
                            .is_some_and(|aloc| existing_roads.contains(&aloc))
                    }) {
                        1u32
                    } else {
                        0u32
                    };

                    let cost = (base_cost + structure_penalty).saturating_sub(adjacency_discount);
                    Some(((nx, ny), cost))
                })
                .collect::<Vec<_>>()
        },
        |&(x, y)| {
            // Heuristic: Chebyshev distance scaled by 10 (matching base cost).
            let dx = (x - goal_pos.0).unsigned_abs() as u32;
            let dy = (y - goal_pos.1).unsigned_abs() as u32;
            dx.max(dy) * 10
        },
        |&(x, y)| x == goal_pos.0 && y == goal_pos.1,
    );

    result.map(|(path, cost)| {
        let locs: Vec<Location> = path
            .into_iter()
            .map(|(x, y)| Location::from_xy(x as u8, y as u8))
            .collect();
        (locs, cost)
    })
}
