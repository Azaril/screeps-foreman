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

use crate::layer::*;
use crate::location::*;
use crate::plan::*;
use crate::terrain::*;
use fnv::{FnvHashMap, FnvHashSet};
use pathfinding::directed::astar::astar;

#[cfg(feature = "shim")]
use crate::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

/// A function that inspects the current placement state and returns the set of
/// locations the road network should route to from the hub.
pub type DestinationGenerator = fn(&PlacementState) -> Vec<Location>;

/// Generates an A* road network from hub to destinations provided by a
/// generator function.  Deterministic (1 candidate).
pub struct RoadNetworkLayer {
    layer_name: &'static str,
    destinations: DestinationGenerator,
}

impl RoadNetworkLayer {
    /// Routes to source containers and controller container only.
    /// Intended to run *before* extensions so they fill around the road trunk.
    pub fn infrastructure() -> Self {
        RoadNetworkLayer {
            layer_name: "road_network_infra",
            destinations: infrastructure_destinations,
        }
    }

    /// Routes to all interactable buildings that lack an adjacent hub-connected
    /// road.  Excludes roads, walls, and ramparts.
    pub fn all_buildings() -> Self {
        RoadNetworkLayer {
            layer_name: "road_network",
            destinations: all_buildings_destinations,
        }
    }
}

impl PlacementLayer for RoadNetworkLayer {
    fn name(&self) -> &str {
        self.layer_name
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
        let mut road_tiles: FnvHashSet<Location> = FnvHashSet::default();
        let mut road_edges: Vec<(Location, Location)> = Vec::new();

        // Seed road_tiles from all existing road structures so the A*
        // algorithm reuses roads placed by earlier layers / instances.
        for (loc, items) in &state.structures {
            if items
                .iter()
                .any(|i| i.structure_type == StructureType::Road)
            {
                road_tiles.insert(*loc);
            }
        }

        // Collect destinations from the generator.
        let mut destinations = (self.destinations)(state);

        // Sort destinations by distance from hub (nearest first). This builds
        // a natural trunk road outward from the hub; farther destinations
        // branch off the existing trunk rather than creating parallel roads.
        destinations.sort_by_key(|d| hub.distance_to(*d));

        // Route to each destination incrementally. Already-placed road tiles
        // have zero movement cost, so A* strongly prefers reusing them.
        for dest in &destinations {
            if let Some((path, _cost)) =
                find_path(hub, *dest, terrain, &state.structures, &road_tiles)
            {
                let mut prev = None;
                for loc in &path {
                    road_tiles.insert(*loc);
                    if let Some(p) = prev {
                        road_edges.push((p, *loc));
                    }
                    prev = Some(*loc);
                }
            }
        }

        // Place road structures, skipping tiles that already have any structure
        for &road_loc in &road_tiles {
            if !new_state.structures.contains_key(&road_loc) {
                new_state
                    .structures
                    .entry(road_loc)
                    .or_default()
                    .push(RoomItem::new(StructureType::Road, 1));
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
        let x = loc.x();
        let y = loc.y();
        let has_adjacent_road = NEIGHBORS_8.iter().any(|&(dx, dy)| {
            let nx = x as i16 + dx as i16;
            let ny = y as i16 + dy as i16;
            if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                return false;
            }
            let nloc = Location::from_coords(nx as u32, ny as u32);
            road_set.contains(&nloc)
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
///   - Swamp:  30
///   - Container: +100 (walkable but adds fatigue; strongly discouraged)
///   - Other occupied structure: +500 (effectively blocked)
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
                    if !(0..50).contains(&nx) || !(0..50).contains(&ny) {
                        return None;
                    }
                    let ux = nx as u8;
                    let uy = ny as u8;
                    if terrain.is_wall(ux, uy) {
                        return None;
                    }
                    let loc = Location::from_coords(ux as u32, uy as u32);

                    // Already-placed roads are free to traverse, strongly
                    // encouraging path merging onto existing roads.
                    if existing_roads.contains(&loc) {
                        return Some(((nx, ny), 0u32));
                    }

                    // Base cost scaled by 10 for tie-breaking resolution.
                    let base_cost = if terrain.is_swamp(ux, uy) {
                        30u32
                    } else {
                        10u32
                    };

                    // Structure penalty based on type.
                    let structure_penalty =
                        if let Some(items) = structures.get(&loc) {
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
                                // Effectively blocked (extensions, spawns, walls, etc.)
                                500u32
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
                        let ax = nx + adx as i16;
                        let ay = ny + ady as i16;
                        if !(0..50).contains(&ax) || !(0..50).contains(&ay) {
                            return false;
                        }
                        let aloc =
                            Location::from_coords(ax as u32, ay as u32);
                        existing_roads.contains(&aloc)
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
            .map(|(x, y)| Location::from_coords(x as u32, y as u32))
            .collect();
        (locs, cost)
    })
}
