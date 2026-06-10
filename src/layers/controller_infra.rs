//! ControllerInfraLayer: places the controller container + link and
//! exports the upgrade-area WORK SLOTS (ADR 0009 D2, operator-directed).
//!
//! A work slot is a walkable tile <=1 from the container AND <=3
//! (upgrade range) from the controller: a creep parked there withdraws
//! and upgrades forever with ZERO move intents — MOVE-light bodies, no
//! pathfinding CPU. Containers are walkable (engine
//! `OBSTACLE_OBJECT_TYPES`), so the container tile itself is a slot
//! (unless the approach reservation consumes it — see below); links are
//! obstacles, so a link tile is never a slot.
//!
//! The (container, link) pair is scored JOINTLY:
//!
//! ```text
//! score = |slots| * WORK_SLOT_WEIGHT + overlap * ERA_OVERLAP_WEIGHT
//! ```
//!
//! where `overlap` = slots also <=1 from the link — the tiles that keep
//! working unchanged when the link era replaces hauled energy (pre-RCL8
//! link throughput and the RCL8 15 e/t cap mean overlap matters more
//! than raw link coverage). The joint score decides whether the link
//! consumes a slot: at weights 3/2 a slot-consuming link wins exactly
//! when it adds >=2 overlap versus the best outside spot. Ties break
//! toward shorter hub haul distance (demoting proximity from objective
//! to tie-breaker — the old layer's whole objective was distance to
//! source #0), then lowest location index for determinism. In open
//! terrain this picks a container at Chebyshev 2 of the controller,
//! where all 9 coverage tiles are in upgrade range.
//!
//! Circulation: the coverage tile nearest the hub is RESERVED as the
//! hauler approach (roads arrive there — `road_network.rs` penalizes
//! slot tiles — and it is never a parking slot; in cramped rooms the
//! approach can land on the container tile itself, which is sound: the
//! hauler stands on the container). Candidate pairs whose approach is
//! hub-unreachable (terrain-sealed pockets) are skipped outright, and
//! the layer walks the ranked candidates through a flood-fill guard
//! (slots + link treated as blocked — parked upgraders ARE obstacles)
//! until one keeps the approach reachable, so a single bad candidate
//! cannot brick the room plan.
//!
//! Rooms with NO legal link spot (within the bot's range-3
//! classification, covering >=1 slot) fall back to container-only
//! placement: a container-era room with no link beats both a dead
//! out-of-classification link (the pre-D2 behavior this replaces) and
//! an unplannable room.
//!
//! Slot and approach tiles are excluded from later placement layers
//! (extensions, defense walls); roads ignore exclusions by design, so
//! the road network may still pave them (penalized, never blocked).

use crate::constants::*;
use crate::layer::*;
use crate::location::*;
use crate::pipeline::analysis::AnalysisOutput;
use crate::terrain::*;
use fnv::{FnvHashMap, FnvHashSet};

use crate::plan::RoomItem;
use screeps::constants::StructureType;

/// Joint-score weight per work slot (container-era parking capacity).
pub const WORK_SLOT_WEIGHT: u32 = 3;
/// Joint-score weight per slot the link also covers (link-era capacity
/// on the SAME tiles).
pub const ERA_OVERLAP_WEIGHT: u32 = 2;
/// Ranked tier-1 candidates kept (best link pairing per container
/// tile). These become SEARCH-TREE candidates: when a downstream
/// validator (reachability's detour bound, extension minimums, defense)
/// rejects a branch, the search backtracks into the next-best
/// placement instead of failing the room — shard3 E11N25's controller
/// crevice is the motivating live case.
const MAX_LINKED_CANDIDATES: usize = 12;
/// Container-only fallback candidates appended after every linked one:
/// a link is strictly desirable, so link-less placements are tried only
/// when every linked branch has failed downstream.
const MAX_FALLBACK_CANDIDATES: usize = 6;

/// One ranked outcome of the joint selection
/// ([`select_controller_infra`] — pure, unit-tested).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ControllerInfraSelection {
    pub container: Location,
    /// `None` when the room has no legal link spot (container-only
    /// fallback tier).
    pub link: Option<Location>,
    /// Reserved hauler approach: the minimum-haul-distance coverage
    /// tile. Excluded from `slots`; roads are steered here. May equal
    /// `container` in cramped rooms.
    pub approach: Location,
    /// Work slots: walkable, <=1 from the container, in upgrade range
    /// of the controller, minus the approach and link tiles. Sorted by
    /// location index (deterministic).
    pub slots: Vec<Location>,
    /// How many of `slots` are also <=1 from the link (0 when no link).
    pub era_overlap: u32,
}

/// Higher wins: score desc, haul asc, container index asc, link index asc.
type RankKey = (
    u32,
    std::cmp::Reverse<u32>,
    std::cmp::Reverse<usize>,
    std::cmp::Reverse<usize>,
);

/// Pure joint selection of ranked (container, link, approach, slots)
/// candidates, best first.
///
/// `free` = tile is interior, not a terrain wall, has no planned
/// structure, not excluded, and not a source/mineral tile. `haul_dist`
/// = hub flood-fill distance (lower = closer; `u32::MAX` for
/// unreachable — candidates whose approach is unreachable are skipped,
/// so terrain-sealed pockets near the controller can never outrank a
/// reachable ledge). Callers with no haul information pass a constant.
///
/// The result is the best (container, link) pairing PER CONTAINER TILE
/// (up to [`MAX_LINKED_CANDIDATES`] — container diversity is what lets
/// downstream backtracking escape a bad area; retrying the same
/// container with a different link rarely does), followed by up to
/// [`MAX_FALLBACK_CANDIDATES`] container-only placements for rooms (or
/// search branches) where no linked candidate survives.
pub fn select_controller_infra(
    ctrl: Location,
    free: impl Fn(Location) -> bool,
    haul_dist: impl Fn(Location) -> u32,
) -> Vec<ControllerInfraSelection> {
    let mut ranked: Vec<(RankKey, ControllerInfraSelection)> = Vec::new();
    let push = |key: RankKey,
                sel: ControllerInfraSelection,
                out: &mut Vec<(RankKey, ControllerInfraSelection)>| {
        out.push((key, sel));
    };
    let mut fallback: Vec<(RankKey, ControllerInfraSelection)> = Vec::new();

    let r = CONTROLLER_CONTAINER_MAX_RANGE as i8;
    for dy in -r..=r {
        for dx in -r..=r {
            if dx == 0 && dy == 0 {
                continue;
            }
            let Some(c) = ctrl.checked_add(dx, dy) else {
                continue;
            };
            if !c.is_interior() || !free(c) {
                continue;
            }

            // Coverage: the container tile + its neighbors that are
            // free and in upgrade range (the controller tile itself is
            // an obstacle and can never be coverage).
            let coverage: Vec<Location> = std::iter::once(c)
                .chain(c.neighbors())
                .filter(|&t| {
                    t != ctrl
                        && t.is_interior()
                        && free(t)
                        && u32::from(t.distance_to(ctrl)) <= CONTROLLER_UPGRADE_RANGE
                })
                .collect();
            // Need the reserved approach plus at least one real slot.
            if coverage.len() < 2 {
                continue;
            }

            // Hauler approach: where the road network will arrive.
            let approach = *coverage
                .iter()
                .min_by_key(|&&t| (haul_dist(t), t.to_index()))
                .expect("coverage checked non-empty");
            // Terrain-sealed pocket: highest-scoring or not, a spot the
            // hauler can never reach is not a candidate.
            if haul_dist(approach) == u32::MAX {
                continue;
            }

            // Container-only fallback entry for this container.
            {
                let mut slots: Vec<Location> =
                    coverage.iter().copied().filter(|&t| t != approach).collect();
                slots.sort_by_key(|t| t.to_index());
                let key: RankKey = (
                    slots.len() as u32 * WORK_SLOT_WEIGHT,
                    std::cmp::Reverse(haul_dist(c)),
                    std::cmp::Reverse(c.to_index()),
                    std::cmp::Reverse(0),
                );
                push(
                    key,
                    ControllerInfraSelection {
                        container: c,
                        link: None,
                        approach,
                        slots,
                        era_overlap: 0,
                    },
                    &mut fallback,
                );
            }

            // Link candidates: anything within 2 of the container can
            // touch a coverage tile (slots are a subset of the
            // container's 1-neighborhood, so no farther link can reach
            // one). Must stay in the bot's controller-link
            // classification range and must not consume the container,
            // approach, or controller tiles.
            for ldy in -2i8..=2 {
                for ldx in -2i8..=2 {
                    let Some(l) = c.checked_add(ldx, ldy) else {
                        continue;
                    };
                    if l == c
                        || l == ctrl
                        || l == approach
                        || !l.is_interior()
                        || !free(l)
                        || u32::from(l.distance_to(ctrl)) > CONTROLLER_LINK_MAX_RANGE
                    {
                        continue;
                    }
                    // The link is an obstacle: placing it on a coverage
                    // tile consumes that slot.
                    let slots: Vec<Location> = coverage
                        .iter()
                        .copied()
                        .filter(|&t| t != approach && t != l)
                        .collect();
                    if slots.is_empty() {
                        continue;
                    }
                    let overlap =
                        slots.iter().filter(|&&s| s.distance_to(l) <= 1).count() as u32;
                    if overlap == 0 {
                        // A link no slot can reach is useless for upgrading.
                        continue;
                    }
                    let score =
                        slots.len() as u32 * WORK_SLOT_WEIGHT + overlap * ERA_OVERLAP_WEIGHT;
                    let key: RankKey = (
                        score,
                        std::cmp::Reverse(haul_dist(c)),
                        std::cmp::Reverse(c.to_index()),
                        std::cmp::Reverse(l.to_index()),
                    );
                    let mut slots = slots;
                    slots.sort_by_key(|t| t.to_index());
                    push(
                        key,
                        ControllerInfraSelection {
                            container: c,
                            link: Some(l),
                            approach,
                            slots,
                            era_overlap: overlap,
                        },
                        &mut ranked,
                    );
                }
            }
        }
    }

    // Best pairing per container, score-ranked, linked tier first.
    let dedup_by_container =
        |mut entries: Vec<(RankKey, ControllerInfraSelection)>, cap: usize| {
            entries.sort_by_key(|entry| std::cmp::Reverse(entry.0));
            let mut seen: FnvHashSet<Location> = FnvHashSet::default();
            let mut out = Vec::new();
            for (_, sel) in entries {
                if seen.insert(sel.container) {
                    out.push(sel);
                    if out.len() == cap {
                        break;
                    }
                }
            }
            out
        };
    let mut result = dedup_by_container(ranked, MAX_LINKED_CANDIDATES);
    result.extend(dedup_by_container(fallback, MAX_FALLBACK_CANDIDATES));
    result
}

/// Flood-fill check: with `blocked` tiles impassable (parked upgraders
/// plus the link) on top of non-walkable structures, is `approach`
/// still reachable from `hub`?
///
/// Approximations (all rare-geometry false PASSES, acceptable for a
/// guard that only needs to reject obviously-sealed placements):
/// room-border tiles count as circulation even though creeps stepping
/// on exits leave the room, and the BFS expands from the hub seed
/// even if the hub tile itself is occupied.
pub(crate) fn approach_reachable_with_parked_slots(
    terrain: &FastRoomTerrain,
    structures: &FnvHashMap<Location, Vec<RoomItem>>,
    hub: Location,
    blocked: &FnvHashSet<Location>,
    approach: Location,
) -> bool {
    if approach == hub {
        return true;
    }
    let (dists, _) = flood_fill_distance_with_obstacles(terrain, &[hub], |x, y| {
        let loc = Location::from_xy(x, y);
        if blocked.contains(&loc) {
            return false;
        }
        match structures.get(&loc) {
            None => true,
            Some(items) => items.iter().all(|i| {
                matches!(
                    i.structure_type,
                    StructureType::Road | StructureType::Container | StructureType::Rampart
                )
            }),
        }
    });
    dists.get_at(approach).is_some()
}

/// Places controller container + link by joint work-slot score and
/// exports the slot set. Emits the ranked, guard-passing selections as
/// SEARCH-TREE candidates (best first): downstream validators
/// (reachability's detour bound, extension minimums, defense) reject a
/// branch and the search backtracks into the next-ranked placement —
/// a single awkward controller area can no longer brick the room.
pub struct ControllerInfraLayer;

impl ControllerInfraLayer {
    /// The ranked, circulation-guard-passing selections for this state.
    fn viable_selections(
        state: &PlacementState,
        analysis: &AnalysisOutput,
        terrain: &FastRoomTerrain,
    ) -> Vec<ControllerInfraSelection> {
        let (ctrl_loc, _, _) = match analysis.controller_distances.first() {
            Some(entry) => entry,
            None => return Vec::new(),
        };
        let ctrl = *ctrl_loc;

        // Haul distances: hub flood fill (terrain-only, same metric as
        // `PlacementState::hub_distances` without needing `&mut state`).
        // Defensive fallbacks: the source #0 distance map, then a
        // constant (which also disables the unreachable-pocket filter)
        // — in the default stack the hub always exists by this layer.
        let haul_map = state
            .get_landmark("hub")
            .map(|hub| flood_fill_distance(terrain, &[hub]).0)
            .or_else(|| analysis.source_distances.first().map(|(_, dm, _)| dm.clone()));
        let haul = |t: Location| -> u32 {
            match &haul_map {
                Some(m) => m.get_at(t).as_ref().copied().unwrap_or(u32::MAX),
                None => 0,
            }
        };
        // Source/mineral tiles are obstacle objects the placement state
        // cannot see (it starts empty) — keep slots and structures off
        // them.
        let object_tiles: FnvHashSet<Location> = analysis
            .source_distances
            .iter()
            .chain(analysis.mineral_distances.iter())
            .map(|(loc, _, _)| *loc)
            .collect();
        let free = |t: Location| -> bool {
            t.is_interior()
                && !terrain.is_wall_at(t)
                && !state.has_any_structure(t)
                && !state.is_excluded(t)
                && !object_tiles.contains(&t)
        };

        let hub = state.get_landmark("hub");
        select_controller_infra(ctrl, free, haul)
            .into_iter()
            .filter(|sel| match hub {
                None => true,
                Some(hub) => {
                    let blocked: FnvHashSet<Location> =
                        sel.slots.iter().copied().chain(sel.link).collect();
                    approach_reachable_with_parked_slots(
                        terrain,
                        &state.structures,
                        hub,
                        &blocked,
                        sel.approach,
                    )
                }
            })
            .collect()
    }
}

impl PlacementLayer for ControllerInfraLayer {
    fn name(&self) -> &str {
        // "_v2" = the D2 work-slot rewrite. The planner fingerprint is
        // a hash of layer names, so this bump invalidates in-flight
        // serialized planner state (segment 60) from the pre-D2 layer
        // instead of letting it bake one last old-semantics plan.
        "controller_infra_v2"
    }

    fn is_applicable(&self, _state: &PlacementState, analysis: &AnalysisOutput) -> bool {
        !analysis.controller_distances.is_empty()
    }

    fn candidate_count(
        &self,
        state: &PlacementState,
        analysis: &AnalysisOutput,
        terrain: &FastRoomTerrain,
    ) -> Option<usize> {
        Some(Self::viable_selections(state, analysis, terrain).len())
    }

    fn candidate(
        &self,
        index: usize,
        state: &PlacementState,
        analysis: &AnalysisOutput,
        terrain: &FastRoomTerrain,
    ) -> Option<Result<PlacementState, ()>> {
        let sel = Self::viable_selections(state, analysis, terrain)
            .into_iter()
            .nth(index)?;

        let mut new_state = state.clone();
        new_state.place_structure(sel.container, StructureType::Container, 0);
        new_state.set_landmark("controller_container", sel.container);
        if let Some(link) = sel.link {
            new_state.place_structure(link, StructureType::Link, 0);
            new_state.set_landmark("controller_link", link);
        }
        // Planner-internal landmark (not exported into Plan): the
        // reserved hauler tile, should later layers or Inc-7 job work
        // need it.
        new_state.set_landmark("upgrade_approach", sel.approach);
        // Protect the parking area from later placement layers
        // (extensions, defense walls). Roads ignore exclusions, so the
        // road network can still pave through (penalized in its A*).
        new_state.exclude_tile(sel.approach);
        for &slot in &sel.slots {
            new_state.add_to_landmark_set("upgrade_area", slot);
            new_state.exclude_tile(slot);
        }

        Some(Ok(new_state))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fnv::FnvHashSet;

    const CTRL: (u8, u8) = (25, 25);

    fn ctrl() -> Location {
        Location::from_xy(CTRL.0, CTRL.1)
    }

    /// `free` closure over a wall set: interior, not the controller,
    /// not a wall.
    fn free_with_walls(walls: FnvHashSet<Location>) -> impl Fn(Location) -> bool {
        move |t: Location| t.is_interior() && t != ctrl() && !walls.contains(&t)
    }

    /// Haul gradient from the west: cheaper toward smaller x.
    fn west_haul(t: Location) -> u32 {
        t.x() as u32
    }

    /// Wall out everything within `r` of the controller, then carve the
    /// given tiles back open.
    fn walls_except(open: &[(u8, u8)], r: i8) -> FnvHashSet<Location> {
        let mut walls = FnvHashSet::default();
        for dy in -r..=r {
            for dx in -r..=r {
                if let Some(t) = ctrl().checked_add(dx, dy) {
                    walls.insert(t);
                }
            }
        }
        for &(x, y) in open {
            walls.remove(&Location::from_xy(x, y));
        }
        walls
    }

    /// The published contract, checked on any selection.
    fn assert_invariants(sel: &ControllerInfraSelection, at: Location) {
        let c = sel.container;
        assert!(u32::from(c.distance_to(at)) <= CONTROLLER_CONTAINER_MAX_RANGE);
        assert!(u32::from(sel.approach.distance_to(c)) <= 1);
        for s in &sel.slots {
            assert!(u32::from(s.distance_to(c)) <= 1, "slot within 1 of container");
            assert!(
                u32::from(s.distance_to(at)) <= CONTROLLER_UPGRADE_RANGE,
                "slot in upgrade range"
            );
            assert_ne!(*s, sel.approach, "approach is never a slot");
        }
        match sel.link {
            Some(l) => {
                assert!(u32::from(l.distance_to(at)) <= CONTROLLER_LINK_MAX_RANGE);
                assert_ne!(l, c);
                assert_ne!(l, sel.approach);
                assert!(!sel.slots.contains(&l), "link tile is never a slot");
                let recomputed =
                    sel.slots.iter().filter(|&&s| s.distance_to(l) <= 1).count() as u32;
                assert_eq!(recomputed, sel.era_overlap);
                assert!(sel.era_overlap >= 1, "link serves at least one slot");
            }
            None => assert_eq!(sel.era_overlap, 0),
        }
    }

    fn best(
        walls: FnvHashSet<Location>,
        haul: impl Fn(Location) -> u32,
    ) -> Option<ControllerInfraSelection> {
        select_controller_infra(ctrl(), free_with_walls(walls), haul)
            .into_iter()
            .next()
    }

    #[test]
    fn selection_is_deterministic_and_ranked() {
        let a = select_controller_infra(ctrl(), free_with_walls(Default::default()), west_haul);
        let b = select_controller_infra(ctrl(), free_with_walls(Default::default()), west_haul);
        assert_eq!(a, b);
        assert!(!a.is_empty());
        for sel in &a {
            assert_invariants(sel, ctrl());
        }
        // Linked tier strictly before the container-only fallbacks, and
        // one candidate per container tile WITHIN each tier (diversity
        // for downstream backtracking; the same container may appear in
        // both tiers).
        let split = a.iter().position(|s| s.link.is_none()).unwrap_or(a.len());
        assert!(
            a[split..].iter().all(|s| s.link.is_none()),
            "all linked candidates precede the container-only tier"
        );
        for tier in [&a[..split], &a[split..]] {
            let containers: FnvHashSet<Location> = tier.iter().map(|s| s.container).collect();
            assert_eq!(containers.len(), tier.len(), "deduped by container within tier");
        }
    }

    /// Open terrain: the exact optimum is a Chebyshev-2 container with
    /// an edge-center link INSIDE the coverage — 7 slots with overlap 5
    /// scores 7*3+5*2=31, beating the best outside-link configuration
    /// (8 slots, overlap 3: 8*3+3*2=30). This pins the consume-iff-
    /// delta-overlap>=2 weight behavior in the canonical geometry: 5 of
    /// 7 parking tiles keep working unchanged through the link era.
    #[test]
    fn open_terrain_maximizes_work_slots() {
        let sel = best(Default::default(), west_haul).expect("open terrain must place");
        assert_eq!(u32::from(sel.container.distance_to(ctrl())), 2, "{sel:?}");
        assert_eq!(sel.slots.len(), 7, "{sel:?}");
        assert_eq!(sel.era_overlap, 5, "{sel:?}");
        let link = sel.link.expect("open terrain places a link");
        assert_eq!(
            u32::from(link.distance_to(sel.container)),
            1,
            "the link consumes a coverage tile here: {sel:?}"
        );
        for s in &sel.slots {
            assert!(west_haul(sel.approach) <= west_haul(*s), "{sel:?}");
        }
        assert_invariants(&sel, ctrl());
    }

    /// When the only outside link spot covers a single slot but an
    /// in-coverage spot covers 5, consuming a slot wins: 7*3+5*2=31
    /// beats 8*3+1*2=26 (the consume-vs-outside boundary is delta
    /// overlap >= 2 at weights 3/2).
    #[test]
    fn link_consumes_a_slot_when_overlap_pays_for_it() {
        // Free: the 3x3 coverage block centered (23,25), plus one bad
        // outside link spot (21,27) touching only the corner (22,26).
        let open: Vec<(u8, u8)> = (22..=24)
            .flat_map(|x| (24..=26).map(move |y| (x, y)))
            .chain([(21, 27)])
            .collect();
        let sel = best(walls_except(&open, 4), west_haul).expect("must place");
        let link = sel.link.expect("a link exists");
        assert_eq!(sel.container, Location::from_xy(23, 25), "{sel:?}");
        assert!(
            u32::from(link.distance_to(sel.container)) <= 1,
            "link consumes a coverage tile: {sel:?}"
        );
        assert_eq!(sel.slots.len(), 7, "{sel:?}");
        assert!(sel.era_overlap >= 5, "{sel:?}");
        assert_invariants(&sel, ctrl());
    }

    /// Walls forcing a 3-tile strip: the haul tie-break drives the
    /// container WEST onto its own approach tile (sound: the hauler
    /// stands on the container), the middle tile is the single slot,
    /// and the link takes the third.
    #[test]
    fn constrained_pocket_still_places() {
        let sel = best(walls_except(&[(22, 25), (23, 25), (24, 25)], 4), west_haul)
            .expect("pocket must still place");
        assert_eq!(sel.approach, Location::from_xy(22, 25), "{sel:?}");
        assert_eq!(sel.container, sel.approach, "container doubles as approach: {sel:?}");
        assert_eq!(sel.slots, vec![Location::from_xy(23, 25)], "{sel:?}");
        assert_eq!(sel.link, Some(Location::from_xy(24, 25)), "{sel:?}");
        assert_eq!(sel.era_overlap, 1, "{sel:?}");
        assert_invariants(&sel, ctrl());
    }

    /// A room where the only link candidate cannot reach any slot falls
    /// back to container-only placement (tier 2) instead of failing the
    /// room outright — a container-era room beats no plan, and beats
    /// the pre-D2 behavior of placing a link the bot never classifies.
    #[test]
    fn no_useful_link_falls_back_to_container_only() {
        // Free: container spot (24,25), one coverage tile (23,25), and
        // a far tile (22,23) that is within 2 of the container but not
        // within 1 of any remaining slot.
        let sel = best(walls_except(&[(24, 25), (23, 25), (22, 23)], 4), west_haul)
            .expect("must fall back to container-only");
        assert_eq!(sel.link, None, "{sel:?}");
        assert!(!sel.slots.is_empty(), "{sel:?}");
        assert_invariants(&sel, ctrl());
    }

    /// A fully-walled controller yields no candidates at all.
    #[test]
    fn sealed_controller_yields_none() {
        assert!(select_controller_infra(ctrl(), free_with_walls(walls_except(&[], 4)), west_haul)
            .is_empty());
    }

    /// Terrain-sealed pocket: free tiles east of the controller score
    /// high but are hub-unreachable (haul == MAX). They must never
    /// outrank the reachable west side — the pre-fix code picked the
    /// pocket and bricked the room plan.
    #[test]
    fn unreachable_pockets_are_never_selected() {
        let haul = |t: Location| {
            if t.x() > CTRL.0 {
                u32::MAX
            } else {
                t.x() as u32
            }
        };
        let all = select_controller_infra(ctrl(), free_with_walls(Default::default()), haul);
        assert!(!all.is_empty());
        for sel in &all {
            assert!(
                haul(sel.approach) != u32::MAX,
                "unreachable approach must be filtered: {sel:?}"
            );
        }
    }

    /// Range-3 containers lose their far-side coverage (Chebyshev 4
    /// from the controller is out of upgrade range), so they only win
    /// when walls force them — never in open terrain.
    #[test]
    fn far_side_tiles_are_not_slots() {
        let open: Vec<(u8, u8)> = [(22u8, 24u8), (22, 25), (22, 26), (21, 24), (21, 25), (21, 26)]
            .into_iter()
            .collect();
        let sel = best(walls_except(&open, 4), west_haul).expect("western pocket must place");
        assert_eq!(u32::from(sel.container.distance_to(ctrl())), 3, "{sel:?}");
        assert_invariants(&sel, ctrl());
    }

    /// Controllers near the room edge are handled by bounds/interiority
    /// checks rather than panicking.
    #[test]
    fn map_edge_controller_is_handled() {
        let edge_ctrl = Location::from_xy(2, 2);
        let free = move |t: Location| t.is_interior() && t != edge_ctrl;
        let all = select_controller_infra(edge_ctrl, free, west_haul);
        assert!(!all.is_empty());
        for sel in &all {
            assert_invariants(sel, edge_ctrl);
        }
    }

    /// The circulation guard itself: sealing the approach behind
    /// blocked (parked) tiles must fail it; an open route must pass.
    #[test]
    fn circulation_guard_detects_sealed_approach() {
        // Terrain: all plain except a wall ring around (40,25) with a
        // single gap at (38,25).
        let mut buffer = vec![0u8; 2500];
        let approach = Location::from_xy(40, 25);
        for dy in -1i32..=1 {
            for dx in -1i32..=1 {
                if dx == 0 && dy == 0 {
                    continue;
                }
                let (x, y) = (40 + dx, 25 + dy);
                buffer[(y * 50 + x) as usize] = 1; // wall
            }
        }
        buffer[(25 * 50 + 39) as usize] = 0; // the gap (39,25)
        let terrain = FastRoomTerrain::new(buffer);
        let hub = Location::from_xy(10, 25);
        let structures = FnvHashMap::default();

        let open: FnvHashSet<Location> = FnvHashSet::default();
        assert!(approach_reachable_with_parked_slots(
            &terrain, &structures, hub, &open, approach
        ));
        let mut sealed = FnvHashSet::default();
        sealed.insert(Location::from_xy(39, 25));
        assert!(!approach_reachable_with_parked_slots(
            &terrain, &structures, hub, &sealed, approach
        ));
    }
}
