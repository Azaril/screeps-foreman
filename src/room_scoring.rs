//! Composable room-candidate scoring — the SHARED stage-1 implementation
//! (screeps-ibex Phase 0, P0.P7: "one room-candidate scoring
//! implementation, not three").
//!
//! WHAT LIVES HERE: cheap, pure heuristics that rank candidate rooms
//! BEFORE full plan scoring — terrain quality (swamp/wall fractions,
//! exit perimeter), object quality (source count, controller presence,
//! mineral type), and caller-composable CONTEXT scorers (proximity to
//! already-selected rooms, route distance to owned rooms). Full
//! plan-quality scoring (the layer stack's `PlanScore`) stays in
//! `crate::layers` — this module is the stage BEFORE the planner runs.
//!
//! CONSUMERS:
//! - `screeps-prospector` (now): stage-1 spawn-site ranking delegates
//!   here; prospector keeps its policy shell (CLI weights, cache
//!   fetch-listing, stage-2 planning of finalists).
//! - live Ibex claim/expansion (rewrite Increment 7, ADR 0009 D3.5 /
//!   IBEX-032): `claim.rs` composes the same core scorers with live
//!   context (`RoomRouteCache` route distances, owned/selected rooms,
//!   threat marks) via [`ScoringContext`].
//! - `screeps-foreman-bench` weight calibration (ADR 0009 D2): with
//!   both stage-1 and plan scoring hosted in foreman, the bench can
//!   calibrate the weights of BOTH stages in one place.
//!
//! PURITY: no game-API calls, no `screeps` feature requirement — the
//! module compiles and runs on wasm (the bot) and native hosts
//! (prospector, the bench) alike. Only pure `screeps-game-api` types
//! (`RoomName`) are used.
//!
//! SHAPE:
//! - [`RoomFacts`] — a pure DTO of per-room facts ([`terrain_stats`]
//!   derives the terrain part from [`FastRoomTerrain`]).
//! - [`RoomScorer`] — one subscore in `[0, 1]`; `None` = "cannot
//!   evaluate" (missing data/context) and drops the scorer's weight for
//!   that room; `disqualify` separately removes a room from candidacy
//!   with a reason while it still receives a score for reporting.
//! - [`ScoringContext`] — caller-supplied cross-room context. Core
//!   scorers ignore it; context scorers read it.
//! - [`WeightedPipeline`] — runs scorers in insertion order and
//!   aggregates `sum(w_i * s_i) / sum(w_i)` over the scorers that
//!   returned `Some` (0.0 when no weight participates), preserving the
//!   exact stage-1 arithmetic prospector pinned.

use crate::room_data::PlanLocation;
use crate::terrain::{FastRoomTerrain, TerrainFlags};
use fnv::{FnvHashMap, FnvHashSet};
use screeps::local::RoomName;
use screeps_common::constants::ROOM_AREA;

/// Exit-tile count at which the exit subscore bottoms out (a fully open
/// 50x50 border has 196 exit tiles; typical rooms have 30-150).
pub const EXIT_TILE_CAP: usize = 200;

// ---------------------------------------------------------------------------
// Room facts (the pure DTO scorers read)
// ---------------------------------------------------------------------------

/// Terrain-derived stats for one room (a pure function of the 2500-tile
/// terrain buffer — see [`terrain_stats`]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TerrainStats {
    /// Swamp tiles / non-wall tiles (swamps matter only where creeps walk).
    pub swamp_fraction: f32,
    /// Wall tiles / all 2500 tiles.
    pub wall_fraction: f32,
    /// Passable border tiles ([`FastRoomTerrain::get_exits`]).
    pub exit_tiles: usize,
    /// Number of room sides (top/right/bottom/left) with at least one exit.
    pub exit_sides: usize,
}

/// Compute [`TerrainStats`] from decoded terrain. Tiles flagged both
/// wall and swamp count as wall (walls dominate).
pub fn terrain_stats(terrain: &FastRoomTerrain) -> TerrainStats {
    let mut walls = 0usize;
    let mut swamps = 0usize;
    for y in 0..50u8 {
        for x in 0..50u8 {
            let flags = terrain.get_xy(x, y);
            if flags.contains(TerrainFlags::WALL) {
                walls += 1;
            } else if flags.contains(TerrainFlags::SWAMP) {
                swamps += 1;
            }
        }
    }
    let exits = terrain.get_exits();
    // Sides: top (y=0), right (x=49), bottom (y=49), left (x=0). Corner
    // tiles count for both adjacent sides.
    let mut sides = [false; 4];
    for exit in &exits {
        if exit.y() == 0 {
            sides[0] = true;
        }
        if exit.x() == 49 {
            sides[1] = true;
        }
        if exit.y() == 49 {
            sides[2] = true;
        }
        if exit.x() == 0 {
            sides[3] = true;
        }
    }
    let non_wall = (ROOM_AREA - walls).max(1);
    TerrainStats {
        swamp_fraction: swamps as f32 / non_wall as f32,
        wall_fraction: walls as f32 / ROOM_AREA as f32,
        exit_tiles: exits.len(),
        exit_sides: sides.iter().filter(|s| **s).count(),
    }
}

/// The room's mineral, when present.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MineralFact {
    pub location: PlanLocation,
    /// `"X"`, `"H"`, ... — `None` when the type is unknown to the caller.
    pub mineral_type: Option<String>,
}

/// Pure per-room facts — everything the scorers may read about the room
/// itself. Cross-room information (selected rooms, route distances,
/// threat) lives in [`ScoringContext`] instead.
#[derive(Debug, Clone, PartialEq)]
pub struct RoomFacts {
    /// Room name (e.g. `"W5N5"`). Kept as the caller's raw string; the
    /// scorers that need coordinates parse it via [`RoomFacts::room_name`]
    /// and treat an unparseable name as missing data.
    pub room: String,
    pub terrain: TerrainStats,
    pub sources: Vec<PlanLocation>,
    pub controller: Option<PlanLocation>,
    pub mineral: Option<MineralFact>,
}

impl RoomFacts {
    /// Facts with the terrain part derived and no objects; callers fill
    /// in the object fields they know about.
    pub fn from_terrain(room: impl Into<String>, terrain: &FastRoomTerrain) -> Self {
        RoomFacts {
            room: room.into(),
            terrain: terrain_stats(terrain),
            sources: Vec::new(),
            controller: None,
            mineral: None,
        }
    }

    /// The parsed room name, when the raw string is a valid one.
    pub fn room_name(&self) -> Option<RoomName> {
        RoomName::new(&self.room).ok()
    }
}

// ---------------------------------------------------------------------------
// Context (the composability point)
// ---------------------------------------------------------------------------

/// Caller-supplied cross-room context. Core scorers ignore it; CONTEXT
/// scorers read the slice of it they care about and return `None` when
/// their slice is absent (missing data — the pipeline drops their
/// weight for that room).
///
/// - prospector composes this from its cache/static data (e.g. rooms it
///   already placed spawns in during a multi-bot bootstrap);
/// - live Ibex (Increment 7) composes it from game state: owned rooms
///   as `selected_rooms`, `RoomRouteCache` distances as
///   `route_distance_to_owned`, hostile-room intel as `threat_rooms`.
#[derive(Debug, Clone, Default)]
pub struct ScoringContext {
    /// Rooms already chosen/owned — read by [`ProximityToSelectedScorer`].
    pub selected_rooms: Vec<RoomName>,
    /// Route distance (in rooms, by actual route — not linear delta)
    /// from each candidate to the nearest owned room — read by
    /// [`RouteDistanceToOwnedScorer`]. Candidates absent from the map
    /// score `None` (missing data); callers that know absence means
    /// unreachable should gate/disqualify before scoring.
    pub route_distance_to_owned: FnvHashMap<RoomName, u32>,
    /// Rooms marked threatened (hostile activity, source keepers, ...).
    /// No reference scorer reads this yet; it is part of the context
    /// contract so threat scorers compose without a signature change.
    pub threat_rooms: FnvHashSet<RoomName>,
}

// ---------------------------------------------------------------------------
// The scorer trait + weighted pipeline
// ---------------------------------------------------------------------------

/// One room-candidate subscore.
pub trait RoomScorer {
    /// Stable name — keys subscores in [`RoomScore`] and weight tables.
    fn name(&self) -> &str;

    /// Weight used when the caller does not supply one
    /// ([`WeightedPipeline::with_scorer`]).
    fn default_weight(&self) -> f32;

    /// Score the room in `[0, 1]`. `None` = this scorer cannot evaluate
    /// the room (missing facts or context) — the pipeline drops the
    /// scorer's weight for this room rather than treating it as 0.
    fn score(&self, facts: &RoomFacts, context: &ScoringContext) -> Option<f32>;

    /// Hard disqualification: `Some(reason)` removes the room from
    /// candidacy. The room still receives a full [`RoomScore`] so
    /// callers can report it (prospector lists disqualified rooms with
    /// their score and reason).
    fn disqualify(&self, _facts: &RoomFacts, _context: &ScoringContext) -> Option<String> {
        None
    }
}

/// One scorer's contribution to a [`RoomScore`].
#[derive(Debug, Clone, PartialEq)]
pub struct SubScore {
    pub name: String,
    /// `None` = the scorer could not evaluate this room (weight dropped).
    pub score: Option<f32>,
    pub weight: f32,
}

/// A scored room: the weighted-average total, every subscore, and the
/// first disqualification reason (in scorer order), if any.
#[derive(Debug, Clone, PartialEq)]
pub struct RoomScore {
    /// `sum(w_i * s_i) / sum(w_i)` over scorers that returned `Some`;
    /// 0.0 when no weight participated. In `[0, 1]` for non-negative
    /// weights.
    pub total: f32,
    /// In pipeline (insertion) order.
    pub subscores: Vec<SubScore>,
    /// First `Some` from [`RoomScorer::disqualify`] in pipeline order.
    pub disqualified: Option<String>,
}

impl RoomScore {
    /// Look up a subscore by scorer name.
    pub fn subscore(&self, name: &str) -> Option<f32> {
        self.subscores
            .iter()
            .find(|s| s.name == name)
            .and_then(|s| s.score)
    }
}

/// Runs scorers in insertion order and aggregates a weighted average.
///
/// Aggregation contract (pinned — prospector's stage-1 rankings depend
/// on it bit-for-bit): accumulate `weighted += w_i * s_i` and
/// `weight_sum += w_i` left-to-right over the scorers that returned
/// `Some`, then `total = weighted / weight_sum` (0.0 when
/// `weight_sum <= 0`). Zero-weight scorers still run and report their
/// subscore; they just contribute nothing.
#[derive(Default)]
pub struct WeightedPipeline {
    scorers: Vec<(Box<dyn RoomScorer>, f32)>,
}

impl WeightedPipeline {
    pub fn new() -> Self {
        Self::default()
    }

    /// The six core scorers at their default weights, in the canonical
    /// order: sources, controller, mineral, swamp, walls, exits.
    pub fn core() -> Self {
        Self::new()
            .with_scorer(SourceCountScorer)
            .with_scorer(ControllerScorer)
            .with_scorer(MineralScorer)
            .with_scorer(SwampScorer)
            .with_scorer(WallsScorer)
            .with_scorer(ExitScorer)
    }

    /// Append a scorer at its [`RoomScorer::default_weight`].
    pub fn with_scorer(self, scorer: impl RoomScorer + 'static) -> Self {
        let weight = scorer.default_weight();
        self.with_weighted(scorer, weight)
    }

    /// Append a scorer at an explicit weight.
    pub fn with_weighted(mut self, scorer: impl RoomScorer + 'static, weight: f32) -> Self {
        self.scorers.push((Box::new(scorer), weight));
        self
    }

    /// Score one room against the context.
    pub fn score_room(&self, facts: &RoomFacts, context: &ScoringContext) -> RoomScore {
        let mut weighted = 0.0f32;
        let mut weight_sum = 0.0f32;
        let mut subscores = Vec::with_capacity(self.scorers.len());
        let mut disqualified = None;
        for (scorer, weight) in &self.scorers {
            let score = scorer.score(facts, context);
            if let Some(score) = score {
                weighted += *weight * score;
                weight_sum += *weight;
            }
            if disqualified.is_none() {
                disqualified = scorer.disqualify(facts, context);
            }
            subscores.push(SubScore {
                name: scorer.name().to_owned(),
                score,
                weight: *weight,
            });
        }
        let total = if weight_sum > 0.0 {
            weighted / weight_sum
        } else {
            0.0
        };
        RoomScore {
            total,
            subscores,
            disqualified,
        }
    }
}

// ---------------------------------------------------------------------------
// Core scorers (context-free; extracted from prospector stage 1)
// ---------------------------------------------------------------------------

/// Source-count curve: 2 strongly preferred (the standard claim
/// target), 1 viable but slow, 3+ unusual (SK-adjacent layouts), 0
/// useless.
pub fn source_count_curve(count: usize) -> f32 {
    match count {
        0 => 0.0,
        1 => 0.45,
        2 => 1.0,
        3 => 0.65,
        _ => 0.5,
    }
}

/// Mineral curve: presence is most of the value; the type bonus prefers
/// X (catalyst — every tier-3 boost needs it), then H/O (the base
/// feedstocks).
pub fn mineral_curve(mineral: Option<&MineralFact>) -> f32 {
    match mineral {
        None => 0.0,
        Some(fact) => {
            let type_bonus = match fact.mineral_type.as_deref() {
                Some("X") => 0.2,
                Some("H") | Some("O") => 0.15,
                _ => 0.1,
            };
            0.8 + type_bonus
        }
    }
}

/// Exit curve: fewer exit tiles on fewer sides = a cheaper, more
/// defensible perimeter. 0 exits would mean a sealed (unreachable) room
/// — scored 0 as almost certainly bad data.
pub fn exit_curve(exit_tiles: usize, exit_sides: usize) -> f32 {
    if exit_tiles == 0 {
        return 0.0;
    }
    let side_score = match exit_sides {
        0 | 1 => 1.0,
        2 => 0.75,
        3 => 0.5,
        _ => 0.25,
    };
    let tile_score = 1.0 - (exit_tiles.min(EXIT_TILE_CAP) as f32 / EXIT_TILE_CAP as f32);
    0.5 * side_score + 0.5 * tile_score
}

/// Source count ([`source_count_curve`]). Disqualifies sourceless rooms
/// — no economy, and the planner's `SourceInfraLayer` fails on missing
/// sources.
pub struct SourceCountScorer;

impl RoomScorer for SourceCountScorer {
    fn name(&self) -> &str {
        "sources"
    }

    fn default_weight(&self) -> f32 {
        3.0
    }

    fn score(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
        Some(source_count_curve(facts.sources.len()))
    }

    fn disqualify(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<String> {
        facts
            .sources
            .is_empty()
            .then(|| "no sources — the room has no economy and room planning would fail".to_owned())
    }
}

/// Controller presence. Disqualifies controllerless rooms — unclaimable,
/// and the planner's `ControllerInfraLayer` fails on missing.
pub struct ControllerScorer;

impl RoomScorer for ControllerScorer {
    fn name(&self) -> &str {
        "controller"
    }

    fn default_weight(&self) -> f32 {
        2.0
    }

    fn score(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
        Some(if facts.controller.is_some() { 1.0 } else { 0.0 })
    }

    fn disqualify(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<String> {
        facts
            .controller
            .is_none()
            .then(|| "no controller — unclaimable, room planning would fail".to_owned())
    }
}

/// Mineral presence + type preference ([`mineral_curve`]).
pub struct MineralScorer;

impl RoomScorer for MineralScorer {
    fn name(&self) -> &str {
        "mineral"
    }

    fn default_weight(&self) -> f32 {
        0.5
    }

    fn score(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
        Some(mineral_curve(facts.mineral.as_ref()))
    }
}

/// `1 - swamp_fraction` (movement/road cost).
pub struct SwampScorer;

impl RoomScorer for SwampScorer {
    fn name(&self) -> &str {
        "swamp"
    }

    fn default_weight(&self) -> f32 {
        1.0
    }

    fn score(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
        Some(1.0 - facts.terrain.swamp_fraction)
    }
}

/// `1 - wall_fraction` (buildable area; extreme cramping is caught by
/// plan failure downstream, not here).
pub struct WallsScorer;

impl RoomScorer for WallsScorer {
    fn name(&self) -> &str {
        "walls"
    }

    fn default_weight(&self) -> f32 {
        0.5
    }

    fn score(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
        Some(1.0 - facts.terrain.wall_fraction)
    }
}

/// Exit count + side distribution ([`exit_curve`]; defensibility —
/// fewer exit tiles on fewer sides = a cheaper perimeter).
pub struct ExitScorer;

impl RoomScorer for ExitScorer {
    fn name(&self) -> &str {
        "exits"
    }

    fn default_weight(&self) -> f32 {
        1.0
    }

    fn score(&self, facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
        Some(exit_curve(
            facts.terrain.exit_tiles,
            facts.terrain.exit_sides,
        ))
    }
}

// ---------------------------------------------------------------------------
// Context scorers (reference implementations of the composition point)
// ---------------------------------------------------------------------------

/// Room-grid Chebyshev distance between two room names (pure room-name
/// math — the linear distance in rooms, ignoring walls/routes).
pub fn room_grid_distance(a: RoomName, b: RoomName) -> u32 {
    let dx = a.x_coord().abs_diff(b.x_coord());
    let dy = a.y_coord().abs_diff(b.y_coord());
    dx.max(dy)
}

/// Whether [`ProximityToSelectedScorer`] rewards being near or far.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistancePreference {
    /// Closer to the selected set is better (e.g. expansion claims that
    /// want short support lines).
    Near,
    /// Farther from the selected set is better (e.g. seating multiple
    /// bots on one world without overlapping remote-mining footprints).
    Far,
}

/// CONTEXT scorer: distance from the candidate to the nearest room in
/// [`ScoringContext::selected_rooms`], normalized against `saturation`
/// (the room-grid distance at which the score stops changing).
///
/// `None` (weight dropped) when the selected set is empty or the
/// candidate's room name does not parse.
pub struct ProximityToSelectedScorer {
    pub preference: DistancePreference,
    /// Distance (in rooms) at which the score saturates; min 1.
    pub saturation: u32,
}

impl Default for ProximityToSelectedScorer {
    fn default() -> Self {
        ProximityToSelectedScorer {
            preference: DistancePreference::Far,
            saturation: 10,
        }
    }
}

impl RoomScorer for ProximityToSelectedScorer {
    fn name(&self) -> &str {
        "proximity_to_selected"
    }

    fn default_weight(&self) -> f32 {
        1.0
    }

    fn score(&self, facts: &RoomFacts, context: &ScoringContext) -> Option<f32> {
        let room = facts.room_name()?;
        let nearest = context
            .selected_rooms
            .iter()
            .map(|selected| room_grid_distance(room, *selected))
            .min()?;
        let saturation = self.saturation.max(1);
        let normalized = nearest.min(saturation) as f32 / saturation as f32;
        Some(match self.preference {
            DistancePreference::Far => normalized,
            DistancePreference::Near => 1.0 - normalized,
        })
    }
}

/// CONTEXT scorer: route distance to the nearest owned room, from the
/// caller-supplied [`ScoringContext::route_distance_to_owned`] map
/// (live Ibex fills this from `RoomRouteCache`; offline callers from
/// whatever route data they have). Nearer is better; the score reaches
/// 0 at `saturation` rooms.
///
/// `None` (weight dropped) when the candidate has no entry or its room
/// name does not parse. Callers that know "no entry" means unreachable
/// should gate those rooms out (or disqualify) before scoring.
pub struct RouteDistanceToOwnedScorer {
    /// Route distance (in rooms) at which the score reaches 0; min 1.
    pub saturation: u32,
}

impl Default for RouteDistanceToOwnedScorer {
    fn default() -> Self {
        RouteDistanceToOwnedScorer { saturation: 10 }
    }
}

impl RoomScorer for RouteDistanceToOwnedScorer {
    fn name(&self) -> &str {
        "route_distance_to_owned"
    }

    fn default_weight(&self) -> f32 {
        1.0
    }

    fn score(&self, facts: &RoomFacts, context: &ScoringContext) -> Option<f32> {
        let room = facts.room_name()?;
        let distance = *context.route_distance_to_owned.get(&room)?;
        let saturation = self.saturation.max(1);
        Some(1.0 - (distance.min(saturation) as f32 / saturation as f32))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use screeps_common::constants::{ROOM_HEIGHT, ROOM_WIDTH};

    /// Terrain fixture: walled border with a 10-tile exit on the top
    /// edge (x in 20..30), interior filled with `interior` (a raw
    /// terrain mask value: 0 plain, 1 wall, 2 swamp).
    fn fixture_terrain(interior: u8) -> FastRoomTerrain {
        let mut buffer = Vec::with_capacity(ROOM_AREA);
        for y in 0..ROOM_HEIGHT {
            for x in 0..ROOM_WIDTH {
                let border = x == 0 || y == 0 || x == ROOM_WIDTH - 1 || y == ROOM_HEIGHT - 1;
                let value = if border {
                    if y == 0 && (20..30).contains(&x) {
                        0
                    } else {
                        1
                    }
                } else {
                    interior
                };
                buffer.push(value);
            }
        }
        FastRoomTerrain::new(buffer)
    }

    fn mineral(kind: Option<&str>) -> MineralFact {
        MineralFact {
            location: PlanLocation::new(40, 40),
            mineral_type: kind.map(str::to_owned),
        }
    }

    /// Facts for an ideal 2-source room on the plain fixture terrain.
    fn ideal_facts(room: &str) -> RoomFacts {
        RoomFacts {
            sources: vec![PlanLocation::new(14, 9), PlanLocation::new(38, 41)],
            controller: Some(PlanLocation::new(21, 32)),
            mineral: Some(mineral(Some("X"))),
            ..RoomFacts::from_terrain(room, &fixture_terrain(0))
        }
    }

    // ---- core curves (pinned values — prospector's rankings rest on these) ----

    #[test]
    fn source_count_curve_strongly_prefers_two() {
        assert_eq!(source_count_curve(0), 0.0);
        assert_eq!(source_count_curve(1), 0.45);
        assert_eq!(source_count_curve(2), 1.0);
        assert_eq!(source_count_curve(3), 0.65);
        assert_eq!(source_count_curve(4), 0.5);
        assert!(source_count_curve(2) > source_count_curve(3));
        assert!(source_count_curve(3) > source_count_curve(1));
        assert!(source_count_curve(1) > source_count_curve(0));
    }

    #[test]
    fn mineral_curve_orders_x_first() {
        assert_eq!(mineral_curve(None), 0.0);
        assert_eq!(mineral_curve(Some(&mineral(Some("X")))), 0.8 + 0.2);
        assert_eq!(mineral_curve(Some(&mineral(Some("H")))), 0.8 + 0.15);
        assert_eq!(mineral_curve(Some(&mineral(Some("O")))), 0.8 + 0.15);
        assert_eq!(mineral_curve(Some(&mineral(Some("Z")))), 0.8 + 0.1);
        assert_eq!(mineral_curve(Some(&mineral(None))), 0.8 + 0.1);
        assert!(
            mineral_curve(Some(&mineral(Some("X")))) > mineral_curve(Some(&mineral(Some("H"))))
        );
        assert!(
            mineral_curve(Some(&mineral(Some("H")))) > mineral_curve(Some(&mineral(Some("Z"))))
        );
    }

    #[test]
    fn exit_curve_prefers_few_tiles_on_few_sides() {
        assert_eq!(exit_curve(0, 0), 0.0, "sealed room = bad data");
        assert_eq!(exit_curve(10, 1), 0.5 + 0.5 * (1.0 - 10.0 / 200.0));
        assert!(exit_curve(10, 1) > exit_curve(150, 1));
        assert!(exit_curve(150, 1) > exit_curve(150, 4));
        // Tile count clamps at the cap rather than going negative.
        assert_eq!(exit_curve(500, 4), 0.5 * 0.25);
    }

    #[test]
    fn terrain_stats_on_fixture() {
        let stats = terrain_stats(&fixture_terrain(2));
        // Border = 196 tiles, 10 opened as exits on the top edge.
        assert_eq!(stats.exit_tiles, 10);
        assert_eq!(stats.exit_sides, 1);
        let walls = 196 - 10;
        assert!((stats.wall_fraction - walls as f32 / 2500.0).abs() < 1e-6);
        // Interior (48*48) is all swamp; the 10 exit tiles are plain.
        let non_wall = 2500 - walls;
        assert!((stats.swamp_fraction - (48.0 * 48.0) / non_wall as f32).abs() < 1e-6);
    }

    // ---- pipeline aggregation ----

    /// Fixed-output scorer for aggregation tests.
    struct Fixed {
        name: &'static str,
        value: Option<f32>,
    }

    impl RoomScorer for Fixed {
        fn name(&self) -> &str {
            self.name
        }

        fn default_weight(&self) -> f32 {
            1.0
        }

        fn score(&self, _facts: &RoomFacts, _context: &ScoringContext) -> Option<f32> {
            self.value
        }
    }

    #[test]
    fn pipeline_weighted_average_matches_hand_arithmetic() {
        let facts = ideal_facts("W1N1");
        let context = ScoringContext::default();
        let pipeline = WeightedPipeline::new()
            .with_weighted(
                Fixed {
                    name: "a",
                    value: Some(1.0),
                },
                3.0,
            )
            .with_weighted(
                Fixed {
                    name: "b",
                    value: Some(0.5),
                },
                1.0,
            );
        let score = pipeline.score_room(&facts, &context);
        assert_eq!(score.total, (3.0 * 1.0 + 1.0 * 0.5) / 4.0);
        assert_eq!(score.subscore("a"), Some(1.0));
        assert_eq!(score.subscore("b"), Some(0.5));
        assert!(score.disqualified.is_none());
    }

    #[test]
    fn pipeline_drops_weight_for_none_scorers() {
        let facts = ideal_facts("W1N1");
        let context = ScoringContext::default();
        let pipeline = WeightedPipeline::new()
            .with_weighted(
                Fixed {
                    name: "present",
                    value: Some(0.5),
                },
                1.0,
            )
            .with_weighted(
                Fixed {
                    name: "missing",
                    value: None,
                },
                100.0,
            );
        let score = pipeline.score_room(&facts, &context);
        assert_eq!(
            score.total, 0.5,
            "a None scorer's weight must not dilute the average"
        );
        assert_eq!(score.subscore("missing"), None);
        // All-None (or all-zero-weight) pipelines score 0.0, not NaN.
        let empty = WeightedPipeline::new()
            .with_weighted(
                Fixed {
                    name: "missing",
                    value: None,
                },
                1.0,
            )
            .score_room(&facts, &context);
        assert_eq!(empty.total, 0.0);
    }

    #[test]
    fn core_pipeline_scores_ideal_room_like_prospector_stage1() {
        let facts = ideal_facts("W1N1");
        let score = WeightedPipeline::core().score_room(&facts, &ScoringContext::default());
        // Hand-computed prospector stage-1 arithmetic over the same facts.
        let stats = facts.terrain;
        let expected = (3.0 * 1.0
            + 2.0 * 1.0
            + 0.5 * 1.0
            + 1.0 * (1.0 - stats.swamp_fraction)
            + 0.5 * (1.0 - stats.wall_fraction)
            + 1.0 * exit_curve(stats.exit_tiles, stats.exit_sides))
            / (3.0 + 2.0 + 0.5 + 1.0 + 0.5 + 1.0);
        assert_eq!(score.total, expected);
        assert!(
            score.total > 0.95,
            "ideal room scores high: {}",
            score.total
        );
        assert!(score.disqualified.is_none());
        assert_eq!(score.subscore("sources"), Some(1.0));
        assert_eq!(score.subscore("controller"), Some(1.0));
        assert_eq!(score.subscore("mineral"), Some(1.0));
    }

    // ---- disqualification semantics ----

    #[test]
    fn missing_controller_disqualifies_with_reason_but_still_scores() {
        let facts = RoomFacts {
            controller: None,
            ..ideal_facts("W4N1")
        };
        let score = WeightedPipeline::core().score_room(&facts, &ScoringContext::default());
        assert!(score
            .disqualified
            .as_deref()
            .unwrap()
            .contains("no controller"));
        assert!(score.total > 0.0, "disqualified rooms still report a score");
        assert_eq!(score.subscore("controller"), Some(0.0));
    }

    #[test]
    fn missing_sources_disqualify_with_reason() {
        let facts = RoomFacts {
            sources: Vec::new(),
            ..ideal_facts("W4N1")
        };
        let score = WeightedPipeline::core().score_room(&facts, &ScoringContext::default());
        assert!(score
            .disqualified
            .as_deref()
            .unwrap()
            .contains("no sources"));
        assert_eq!(score.subscore("sources"), Some(0.0));
    }

    // ---- context scorers / composition ----

    #[test]
    fn room_grid_distance_is_chebyshev_on_room_coords() {
        let name = |s: &str| RoomName::new(s).unwrap();
        assert_eq!(room_grid_distance(name("W1N1"), name("W1N1")), 0);
        assert_eq!(room_grid_distance(name("W1N1"), name("W3N1")), 2);
        assert_eq!(room_grid_distance(name("W1N1"), name("W2N4")), 3);
        // E0/S0 and W0/N0 are adjacent (no room zero is shared).
        assert_eq!(room_grid_distance(name("E0S0"), name("W0N0")), 1);
    }

    #[test]
    fn proximity_scorer_needs_a_selected_set() {
        let scorer = ProximityToSelectedScorer::default();
        let facts = ideal_facts("W5N5");
        let context = ScoringContext::default();
        assert_eq!(
            scorer.score(&facts, &context),
            None,
            "empty selected set = missing data"
        );
        let unparseable = RoomFacts {
            room: "not-a-room".to_owned(),
            ..ideal_facts("W5N5")
        };
        let mut context = ScoringContext::default();
        context.selected_rooms.push(RoomName::new("W1N1").unwrap());
        assert_eq!(scorer.score(&unparseable, &context), None);
    }

    #[test]
    fn proximity_scorer_scores_distance_to_nearest_selected() {
        let scorer = ProximityToSelectedScorer {
            preference: DistancePreference::Far,
            saturation: 10,
        };
        let mut context = ScoringContext::default();
        context.selected_rooms.push(RoomName::new("W1N1").unwrap());
        context.selected_rooms.push(RoomName::new("W9N9").unwrap());
        // W2N1 is 1 from W1N1 (the nearest of the two).
        let near = scorer.score(&ideal_facts("W2N1"), &context).unwrap();
        let far = scorer.score(&ideal_facts("W20N20"), &context).unwrap();
        assert_eq!(near, 0.1);
        assert_eq!(far, 1.0, "saturates at `saturation` rooms");
        assert!(far > near, "Far preference rewards distance");
        let near_pref = ProximityToSelectedScorer {
            preference: DistancePreference::Near,
            saturation: 10,
        };
        assert_eq!(near_pref.score(&ideal_facts("W2N1"), &context), Some(0.9));
    }

    #[test]
    fn selected_set_changes_core_ranking_via_composition() {
        // Two rooms with IDENTICAL room facts — the core pipeline ties.
        let a = ideal_facts("W2N1");
        let b = ideal_facts("W20N20");
        let core = WeightedPipeline::core();
        let no_context = ScoringContext::default();
        assert_eq!(
            core.score_room(&a, &no_context).total,
            core.score_room(&b, &no_context).total,
            "identical facts tie without context"
        );

        // Compose the proximity scorer; a selected room next to A breaks
        // the tie in B's favor (Far preference).
        let composed = WeightedPipeline::core().with_scorer(ProximityToSelectedScorer::default());
        let mut context = ScoringContext::default();
        context.selected_rooms.push(RoomName::new("W1N1").unwrap());
        let score_a = composed.score_room(&a, &context).total;
        let score_b = composed.score_room(&b, &context).total;
        assert!(
            score_b > score_a,
            "selected set must re-rank: {score_b} <= {score_a}"
        );
        // Without context the composed pipeline still ties (the context
        // scorer returns None and its weight is dropped).
        assert_eq!(
            composed.score_room(&a, &no_context).total,
            composed.score_room(&b, &no_context).total
        );
    }

    #[test]
    fn route_distance_scorer_reads_the_supplied_map() {
        let scorer = RouteDistanceToOwnedScorer { saturation: 10 };
        let mut context = ScoringContext::default();
        context
            .route_distance_to_owned
            .insert(RoomName::new("W2N1").unwrap(), 2);
        context
            .route_distance_to_owned
            .insert(RoomName::new("W9N9").unwrap(), 12);
        assert_eq!(scorer.score(&ideal_facts("W2N1"), &context), Some(0.8));
        assert_eq!(
            scorer.score(&ideal_facts("W9N9"), &context),
            Some(0.0),
            "clamps at saturation"
        );
        assert_eq!(
            scorer.score(&ideal_facts("W5N5"), &context),
            None,
            "no entry = missing data, weight dropped"
        );
    }
}
