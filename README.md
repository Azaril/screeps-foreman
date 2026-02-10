# screeps-foreman

A room base-planning library for [Screeps](https://screeps.com/), written in Rust. screeps-foreman generates optimized room layouts through a layered pipeline of placement algorithms, producing a complete build plan from RCL 1 through RCL 8.

**Decoupled from the Screeps runtime:** The planning algorithms use only pure-Rust data types from `screeps-game-api` (which compile on native targets). Game API calls for executing plans and rendering visuals are gated behind the optional `screeps` feature, allowing offline testing, benchmarking, and integration into non-Screeps projects.

## Features

| Feature | Default | Description |
|---------|---------|-------------|
| *(none)* | yes | Core planning library. Compiles on native targets for offline use. |
| `screeps` | | Convenience functions that call the Screeps game API: `execute_operations()`, `snapshot_structures()`, and `impl RoomVisualizer for RoomVisual`. |
| `profile` | | Profiling instrumentation via `screeps-timing`. |

## Architecture

The planner runs a multi-layer pipeline that progressively builds up a room plan:

```
┌─────────────────────────────────────────────────────────┐
│                    PlannerRoomDataSource                  │
│  (terrain, sources, mineral, controller position, etc.)  │
└──────────────────────────┬──────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                    Planning Pipeline                      │
│                                                          │
│  1. Hub placement (spawn + storage + link + terminal)    │
│  2. Extension placement (stamps or flood-fill)           │
│  3. Source infrastructure (containers, links, roads)     │
│  4. Mineral infrastructure (extractor, container, road)  │
│  5. Controller infrastructure (link, container, road)    │
│  6. Tower placement                                      │
│  7. Spawn placement                                      │
│  8. Defense walls and ramparts                           │
│  9. Road network (inter-structure connectivity)          │
│  10. Road pruning (remove redundant roads)               │
│  11. Reachability validation                             │
│  12. RCL assignment (order structures by RCL)            │
│  13. Scoring (quality metrics)                           │
│  14. Finalization                                        │
│                                                          │
└──────────────────────────┬──────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────┐
│                         Plan                              │
│  • Per-tile structure assignments with RCL requirements   │
│  • Build/cleanup operations for incremental construction  │
│  • Visualization support via RoomVisualizer trait         │
└─────────────────────────────────────────────────────────┘
```

## Key Types

### `Plan`

The output of the planner. Contains a grid of `RoomItem` entries (structure type + required RCL) indexed by `Location`.

```rust
// Get structures to build at a given RCL
let items = plan.get_build_operations(current_rcl, &existing_structures);

// Get structures to remove (misplaced or obsolete)
let cleanup = plan.get_cleanup_operations(current_rcl, &existing_structures);
```

### `PlannerRoomDataSource`

Trait that provides room data to the planner. Implement this to feed terrain, source positions, mineral type, controller position, etc.

### `RoomVisualizer`

Trait for rendering plan structures. The `screeps` feature provides `impl RoomVisualizer for RoomVisual`.

```rust
pub trait RoomVisualizer {
    fn render(&mut self, location: Location, structure: StructureType);
}
```

### `PlanOperation`

Describes a single build or destroy action:

```rust
pub enum PlanOperation {
    CreateSite { location: Location, structure_type: StructureType },
    DestroyStructure { location: Location, structure_type: StructureType, safe_only: bool },
}
```

## Integration Guide

### Offline / Native Usage

```toml
[dependencies]
screeps-foreman = { path = "../screeps-foreman" }
```

The planner runs entirely in Rust with no game API calls. Provide room data via `PlannerRoomDataSource` and receive a `Plan` with structure placements.

### In-Game Usage

```toml
[dependencies]
screeps-foreman = { path = "../screeps-foreman", features = ["screeps"] }
```

With the `screeps` feature enabled, you get:

- **`execute_operations(room, operations)`** — Creates construction sites and destroys structures as directed by the plan operations list.
- **`snapshot_structures(structures)`** — Converts the game's `StructureObject` slice into the lightweight `ExistingStructure` representation.
- **`impl RoomVisualizer for RoomVisual`** — Renders plan structures using the Screeps room visual API (via `screeps-visual`).

## Dependencies

| Crate | Purpose |
|-------|---------|
| [screeps-game-api](https://github.com/rustyscreeps/screeps-game-api) | `StructureType` and other pure-Rust data types |
| [screeps-visual](https://github.com/Azaril/screeps-visual) | Structure rendering primitives |
| [screeps-cache](https://github.com/Azaril/screeps-cache) | Lazy evaluation and caching utilities |
| [pathfinding](https://crates.io/crates/pathfinding) | A* and other graph search algorithms |
| [rs-graph](https://crates.io/crates/rs-graph) | Graph data structures for road networks |
| [serde](https://serde.rs/) | Serialization for plans and room data |
| [itertools](https://crates.io/crates/itertools) | Iterator combinators |
| [bitflags](https://crates.io/crates/bitflags) | Bitflag types for terrain and structure masks |
| [uuid](https://crates.io/crates/uuid) | Unique identifiers for plan elements |
| [fnv](https://crates.io/crates/fnv) | Fast hash maps for location-indexed data |

## License

See repository root for license information.
