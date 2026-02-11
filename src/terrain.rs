use crate::constants::*;
use crate::location::*;
use bitflags::*;
use serde::{Deserialize, Serialize};

bitflags! {
    #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
    pub struct TerrainFlags: u8 {
        const NONE = 0;
        const WALL = 1;
        const SWAMP = 2;
        const LAVA = 4;
    }
}

#[derive(Clone)]
pub struct FastRoomTerrain {
    buffer: Vec<u8>,
}

impl FastRoomTerrain {
    pub fn new(buffer: Vec<u8>) -> FastRoomTerrain {
        FastRoomTerrain { buffer }
    }

    pub fn get(&self, pos: &Location) -> TerrainFlags {
        self.get_xy(pos.x(), pos.y())
    }

    pub fn get_xy(&self, x: u8, y: u8) -> TerrainFlags {
        let index = (y as usize * ROOM_WIDTH as usize) + (x as usize);
        TerrainFlags::from_bits_truncate(self.buffer[index])
    }

    pub fn is_wall(&self, x: u8, y: u8) -> bool {
        self.get_xy(x, y).contains(TerrainFlags::WALL)
    }

    pub fn is_swamp(&self, x: u8, y: u8) -> bool {
        self.get_xy(x, y).contains(TerrainFlags::SWAMP)
    }

    /// Returns an iterator over all passable exit tiles (tiles on the room border that are not walls).
    pub fn get_exits(&self) -> Vec<Location> {
        let mut exits = Vec::new();
        // Top edge
        for x in 0..ROOM_WIDTH {
            if !self.is_wall(x, 0) {
                exits.push(Location::from_coords(x as u32, 0));
            }
        }
        // Right edge
        for y in 1..ROOM_HEIGHT - 1 {
            if !self.is_wall(ROOM_WIDTH - 1, y) {
                exits.push(Location::from_coords(ROOM_WIDTH as u32 - 1, y as u32));
            }
        }
        // Bottom edge
        for x in 0..ROOM_WIDTH {
            if !self.is_wall(x, ROOM_HEIGHT - 1) {
                exits.push(Location::from_coords(x as u32, ROOM_HEIGHT as u32 - 1));
            }
        }
        // Left edge
        for y in 1..ROOM_HEIGHT - 1 {
            if !self.is_wall(0, y) {
                exits.push(Location::from_coords(0, y as u32));
            }
        }
        exits
    }
}

/// A 50x50 array for room-sized data.
#[derive(Clone)]
pub struct RoomDataArray<T: Copy> {
    data: Vec<T>,
}

impl<T: Copy> RoomDataArray<T> {
    pub fn new(initial: T) -> Self {
        RoomDataArray {
            data: vec![initial; (ROOM_WIDTH as usize) * (ROOM_HEIGHT as usize)],
        }
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize) -> &T {
        let index = y * (ROOM_WIDTH as usize) + x;
        &self.data[index]
    }

    #[inline]
    pub fn get_mut(&mut self, x: usize, y: usize) -> &mut T {
        let index = y * (ROOM_WIDTH as usize) + x;
        &mut self.data[index]
    }

    #[inline]
    pub fn set(&mut self, x: usize, y: usize, value: T) {
        *self.get_mut(x, y) = value;
    }

    pub fn iter(&self) -> impl Iterator<Item = ((usize, usize), &T)> {
        self.data.iter().enumerate().map(|(i, v)| {
            let x = i % (ROOM_WIDTH as usize);
            let y = i / (ROOM_WIDTH as usize);
            ((x, y), v)
        })
    }
}

impl<T: Copy + Serialize> Serialize for RoomDataArray<T> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        self.data.serialize(serializer)
    }
}

impl<'de, T: Copy + Deserialize<'de>> Deserialize<'de> for RoomDataArray<T> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        let data = Vec::<T>::deserialize(deserializer)?;
        if data.len() != (ROOM_WIDTH as usize) * (ROOM_HEIGHT as usize) {
            return Err(serde::de::Error::custom("Invalid room data array size"));
        }
        Ok(RoomDataArray { data })
    }
}

/// Compact serialization for `RoomDataArray<Option<u32>>`.
///
/// Flood-fill distances in a 50x50 room fit comfortably in a `u16`
/// (max distance ~100). This module encodes each element as a `u16`,
/// using `u16::MAX` (65535) as a sentinel for `None`. This halves the
/// serialized size compared to bincode's default `Option<u32>` encoding
/// (2 bytes vs 5 bytes per element).
pub mod compact_distance_serde {
    use super::*;
    use serde::{Deserializer, Serializer};

    const NONE_SENTINEL: u16 = u16::MAX;

    pub fn serialize<S>(data: &RoomDataArray<Option<u32>>, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let compact: Vec<u16> = data
            .data
            .iter()
            .map(|v| match v {
                Some(d) => {
                    // Clamp to u16 range (values > 65534 are extremely unlikely
                    // in a 50x50 room but we handle it defensively).
                    (*d).min(NONE_SENTINEL as u32 - 1) as u16
                }
                None => NONE_SENTINEL,
            })
            .collect();
        compact.serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<RoomDataArray<Option<u32>>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let compact = Vec::<u16>::deserialize(deserializer)?;
        let expected = (ROOM_WIDTH as usize) * (ROOM_HEIGHT as usize);
        if compact.len() != expected {
            return Err(serde::de::Error::custom(
                "Invalid compact distance array size",
            ));
        }
        let data: Vec<Option<u32>> = compact
            .into_iter()
            .map(|v| {
                if v == NONE_SENTINEL {
                    None
                } else {
                    Some(v as u32)
                }
            })
            .collect();
        Ok(RoomDataArray { data })
    }
}

/// A distance entry: (seed location, distance map, max distance).
pub type DistanceEntry = (Location, RoomDataArray<Option<u32>>, u32);

/// Compact serialization for `Vec<DistanceEntry>`.
///
/// Used for source_distances, controller_distances, mineral_distances in
/// `AnalysisOutput`. Each tuple element is serialized with the distance
/// array using the compact u16 encoding.
pub mod compact_distance_vec_serde {
    use super::*;
    use serde::{Deserialize, Deserializer, Serialize, Serializer};

    #[derive(Serialize, Deserialize)]
    struct CompactEntry {
        loc: Location,
        #[serde(with = "super::compact_distance_serde")]
        distances: RoomDataArray<Option<u32>>,
        max_dist: u32,
    }

    pub fn serialize<S>(data: &[DistanceEntry], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let entries: Vec<CompactEntry> = data
            .iter()
            .map(|(loc, distances, max_dist)| CompactEntry {
                loc: *loc,
                distances: distances.clone(),
                max_dist: *max_dist,
            })
            .collect();
        entries.serialize(serializer)
    }

    pub fn deserialize<'de, D>(deserializer: D) -> Result<Vec<DistanceEntry>, D::Error>
    where
        D: Deserializer<'de>,
    {
        let entries = Vec::<CompactEntry>::deserialize(deserializer)?;
        Ok(entries
            .into_iter()
            .map(|e| (e.loc, e.distances, e.max_dist))
            .collect())
    }
}

/// Neighbor offsets for 8-directional movement.
pub const NEIGHBORS_8: [(i8, i8); 8] = [
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
];

/// Neighbor offsets for 4-directional (cardinal) movement.
pub const NEIGHBORS_4: [(i8, i8); 4] = [(-1, 0), (0, 1), (1, 0), (0, -1)];

/// Compute a distance transform: for each tile, the Chebyshev distance to the nearest wall or room edge.
/// Walls and out-of-build-bounds tiles get distance 0.
pub fn distance_transform(terrain: &FastRoomTerrain) -> RoomDataArray<u8> {
    let mut result = RoomDataArray::new(0u8);

    // Initialize: walls and border tiles get 0, others get 255 (max)
    for y in 0..ROOM_HEIGHT as usize {
        for x in 0..ROOM_WIDTH as usize {
            if terrain.is_wall(x as u8, y as u8)
                || x == 0
                || y == 0
                || x == (ROOM_WIDTH as usize - 1)
                || y == (ROOM_HEIGHT as usize - 1)
            {
                result.set(x, y, 0);
            } else {
                result.set(x, y, 255);
            }
        }
    }

    // Forward pass
    for y in 1..ROOM_HEIGHT as usize {
        for x in 1..ROOM_WIDTH as usize {
            let current = *result.get(x, y);
            let top = result.get(x, y - 1).saturating_add(1);
            let left = result.get(x - 1, y).saturating_add(1);
            let top_left = result.get(x - 1, y - 1).saturating_add(1);
            let top_right = if x + 1 < ROOM_WIDTH as usize {
                result.get(x + 1, y - 1).saturating_add(1)
            } else {
                1
            };
            let min_val = current.min(top).min(left).min(top_left).min(top_right);
            result.set(x, y, min_val);
        }
    }

    // Backward pass
    for y in (0..ROOM_HEIGHT as usize - 1).rev() {
        for x in (0..ROOM_WIDTH as usize - 1).rev() {
            let current = *result.get(x, y);
            let bottom = result.get(x, y + 1).saturating_add(1);
            let right = result.get(x + 1, y).saturating_add(1);
            let bottom_right = result.get(x + 1, y + 1).saturating_add(1);
            let bottom_left = if x > 0 {
                result.get(x - 1, y + 1).saturating_add(1)
            } else {
                1
            };
            let min_val = current
                .min(bottom)
                .min(right)
                .min(bottom_right)
                .min(bottom_left);
            result.set(x, y, min_val);
        }
    }

    result
}

/// BFS flood-fill distance from a set of seed locations, respecting terrain walls.
/// Returns the distance map and the maximum distance reached.
pub fn flood_fill_distance(
    terrain: &FastRoomTerrain,
    seeds: &[Location],
) -> (RoomDataArray<Option<u32>>, u32) {
    let mut data: RoomDataArray<Option<u32>> = RoomDataArray::new(None);
    let mut queue = std::collections::VecDeque::new();

    for seed in seeds {
        data.set(seed.x() as usize, seed.y() as usize, Some(0));
        queue.push_back((*seed, 0u32));
    }

    let mut max_distance = 0u32;

    while let Some((loc, dist)) = queue.pop_front() {
        let next_dist = dist + 1;
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = loc.x() as i16 + dx as i16;
            let ny = loc.y() as i16 + dy as i16;
            if nx >= 0 && nx < ROOM_WIDTH as i16 && ny >= 0 && ny < ROOM_HEIGHT as i16 {
                let ux = nx as usize;
                let uy = ny as usize;
                if data.get(ux, uy).is_none() && !terrain.is_wall(nx as u8, ny as u8) {
                    data.set(ux, uy, Some(next_dist));
                    if next_dist > max_distance {
                        max_distance = next_dist;
                    }
                    queue.push_back((Location::from_coords(ux as u32, uy as u32), next_dist));
                }
            }
        }
    }

    (data, max_distance)
}

/// BFS flood-fill distance that also avoids tiles blocked by structures.
/// `is_passable` is called for each tile to determine if it can be traversed.
pub fn flood_fill_distance_with_obstacles<F>(
    terrain: &FastRoomTerrain,
    seeds: &[Location],
    is_passable: F,
) -> (RoomDataArray<Option<u32>>, u32)
where
    F: Fn(u8, u8) -> bool,
{
    let mut data: RoomDataArray<Option<u32>> = RoomDataArray::new(None);
    let mut queue = std::collections::VecDeque::new();

    for seed in seeds {
        data.set(seed.x() as usize, seed.y() as usize, Some(0));
        queue.push_back((*seed, 0u32));
    }

    let mut max_distance = 0u32;

    while let Some((loc, dist)) = queue.pop_front() {
        let next_dist = dist + 1;
        for &(dx, dy) in &NEIGHBORS_8 {
            let nx = loc.x() as i16 + dx as i16;
            let ny = loc.y() as i16 + dy as i16;
            if nx >= 0 && nx < ROOM_WIDTH as i16 && ny >= 0 && ny < ROOM_HEIGHT as i16 {
                let ux = nx as usize;
                let uy = ny as usize;
                if data.get(ux, uy).is_none()
                    && !terrain.is_wall(nx as u8, ny as u8)
                    && is_passable(nx as u8, ny as u8)
                {
                    data.set(ux, uy, Some(next_dist));
                    if next_dist > max_distance {
                        max_distance = next_dist;
                    }
                    queue.push_back((Location::from_coords(ux as u32, uy as u32), next_dist));
                }
            }
        }
    }

    (data, max_distance)
}
