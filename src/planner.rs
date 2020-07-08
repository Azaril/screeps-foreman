#![allow(dead_code)]
use super::location::*;
use super::visual::*;
use super::*;
use crate::constants::*;
use bitflags::*;
use log::*;
use pathfinding::directed::astar::*;
use rs_graph::linkedlistgraph::*;
use rs_graph::maxflow::*;
use rs_graph::traits::*;
use rs_graph::{Buildable, Builder};
use serde::*;
use std::cell::RefCell;
use std::collections::hash_map::*;
use std::collections::*;
use std::convert::*;
use fnv::*;

pub const ONE_OFFSET_SQUARE: &[(i8, i8)] = &[
    (-1, -1),
    (-1, 0),
    (-1, 1),
    (0, 1),
    (1, 1),
    (1, 0),
    (1, -1),
    (0, -1),
];

pub const ONE_OFFSET_CROSS: &[(i8, i8)] = &[(-1, 0), (0, 1), (1, 0), (0, -1)];

pub const TWO_OFFSET_SQUARE: &[(i8, i8)] = &[
    (-2, -2),
    (-2, -1),
    (-2, 0),
    (-2, 1),
    (-2, 2),
    (-1, 2),
    (0, 2),
    (1, 2),
    (2, 2),
    (2, 1),
    (2, 0),
    (2, -1),
    (2, -2),
    (1, -2),
    (0, -2),
    (-1, -2),
];

pub const ONE_OFFSET_DIAMOND: &[(i8, i8)] = &[(-1, 0), (0, 1), (1, 0), (0, -1)];
pub const TWO_OFFSET_DIAMOND: &[(i8, i8)] = &[
    (0, -2),
    (-1, -1),
    (-2, 0),
    (-1, 1),
    (0, 2),
    (1, 1),
    (2, 0),
    (1, -1),
];
pub const TWO_OFFSET_DIAMOND_POINTS: &[(i8, i8)] = &[(0, -2), (-2, 0), (0, 2), (2, 0)];

pub fn in_room_bounds<T>(x: T, y: T) -> bool
where
    T: Into<i32>,
{
    let x = x.into();
    let y = y.into();

    x >= 0 && x < (ROOM_WIDTH as i32) && y >= 0 && y < (ROOM_HEIGHT as i32)
}

pub fn in_room_bounds_unsigned<T>(x: T, y: T) -> bool
where
    T: Into<u32>,
{
    let x = x.into();
    let y = y.into();

    x < (ROOM_WIDTH as u32) && y < (ROOM_HEIGHT as u32)
}

pub fn in_room_from_edge<T, E>(x: T, y: T, edge: E) -> bool
where
    T: Into<i32>,
    E: Into<u32>,
{
    let x = x.into();
    let y = y.into();
    let edge = edge.into() as i32;

    (x >= edge) && (x < ROOM_WIDTH as i32 - edge) && (y >= edge) && (y < ROOM_HEIGHT as i32 - edge)
}

pub fn in_room_from_edge_unsigned<T, E>(x: T, y: T, edge: E) -> bool
where
    T: Into<u32>,
    E: Into<u32>,
{
    let x = x.into();
    let y = y.into();
    let edge = edge.into();

    (x >= edge) && (x < ROOM_WIDTH as u32 - edge) && (y >= edge) && (y < ROOM_HEIGHT as u32 - edge)
}

pub fn in_room_build_bounds<T>(x: T, y: T) -> bool
where
    T: Into<i32>,
{
    in_room_from_edge(x, y, ROOM_BUILD_BORDER)
}

pub fn in_room_build_bounds_unsigned<T>(x: T, y: T) -> bool
where
    T: Into<u32>,
{
    in_room_from_edge_unsigned(x, y, ROOM_BUILD_BORDER)
}

pub trait InBounds {
    fn in_room_bounds(&self) -> bool;
    fn in_room_from_edge(&self, edge: u32) -> bool;
    fn in_room_build_bounds(&self) -> bool;
}

pub trait InBoundsUnsigned {
    fn in_room_bounds(&self) -> bool;
    fn in_room_from_edge(&self, edge: u32) -> bool;
    fn in_room_build_bounds(&self) -> bool;
}

impl<T> InBounds for (T, T)
where
    T: Into<i32> + Copy,
{
    fn in_room_bounds(&self) -> bool {
        in_room_bounds(self.0, self.1)
    }

    fn in_room_from_edge(&self, edge: u32) -> bool {
        in_room_from_edge(self.0, self.1, edge)
    }

    fn in_room_build_bounds(&self) -> bool {
        in_room_build_bounds(self.0, self.1)
    }
}

impl<T> InBoundsUnsigned for (T, T)
where
    T: Into<u32> + Copy,
{
    fn in_room_bounds(&self) -> bool {
        in_room_bounds_unsigned(self.0, self.1)
    }

    fn in_room_from_edge(&self, edge: u32) -> bool {
        in_room_from_edge_unsigned(self.0, self.1, edge)
    }

    fn in_room_build_bounds(&self) -> bool {
        in_room_build_bounds_unsigned(self.0, self.1)
    }
}

#[derive(Copy, Clone, Serialize, Deserialize, Debug)]
pub struct RoomItem {
    #[serde(rename = "s")]
    structure_type: StructureType,
    #[serde(rename = "r")]
    required_rcl: u8,
}

impl RoomItem {
    pub fn structure_type(&self) -> StructureType {
        self.structure_type
    }

    pub fn required_rcl(&self) -> u8 {
        self.required_rcl
    }
}

impl InBoundsUnsigned for Location {
    fn in_room_bounds(&self) -> bool {
        in_room_bounds_unsigned(self.x(), self.y())
    }

    fn in_room_from_edge(&self, edge: u32) -> bool {
        in_room_from_edge_unsigned(self.x(), self.y(), edge)
    }

    fn in_room_build_bounds(&self) -> bool {
        in_room_build_bounds_unsigned(self.x(), self.y())
    }
}

impl TryFrom<PlanLocation> for Location {
    type Error = ();

    fn try_from(value: PlanLocation) -> Result<Self, Self::Error> {
        if value.in_room_bounds() {
            Ok(Location::from_coords(value.x() as u32, value.y() as u32))
        } else {
            Err(())
        }
    }
}

impl TryFrom<&PlanLocation> for Location {
    type Error = ();

    fn try_from(value: &PlanLocation) -> Result<Self, Self::Error> {
        if value.in_room_bounds() {
            Ok(Location::from_coords(value.x() as u32, value.y() as u32))
        } else {
            Err(())
        }
    }
}

fn get_min_rcl_for_spawn(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1 => Some(1),
        2 => Some(7),
        3 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_extension(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=5 => Some(2),
        6..=10 => Some(3),
        11..=20 => Some(4),
        21..=30 => Some(5),
        31..=40 => Some(6),
        41..=50 => Some(7),
        51..=60 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_link(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=2 => Some(5),
        3..=3 => Some(6),
        4..=4 => Some(7),
        5..=6 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_storage(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(4),
        _ => None,
    }
}

fn get_min_rcl_for_tower(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(3),
        2..=2 => Some(5),
        3..=3 => Some(7),
        4..=6 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_observer(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_power_spawn(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(8),
        _ => None,
    }
}

pub fn get_min_rcl_for_extractor(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(6),
        _ => None,
    }
}

fn get_min_rcl_for_terminal(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(6),
        _ => None,
    }
}

fn get_min_rcl_for_lab(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=3 => Some(6),
        4..=6 => Some(7),
        7..=10 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_container(count: u8) -> Option<u8> {
    match count {
        0..=5 => Some(0),
        _ => None,
    }
}

fn get_min_rcl_for_nuker(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(8),
        _ => None,
    }
}

fn get_min_rcl_for_factory(count: u8) -> Option<u8> {
    match count {
        0 => Some(0),
        1..=1 => Some(7),
        _ => None,
    }
}

pub type PlanState = FnvHashMap<Location, Vec<RoomItem>>;

#[derive(Clone, Serialize, Deserialize)]
pub struct PlannerStateLayer {
    #[serde(rename = "d")]
    data: FnvHashMap<Location, Vec<RoomItem>>,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct PlannerStateCacheLayer {
    #[serde(rename = "s")]
    structure_counts: FnvHashMap<StructureType, u8>,
    #[serde(skip)]
    data_cache: RefCell<FnvHashMap<Location, Option<Vec<RoomItem>>>>,
    #[serde(skip)]
    structure_distances: RefCell<FnvHashMap<StructureType, (RoomDataArray<Option<u32>>, u32)>>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl PlannerStateLayer {
    pub fn new() -> PlannerStateLayer {
        PlannerStateLayer {
            data: FnvHashMap::default(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn insert(&mut self, location: Location, item: RoomItem) {
        let slot = self.data.entry(location).or_insert_with(Vec::new);

        slot.push(item);
    }

    pub fn get(&self, location: &Location) -> Option<&[RoomItem]> {
        self.data.get(location).map(|slot| slot.as_slice())
    }

    pub fn get_locations(&self, structure_type: StructureType) -> impl Iterator<Item = &Location> {
        self.data
            .iter()
            .filter(move |(_, entries)| {
                entries
                    .iter()
                    .any(|entry| entry.structure_type == structure_type)
            })
            .map(|(location, _)| location)
    }

    pub fn get_all_locations(&self) -> impl Iterator<Item = &Location> {
        self.data.iter().map(|(location, _)| location)
    }

    pub fn complete(self) -> FnvHashMap<Location, Vec<RoomItem>> {
        self.data
    }

    pub fn visualize<T>(&self, visualizer: &mut T)
    where
        T: RoomVisualizer,
    {
        let items = self
            .data
            .iter()
            .flat_map(|(location, entries)| entries.iter().map(move |entry| (location, entry)));

        visualize_room_items(items, visualizer);
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl PlannerStateCacheLayer {
    pub fn new(structure_counts: FnvHashMap<StructureType, u8>) -> PlannerStateCacheLayer {
        PlannerStateCacheLayer {
            structure_counts,
            data_cache: RefCell::new(FnvHashMap::default()),
            structure_distances: RefCell::new(FnvHashMap::default()),
        }
    }

    pub fn insert(&mut self, location: Location, item: RoomItem) {
        let current_count = self
            .structure_counts
            .entry(item.structure_type)
            .or_insert(0);

        *current_count += 1;

        self.structure_distances.borrow_mut().clear();

        if let Some(entries) = self
            .data_cache
            .borrow_mut()
            .get_mut(&location)
            .and_then(|v| v.as_mut())
        {
            entries.push(item);
        }
    }

    pub fn get_count(&self, structure_type: StructureType) -> u8 {
        *self.structure_counts.get(&structure_type).unwrap_or(&0)
    }

    pub fn with_structure_distances<G, F, R>(
        &self,
        structure_type: StructureType,
        generator: G,
        callback: F,
    ) -> R
    where
        F: FnOnce((&RoomDataArray<Option<u32>>, u32)) -> R,
        G: FnOnce() -> (RoomDataArray<Option<u32>>, u32),
    {
        let mut structure_data = self.structure_distances.borrow_mut();
        let (data, max_distance) = structure_data
            .entry(structure_type)
            .or_insert_with(generator);

        callback((data, *max_distance))
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct PlannerState {
    #[serde(rename = "l")]
    layers: Vec<PlannerStateLayer>,
    #[serde(rename = "c")]
    cache_layers: Vec<PlannerStateCacheLayer>,
}

impl PlannerState {
    pub fn new() -> PlannerState {
        PlannerState {
            layers: vec![PlannerStateLayer::new()],
            cache_layers: vec![PlannerStateCacheLayer::new(FnvHashMap::default())],
        }
    }

    pub fn push_layer(&mut self) {
        let counts = self
            .cache_layers
            .last()
            .map(|cache_layer| cache_layer.structure_counts.clone())
            .unwrap_or_else(|| FnvHashMap::default());

        self.layers.push(PlannerStateLayer::new());
        self.cache_layers.push(PlannerStateCacheLayer::new(counts));
    }

    fn pop_layer(&mut self) {
        self.layers.pop();
        self.cache_layers.pop();
    }

    pub fn get(&self, location: &Location) -> Option<Vec<RoomItem>> {
        let flush_index = self
            .cache_layers
            .iter()
            .enumerate()
            .rev()
            .find(|(_index, cache_layer)| cache_layer.data_cache.borrow().get(location).is_some())
            .map(|(index, _)| index + 1)
            .unwrap_or(0);

        for index in flush_index..self.cache_layers.len() {
            let cache_layer = &self.cache_layers[index];
            let layer = &self.layers[index];

            let mut items = layer
                .get(location)
                .map(|entry| entry.to_owned())
                .unwrap_or_else(Vec::new);

            if index > 0 {
                self.cache_layers[index - 1]
                    .data_cache
                    .borrow()
                    .get(location)
                    .and_then(|v| v.as_ref())
                    .map(|v| items.extend(v.iter()));
            }

            cache_layer
                .data_cache
                .borrow_mut()
                .insert(*location, Some(items));
        }

        self.cache_layers.last().and_then(|layer| {
            layer
                .data_cache
                .borrow()
                .get(location)
                .and_then(|v| v.to_owned())
        })
    }

    pub fn get_count(&self, structure_type: StructureType) -> u8 {
        self.cache_layers
            .last()
            .map(|l| l.get_count(structure_type))
            .unwrap_or(0)
    }

    pub fn get_locations(&self, structure_type: StructureType) -> Vec<Location> {
        let locations = self
            .layers
            .iter()
            .flat_map(|l| l.get_locations(structure_type))
            .collect::<FnvHashSet<_>>();

        locations
            .into_iter()
            .filter(|location| self.get(location).is_some())
            .map(|location| *location)
            .collect()
    }

    pub fn get_all_locations(&self) -> Vec<Location> {
        let locations = self
            .layers
            .iter()
            .flat_map(|l| l.get_all_locations())
            .collect::<FnvHashSet<_>>();

        locations
            .into_iter()
            .filter(|location| self.get(location).is_some())
            .map(|location| *location)
            .collect()
    }

    pub fn get_all(&self) -> Vec<(Location, RoomItem)> {
        let locations = self
            .layers
            .iter()
            .flat_map(|l| l.get_all_locations())
            .collect::<HashSet<_>>();

        locations
            .into_iter()
            .filter_map(|location| {
                if let Some(entries) = self.get(location) {
                    Some((*location, entries))
                } else {
                    None
                }
            })
            .flat_map(|(location, entries)| entries.into_iter().map(move |entry| (location, entry)))
            .collect()
    }

    pub fn get_pathfinding_distance_to_structure(
        &self,
        position: PlanLocation,
        structure_type: StructureType,
        range: u32,
        terrain: &FastRoomTerrain,
    ) -> Option<(Vec<PlanLocation>, u32)> {
        let is_passable = |location: PlanLocation| {
            if let Ok(location) = Location::try_from(location) {
                if terrain.get(&location).contains(TerrainFlags::WALL) {
                    return false;
                }

                let blocked = self
                    .get(&location)
                    .iter()
                    .flat_map(|v| v.iter())
                    .any(|item| match item.structure_type {
                        StructureType::Road => false,
                        StructureType::Container => false,
                        StructureType::Rampart => false,
                        _ => true,
                    });

                !blocked
            } else {
                false
            }
        };

        let get_neighbours = |location: &PlanLocation| {
            let start_location = *location;

            ONE_OFFSET_SQUARE
                .iter()
                .map(move |offset| start_location + *offset)
                .filter(|location| is_passable(*location))
                .map(|location| (location, 1))
        };

        let goals: Vec<PlanLocation> = self
            .get_locations(structure_type)
            .iter()
            .map(|l| l.into())
            .collect();

        if !goals.is_empty() {
            let distance_to_goals = |p: &PlanLocation| {
                goals
                    .iter()
                    .map(|g| p.distance_to(*g) as u32)
                    .min()
                    .unwrap()
            };

            astar(&position, get_neighbours, distance_to_goals, |p| {
                distance_to_goals(p) <= range
            })
        } else {
            None
        }
    }

    pub fn get_linear_distance_to_structure(
        &self,
        position: PlanLocation,
        structure_type: StructureType,
        range: u32,
    ) -> Option<u32> {
        self.get_locations(structure_type)
            .iter()
            .map(|goal_location| {
                let distance = position.distance_to(goal_location.into()) as u32;

                if distance >= range {
                    distance - range
                } else {
                    0
                }
            })
            .min()
    }

    pub fn with_structure_distances<F, R>(
        &self,
        structure_type: StructureType,
        terrain: &FastRoomTerrain,
        callback: F,
    ) -> R
    where
        F: FnOnce(Option<(&RoomDataArray<Option<u32>>, u32)>) -> R,
    {
        let layer = self
            .layers
            .iter()
            .rev()
            .zip(self.cache_layers.iter().rev())
            .find(|(layer, _)| !layer.is_empty());

        if let Some((_top_non_empty_layer, top_non_empty_cache_layer)) = layer {
            let generator = || {
                let mut data: RoomDataArray<Option<u32>> = RoomDataArray::new(None);
                let mut to_apply: FnvHashSet<PlanLocation> = FnvHashSet::default();

                let structure_locations = self.get_locations(structure_type);

                for location in &structure_locations {
                    to_apply.insert(location.into());
                }

                let is_passable = |location: PlanLocation| {
                    if let Ok(location) = Location::try_from(location) {
                        let blocked = self.get(&location).iter().flat_map(|v| v.iter()).any(
                            |item| match item.structure_type {
                                StructureType::Road => false,
                                StructureType::Container => false,
                                StructureType::Rampart => false,
                                _ => true,
                            },
                        );

                        !blocked
                    } else {
                        false
                    }
                };

                let max_distance = flood_fill_distance(to_apply, terrain, &mut data, is_passable);

                (data, max_distance)
            };

            top_non_empty_cache_layer.with_structure_distances(
                structure_type,
                generator,
                |(data, max_distance)| callback(Some((data, max_distance))),
            )
        } else {
            callback(None)
        }
    }

    pub fn insert(&mut self, location: Location, item: RoomItem) {
        let cache_layer = self.cache_layers.last_mut().unwrap();
        let layer = self.layers.last_mut().unwrap();

        cache_layer.insert(location, item);
        layer.insert(location, item);
    }

    pub fn snapshot(&self) -> PlanState {
        let mut state = PlanState::default();

        for layer in &self.layers {
            for (location, item) in layer.data.iter() {
                state
                    .entry(*location)
                    .and_modify(|entries| entries.extend(item.iter()))
                    .or_insert_with(|| item.to_owned());
            }
        }

        state
    }

    pub fn visualize<V>(&self, visualizer: &mut V)
    where
        V: RoomVisualizer,
    {
        for layer in &self.layers {
            layer.visualize(visualizer);
        }
    }

    pub fn get_rcl_for_next_structure(&self, structure_type: StructureType) -> Option<u8> {
        let current_count = self.get_count(structure_type);

        match structure_type {
            StructureType::Spawn => get_min_rcl_for_spawn(current_count + 1),
            StructureType::Extension => get_min_rcl_for_extension(current_count + 1),
            StructureType::Road => Some(1),
            StructureType::Wall => Some(2),
            StructureType::Rampart => Some(2),
            StructureType::Link => get_min_rcl_for_link(current_count + 1),
            StructureType::Storage => get_min_rcl_for_storage(current_count + 1),
            StructureType::Tower => get_min_rcl_for_tower(current_count + 1),
            StructureType::Observer => get_min_rcl_for_observer(current_count + 1),
            StructureType::PowerSpawn => get_min_rcl_for_power_spawn(current_count + 1),
            StructureType::Extractor => get_min_rcl_for_extractor(current_count + 1),
            StructureType::Lab => get_min_rcl_for_lab(current_count + 1),
            StructureType::Terminal => get_min_rcl_for_terminal(current_count + 1),
            StructureType::Container => get_min_rcl_for_container(current_count + 1),
            StructureType::Nuker => get_min_rcl_for_nuker(current_count + 1),
            StructureType::Factory => get_min_rcl_for_factory(current_count + 1),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Eq, PartialEq, Hash, Debug)]
pub struct PlanLocation {
    x: i8,
    y: i8,
}

impl PlanLocation {
    pub fn new(x: i8, y: i8) -> PlanLocation {
        PlanLocation { x, y }
    }

    pub fn x(&self) -> i8 {
        self.x
    }

    pub fn y(&self) -> i8 {
        self.y
    }

    pub fn as_location(&self) -> Option<Location> {
        if self.in_room_bounds() {
            Some(Location::from_coords(self.x as u32, self.y as u32))
        } else {
            None
        }
    }

    pub fn as_build_location(&self) -> Option<Location> {
        if self.in_room_build_bounds() {
            Some(Location::from_coords(self.x as u32, self.y as u32))
        } else {
            None
        }
    }

    pub fn distance_to(self, other: Self) -> u8 {
        let dx = self.x() - other.x();
        let dy = self.y() - other.y();

        dx.abs().max(dy.abs()) as u8
    }

    pub fn distance_to_xy(self, x: i8, y: i8) -> u8 {
        let dx = self.x() - x;
        let dy = self.y() - y;

        dx.abs().max(dy.abs()) as u8
    }

    #[inline]
    pub fn packed_repr(self) -> u16 {
        let x = (self.x as u8) as u16;
        let y = (self.y as u8) as u16;

        x | (y << 8)
    }

    #[inline]
    pub fn from_packed(packed: u16) -> Self {
        let x = ((packed & 0xFF) as u8) as i8;
        let y = (((packed >> 8) & 0xFF) as u8) as i8;

        PlanLocation { x, y }
    }
}

impl Serialize for PlanLocation {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.packed_repr().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for PlanLocation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        u16::deserialize(deserializer).map(PlanLocation::from_packed)
    }
}

impl<T> From<T> for PlanLocation
where
    T: std::borrow::Borrow<Location>,
{
    fn from(value: T) -> PlanLocation {
        let value = value.borrow();

        PlanLocation {
            x: value.x() as i8,
            y: value.y() as i8,
        }
    }
}

impl InBounds for PlanLocation {
    fn in_room_bounds(&self) -> bool {
        in_room_bounds(self.x(), self.y())
    }

    fn in_room_from_edge(&self, edge: u32) -> bool {
        in_room_from_edge(self.x(), self.y(), edge)
    }

    fn in_room_build_bounds(&self) -> bool {
        in_room_build_bounds(self.x(), self.y())
    }
}

impl std::ops::Add for PlanLocation {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
}

impl std::ops::Add<(i8, i8)> for PlanLocation {
    type Output = Self;

    fn add(self, other: (i8, i8)) -> Self {
        Self {
            x: self.x + other.0,
            y: self.y + other.1,
        }
    }
}

impl std::ops::Add<&(i8, i8)> for PlanLocation {
    type Output = Self;

    fn add(self, other: &(i8, i8)) -> Self {
        Self {
            x: self.x + other.0,
            y: self.y + other.1,
        }
    }
}

impl std::ops::Sub for PlanLocation {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
}

fn visualize_room_items<'a, T: IntoIterator<Item = (&'a Location, &'a RoomItem)>, V>(
    data: T,
    visualizer: &mut V,
) where
    V: RoomVisualizer,
{
    for (loc, entry) in data.into_iter() {
        visualizer.render(*loc, entry.structure_type);
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct Plan {
    #[serde(rename = "s")]
    state: PlanState,
}

#[derive(Copy, Clone, PartialEq, Eq, Hash, Debug, Ord, PartialOrd)]
pub enum BuildPriority {
    VeryLow,
    Low,
    Medium,
    High,
    Critical,
}

pub fn get_build_priority(structure: StructureType) -> BuildPriority {
    match structure {
        StructureType::Spawn => BuildPriority::Critical,
        StructureType::Storage => BuildPriority::High,
        StructureType::Container => BuildPriority::High,
        StructureType::Tower => BuildPriority::High,
        StructureType::Wall => BuildPriority::Low,
        StructureType::Rampart => BuildPriority::Low,
        StructureType::Road => BuildPriority::VeryLow,
        _ => BuildPriority::Medium,
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl Plan {
    #[cfg(not(feature = "shim"))]
    pub fn execute(&self, room: &Room, max_placements: u32) {
        let room_name = room.name();
        let room_level = room.controller().map(|c| c.level()).unwrap_or(0);

        let mut current_placements = 0;

        let mut ordered_entries: Vec<_> = self
            .state
            .iter()
            .flat_map(|(loc, entries)| entries.iter().map(move |item| (loc, item)))
            .collect();

        ordered_entries.sort_by_key(|(_, item)| get_build_priority(item.structure_type()));

        for (loc, entry) in ordered_entries.iter().rev() {
            let required_rcl = entry.required_rcl.into();

            if entry.structure_type == StructureType::Storage && room_level < required_rcl {
                match room.create_construction_site(
                    &RoomPosition::new(loc.x() as u32, loc.y() as u32, room_name),
                    StructureType::Container,
                ) {
                    ReturnCode::Ok => {
                        current_placements += 1;
                    }
                    _ => {}
                }
            } else if room_level >= required_rcl {
                if entry.structure_type == StructureType::Storage {
                    let structures = room.look_for_at(
                        look::STRUCTURES,
                        &RoomPosition::new(loc.x() as u32, loc.y() as u32, room_name),
                    );

                    for structure in &structures {
                        match structure {
                            Structure::Container(container) => {
                                container.destroy();
                            }
                            _ => {}
                        }
                    }
                }

                match room.create_construction_site(
                    &RoomPosition::new(loc.x() as u32, loc.y() as u32, room_name),
                    entry.structure_type,
                ) {
                    ReturnCode::Ok => {
                        current_placements += 1;
                    }
                    _ => {}
                }
            }

            if current_placements >= max_placements {
                return;
            }
        }
    }

    #[cfg(not(feature = "shim"))]
    pub fn cleanup(&self, structures: &[Structure]) {
        let mut invalid_structures = Vec::new();
        let mut valid_structures = Vec::new();

        for structure in structures {
            let structure_pos = structure.pos();
            let structure_type = structure.structure_type();

            let is_valid = self
                .state
                .get(&Location::from_coords(structure_pos.x(), structure_pos.y()))
                .iter()
                .flat_map(|v| *v)
                .any(|r| r.structure_type() == structure_type);

            if is_valid {
                valid_structures.push(structure);
            } else {
                invalid_structures.push(structure);
            }
        }

        let has_valid_spawn = valid_structures
            .iter()
            .any(|s| s.structure_type() == StructureType::Spawn);

        for structure in invalid_structures {
            let can_destroy = match structure.structure_type() {
                StructureType::Spawn => has_valid_spawn,
                _ => true,
            };

            let has_store = structure
                .as_has_store()
                .map(|s| {
                    let resources = s.store_types();

                    resources.iter().any(|r| s.store_of(*r) > 0)
                })
                .unwrap_or(false);

            if can_destroy && !has_store {
                structure.destroy();
            }
        }
    }

    pub fn visualize<V>(&self, visualizer: &mut V)
    where
        V: RoomVisualizer,
    {
        let items = self
            .state
            .iter()
            .flat_map(|(location, entries)| entries.iter().map(move |entry| (location, entry)));

        visualize_room_items(items, visualizer);
    }
}

struct RoomDataArrayIterator<'a, T>
where
    T: Copy,
{
    data: &'a RoomDataArray<T>,
    x: u8,
    y: u8,
}

impl<'a, T> Iterator for RoomDataArrayIterator<'a, T>
where
    T: Copy,
{
    type Item = ((usize, usize), &'a T);

    fn next(&mut self) -> Option<Self::Item> {
        if self.x < ROOM_WIDTH && self.y < ROOM_HEIGHT {
            let current_x = self.x as usize;
            let current_y = self.y as usize;

            self.x += 1;

            if self.x >= ROOM_WIDTH {
                self.x = 0;
                self.y += 1;
            }

            Some(((current_x, current_y), self.data.get(current_x, current_y)))
        } else {
            None
        }
    }
}

#[derive(Clone)]
pub struct RoomDataArray<T>
where
    T: Copy,
{
    data: [T; (ROOM_WIDTH as usize) * (ROOM_HEIGHT as usize)],
}

impl<T> RoomDataArray<T>
where
    T: Copy,
{
    pub fn new(initial: T) -> Self {
        RoomDataArray {
            data: [initial; (ROOM_WIDTH as usize) * (ROOM_HEIGHT as usize)],
        }
    }

    pub fn get(&self, x: usize, y: usize) -> &T {
        let index = (y * (ROOM_WIDTH as usize)) + x;
        &self.data[index]
    }

    pub fn get_mut(&mut self, x: usize, y: usize) -> &mut T {
        let index = (y * (ROOM_WIDTH as usize)) + x;
        &mut self.data[index]
    }

    pub fn set(&mut self, x: usize, y: usize, value: T) {
        *self.get_mut(x, y) = value;
    }

    pub fn iter(&self) -> impl Iterator<Item = ((usize, usize), &T)> {
        RoomDataArrayIterator {
            data: &self,
            x: 0,
            y: 0,
        }
    }
}

#[derive(Clone)]
pub enum PlanNodeChild<'a> {
    GlobalPlacement(&'a dyn PlanGlobalPlacementNode),
    LocationPlacement(PlanLocation, &'a dyn PlanLocationPlacementNode),
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanNodeChild<'a> {
    fn name(&self) -> &str {
        match self {
            PlanNodeChild::GlobalPlacement(n) => n.name(),
            PlanNodeChild::LocationPlacement(_, n) => n.name(),
        }
    }

    fn placement_phase(&self) -> PlacementPhase {
        match self {
            PlanNodeChild::GlobalPlacement(n) => n.placement_phase(),
            PlanNodeChild::LocationPlacement(_, n) => n.placement_phase(),
        }
    }

    fn must_place(&self) -> bool {
        match self {
            PlanNodeChild::GlobalPlacement(n) => n.must_place(),
            PlanNodeChild::LocationPlacement(_, n) => n.must_place(),
        }
    }

    fn place(&self, context: &mut NodeContext, state: &mut PlannerState) -> Result<(), ()> {
        match self {
            PlanNodeChild::GlobalPlacement(node) => node.place(context, state),
            PlanNodeChild::LocationPlacement(location, node) => {
                node.place(*location, context, state)
            }
        }
    }

    fn get_score(&self, context: &mut NodeContext, state: &PlannerState) -> Option<f32> {
        match self {
            PlanNodeChild::GlobalPlacement(node) => node.get_score(context, state),
            PlanNodeChild::LocationPlacement(location, node) => {
                node.get_score(*location, context, state)
            }
        }
    }

    fn mark_visited(&self, gather_data: &mut PlanGatherChildrenData<'a>) {
        match self {
            PlanNodeChild::GlobalPlacement(node) => {
                gather_data.mark_visited_global(node.as_global())
            }
            PlanNodeChild::LocationPlacement(location, node) => {
                gather_data.mark_visited_location(*location, node.as_location())
            }
        }
    }

    fn get_children(
        &self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'a>,
    ) {
        match self {
            PlanNodeChild::GlobalPlacement(node) => node.get_children(context, state, gather_data),
            PlanNodeChild::LocationPlacement(location, node) => {
                node.get_children(*location, context, state, gather_data)
            }
        }
    }

    fn desires_placement(
        &self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'a>,
    ) -> bool {
        match self {
            PlanNodeChild::GlobalPlacement(node) => {
                gather_data.desires_placement(node.as_base(), context, state)
            }
            PlanNodeChild::LocationPlacement(_, node) => {
                gather_data.desires_placement(node.as_base(), context, state)
            }
        }
    }

    fn desires_location(
        &self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'a>,
    ) -> bool {
        match self {
            PlanNodeChild::GlobalPlacement(_) => true,
            PlanNodeChild::LocationPlacement(location, node) => {
                gather_data.desires_location(*location, node.as_location(), context, state)
            }
        }
    }

    fn ready_for_placement(&self, context: &mut NodeContext, state: &PlannerState) -> bool {
        match self {
            PlanNodeChild::GlobalPlacement(node) => node.ready_for_placement(context, state),
            PlanNodeChild::LocationPlacement(_, node) => node.ready_for_placement(context, state),
        }
    }

    fn insert(&self, gather_data: &mut PlanGatherChildrenData<'a>) -> bool {
        match self {
            PlanNodeChild::GlobalPlacement(node) => gather_data.insert_global_placement(*node),
            PlanNodeChild::LocationPlacement(location, node) => {
                gather_data.insert_location_placement(*location, *node)
            }
        }
    }

    fn to_serialized(&self, index_lookup: &FnvHashMap<uuid::Uuid, usize>) -> SerializedPlanNodeChild {
        match self {
            PlanNodeChild::GlobalPlacement(node) => {
                let node_type = 0;
                let node = index_lookup.get(node.id()).unwrap();
                if (node & !0x7F) != 0 {
                    panic!("Not enough bits to represent packed value!");
                }
                let node = node & 0x7F;

                let packed = (node_type) | ((node as u32) << 1);

                SerializedPlanNodeChild { packed }
            }
            PlanNodeChild::LocationPlacement(location, node) => {
                let node_type = 1;
                let location = location.packed_repr();
                let node = index_lookup.get(node.id()).unwrap();
                if (node & !0x7F) != 0 {
                    panic!("Not enough bits to represent packed value!");
                }
                let node = node & 0x7F;

                let packed = (node_type) | ((node as u32) << 1) | ((location as u32) << 16);

                SerializedPlanNodeChild { packed }
            }
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
#[repr(transparent)]
#[serde(transparent)]
struct SerializedPlanNodeChild {
    packed: u32,
}

impl SerializedPlanNodeChild {
    pub fn as_entry<'b>(
        &self,
        nodes: &PlanGatherNodesData<'b>,
        index_lookup: &Vec<uuid::Uuid>,
    ) -> Result<PlanNodeChild<'b>, String> {
        let node_type = self.packed & 0x1;

        match node_type {
            0 => {
                let node_index = (self.packed >> 1) & 0x7F;
                let node_id = index_lookup
                    .get(node_index as usize)
                    .ok_or("Invalid node id")?;
                let node = nodes
                    .global_placement_nodes
                    .get(node_id)
                    .ok_or("Invalid node")?;

                Ok(PlanNodeChild::GlobalPlacement(*node))
            }
            1 => {
                let node_index = (self.packed >> 1) & 0x7F;
                let node_id = index_lookup
                    .get(node_index as usize)
                    .ok_or("Invalid node id")?;
                let node = nodes
                    .location_placement_nodes
                    .get(node_id)
                    .ok_or("Invalid node")?;

                let location = PlanLocation::from_packed((self.packed >> 16) as u16);

                Ok(PlanNodeChild::LocationPlacement(location, *node))
            }
            _ => Err("Unknown node type".to_string()),
        }
    }
}

pub struct PlanGatherNodesData<'b> {
    global_placement_nodes: FnvHashMap<uuid::Uuid, &'b dyn PlanGlobalPlacementNode>,
    location_placement_nodes: FnvHashMap<uuid::Uuid, &'b dyn PlanLocationPlacementNode>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'b> PlanGatherNodesData<'b> {
    pub fn new<'a>() -> PlanGatherNodesData<'a> {
        PlanGatherNodesData {
            global_placement_nodes: FnvHashMap::default(),
            location_placement_nodes: FnvHashMap::default(),
        }
    }

    pub fn get_all_ids(&self) -> Vec<uuid::Uuid> {
        self.global_placement_nodes
            .keys()
            .chain(self.location_placement_nodes.keys())
            .cloned()
            .collect()
    }

    pub fn insert_global_placement(
        &mut self,
        id: uuid::Uuid,
        node: &'b dyn PlanGlobalPlacementNode,
    ) -> bool {
        match self.global_placement_nodes.entry(id) {
            Entry::Occupied(_) => false,
            Entry::Vacant(e) => {
                e.insert(node);

                true
            }
        }
    }

    pub fn insert_location_placement(
        &mut self,
        id: uuid::Uuid,
        node: &'b dyn PlanLocationPlacementNode,
    ) -> bool {
        match self.location_placement_nodes.entry(id) {
            Entry::Occupied(_) => false,
            Entry::Vacant(e) => {
                e.insert(node);

                true
            }
        }
    }
}
struct PlanGatherChildrenGlobalData<'s> {
    visited: Vec<&'s dyn PlanGlobalNode>,
    inserted: Vec<&'s dyn PlanGlobalPlacementNode>,
}

impl<'s> PlanGatherChildrenGlobalData<'s> {
    pub fn has_visited(&self, node: &dyn PlanGlobalNode) -> bool {
        self.visited.iter().any(|other| std::ptr::eq(node, *other))
    }

    pub fn mark_visited(&mut self, node: &'s dyn PlanGlobalNode) {
        if !self.has_visited(node) {
            self.visited.push(node);
        }
    }

    pub fn insert(&mut self, node: &'s dyn PlanGlobalPlacementNode) -> bool {
        if !self.inserted.iter().any(|other| std::ptr::eq(node, *other)) {
            self.inserted.push(node);

            true
        } else {
            false
        }
    }
}

struct PlanGatherChildrenLocationData<'s> {
    desires_location_cache: Vec<(&'s dyn PlanLocationNode, bool)>,
    visited: Vec<&'s dyn PlanLocationNode>,
    inserted: Vec<&'s dyn PlanLocationPlacementNode>,
}

impl<'s> PlanGatherChildrenLocationData<'s> {
    pub fn has_visited(&self, node: &dyn PlanLocationNode) -> bool {
        self.visited.iter().any(|other| std::ptr::eq(node, *other))
    }

    pub fn mark_visited(&mut self, node: &'s dyn PlanLocationNode) {
        if !self.has_visited(node) {
            self.visited.push(node);
        }
    }

    pub fn insert(&mut self, node: &'s dyn PlanLocationPlacementNode) -> bool {
        if !self.inserted.iter().any(|other| std::ptr::eq(node, *other)) {
            self.inserted.push(node);

            true
        } else {
            false
        }
    }
}

pub struct PlanGatherChildrenData<'a> {
    desires_placement_cache: Vec<(&'a dyn PlanBaseNode, bool)>,
    global_nodes: PlanGatherChildrenGlobalData<'a>,
    location_nodes: FnvHashMap<PlanLocation, PlanGatherChildrenLocationData<'a>>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanGatherChildrenData<'a> {
    pub fn new<'b>() -> PlanGatherChildrenData<'b> {
        PlanGatherChildrenData {
            desires_placement_cache: Vec::new(),
            global_nodes: PlanGatherChildrenGlobalData {
                visited: Vec::new(),
                inserted: Vec::new(),
            },
            location_nodes: FnvHashMap::default(),
        }
    }

    pub fn desires_placement(
        &mut self,
        node: &'a dyn PlanBaseNode,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> bool {
        match self
            .desires_placement_cache
            .iter()
            .position(|(other, _)| std::ptr::eq(node, *other))
        {
            Some(index) => self.desires_placement_cache[index].1,
            None => {
                let desires_placement = node.desires_placement(context, state, self);

                self.desires_placement_cache.push((node, desires_placement));

                desires_placement
            }
        }
    }

    pub fn desires_location(
        &mut self,
        position: PlanLocation,
        node: &'a dyn PlanLocationNode,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> bool {
        {
            if let Some(location_data) = self.try_get_location_data(position) {
                if let Some(index) = location_data
                    .desires_location_cache
                    .iter()
                    .position(|(other, _)| std::ptr::eq(node, *other))
                {
                    return location_data.desires_location_cache[index].1;
                }
            }
        }

        let desires_location = node.desires_location(position, context, state, self);

        let location_data = self.get_location_data(position);

        if !location_data
            .desires_location_cache
            .iter()
            .any(|(other, _)| std::ptr::eq(node, *other))
        {
            location_data
                .desires_location_cache
                .push((node, desires_location));
        }

        desires_location
    }

    fn get_location_data(
        &mut self,
        position: PlanLocation,
    ) -> &mut PlanGatherChildrenLocationData<'a> {
        self.location_nodes
            .entry(position)
            .or_insert_with(|| PlanGatherChildrenLocationData {
                desires_location_cache: Vec::new(),
                visited: Vec::new(),
                inserted: Vec::new(),
            })
    }

    fn try_get_location_data(
        &self,
        position: PlanLocation,
    ) -> Option<&PlanGatherChildrenLocationData<'a>> {
        self.location_nodes.get(&position)
    }

    pub fn has_visited_global(&self, node: &'a dyn PlanGlobalNode) -> bool {
        self.global_nodes.has_visited(node)
    }

    pub fn mark_visited_global(&mut self, node: &'a dyn PlanGlobalNode) {
        self.global_nodes.mark_visited(node);
    }

    pub fn has_visited_location(
        &self,
        position: PlanLocation,
        node: &'a dyn PlanLocationNode,
    ) -> bool {
        self.try_get_location_data(position)
            .map(|l| l.has_visited(node))
            .unwrap_or(false)
    }

    pub fn mark_visited_location(
        &mut self,
        position: PlanLocation,
        node: &'a dyn PlanLocationNode,
    ) {
        let location_data = self.get_location_data(position);

        location_data.mark_visited(node);
    }

    pub fn insert_global_placement(&mut self, node: &'a dyn PlanGlobalPlacementNode) -> bool {
        self.global_nodes.insert(node)
    }

    pub fn insert_location_placement(
        &mut self,
        position: PlanLocation,
        node: &'a dyn PlanLocationPlacementNode,
    ) -> bool {
        let location_data = self.get_location_data(position);

        location_data.insert(node)
    }

    pub fn collect(self) -> Vec<PlanNodeChild<'a>> {
        let globals = self
            .global_nodes
            .inserted
            .iter()
            .map(|node| PlanNodeChild::GlobalPlacement(*node));

        self.location_nodes
            .iter()
            .flat_map(|(location, location_data)| {
                location_data
                    .inserted
                    .iter()
                    .map(move |node| PlanNodeChild::LocationPlacement(*location, *node))
            })
            .chain(globals)
            .collect()
    }
}

pub struct NodeContext<'d> {
    data_source: &'d mut dyn PlannerRoomDataSource,

    wall_distance: Option<RoomDataArray<Option<u32>>>,
    source_distances: Option<Vec<(RoomDataArray<Option<u32>>, u32)>>,
}

impl<'d> NodeContext<'d> {
    pub fn new<'a>(data_source: &'a mut dyn PlannerRoomDataSource) -> NodeContext<'a> {
        NodeContext {
            data_source,
            wall_distance: None,
            source_distances: None,
        }
    }

    pub fn terrain(&mut self) -> &FastRoomTerrain {
        self.data_source.get_terrain()
    }

    pub fn controllers(&mut self) -> &[PlanLocation] {
        self.data_source.get_controllers()
    }

    pub fn sources(&mut self) -> &[PlanLocation] {
        self.data_source.get_sources()
    }

    pub fn minerals(&mut self) -> &[PlanLocation] {
        self.data_source.get_minerals()
    }

    pub fn wall_distance(&mut self) -> &RoomDataArray<Option<u32>> {
        if self.wall_distance.is_none() {
            let mut data: RoomDataArray<Option<u32>> = RoomDataArray::new(None);
            let mut to_apply: FnvHashSet<PlanLocation> = FnvHashSet::default();

            let terrain = self.terrain();

            for y in 0..ROOM_HEIGHT {
                for x in 0..ROOM_WIDTH {
                    let terrain_cell = terrain.get_xy(x, y);

                    if terrain_cell.contains(TerrainFlags::WALL) || !in_room_build_bounds(x, y) {
                        to_apply.insert(PlanLocation::new(x as i8, y as i8));
                    }
                }
            }

            flood_fill_distance(to_apply, terrain, &mut data, |_| true);

            self.wall_distance = Some(data);
        }

        self.wall_distance.as_ref().unwrap()
    }

    pub fn source_distances(&mut self) -> &[(RoomDataArray<Option<u32>>, u32)] {
        if self.source_distances.is_none() {
            let mut sources_data = Vec::new();

            let sources = { self.sources().to_vec() };
            let terrain = self.terrain();

            for source in sources.iter() {
                let mut data: RoomDataArray<Option<u32>> = RoomDataArray::new(None);
                let mut to_apply: FnvHashSet<PlanLocation> = FnvHashSet::default();

                to_apply.insert(*source);

                let max_distance = flood_fill_distance(to_apply, terrain, &mut data, |_| true);

                sources_data.push((data, max_distance));
            }

            self.source_distances = Some(sources_data);
        }

        self.source_distances.as_ref().unwrap()
    }
}

pub trait PlanBaseNode {
    fn name(&self) -> &str;

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool;

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>);
}

pub trait PlanGlobalNode: PlanBaseNode {
    fn as_base(&self) -> &dyn PlanBaseNode;

    fn get_children<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    );
}

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
pub enum PlacementPhase {
    Pre,
    Normal,
    Post,
}

pub trait PlanGlobalPlacementNode: PlanGlobalNode {
    fn as_global(&self) -> &dyn PlanGlobalNode;

    fn id(&self) -> &uuid::Uuid;

    fn placement_phase(&self) -> PlacementPhase;

    fn must_place(&self) -> bool;

    fn get_maximum_score(&self, context: &mut NodeContext, state: &PlannerState) -> Option<f32>;

    fn get_score(&self, context: &mut NodeContext, state: &PlannerState) -> Option<f32>;

    fn ready_for_placement(&self, context: &mut NodeContext, state: &PlannerState) -> bool;

    fn place(&self, context: &mut NodeContext, state: &mut PlannerState) -> Result<(), ()>;
}

pub trait PlanGlobalExpansionNode: PlanGlobalNode {
    fn as_global(&self) -> &dyn PlanGlobalNode;
}

pub trait PlanLocationNode: PlanBaseNode {
    fn as_base(&self) -> &dyn PlanBaseNode;

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool;

    fn get_children<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    );
}

pub trait PlanLocationPlacementNode: PlanLocationNode {
    fn as_location(&self) -> &dyn PlanLocationNode;

    fn id(&self) -> &uuid::Uuid;

    fn placement_phase(&self) -> PlacementPhase;

    fn must_place(&self) -> bool;

    fn get_maximum_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32>;

    fn get_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32>;

    fn ready_for_placement(&self, context: &mut NodeContext, state: &PlannerState) -> bool;

    fn place(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &mut PlannerState,
    ) -> Result<(), ()>;
}

pub trait PlanPlacementExpansionNode: PlanLocationNode {
    fn as_location(&self) -> &dyn PlanLocationNode;
}

pub enum PlanNodeStorage<'a> {
    Empty,
    GlobalPlacement(&'a dyn PlanGlobalPlacementNode),
    GlobalExpansion(&'a dyn PlanGlobalExpansionNode),
    LocationPlacement(&'a dyn PlanLocationPlacementNode),
    LocationExpansion(&'a dyn PlanPlacementExpansionNode),
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanNodeStorage<'a> {
    fn gather_nodes(&self, data: &mut PlanGatherNodesData<'a>) {
        match self {
            PlanNodeStorage::Empty => {}
            PlanNodeStorage::GlobalPlacement(n) => n.gather_nodes(data),
            PlanNodeStorage::GlobalExpansion(n) => n.gather_nodes(data),
            PlanNodeStorage::LocationPlacement(n) => n.gather_nodes(data),
            PlanNodeStorage::LocationExpansion(n) => n.gather_nodes(data),
        }
    }

    fn desires_placement(
        &self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'a>,
    ) -> bool {
        match self {
            PlanNodeStorage::Empty => false,
            PlanNodeStorage::GlobalPlacement(n) => {
                gather_data.desires_placement(n.as_base(), context, state)
            }
            PlanNodeStorage::GlobalExpansion(n) => {
                gather_data.desires_placement(n.as_base(), context, state)
            }
            PlanNodeStorage::LocationPlacement(n) => {
                gather_data.desires_placement(n.as_base(), context, state)
            }
            PlanNodeStorage::LocationExpansion(n) => {
                gather_data.desires_placement(n.as_base(), context, state)
            }
        }
    }

    fn desires_location(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'a>,
    ) -> bool {
        match self {
            PlanNodeStorage::Empty => false,
            PlanNodeStorage::GlobalPlacement(_) => true,
            PlanNodeStorage::GlobalExpansion(_) => true,
            PlanNodeStorage::LocationPlacement(n) => {
                gather_data.desires_location(position, n.as_location(), context, state)
            }
            PlanNodeStorage::LocationExpansion(n) => {
                gather_data.desires_location(position, n.as_location(), context, state)
            }
        }
    }

    fn insert_or_expand(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'a>,
    ) {
        match self {
            PlanNodeStorage::Empty => {}
            PlanNodeStorage::GlobalPlacement(n) => {
                gather_data.insert_global_placement(*n);
            }
            PlanNodeStorage::GlobalExpansion(n) => n.get_children(context, state, gather_data),
            PlanNodeStorage::LocationPlacement(n) => {
                gather_data.insert_location_placement(position, *n);
            }
            PlanNodeStorage::LocationExpansion(n) => {
                n.get_children(position, context, state, gather_data)
            }
        }
    }
}

fn flood_fill_distance<F>(
    initial_seeds: FnvHashSet<PlanLocation>,
    terrain: &FastRoomTerrain,
    data: &mut RoomDataArray<Option<u32>>,
    is_passable: F,
) -> u32
where
    F: Fn(PlanLocation) -> bool,
{
    let mut to_apply = initial_seeds;
    let mut current_distance: u32 = 0;

    loop {
        let eval_locations = std::mem::replace(&mut to_apply, FnvHashSet::default());

        for pos in &eval_locations {
            let current = data.get_mut(pos.x() as usize, pos.y() as usize);

            let allow_expand = if current.is_none() {
                if is_passable(*pos) {
                    *current = Some(current_distance);

                    true
                } else {
                    current_distance == 0
                }
            } else {
                false
            };

            if allow_expand {
                for offset in ONE_OFFSET_SQUARE {
                    let next_location = *pos + offset;
                    if next_location.in_room_bounds() {
                        let terrain =
                            terrain.get_xy(next_location.x() as u8, next_location.y() as u8);
                        if !terrain.contains(TerrainFlags::WALL) {
                            to_apply.insert(next_location);
                        }
                    }
                }
            }
        }

        if to_apply.is_empty() {
            break current_distance;
        }

        current_distance += 1;
    }
}

pub struct PlaceAwayFromWallsNode<'a> {
    pub wall_distance: u32,
    pub child: PlanNodeStorage<'a>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for PlaceAwayFromWallsNode<'a> {
    fn name(&self) -> &str {
        "Place Away From Walls"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        self.child.gather_nodes(data);
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.child.desires_placement(context, state, gather_data)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanGlobalNode for PlaceAwayFromWallsNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn get_children<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_global(self) {
            gather_data.mark_visited_global(self);

            if self.child.desires_placement(context, state, gather_data) {
                let locations: Vec<PlanLocation> = context
                    .wall_distance()
                    .iter()
                    .filter(|(_, distance)| {
                        distance.map(|d| d >= self.wall_distance).unwrap_or(false)
                    })
                    .map(|((x, y), _)| PlanLocation::new(x as i8, y as i8))
                    .collect();

                for location in &locations {
                    if self
                        .child
                        .desires_location(*location, context, state, gather_data)
                    {
                        self.child
                            .insert_or_expand(*location, context, state, gather_data);
                    }
                }
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanGlobalExpansionNode for PlaceAwayFromWallsNode<'a> {
    fn as_global(&self) -> &dyn PlanGlobalNode {
        self
    }
}

#[derive(Copy, Clone)]
pub struct PlanPlacement {
    structure_type: StructureType,
    offset: PlanLocation,
    optional: bool,
    rcl_override: Option<u8>,
}

impl PlanPlacement {
    pub const fn optional(self) -> Self {
        Self {
            optional: true,
            ..self
        }
    }

    pub const fn rcl(self, rcl: u8) -> Self {
        Self {
            rcl_override: Some(rcl),
            ..self
        }
    }

    fn can_place(
        &self,
        plan_location: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> bool {
        if let Some(placement_location) = plan_location.as_build_location() {
            if self.structure_type == StructureType::Extractor {
                if !context.minerals().contains(&plan_location) {
                    return false;
                }
            } else if context
                .terrain()
                .get(&placement_location)
                .contains(TerrainFlags::WALL)
            {
                return false;
            } else if !placement_location.in_room_from_edge(ROOM_BUILD_BORDER as u32 + 1) {
                return false;
            }

            for existing in state.get(&placement_location).iter().flat_map(|v| v.iter()) {
                let valid = match existing.structure_type {
                    StructureType::Road => self.structure_type == StructureType::Road,
                    StructureType::Rampart => true,
                    _ => self.structure_type == StructureType::Rampart,
                };

                if !valid {
                    return false;
                }
            }
        } else {
            return false;
        }

        true
    }
}

pub const fn placement(structure_type: StructureType, x: i8, y: i8) -> PlanPlacement {
    PlanPlacement {
        structure_type,
        offset: PlanLocation { x, y },
        optional: false,
        rcl_override: None,
    }
}

pub struct FixedPlanNode<'a> {
    pub id: uuid::Uuid,
    pub placement_phase: PlacementPhase,
    pub must_place: bool,
    pub placements: &'a [PlanPlacement],
    pub child: PlanNodeStorage<'a>,
    pub desires_placement: fn(context: &mut NodeContext, state: &PlannerState) -> bool,
    pub desires_location:
        fn(position: PlanLocation, context: &mut NodeContext, state: &PlannerState) -> bool,
    pub maximum_scorer:
        fn(position: PlanLocation, context: &mut NodeContext, state: &PlannerState) -> Option<f32>,
    pub scorer:
        fn(position: PlanLocation, context: &mut NodeContext, state: &PlannerState) -> Option<f32>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for FixedPlanNode<'a> {
    fn name(&self) -> &str {
        "Fixed"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        if data.insert_location_placement(self.id, self) {
            self.child.gather_nodes(data);
        }
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        _gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        (self.desires_placement)(context, state)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for FixedPlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        _gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        if (self.desires_location)(position, context, state) {
            self.placements.iter().all(|placement| {
                placement.optional
                    || placement.can_place(position + placement.offset, context, state)
            })
        } else {
            false
        }
    }

    fn get_children<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_location(position, self) {
            gather_data.mark_visited_location(position, self);

            if self.child.desires_placement(context, state, gather_data)
                && self
                    .child
                    .desires_location(position, context, state, gather_data)
            {
                self.child
                    .insert_or_expand(position, context, state, gather_data);
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationPlacementNode for FixedPlanNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }

    fn id(&self) -> &uuid::Uuid {
        &self.id
    }

    fn placement_phase(&self) -> PlacementPhase {
        self.placement_phase
    }

    fn must_place(&self) -> bool {
        self.must_place
    }

    fn get_maximum_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32> {
        (self.maximum_scorer)(position, context, state)
    }

    fn get_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32> {
        (self.scorer)(position, context, state)
    }

    fn ready_for_placement(&self, _context: &mut NodeContext, _state: &PlannerState) -> bool {
        true
    }

    fn place(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &mut PlannerState,
    ) -> Result<(), ()> {
        let mut min_rcl = None;

        for placement in self
            .placements
            .iter()
            .filter(|p| p.structure_type != StructureType::Road)
        {
            let placement_location = (position + placement.offset).as_location().unwrap();

            if !placement.optional || placement.can_place(placement_location.into(), context, state)
            {
                let rcl = if let Some(rcl) = placement.rcl_override {
                    rcl
                } else {
                    //TODO: This isn't quite right - should find the lowest unused RCL.
                    state
                        .get_rcl_for_next_structure(placement.structure_type)
                        .ok_or(())?
                };

                min_rcl = min_rcl.map(|r| if rcl < r { rcl } else { r }).or(Some(rcl));

                state.insert(
                    placement_location,
                    RoomItem {
                        structure_type: placement.structure_type,
                        required_rcl: rcl,
                    },
                );
            }
        }

        let road_rcl = min_rcl.unwrap_or(1);

        for placement in self
            .placements
            .iter()
            .filter(|p| p.structure_type == StructureType::Road)
        {
            let placement_location = (position + placement.offset).as_location().unwrap();

            if !placement.optional || placement.can_place(placement_location.into(), context, state)
            {
                state.insert(
                    placement_location,
                    RoomItem {
                        structure_type: placement.structure_type,
                        required_rcl: road_rcl,
                    },
                );
            }
        }

        Ok(())
    }
}

pub struct OffsetPlanNode<'a> {
    pub offsets: &'a [(i8, i8)],
    pub child: PlanNodeStorage<'a>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for OffsetPlanNode<'a> {
    fn name(&self) -> &str {
        "Offset"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        self.child.gather_nodes(data);
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.child.desires_placement(context, state, gather_data)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for OffsetPlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.offsets.iter().any(|offset| {
            let offset_position = position + offset;

            self.child
                .desires_location(offset_position, context, state, gather_data)
        })
    }

    fn get_children<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_location(position, self) {
            gather_data.mark_visited_location(position, self);

            if self.child.desires_placement(context, state, gather_data) {
                for offset in self.offsets.iter() {
                    let offset_position = position + offset;

                    if self
                        .child
                        .desires_location(offset_position, context, state, gather_data)
                    {
                        self.child
                            .insert_or_expand(offset_position, context, state, gather_data);
                    }
                }
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanPlacementExpansionNode for OffsetPlanNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }
}

pub struct MultiPlacementExpansionNode<'a> {
    pub children: &'a [PlanNodeStorage<'a>],
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for MultiPlacementExpansionNode<'a> {
    fn name(&self) -> &str {
        "Multi Placement Expansion"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        for child in self.children.iter() {
            child.gather_nodes(data);
        }
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.children
            .iter()
            .any(|child| child.desires_placement(context, state, gather_data))
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for MultiPlacementExpansionNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.children
            .iter()
            .any(|child| child.desires_location(position, context, state, gather_data))
    }

    fn get_children<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_location(position, self) {
            gather_data.mark_visited_location(position, self);

            for child in self.children.iter() {
                if child.desires_placement(context, state, gather_data)
                    && child.desires_location(position, context, state, gather_data)
                {
                    child.insert_or_expand(position, context, state, gather_data);
                }
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanPlacementExpansionNode for MultiPlacementExpansionNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }
}

pub struct LazyPlanNode<'a> {
    pub child: fn() -> PlanNodeStorage<'a>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for LazyPlanNode<'a> {
    fn name(&self) -> &str {
        "Lazy"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        let node = (self.child)();

        node.gather_nodes(data);
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        let node = (self.child)();

        node.desires_placement(context, state, gather_data)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for LazyPlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        let node = (self.child)();

        node.desires_location(position, context, state, gather_data)
    }

    fn get_children<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_location(position, self) {
            gather_data.mark_visited_location(position, self);

            let node = (self.child)();

            if node.desires_placement(context, state, gather_data)
                && node.desires_location(position, context, state, gather_data)
            {
                node.insert_or_expand(position, context, state, gather_data);
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanPlacementExpansionNode for LazyPlanNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }
}

pub struct FixedLocationPlanNode<'a> {
    pub locations: fn(context: &mut NodeContext) -> Vec<PlanLocation>,
    pub child: PlanNodeStorage<'a>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for FixedLocationPlanNode<'a> {
    fn name(&self) -> &str {
        "Fixed Locations"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        self.child.gather_nodes(data);
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.child.desires_placement(context, state, gather_data)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanGlobalNode for FixedLocationPlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn get_children<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_global(self) {
            gather_data.mark_visited_global(self);

            if self.child.desires_placement(context, state, gather_data) {
                let locations = (self.locations)(context);

                for location in locations {
                    if self
                        .child
                        .desires_location(location, context, state, gather_data)
                    {
                        self.child
                            .insert_or_expand(location, context, state, gather_data);
                    }
                }
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanGlobalExpansionNode for FixedLocationPlanNode<'a> {
    fn as_global(&self) -> &dyn PlanGlobalNode {
        self
    }
}

pub struct MinCutWallsPlanNode {
    pub id: uuid::Uuid,
    pub placement_phase: PlacementPhase,
    pub must_place: bool,
    pub desires_placement: fn(context: &mut NodeContext, state: &PlannerState) -> bool,
    pub ready_for_placement: fn(context: &mut NodeContext, state: &PlannerState) -> bool,
    pub rcl_override: Option<u8>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl PlanBaseNode for MinCutWallsPlanNode {
    fn name(&self) -> &str {
        "Min Cut Walls"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        data.insert_global_placement(self.id, self);
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        _gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        (self.desires_placement)(context, state)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl PlanGlobalNode for MinCutWallsPlanNode {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn get_children<'s>(
        &'s self,
        _context: &mut NodeContext,
        _state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_global(self) {
            gather_data.mark_visited_global(self);
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl PlanGlobalPlacementNode for MinCutWallsPlanNode {
    fn as_global(&self) -> &dyn PlanGlobalNode {
        self
    }

    fn id(&self) -> &uuid::Uuid {
        &self.id
    }

    fn placement_phase(&self) -> PlacementPhase {
        self.placement_phase
    }

    fn must_place(&self) -> bool {
        self.must_place
    }

    fn get_maximum_score(&self, _context: &mut NodeContext, _state: &PlannerState) -> Option<f32> {
        None
    }

    fn get_score(&self, _context: &mut NodeContext, _state: &PlannerState) -> Option<f32> {
        Some(0.0)
    }

    fn ready_for_placement(&self, context: &mut NodeContext, state: &PlannerState) -> bool {
        (self.ready_for_placement)(context, state)
    }

    fn place(&self, context: &mut NodeContext, state: &mut PlannerState) -> Result<(), ()> {
        let mut builder = LinkedListGraph::<u32>::new_builder();

        let top_nodes = builder.add_nodes(50 * 50);
        let bottom_nodes = builder.add_nodes(50 * 50);

        // source (protected) and sink (exit)
        let source = builder.add_node();
        let sink = builder.add_node();

        // unbuildable is for tiles near room exits that can't be ramparted
        let mut unbuildable = FnvHashSet::default();

        // and exits is for the exit tiles themselves, for later attachment to the sink
        let mut exits = FnvHashSet::default();

        for exit_position in context.terrain().get_exits() {
            unbuildable.insert(exit_position);
            exits.insert(exit_position);

            // and mark all tiles within range 1 as unbuildable
            let adjacent_positions = ONE_OFFSET_SQUARE
                .iter()
                .map(|offset| {
                    PlanLocation::new(exit_position.x() as i8, exit_position.y() as i8) + offset
                })
                .filter_map(|offset_location| offset_location.try_into().ok());

            for exit_adjacent_position in adjacent_positions {
                unbuildable.insert(exit_adjacent_position);
            }
        }

        // protected is for tiles that will hook to the source
        let mut protected = FnvHashSet::default();

        let room_items = state.get_all();

        // Protect all tiles we've put structures on so far
        for (location, room_item) in room_items.iter() {
            let should_protect = match room_item.structure_type {
                StructureType::KeeperLair | StructureType::Portal | StructureType::InvaderCore => {
                    false
                }
                StructureType::Wall | StructureType::Rampart => false,
                _ => true,
            };

            if should_protect {
                protected.insert(*location);
            }
        }

        // also explicitly protect range:1 of the controller
        for controller_position in context.controllers() {
            if let Some(controller_location) = controller_position.try_into().ok() {
                protected.insert(controller_location);

                let adjacent_positions = ONE_OFFSET_SQUARE
                    .iter()
                    .map(|offset| *controller_position + offset)
                    .filter(|offset_location| offset_location.in_room_build_bounds())
                    .filter_map(|offset_location| offset_location.try_into().ok());

                for controller_adjacent_position in adjacent_positions {
                    protected.insert(controller_adjacent_position);
                }
            }
        }

        // TODO improve this to support tunnels - top should hook to bottom if it's a wall, (assuming can't rampart a tunnel?)
        // hook to neighboring walls like they're walkable if they're a road
        // big ol' vector of the weights of edges we create
        let mut edge_weights = vec![];

        {
            let terrain = context.terrain();

            // step over all tiles in the room, creating a mesh of flow connections
            // walkable tiles have a weight: 1 edge from their 'top' node to their 'bot' node,
            // which is what limits the 'flow' through the tile and what will ultimately be cut if
            // that tile should be protected.  Then, the bottom tile connects with max weight to
            // walkable neighbors, with high weight to prevent these from being the bottleneck to cut
            for x in 0..ROOM_WIDTH as u32 {
                for y in 0..ROOM_HEIGHT as u32 {
                    // for each tile there's a 'top' and 'bottom'
                    // 'top' is at y * 50 + x
                    // 'bottom' is at 2500 + top
                    // top hooks to bottom with cost 1 if it's a normal tile, max if non-buildable
                    // bottom hooks to surrounding tiles as long as they're not protected tiles
                    // protected tiles top hooks to source
                    // edge tiles' bottom hooks to the sink
                    let current_location = Location::from_coords(x, y);

                    let terrain_mask = terrain.get(&current_location);

                    if terrain_mask.contains(TerrainFlags::WALL) {
                        continue;
                    }

                    if unbuildable.contains(&current_location) {
                        // no cutting here, make a max value edge from top to bottom
                        builder.add_edge(
                            top_nodes[(x + y * 50) as usize],
                            bottom_nodes[(x + y * 50) as usize],
                        );
                        edge_weights.push(std::usize::MAX);
                    } else {
                        // make an edge costing 1 from top to bottom
                        builder.add_edge(
                            top_nodes[(x + y * 50) as usize],
                            bottom_nodes[(x + y * 50) as usize],
                        );
                        edge_weights.push(1);
                    }

                    // if it's an edge tile, connect bot to sink
                    if exits.contains(&current_location) {
                        builder.add_edge(bottom_nodes[(x + y * 50) as usize], sink);
                        edge_weights.push(std::usize::MAX);
                    }

                    // if it's a protected tile, connect source to top
                    if protected.contains(&current_location) {
                        builder.add_edge(source, top_nodes[(x + y * 50) as usize]);
                        edge_weights.push(std::usize::MAX);
                    }

                    let adjacent_locations = ONE_OFFSET_SQUARE
                        .iter()
                        .map(|offset| {
                            PlanLocation::new(
                                current_location.x() as i8,
                                current_location.y() as i8,
                            ) + offset
                        })
                        .filter_map(|offset_location| offset_location.try_into().ok());

                    for adjacent_location in adjacent_locations {
                        let adjacent_terrain_mask = terrain.get(&adjacent_location);

                        if adjacent_terrain_mask.contains(TerrainFlags::WALL) {
                            // good wall
                            continue;
                        }

                        if !protected.contains(&adjacent_location) {
                            // walkable, link from this bottom to that top if it's not protected
                            builder.add_edge(
                                bottom_nodes[(x + y * 50) as usize],
                                top_nodes[(adjacent_location.x() as u32
                                    + adjacent_location.y() as u32 * 50)
                                    as usize],
                            );
                            edge_weights.push(std::usize::MAX);
                        }
                    }
                }
            }
        }

        let network = builder.to_graph();

        // get the big math guns in here
        let (_, _, mincut) = dinic(&network, source, sink, |e| edge_weights[e.index()]);

        // tracking for nodes of each 'type' that have been evaluated as 'part of the cut'
        // (here meaning, on the 'source' side of protected).
        // to find which tiles we want ramparts in, we want to find out which tiles have their
        // top node in the set but their bottom node not in the set, meaning we cut the edge between
        // the top and bottom for that tile.
        let mut top_cut = FnvHashSet::default();
        let mut bot_cut = FnvHashSet::default();

        for node in mincut {
            let node_id = network.node_id(node);

            let room_node_count = ROOM_WIDTH as usize * ROOM_HEIGHT as usize;

            //
            // NOTE: This relies on room nodes to be added first in order to the graph.
            //

            if node_id < room_node_count {
                top_cut.insert(node_id);
            } else if room_node_count < room_node_count * 2 {
                bot_cut.insert(node_id - room_node_count);
            }
        }

        let terrain = context.terrain();

        let mut candidates: FnvHashSet<_> = top_cut.difference(&bot_cut).collect();

        while !candidates.is_empty() {
            let mut to_process: Vec<(Location, StructureType)> = Vec::new();

            let candidate_node = **candidates.iter().next().expect("Expected seed");

            let location =
                Location::from_coords((candidate_node % 50) as u32, (candidate_node / 50) as u32);

            to_process.push((location, StructureType::Rampart));

            while let Some((location, structure_type)) = to_process.pop() {
                let candidate_node = location.x() as usize + (location.y() as usize * 50);

                if candidates.remove(&candidate_node) {
                    let terrain_mask = terrain.get(&location);

                    if !terrain_mask.contains(TerrainFlags::WALL) {
                        if let Some(rcl) = self
                            .rcl_override
                            .or_else(|| state.get_rcl_for_next_structure(structure_type))
                        {
                            state.insert(
                                location,
                                RoomItem {
                                    structure_type: structure_type,
                                    required_rcl: rcl,
                                },
                            );

                            let adjacent_positions = ONE_OFFSET_CROSS
                                .iter()
                                .map(|offset| PlanLocation::from(location) + offset)
                                .filter(|offset_location| offset_location.in_room_build_bounds())
                                .filter_map(|offset_location| offset_location.try_into().ok());

                            for adjacent_position in adjacent_positions {
                                let next_structure = if structure_type == StructureType::Rampart {
                                    if state
                                        .get(&adjacent_position)
                                        .map(|e| e.is_empty())
                                        .unwrap_or(true)
                                    {
                                        StructureType::Wall
                                    } else {
                                        StructureType::Rampart
                                    }
                                } else {
                                    StructureType::Rampart
                                };

                                to_process.push((adjacent_position, next_structure));
                            }
                        }
                    }
                }
            }
        }

        //TODO: Validate min cut actually succeeded...
        Ok(())
    }
}

pub struct FloodFillPlanNodeLevel<'a> {
    pub offsets: &'a [(i8, i8)],
    pub node: &'a dyn PlanLocationPlacementNode,
}

pub struct FloodFillPlanNode<'a> {
    pub id: uuid::Uuid,
    pub placement_phase: PlacementPhase,
    pub must_place: bool,
    pub start_offsets: &'a [(i8, i8)],
    pub expansion_offsets: &'a [(i8, i8)],
    pub maximum_expansion: u32,
    pub minimum_candidates: usize,
    pub levels: &'a [FloodFillPlanNodeLevel<'a>],
    pub desires_placement: fn(context: &mut NodeContext, state: &PlannerState) -> bool,
    pub scorer:
        fn(position: PlanLocation, context: &mut NodeContext, state: &PlannerState) -> Option<f32>,
    pub validator: fn(context: &mut NodeContext, state: &PlannerState) -> Result<(), ()>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for FloodFillPlanNode<'a> {
    fn name(&self) -> &str {
        "Flood Fill"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        if data.insert_location_placement(*self.id(), self) {
            for lod in self.levels.iter() {
                lod.node.gather_nodes(data);
            }
        }
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        (self.desires_placement)(context, state)
            && self
                .levels
                .iter()
                .any(|l| l.node.desires_placement(context, state, gather_data))
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for FloodFillPlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        let mut locations: FnvHashSet<_> = self
            .start_offsets
            .into_iter()
            .map(|o| position + o)
            .collect();

        for lod in self.levels.iter() {
            let mut expanded_locations: FnvHashSet<PlanLocation> = locations
                .iter()
                .flat_map(|&location| lod.offsets.iter().map(move |offset| location + *offset))
                .collect();

            if expanded_locations.iter().any(|location| {
                lod.node
                    .desires_location(*location, context, state, gather_data)
            }) {
                return true;
            }

            locations = std::mem::replace(&mut expanded_locations, FnvHashSet::default());
        }

        false
    }

    fn get_children<'s>(
        &'s self,
        _position: PlanLocation,
        _context: &mut NodeContext,
        _state: &PlannerState,
        _gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationPlacementNode for FloodFillPlanNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }

    fn placement_phase(&self) -> PlacementPhase {
        self.placement_phase
    }

    fn must_place(&self) -> bool {
        self.must_place
    }

    fn id(&self) -> &uuid::Uuid {
        &self.id
    }

    fn get_maximum_score(
        &self,
        _position: PlanLocation,
        _context: &mut NodeContext,
        _state: &PlannerState,
    ) -> Option<f32> {
        None
    }

    fn get_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32> {
        (self.scorer)(position, context, state)
    }

    fn ready_for_placement(&self, _context: &mut NodeContext, _state: &PlannerState) -> bool {
        //TODO: Provide customization option?
        true
    }

    fn place(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &mut PlannerState,
    ) -> Result<(), ()> {
        let mut locations: FnvHashSet<_> = self
            .start_offsets
            .into_iter()
            .map(|o| position + o)
            .collect();
        let mut next_locations: FnvHashSet<_> = FnvHashSet::default();
        let mut visited_locations: FnvHashSet<_> = FnvHashSet::default();

        let mut current_expansion = 0;

        let mut candidates = Vec::new();

        while current_expansion < self.maximum_expansion && !locations.is_empty() {
            let mut current_gather_data = PlanGatherChildrenData::<'a>::new();

            while current_expansion < self.maximum_expansion
                && !locations.is_empty()
                && candidates.len() < self.minimum_candidates
            {
                for root_location in locations.iter() {
                    if !visited_locations.contains(root_location) {
                        visited_locations.insert(*root_location);

                        let mut lod_locations = vec![*root_location];

                        for lod in self.levels.iter() {
                            let expanded_locations = lod_locations.iter().flat_map(|&location| {
                                lod.offsets.iter().map(move |offset| location + *offset)
                            });

                            let mut next_lod_locations = Vec::new();

                            for lod_location in expanded_locations {
                                if !current_gather_data
                                    .has_visited_location(lod_location, lod.node.as_location())
                                {
                                    current_gather_data.mark_visited_location(
                                        lod_location,
                                        lod.node.as_location(),
                                    );

                                    let got_candidate = if current_gather_data.desires_placement(
                                        lod.node.as_base(),
                                        context,
                                        state,
                                    ) && current_gather_data
                                        .desires_location(
                                            lod_location,
                                            lod.node.as_location(),
                                            context,
                                            state,
                                        ) {
                                        let max_score = lod.node.get_maximum_score(
                                            lod_location,
                                            context,
                                            state,
                                        );

                                        candidates.push((lod_location, lod.node, max_score));

                                        true
                                    } else {
                                        false
                                    };

                                    if got_candidate {
                                        for offset in self.expansion_offsets.into_iter() {
                                            let next_location = *root_location + *offset;

                                            next_locations.insert(next_location);
                                        }
                                    } else {
                                        next_lod_locations.push(lod_location);
                                    }
                                }
                            }

                            if next_lod_locations.is_empty() {
                                break;
                            }

                            lod_locations = next_lod_locations;
                        }
                    }
                }

                current_expansion += 1;

                locations = std::mem::replace(&mut next_locations, FnvHashSet::default());
            }

            while (candidates.len() >= self.minimum_candidates
                || locations.is_empty()
                || current_expansion >= self.maximum_expansion)
                && !candidates.is_empty()
            {
                candidates.sort_by(|(_, _, max_score_a), (_, _, max_score_b)| {
                    max_score_a.partial_cmp(&max_score_b).unwrap()
                });

                let mut current_gather_data = PlanGatherChildrenData::<'a>::new();

                let mut best_candidate = None;

                let mut to_remove = Vec::new();

                for (index, (location, node, max_score)) in candidates.iter_mut().enumerate().rev()
                {
                    let can_exceed_best_score = best_candidate
                        .as_ref()
                        .map(|(best_score, _)| best_score)
                        .and_then(|best_score| max_score.map(|max| max > *best_score))
                        .unwrap_or(true);

                    if can_exceed_best_score {
                        let can_place =
                            current_gather_data.desires_placement(node.as_base(), context, state)
                                && current_gather_data.desires_location(
                                    *location,
                                    node.as_location(),
                                    context,
                                    state,
                                );

                        if can_place {
                            if let Some(score) = node.get_score(*location, context, state) {
                                //TODO: Only allow modifying score if hint is set that score can only get worse?
                                *max_score = Some(score);

                                if best_candidate
                                    .as_ref()
                                    .map(|(best_score, _)| score > *best_score)
                                    .unwrap_or(true)
                                {
                                    best_candidate = Some((score, (*location, node, index)));
                                }
                            } else {
                                to_remove.push(index);
                            }
                        } else {
                            //TODO: Should consider pushing to next LOD?

                            to_remove.push(index);
                        }
                    }
                }

                if let Some((_, (location, node, index))) = best_candidate {
                    node.place(location, context, state)?;

                    match to_remove.binary_search_by(|probe| probe.cmp(&index).reverse()) {
                        Ok(_) => {}
                        Err(pos) => to_remove.insert(pos, index),
                    }
                }

                for index in to_remove.into_iter() {
                    candidates.remove(index);
                }
            }
        }

        (self.validator)(context, state)
    }
}

pub struct FirstPossiblePlanNode<'a> {
    pub id: uuid::Uuid,
    pub placement_phase: PlacementPhase,
    pub must_place: bool,
    pub options: &'a [&'a dyn PlanLocationPlacementNode],
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for FirstPossiblePlanNode<'a> {
    fn name(&self) -> &str {
        "First Possible"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        if data.insert_location_placement(*self.id(), self) {
            for option in self.options.iter() {
                option.gather_nodes(data);
            }
        }
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.options
            .iter()
            .any(|option| option.desires_placement(context, state, gather_data))
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for FirstPossiblePlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        self.options
            .iter()
            .any(|option| option.desires_location(position, context, state, gather_data))
    }

    fn get_children<'s>(
        &'s self,
        _position: PlanLocation,
        _context: &mut NodeContext,
        _state: &PlannerState,
        _gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationPlacementNode for FirstPossiblePlanNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }

    fn placement_phase(&self) -> PlacementPhase {
        self.placement_phase
    }

    fn must_place(&self) -> bool {
        self.must_place
    }

    fn id(&self) -> &uuid::Uuid {
        &self.id
    }

    fn get_maximum_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32> {
        self.options
            .iter()
            .filter_map(|option| option.get_maximum_score(position, context, state))
            .max_by(|a, b| a.partial_cmp(b).unwrap())
    }

    fn get_score(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
    ) -> Option<f32> {
        let mut current_gather_data = PlanGatherChildrenData::<'a>::new();

        self.options
            .iter()
            .filter_map(|option| {
                if current_gather_data.desires_placement(option.as_base(), context, state)
                    && current_gather_data.desires_location(
                        position,
                        option.as_location(),
                        context,
                        state,
                    )
                {
                    option.get_score(position, context, state)
                } else {
                    None
                }
            })
            .next()
    }

    fn ready_for_placement(&self, _context: &mut NodeContext, _state: &PlannerState) -> bool {
        //TODO: Provide customization option?
        true
    }

    fn place(
        &self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &mut PlannerState,
    ) -> Result<(), ()> {
        let mut current_gather_data = PlanGatherChildrenData::<'a>::new();

        for option in self.options.iter() {
            if current_gather_data.desires_placement(option.as_base(), context, state)
                && current_gather_data.desires_location(
                    position,
                    option.as_location(),
                    context,
                    state,
                )
                && current_gather_data.insert_location_placement(position, *option)
            {
                //TODO: Should this allow recovery?
                option.place(position, context, state)?;

                break;
            }
        }

        Ok(())
    }
}

pub struct NearestToStructureExpansionPlanNode<'a> {
    pub structure_type: StructureType,
    pub child: PlanNodeStorage<'a>,
    pub path_distance: u32,
    pub desires_placement: fn(context: &mut NodeContext, state: &PlannerState) -> bool,
    pub desires_location:
        fn(position: PlanLocation, context: &mut NodeContext, state: &PlannerState) -> bool,
    pub scorer:
        fn(position: PlanLocation, context: &mut NodeContext, state: &PlannerState) -> Option<f32>,
}

impl<'a> NearestToStructureExpansionPlanNode<'a> {
    fn get_child_locations<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> Vec<PlanLocation> {
        let mut result = Vec::new();

        if self.child.desires_placement(context, state, gather_data) {
            if let Some((path, _distance)) = state.get_pathfinding_distance_to_structure(
                position,
                self.structure_type,
                1,
                context.terrain(),
            ) {
                for offset_location in path.iter() {
                    let distance = offset_location.distance_to(position) as u32;

                    if distance == self.path_distance {
                        result.push(*offset_location);
                    } else if distance > self.path_distance {
                        break;
                    }
                }
            }
        }

        result
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanBaseNode for NearestToStructureExpansionPlanNode<'a> {
    fn name(&self) -> &str {
        "Nearest To Structure"
    }

    fn gather_nodes<'b>(&'b self, data: &mut PlanGatherNodesData<'b>) {
        self.child.gather_nodes(data);
    }

    fn desires_placement<'s>(
        &'s self,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        (self.desires_placement)(context, state)
            && self.child.desires_placement(context, state, gather_data)
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanLocationNode for NearestToStructureExpansionPlanNode<'a> {
    fn as_base(&self) -> &dyn PlanBaseNode {
        self
    }

    fn desires_location<'s>(
        &'s self,
        _position: PlanLocation,
        _context: &mut NodeContext,
        _state: &PlannerState,
        _gather_data: &mut PlanGatherChildrenData<'s>,
    ) -> bool {
        true

        /*
        self.allowed_offsets.iter().any(|offset| {
            self.child
                .desires_location(position + *offset, context, state, gather_data)
        })
        */
    }

    fn get_children<'s>(
        &'s self,
        position: PlanLocation,
        context: &mut NodeContext,
        state: &PlannerState,
        gather_data: &mut PlanGatherChildrenData<'s>,
    ) {
        if !gather_data.has_visited_location(position, self) {
            gather_data.mark_visited_location(position, self);

            if self.child.desires_placement(context, state, gather_data) {
                if let Some((path, _distance)) = state.get_pathfinding_distance_to_structure(
                    position,
                    self.structure_type,
                    1,
                    context.terrain(),
                ) {
                    for offset_location in path.iter() {
                        let distance = offset_location.distance_to(position) as u32;

                        if distance == self.path_distance {
                            if self.child.desires_location(
                                *offset_location,
                                context,
                                state,
                                gather_data,
                            ) {
                                self.child.insert_or_expand(
                                    *offset_location,
                                    context,
                                    state,
                                    gather_data,
                                );

                                break;
                            }
                        } else if distance > self.path_distance {
                            break;
                        }
                    }
                }
            }
        }
    }
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'a> PlanPlacementExpansionNode for NearestToStructureExpansionPlanNode<'a> {
    fn as_location(&self) -> &dyn PlanLocationNode {
        self
    }
}

pub struct FastRoomTerrain {
    buffer: Vec<u8>,
}

bitflags! {
    pub struct TerrainFlags: u8 {
        const NONE = 0;
        const WALL = TERRAIN_MASK_WALL;
        const SWAMP = TERRAIN_MASK_SWAMP;
        const LAVA = TERRAIN_MASK_LAVA;
    }
}

enum ExitSide {
    Top,
    Right,
    Bottom,
    Left,
}

pub struct ExitIterator<'a> {
    terrain: &'a FastRoomTerrain,
    side: Option<ExitSide>,
    index: u32,
}

impl<'a> Iterator for ExitIterator<'a> {
    type Item = Location;

    fn next(&mut self) -> Option<Location> {
        loop {
            let current = match self.side {
                Some(ExitSide::Top) => {
                    let res = Location::from_coords(self.index, 0);

                    self.index += 1;

                    if self.index >= ROOM_WIDTH as u32 - 1 {
                        self.index = 0;
                        self.side = Some(ExitSide::Right)
                    }

                    res
                }
                Some(ExitSide::Right) => {
                    let res = Location::from_coords(ROOM_WIDTH as u32 - 1, self.index);

                    self.index += 1;

                    if self.index >= ROOM_HEIGHT as u32 - 1 {
                        self.index = 0;
                        self.side = Some(ExitSide::Bottom)
                    }

                    res
                }
                Some(ExitSide::Bottom) => {
                    let res = Location::from_coords(
                        (ROOM_WIDTH as u32 - 1) - self.index,
                        ROOM_HEIGHT as u32 - 1,
                    );

                    self.index += 1;

                    if self.index >= ROOM_WIDTH as u32 - 1 {
                        self.index = 0;
                        self.side = Some(ExitSide::Left)
                    }

                    res
                }
                Some(ExitSide::Left) => {
                    let res = Location::from_coords(0, (ROOM_HEIGHT as u32 - 1) - self.index);

                    self.index += 1;

                    if self.index >= ROOM_HEIGHT as u32 - 1 {
                        self.index = 0;
                        self.side = None;
                    }

                    res
                }
                None => {
                    return None;
                }
            };

            let terrain_mask = self.terrain.get_xy(current.x(), current.y());

            if !terrain_mask.intersects(TerrainFlags::WALL) {
                return Some(current);
            }
        }
    }
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

    pub fn get_exits(&self) -> ExitIterator {
        ExitIterator {
            terrain: self,
            side: Some(ExitSide::Top),
            index: 0,
        }
    }
}

struct EvaluationStackEntry<'b> {
    children: Vec<PlanNodeChild<'b>>,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'b> EvaluationStackEntry<'b> {
    pub fn to_serialized(
        &self,
        index_lookup: &FnvHashMap<uuid::Uuid, usize>,
    ) -> SerializedEvaluationStackEntry {
        SerializedEvaluationStackEntry {
            children: self
                .children
                .iter()
                .map(|c| c.to_serialized(index_lookup))
                .collect(),
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
struct SerializedEvaluationStackEntry {
    #[serde(rename = "c")]
    children: Vec<SerializedPlanNodeChild>,
}

impl SerializedEvaluationStackEntry {
    pub fn as_entry<'b>(
        &self,
        nodes: &PlanGatherNodesData<'b>,
        index_lookup: &Vec<uuid::Uuid>,
    ) -> Result<EvaluationStackEntry<'b>, String> {
        let mut children = Vec::new();

        for serialized_child in &self.children {
            let child = serialized_child.as_entry(nodes, index_lookup)?;

            children.push(child);
        }

        Ok(EvaluationStackEntry { children })
    }
}

#[derive(Clone, Serialize, Deserialize)]
struct SerializedEvaluationStack {
    identifiers: Vec<uuid::Uuid>,
    entries: Vec<SerializedEvaluationStackEntry>,
}

impl SerializedEvaluationStack {
    pub fn from_stack(
        gathered_nodes: &PlanGatherNodesData,
        entries: &Vec<EvaluationStackEntry>,
    ) -> SerializedEvaluationStack {
        let identifiers: Vec<_> = gathered_nodes.get_all_ids();
        let index_lookup: FnvHashMap<_, _> = identifiers
            .iter()
            .enumerate()
            .map(|(index, id)| (*id, index))
            .collect();

        let serialized_entries = entries
            .iter()
            .map(|e| e.to_serialized(&index_lookup))
            .collect();

        SerializedEvaluationStack {
            identifiers,
            entries: serialized_entries,
        }
    }

    pub fn to_stack<'b>(
        &self,
        gathered_nodes: &PlanGatherNodesData<'b>,
    ) -> Result<Vec<EvaluationStackEntry<'b>>, String> {
        let mut stack = Vec::new();

        for serialized_entry in self.entries.iter() {
            let entry = serialized_entry.as_entry(&gathered_nodes, &self.identifiers)?;

            stack.push(entry);
        }

        Ok(stack)
    }
}

enum TreePlannerResult {
    Complete,
    Running(SerializedEvaluationStack),
}

struct TreePlanner<'t, H>
where
    H: FnMut(&PlannerState, &mut NodeContext),
{
    data_source: &'t mut dyn PlannerRoomDataSource,
    handler: H,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<'t, H> TreePlanner<'t, H>
where
    H: FnMut(&PlannerState, &mut NodeContext),
{
    pub fn new<'a>(
        data_source: &'a mut dyn PlannerRoomDataSource,
        handler: H,
    ) -> TreePlanner<'a, H> {
        TreePlanner {
            data_source,
            handler,
        }
    }

    pub fn seed<'r, 's>(
        &mut self,
        root_nodes: &[&'r dyn PlanGlobalExpansionNode],
        state: &'s mut PlannerState,
    ) -> Result<TreePlannerResult, String> {
        let mut context = NodeContext::new(self.data_source);

        let mut stack = Vec::new();

        let mut gathered_children = PlanGatherChildrenData::<'s>::new();

        for node in root_nodes.iter() {
            if gathered_children.desires_placement(node.as_base(), &mut context, state) {
                node.get_children(&mut context, state, &mut gathered_children);
            }
        }

        let children = gathered_children.collect();

        let mut ordered_children: Vec<_> = children
            .into_iter()
            .filter_map(|node| {
                node.get_score(&mut context, state)
                    .map(|score| (node, score))
            })
            .collect();

        ordered_children.sort_by(|(node_a, score_a), (node_b, score_b)| {
            node_a
                .placement_phase()
                .cmp(&node_b.placement_phase())
                .reverse()
                .then_with(|| node_a.must_place().cmp(&node_b.must_place()))
                .then_with(|| score_a.partial_cmp(score_b).unwrap())
        });

        stack.push(EvaluationStackEntry {
            children: ordered_children.into_iter().map(|(node, _)| node).collect(),
        });

        let mut gathered_nodes = PlanGatherNodesData::new::<'r>();

        for node in root_nodes {
            node.gather_nodes(&mut gathered_nodes);
        }

        let serialized = SerializedEvaluationStack::from_stack(&gathered_nodes, &stack);

        Ok(TreePlannerResult::Running(serialized))
    }

    pub fn process<'r, 's, F>(
        &mut self,
        root_nodes: &[&'r dyn PlanGlobalExpansionNode],
        state: &'s mut PlannerState,
        serialized_stack: &SerializedEvaluationStack,
        should_continue: F,
    ) -> Result<TreePlannerResult, String>
    where
        F: Fn() -> bool,
    {
        let mut context = NodeContext::new(self.data_source);

        let mut processed_entries = 0;

        let mut gathered_nodes = PlanGatherNodesData::new::<'r>();

        for node in root_nodes {
            node.gather_nodes(&mut gathered_nodes);
        }

        let mut stack = serialized_stack.to_stack(&gathered_nodes)?;

        while !stack.is_empty() && should_continue() {
            let mut placed_nodes = Vec::new();

            let (entry_failed, finished_entry) = {
                let entry = stack.last_mut().unwrap();
                let mut entry_failed = false;

                while !entry.children.is_empty()
                    && placed_nodes.is_empty()
                    && !entry_failed
                    && should_continue()
                {
                    let mut to_place = Vec::new();

                    let mut current_phase = None;

                    let placeable_children = entry
                        .children
                        .iter()
                        .enumerate()
                        .rev()
                        .filter(|(_, c)| c.ready_for_placement(&mut context, state));

                    for (index, child) in placeable_children {
                        let matches_phase = current_phase
                            .map(|phase| phase == child.placement_phase())
                            .unwrap_or(true);

                        if child.must_place() && matches_phase {
                            to_place.push(index);

                            current_phase = Some(child.placement_phase());
                        } else if to_place.is_empty() {
                            to_place.push(index);

                            break;
                        } else {
                            break;
                        }
                    }

                    if !to_place.is_empty() {
                        processed_entries += to_place.len();

                        let to_place_nodes =
                            to_place.iter().map(|index| entry.children.remove(*index));

                        state.push_layer();

                        let mut validate_location = false;

                        for child in to_place_nodes {
                            if validate_location
                                && !child.desires_location(
                                    &mut context,
                                    state,
                                    &mut PlanGatherChildrenData::new(),
                                )
                            {
                                entry_failed = true;

                                break;
                            }

                            match child.place(&mut context, state) {
                                Ok(()) => {}
                                Err(()) => {
                                    entry_failed = true;

                                    break;
                                }
                            }

                            placed_nodes.push(child);

                            validate_location = true;
                        }
                    } else {
                        entry_failed = true;
                    }

                    if !entry_failed {
                        (self.handler)(state, &mut context);
                    } else {
                        state.pop_layer();
                    }
                }

                (entry_failed, entry.children.is_empty())
            };

            if entry_failed {
                state.pop_layer();

                stack.pop();
            } else if !placed_nodes.is_empty() {
                let mut gathered_children = PlanGatherChildrenData::<'s>::new();

                for child in placed_nodes.iter() {
                    child.get_children(&mut context, state, &mut gathered_children);
                }

                for existing_child in stack.last().unwrap().children.iter() {
                    if existing_child.desires_placement(&mut context, state, &mut gathered_children)
                        && existing_child.desires_location(
                            &mut context,
                            state,
                            &mut gathered_children,
                        )
                    {
                        existing_child.insert(&mut gathered_children);
                    }
                }

                let children = gathered_children.collect();

                let mut ordered_children: Vec<_> = children
                    .into_iter()
                    .filter_map(|node| {
                        node.get_score(&mut context, state)
                            .map(|score| (node, score))
                    })
                    .collect();

                ordered_children.sort_by(|(node_a, score_a), (node_b, score_b)| {
                    node_a
                        .placement_phase()
                        .cmp(&node_b.placement_phase())
                        .reverse()
                        .then_with(|| node_a.must_place().cmp(&node_b.must_place()))
                        .then_with(|| score_a.partial_cmp(score_b).unwrap())
                });

                stack.push(EvaluationStackEntry {
                    children: ordered_children.into_iter().map(|(node, _)| node).collect(),
                });
            } else if finished_entry {
                state.pop_layer();

                stack.pop();
            }
        }

        info!(
            "Processed planning entries: {} - Known children: {}",
            processed_entries,
            stack.iter().map(|e| e.children.len()).sum::<usize>()
        );

        if stack.is_empty() {
            Ok(TreePlannerResult::Complete)
        } else {
            let serialized = SerializedEvaluationStack::from_stack(&gathered_nodes, &stack);

            Ok(TreePlannerResult::Running(serialized))
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct BestPlanData {
    score: f32,
    state: PlanState,
}

#[derive(Clone, Serialize, Deserialize)]
pub struct PlanRunningStateData {
    planner_state: PlannerState,
    stack: SerializedEvaluationStack,
    best_plan: Option<BestPlanData>,
}

impl PlanRunningStateData {
    pub fn visualize<V>(&self, visualizer: &mut V)
    where
        V: RoomVisualizer,
    {
        self.planner_state.visualize(visualizer);
    }

    pub fn visualize_best<V>(&self, visualizer: &mut V)
    where
        V: RoomVisualizer,
    {
        if let Some(best_plan) = &self.best_plan {
            let items = best_plan
                .state
                .iter()
                .flat_map(|(location, entries)| entries.iter().map(move |entry| (location, entry)));

            visualize_room_items(items, visualizer);
        }
    }
}

pub enum PlanSeedResult {
    Complete(Option<Plan>),
    Running(PlanRunningStateData),
}

pub enum PlanEvaluationResult {
    Complete(Option<Plan>),
    Running(),
}

pub trait PlannerRoomDataSource {
    fn get_terrain(&mut self) -> &FastRoomTerrain;
    fn get_controllers(&mut self) -> &[PlanLocation];
    fn get_sources(&mut self) -> &[PlanLocation];
    fn get_minerals(&mut self) -> &[PlanLocation];
}

pub struct Planner<S>
where
    S: Fn(&PlannerState, &mut NodeContext) -> Option<f32>,
{
    scorer: S,
}

#[cfg_attr(feature = "profile", screeps_timing_annotate::timing)]
impl<S> Planner<S>
where
    S: Fn(&PlannerState, &mut NodeContext) -> Option<f32>,
{
    pub fn new(scorer: S) -> Planner<S> {
        Planner { scorer }
    }

    pub fn seed(
        &self,
        root_nodes: &[&dyn PlanGlobalExpansionNode],
        data_source: &mut dyn PlannerRoomDataSource,
    ) -> Result<PlanSeedResult, String> {
        let mut planner_state = PlannerState::new();

        let mut best_plan = None;

        let mut state_handler = |new_state: &PlannerState, context: &mut NodeContext| {
            if let Some(score) = (self.scorer)(new_state, context) {
                best_plan = Some(BestPlanData {
                    score,
                    state: new_state.snapshot(),
                });
            }
        };

        let mut planner = TreePlanner::new(data_source, &mut state_handler);

        let seed_result = match planner.seed(root_nodes, &mut planner_state)? {
            TreePlannerResult::Complete => {
                let plan = best_plan.take().map(|p| Plan { state: p.state });

                PlanSeedResult::Complete(plan)
            }
            TreePlannerResult::Running(stack) => {
                let running_data = PlanRunningStateData {
                    planner_state,
                    stack,
                    best_plan,
                };

                PlanSeedResult::Running(running_data)
            }
        };

        Ok(seed_result)
    }

    pub fn evaluate<F>(
        &self,
        root_nodes: &[&dyn PlanGlobalExpansionNode],
        data_source: &mut dyn PlannerRoomDataSource,
        evaluation_state: &mut PlanRunningStateData,
        should_continue: F,
    ) -> Result<PlanEvaluationResult, String>
    where
        F: Fn() -> bool,
    {
        let mut current_best = evaluation_state.best_plan.as_ref().map(|p| p.score);
        let mut new_best_plan = None;

        let mut state_handler = |new_state: &PlannerState, context: &mut NodeContext| {
            if let Some(score) = (self.scorer)(new_state, context) {
                if current_best.map(|s| score > s).unwrap_or(true) {
                    new_best_plan = Some(BestPlanData {
                        score,
                        state: new_state.snapshot(),
                    });

                    current_best = Some(score);
                }
            }
        };

        let mut planner = TreePlanner::new(data_source, &mut state_handler);

        let evaluate_result = match planner.process(
            root_nodes,
            &mut evaluation_state.planner_state,
            &evaluation_state.stack,
            should_continue,
        )? {
            TreePlannerResult::Complete => {
                if new_best_plan.is_some() {
                    evaluation_state.best_plan = new_best_plan;
                }

                let plan = evaluation_state
                    .best_plan
                    .take()
                    .map(|p| Plan { state: p.state });

                PlanEvaluationResult::Complete(plan)
            }
            TreePlannerResult::Running(stack) => {
                if new_best_plan.is_some() {
                    evaluation_state.best_plan = new_best_plan;
                }

                evaluation_state.stack = stack;

                PlanEvaluationResult::Running()
            }
        };

        Ok(evaluate_result)
    }
}
