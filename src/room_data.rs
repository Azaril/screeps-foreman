use crate::location::*;
use crate::terrain::*;
use serde::{Deserialize, Serialize};

/// Compact signed coordinate used during planning (supports negative offsets).
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
        if self.x >= 0 && self.y >= 0 && self.x < 50 && self.y < 50 {
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
        S: serde::Serializer,
    {
        self.packed_repr().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for PlanLocation {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        u16::deserialize(deserializer).map(PlanLocation::from_packed)
    }
}

impl From<Location> for PlanLocation {
    fn from(loc: Location) -> Self {
        PlanLocation {
            x: loc.x() as i8,
            y: loc.y() as i8,
        }
    }
}

impl From<&Location> for PlanLocation {
    fn from(loc: &Location) -> Self {
        PlanLocation {
            x: loc.x() as i8,
            y: loc.y() as i8,
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

/// Trait for providing room data to the planner.
/// Implementations exist for both in-game (screeps API) and offline (bench) use.
pub trait PlannerRoomDataSource {
    fn get_terrain(&self) -> &FastRoomTerrain;
    fn get_controllers(&self) -> &[PlanLocation];
    fn get_sources(&self) -> &[PlanLocation];
    fn get_minerals(&self) -> &[PlanLocation];
}
