use serde::*;

#[derive(Copy, Clone, Eq, PartialEq, Hash, Debug)]
#[repr(transparent)]
pub struct Location {
    packed: u16,
}

impl Location {
    pub fn from_coords(x: u32, y: u32) -> Self {
        Location {
            packed: ((x << 8) | y) as u16,
        }
    }

    #[inline]
    pub fn x(self) -> u8 {
        ((self.packed >> 8) & 0xFF) as u8
    }

    #[inline]
    pub fn y(self) -> u8 {
        (self.packed & 0xFF) as u8
    }

    #[inline]
    pub fn packed_repr(self) -> u16 {
        self.packed
    }

    #[inline]
    pub fn from_packed(packed: u16) -> Self {
        Location { packed }
    }

    pub fn distance_to(self, other: Self) -> u8 {
        let dx = (self.x() as i8) - (other.x() as i8);
        let dy = (self.y() as i8) - (other.y() as i8);

        dx.abs().max(dy.abs()) as u8
    }

    pub fn distance_to_xy(self, x: i8, y: i8) -> u8 {
        let dx = (self.x() as i8) - x;
        let dy = (self.y() as i8) - y;

        dx.abs().max(dy.abs()) as u8
    }
}

impl Serialize for Location {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.packed_repr().serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for Location {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        u16::deserialize(deserializer).map(Location::from_packed)
    }
}
