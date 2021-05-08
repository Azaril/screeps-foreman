#[cfg(feature = "shim")]
use super::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

use super::location::*;

pub trait RoomVisualizer {
    fn render(&mut self, location: Location, structure: StructureType);
}

#[cfg(not(feature = "shim"))]
impl RoomVisualizer for RoomVisual {
    fn render(&mut self, location: Location, structure: StructureType) {
        match structure {
            StructureType::Spawn => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Green").opacity(1.0)),
                );
            }
            StructureType::Extension => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Purple").opacity(1.0)),
                );
            }
            StructureType::Container => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Blue").opacity(1.0)),
                );
            }
            StructureType::Storage => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Red").opacity(1.0)),
                );
            }
            StructureType::Link => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Orange").opacity(1.0)),
                );
            }
            StructureType::Terminal => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Pink").opacity(1.0)),
                );
            }
            StructureType::Nuker => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Black").opacity(1.0)),
                );
            }
            StructureType::Lab => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Aqua").opacity(1.0)),
                );
            }
            StructureType::PowerSpawn => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Fuschia").opacity(1.0)),
                );
            }
            StructureType::Observer => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Lime").opacity(1.0)),
                );
            }
            StructureType::Factory => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Brown").opacity(1.0)),
                );
            }
            StructureType::Rampart => {
                RoomVisual::rect(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    1.0,
                    1.0,
                    Some(RectStyle::default().fill("Green").opacity(0.3)),
                );
            }
            _ => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("Yellow").opacity(1.0)),
                );
            }
        }
    }
}
