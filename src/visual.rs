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
                    Some(CircleStyle::default().fill("green").opacity(1.0)),
                );
            }
            StructureType::Extension => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("purple").opacity(1.0)),
                );
            }
            StructureType::Container => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("blue").opacity(1.0)),
                );
            }
            StructureType::Storage => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("red").opacity(1.0)),
                );
            }
            StructureType::Link => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("orange").opacity(1.0)),
                );
            }
            StructureType::Terminal => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("pink").opacity(1.0)),
                );
            }
            StructureType::Nuker => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("black").opacity(1.0)),
                );
            }
             StructureType::Lab => {
                RoomVisual::circle(
                    self,
                    location.x() as f32,
                    location.y() as f32,
                    Some(CircleStyle::default().fill("aqua").opacity(1.0)),
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
                    Some(CircleStyle::default().fill("yellow").opacity(1.0)),
                );
            }
        }
    }
}