//! Tower placement is algorithmic (not stamp-based) -- towers are placed
//! to maximize minimum damage across the rampart perimeter.
//! This module provides the damage calculation utilities.

/// Calculate tower damage at a given range.
/// Screeps tower damage formula:
/// - Range <= 5: 600 damage
/// - Range >= 20: 150 damage
/// - Between 5 and 20: linear interpolation
pub fn tower_damage_at_range(range: u32) -> u32 {
    if range <= 5 {
        600
    } else if range >= 20 {
        150
    } else {
        // Linear interpolation between (5, 600) and (20, 150)
        let t = (range - 5) as f32 / 15.0;
        (600.0 - t * 450.0) as u32
    }
}

/// Calculate tower heal at a given range.
pub fn tower_heal_at_range(range: u32) -> u32 {
    if range <= 5 {
        400
    } else if range >= 20 {
        100
    } else {
        let t = (range - 5) as f32 / 15.0;
        (400.0 - t * 300.0) as u32
    }
}

/// Calculate tower repair at a given range.
pub fn tower_repair_at_range(range: u32) -> u32 {
    if range <= 5 {
        800
    } else if range >= 20 {
        200
    } else {
        let t = (range - 5) as f32 / 15.0;
        (800.0 - t * 600.0) as u32
    }
}
