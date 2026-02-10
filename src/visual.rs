#[cfg(feature = "shim")]
use super::shim::*;

#[cfg(not(feature = "shim"))]
use screeps::*;

use super::location::*;

pub trait RoomVisualizer {
    fn render(&mut self, location: Location, structure: StructureType);
}

/// Adapter that implements [`screeps_visual::render::VisualBackend`] by
/// forwarding draw calls to a [`screeps::RoomVisual`].
#[cfg(not(feature = "shim"))]
struct RoomVisualBackend<'a> {
    vis: &'a mut RoomVisual,
}

#[cfg(not(feature = "shim"))]
impl<'a> screeps_visual::render::VisualBackend for RoomVisualBackend<'a> {
    fn circle(
        &mut self,
        x: f32,
        y: f32,
        radius: f32,
        fill: Option<&str>,
        stroke: Option<&str>,
        stroke_width: f32,
        opacity: f32,
    ) {
        let mut style = CircleStyle::default().radius(radius).opacity(opacity);
        if let Some(f) = fill {
            style = style.fill(f);
        }
        if let Some(s) = stroke {
            style = style.stroke(s).stroke_width(stroke_width);
        }
        self.vis.circle(x, y, Some(style));
    }

    fn rect(
        &mut self,
        x: f32,
        y: f32,
        w: f32,
        h: f32,
        fill: Option<&str>,
        stroke: Option<&str>,
        stroke_width: f32,
        opacity: f32,
    ) {
        let mut style = RectStyle::default().opacity(opacity);
        if let Some(f) = fill {
            style = style.fill(f);
        }
        if let Some(s) = stroke {
            style = style.stroke(s).stroke_width(stroke_width);
        }
        self.vis.rect(x, y, w, h, Some(style));
    }

    fn poly(
        &mut self,
        points: &[(f32, f32)],
        fill: Option<&str>,
        stroke: Option<&str>,
        stroke_width: f32,
        opacity: f32,
    ) {
        let mut style = PolyStyle::default().opacity(opacity);
        if let Some(f) = fill {
            style = style.fill(f);
        }
        if let Some(s) = stroke {
            style = style.stroke(s).stroke_width(stroke_width);
        }
        self.vis.poly(points.to_vec(), Some(style));
    }

    fn line(
        &mut self,
        from: (f32, f32),
        to: (f32, f32),
        color: Option<&str>,
        width: f32,
        opacity: f32,
    ) {
        let mut style = LineStyle::default().width(width).opacity(opacity);
        if let Some(c) = color {
            style = style.color(c);
        }
        self.vis.line(from, to, Some(style));
    }
}

#[cfg(not(feature = "shim"))]
impl RoomVisualizer for RoomVisual {
    fn render(&mut self, location: Location, structure: StructureType) {
        let mut backend = RoomVisualBackend { vis: self };
        screeps_visual::render::render_structure(
            &mut backend,
            location.x() as f32,
            location.y() as f32,
            structure,
            1.0,
        );
    }
}
