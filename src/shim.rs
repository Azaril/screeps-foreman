use serde::*;

pub const TERRAIN_MASK_WALL: u8 = 1;
pub const TERRAIN_MASK_SWAMP: u8 = 2;
pub const TERRAIN_MASK_LAVA: u8 = 4;

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Eq, PartialEq, Hash)]
pub enum StructureType {
    Spawn = 0,
    Extension = 1,
    Road = 2,
    Wall = 3,
    Rampart = 4,
    KeeperLair = 5,
    Portal = 6,
    Controller = 7,
    Link = 8,
    Storage = 9,
    Tower = 10,
    Observer = 11,
    PowerBank = 12,
    PowerSpawn = 13,
    Extractor = 14,
    Lab = 15,
    Terminal = 16,
    Container = 17,
    Nuker = 18,
    Factory = 19,
    InvaderCore = 20,
}

#[derive(Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct CircleStyle {
    #[serde(skip_serializing_if = "Option::is_none")]
    radius: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    fill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    opacity: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke_width: Option<f32>,
}

impl CircleStyle {
    pub fn radius(mut self, val: f32) -> CircleStyle {
        self.radius = Some(val);
        self
    }

    pub fn fill(mut self, val: &str) -> CircleStyle {
        self.fill = Some(val.to_string());
        self
    }

    pub fn opacity(mut self, val: f32) -> CircleStyle {
        self.opacity = Some(val);
        self
    }

    pub fn stroke(mut self, val: &str) -> CircleStyle {
        self.stroke = Some(val.to_string());
        self
    }

    pub fn stroke_width(mut self, val: f32) -> CircleStyle {
        self.stroke_width = Some(val);
        self
    }
}

#[derive(Clone, Serialize)]
pub struct CircleData {
    x: f32,
    y: f32,
    #[serde(rename = "s", skip_serializing_if = "Option::is_none")]
    style: Option<CircleStyle>,
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum LineDrawStyle {
    Solid,
    Dashed,
    Dotted,
}

impl Default for LineDrawStyle {
    fn default() -> LineDrawStyle {
        LineDrawStyle::Solid
    }
}

impl LineDrawStyle {
    pub fn is_solid(&self) -> bool {
        match self {
            LineDrawStyle::Solid => true,
            _ => false,
        }
    }
}

#[derive(Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct LineStyle {
    #[serde(skip_serializing_if = "Option::is_none")]
    width: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    opacity: Option<f32>,
    #[serde(skip_serializing_if = "LineDrawStyle::is_solid")]
    line_style: LineDrawStyle,
}

impl LineStyle {
    pub fn width(mut self, val: f32) -> LineStyle {
        self.width = Some(val);
        self
    }

    pub fn color(mut self, val: &str) -> LineStyle {
        self.color = Some(val.to_string());
        self
    }

    pub fn opacity(mut self, val: f32) -> LineStyle {
        self.opacity = Some(val);
        self
    }

    pub fn line_style(mut self, val: LineDrawStyle) -> LineStyle {
        self.line_style = val;
        self
    }
}

#[derive(Clone, Serialize)]
pub struct LineData {
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
    #[serde(rename = "s", skip_serializing_if = "Option::is_none")]
    style: Option<LineStyle>,
}

#[derive(Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RectStyle {
    #[serde(skip_serializing_if = "Option::is_none")]
    fill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    opacity: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke_width: Option<f32>,
    #[serde(skip_serializing_if = "LineDrawStyle::is_solid")]
    line_style: LineDrawStyle,
}

impl RectStyle {
    pub fn fill(mut self, val: &str) -> RectStyle {
        self.fill = Some(val.to_string());
        self
    }

    pub fn opacity(mut self, val: f32) -> RectStyle {
        self.opacity = Some(val);
        self
    }

    pub fn stroke(mut self, val: &str) -> RectStyle {
        self.stroke = Some(val.to_string());
        self
    }

    pub fn stroke_width(mut self, val: f32) -> RectStyle {
        self.stroke_width = Some(val);
        self
    }

    pub fn line_style(mut self, val: LineDrawStyle) -> RectStyle {
        self.line_style = val;
        self
    }
}

#[derive(Clone, Serialize)]
pub struct RectData {
    x: f32,
    y: f32,
    #[serde(rename = "w")]
    width: f32,
    #[serde(rename = "h")]
    height: f32,
    #[serde(rename = "s", skip_serializing_if = "Option::is_none")]
    style: Option<RectStyle>,
}

#[derive(Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PolyStyle {
    #[serde(skip_serializing_if = "Option::is_none")]
    fill: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    opacity: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke_width: Option<f32>,
    #[serde(skip_serializing_if = "LineDrawStyle::is_solid")]
    line_style: LineDrawStyle,
}

impl PolyStyle {
    pub fn fill(mut self, val: &str) -> PolyStyle {
        self.fill = Some(val.to_string());
        self
    }

    pub fn opacity(mut self, val: f32) -> PolyStyle {
        self.opacity = Some(val);
        self
    }

    pub fn stroke(mut self, val: &str) -> PolyStyle {
        self.stroke = Some(val.to_string());
        self
    }

    pub fn stroke_width(mut self, val: f32) -> PolyStyle {
        self.stroke_width = Some(val);
        self
    }

    pub fn line_style(mut self, val: LineDrawStyle) -> PolyStyle {
        self.line_style = val;
        self
    }
}

#[derive(Clone, Serialize)]
pub struct PolyData {
    points: Vec<(f32, f32)>,
    #[serde(rename = "s", skip_serializing_if = "Option::is_none")]
    style: Option<PolyStyle>,
}

#[derive(Clone, Serialize)]
#[serde(untagged)]
pub enum FontStyle {
    Size(f32),
    Custom(String),
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub enum TextAlign {
    Center,
    Left,
    Right,
}

impl Default for TextAlign {
    fn default() -> TextAlign {
        TextAlign::Center
    }
}

impl TextAlign {
    pub fn is_center(&self) -> bool {
        match self {
            TextAlign::Center => true,
            _ => false,
        }
    }
}

#[derive(Clone, Default, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct TextStyle {
    #[serde(skip_serializing_if = "Option::is_none")]
    color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    font: Option<FontStyle>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    stroke_width: Option<f32>,
    #[serde(skip_serializing_if = "Option::is_none")]
    background_color: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    background_padding: Option<f32>,
    #[serde(skip_serializing_if = "TextAlign::is_center")]
    align: TextAlign,
    #[serde(skip_serializing_if = "Option::is_none")]
    opacity: Option<f32>,
}

impl TextStyle {
    pub fn color(mut self, val: &str) -> TextStyle {
        self.color = Some(val.to_string());
        self
    }

    pub fn font(mut self, val: f32) -> TextStyle {
        self.font = Some(FontStyle::Size(val));
        self
    }

    pub fn custom_font(mut self, val: &str) -> TextStyle {
        self.font = Some(FontStyle::Custom(val.to_string()));
        self
    }

    pub fn stroke(mut self, val: &str) -> TextStyle {
        self.stroke = Some(val.to_string());
        self
    }

    pub fn stroke_width(mut self, val: f32) -> TextStyle {
        self.opacity = Some(val);
        self
    }

    pub fn background_color(mut self, val: &str) -> TextStyle {
        self.background_color = Some(val.to_string());
        self
    }

    pub fn background_padding(mut self, val: f32) -> TextStyle {
        self.opacity = Some(val);
        self
    }

    pub fn align(mut self, val: TextAlign) -> TextStyle {
        self.align = val;
        self
    }

    pub fn opacity(mut self, val: f32) -> TextStyle {
        self.opacity = Some(val);
        self
    }
}

#[derive(Clone, Serialize)]
pub struct TextData {
    text: String,
    x: f32,
    y: f32,
    #[serde(rename = "s", skip_serializing_if = "Option::is_none")]
    style: Option<TextStyle>,
}

#[derive(Clone, Serialize)]
#[serde(tag = "t")]
pub enum Visual {
    #[serde(rename = "c")]
    Circle(CircleData),
    #[serde(rename = "l")]
    Line(LineData),
    #[serde(rename = "r")]
    Rect(RectData),
    #[serde(rename = "p")]
    Poly(PolyData),
    #[serde(rename = "t")]
    Text(TextData),
}

impl Visual {
    pub fn circle(x: f32, y: f32, style: Option<CircleStyle>) -> Visual {
        Visual::Circle(CircleData { x, y, style })
    }

    pub fn line(from: (f32, f32), to: (f32, f32), style: Option<LineStyle>) -> Visual {
        Visual::Line(LineData {
            x1: from.0,
            y1: from.1,
            x2: to.0,
            y2: to.1,
            style,
        })
    }

    pub fn rect(x: f32, y: f32, width: f32, height: f32, style: Option<RectStyle>) -> Visual {
        Visual::Rect(RectData {
            x,
            y,
            width,
            height,
            style,
        })
    }

    pub fn poly(points: Vec<(f32, f32)>, style: Option<PolyStyle>) -> Visual {
        Visual::Poly(PolyData { points, style })
    }

    pub fn text(x: f32, y: f32, text: String, style: Option<TextStyle>) -> Visual {
        Visual::Text(TextData { x, y, text, style })
    }
}