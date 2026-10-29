#[derive(Debug, Clone, Copy, PartialEq)]
pub struct State {
    pub x: f32,
    pub y: f32,
    pub heading: f32,  // radians
    pub speed: f32,    // px/s
    pub steering: f32, // radians
}