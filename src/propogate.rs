use crate::dynamics::{arc_pose, max_steer_at_speed};
use crate::grid::DistanceField;
use crate::params::VehicleParams;
use crate::state::State;

#[derive(Debug, Clone, Copy)]
pub struct Control {
    pub accel: f32,        // m/s², held for the whole edge
    pub steer_rate: f32,   // rad/s, held for the whole edge
    pub frames: u32,       // number of PLAN_DT steps
}

#[derive(Debug, Clone, Copy)]
pub struct Edge {
    pub end: State,
    pub cost: f32,
    pub time: f32,
}

pub fn propagate(start: &State, ctl: &Control, sdf: &DistanceField, params: &VehicleParams) -> Option<Edge>
{
    todo!()
}


pub fn step(s: &State, accel: f32, steer_rate: f32, p: &VehicleParams) -> State {
    let n_speed = (s.speed + accel * p.plan_dt).clamp(-p.max_reverse, p.max_speed);
    let lim = max_steer_at_speed(n_speed.abs().max(s.speed.abs()), p);
    let n_steer = (s.steering + steer_rate * p.plan_dt).clamp(-lim, lim);
    let avg_speed = (s.speed + n_speed) / 2.0;
    let distance = avg_speed * p.plan_dt;
    let curvature = ((s.steering + n_steer) / 2.0).tan() / p.wheelbase;
    let (x, y, heading) = arc_pose(s.x, s.y, s.heading, distance, curvature);
    State{x,y,heading,speed: n_speed, steering: n_steer }
}
