use crate::dynamics::{accel_limits, arc_pose, max_steer_at_speed};
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

//Take a single dt step and return new position
pub fn step(s: &State, accel: f32, steer_rate: f32, p: &VehicleParams) -> State {
    let (lo, hi) = accel_limits(s.speed, p);
    let accel = accel.clamp(lo, hi);
    let n_speed = (s.speed + accel * p.plan_dt).clamp(-p.max_reverse, p.max_speed);
    let lim = max_steer_at_speed(n_speed.abs().max(s.speed.abs()), p);
    let n_steer = (s.steering + steer_rate * p.plan_dt).clamp(-lim, lim);
    let avg_speed = (s.speed + n_speed) / 2.0;
    let distance = avg_speed * p.plan_dt;
    let curvature = ((s.steering + n_steer) / 2.0).tan() / p.wheelbase;
    let (x, y, heading) = arc_pose(s.x, s.y, s.heading, distance, curvature);
    State{x,y,heading,speed: n_speed, steering: n_steer }

}

pub fn propagate(start: &State, ctl: &Control, sdf:
&DistanceField, params: &VehicleParams)-> Option<Edge>{
    let mut s = *start;
    let mut since_check = 0.0_f32;
    let mut steer_cost = 0.0_f32;
    let mut budget = params.check_spacing; //Distance we cna go before next checks safeklky
    for _ in 0..ctl.frames{
        let prev = s;
        s = step(&s, ctl.accel, ctl.steer_rate, params);
        let dist = (prev.speed+s.speed)/2.0*params.plan_dt;
        let dh = s.heading-prev.heading;
        let max_off = params.footprint_offsets.iter().cloned().fold(0.0_f32, f32::max);

        since_check+=dist.abs()+dh.abs()*max_off;

        if since_check >= budget{
            let slack = sdf.pose_clear(params, s.x, s.y, s.heading)?;
            budget = params.check_spacing.max(slack - params.cell_error);
            since_check = 0.0;
        }
        steer_cost+= s.steering.abs()*params.plan_dt;
    }

    let slack = sdf.pose_clear(params, s.x, s.y, s.heading)?;


    let time = ctl.frames as f32 * params.plan_dt;
    let cost = time + 10.8 * steer_cost;     
    Some(Edge { end: s, cost, time })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{Grid, FREE, OCCUPIED};

    // 20 m x 20 m all-free map, 0.1 m cells
    fn open_map() -> Grid {
        let mut g = Grid::new(200, 200, 0.1, 0.0, 0.0);
        for r in 0..200 {
            for c in 0..200 {
                g.set(c, r, FREE);
            }
        }
        g
    }

    fn sdf_of(g: &Grid) -> DistanceField {
        DistanceField {
            width: g.width,
            height: g.height,
            resolution: g.resolution,
            origin_x: g.origin_x,
            origin_y: g.origin_y,
            data: g.distance_field(OCCUPIED, 2.0, true),
        }
    }

    // car at (5, 10) facing +x at 1 m/s
    fn start() -> State {
        State { x: 5.0, y: 10.0, heading: 0.0, speed: 1.0, steering: 0.0 }
    }

    #[test]
    fn step_accelerates_from_rest() {
        let p = VehicleParams::default();
        let mut s = State { x: 0.0, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.0 };
        for _ in 0..80 {
            s = step(&s, 1.2, 0.0, &p);
        }
        // 1 s at 1.2 m/s^2: v = 1.2 m/s, distance = 0.5 * 1.2 * 1^2 = 0.6 m
        assert!((s.speed - 1.2).abs() < 1e-3, "speed was {}", s.speed);
        assert!((s.x - 0.6).abs() < 1e-3, "x was {}", s.x);
        assert!(s.y.abs() < 1e-6);
    }

    #[test]
    fn step_never_exceeds_max_speed() {
        let p = VehicleParams::default();
        let mut s = State { x: 0.0, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.0 };
        for _ in 0..800 {
            s = step(&s, p.max_accel, 0.0, &p);
        }
        assert!((s.speed - p.max_speed).abs() < 1e-5, "speed was {}", s.speed);
    }

    #[test]
    fn step_respects_lateral_limit_at_speed() {
        let p = VehicleParams::default();
        let mut s = State { x: 0.0, y: 0.0, heading: 0.0, speed: p.max_speed, steering: 0.0 };
        for _ in 0..80 {
            s = step(&s, 0.0, 5.0, &p); // crank the wheel as fast as possible
        }
        let lim = max_steer_at_speed(s.speed, &p);
        assert!(lim < p.max_steer, "limit at top speed should be below full lock");
        assert!(s.steering <= lim + 1e-6, "steering {} exceeded limit {}", s.steering, lim);
        assert!(s.steering >= -lim - 1e-6);
    }

    #[test]
    fn straight_edge_is_clear_and_moves_forward() {
        let p = VehicleParams::default();
        let sdf = sdf_of(&open_map());
        let ctl = Control { accel: 0.0, steer_rate: 0.0, frames: 40 }; // 0.5 s at 1 m/s

        let edge = propagate(&start(), &ctl, &sdf, &p).expect("edge should be clear");
        assert!((edge.end.x - 5.5).abs() < 1e-3, "x was {}", edge.end.x);
        assert!((edge.end.y - 10.0).abs() < 1e-3);
    }

    #[test]
    fn obstacle_ahead_blocks_the_edge() {
        let p = VehicleParams::default();
        let mut g = open_map();
        // wall at x = 6.8 m: the front circle reaches 6.4 m by the end of the edge,
        // which is closer than radius + margin (0.6 m)
        for r in 90..110 {
            g.set(68, r, OCCUPIED);
        }
        let sdf = sdf_of(&g);
        let ctl = Control { accel: 0.0, steer_rate: 0.0, frames: 40 };

        assert!(propagate(&start(), &ctl, &sdf, &p).is_none());
    }

    #[test]
    fn short_edge_short_of_the_wall_is_clear() {
        let p = VehicleParams::default();
        let mut g = open_map();
        for r in 90..110 {
            g.set(68, r, OCCUPIED);
        }
        let sdf = sdf_of(&g);
        let ctl = Control { accel: 0.0, steer_rate: 0.0, frames: 8 }; // 0.1 m

        assert!(propagate(&start(), &ctl, &sdf, &p).is_some());
    }

    #[test]
    fn edge_leaving_the_map_is_rejected() {
        let p = VehicleParams::default();
        let sdf = sdf_of(&open_map());
        let near_edge = State { x: 19.5, y: 10.0, heading: 0.0, speed: 1.0, steering: 0.0 };
        let ctl = Control { accel: 0.0, steer_rate: 0.0, frames: 40 };

        assert!(propagate(&near_edge, &ctl, &sdf, &p).is_none());
    }

    #[test]
    fn time_is_frames_times_dt() {
        let p = VehicleParams::default();
        let sdf = sdf_of(&open_map());
        let ctl = Control { accel: 0.0, steer_rate: 0.0, frames: 40 };

        let edge = propagate(&start(), &ctl, &sdf, &p).unwrap();
        assert!((edge.time - 40.0 / 80.0).abs() < 1e-6);
    }

    #[test]
    fn turning_costs_more_than_driving_straight() {
        let p = VehicleParams::default();
        let sdf = sdf_of(&open_map());
        let straight = Control { accel: 0.0, steer_rate: 0.0, frames: 40 };
        let turning = Control { accel: 0.0, steer_rate: 0.3, frames: 40 };

        let a = propagate(&start(), &straight, &sdf, &p).unwrap();
        let b = propagate(&start(), &turning, &sdf, &p).unwrap();
        assert!(b.cost > a.cost, "turning {} vs straight {}", b.cost, a.cost);
        assert!(b.end.heading > 0.0, "left steering should turn the heading positive");
    }

    fn moving(speed: f32) -> State {
        State { x: 0.0, y: 0.0, heading: 0.0, speed, steering: 0.0 }
    }

    #[test]
    fn step_clamps_throttle_when_moving_forward() {
        let p = VehicleParams::default();
        let s = step(&moving(1.0), 100.0, 0.0, &p);
        assert!((s.speed - (1.0 + p.max_accel * p.plan_dt)).abs() < 1e-6, "speed was {}", s.speed);
    }

    #[test]
    fn step_clamps_braking_when_moving_forward() {
        let p = VehicleParams::default();
        let s = step(&moving(1.0), -100.0, 0.0, &p);
        assert!((s.speed - (1.0 - p.max_brake * p.plan_dt)).abs() < 1e-6, "speed was {}", s.speed);
    }

    #[test]
    fn step_clamps_braking_when_reversing() {
        let p = VehicleParams::default();
        let s = step(&moving(-0.5), 100.0, 0.0, &p);
        assert!((s.speed - (-0.5 + p.max_brake * p.plan_dt)).abs() < 1e-6, "speed was {}", s.speed);
    }

    #[test]
    fn step_clamps_throttle_when_reversing() {
        let p = VehicleParams::default();
        let s = step(&moving(-0.5), -100.0, 0.0, &p);
        assert!((s.speed - (-0.5 - p.max_accel * p.plan_dt)).abs() < 1e-6, "speed was {}", s.speed);
    }
}
