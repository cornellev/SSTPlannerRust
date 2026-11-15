use crate::params::VehicleParams;

//Used in propogate (which essentially calls this 80 times a second)
//It moves the car a bit assuming constant curvature and speed
pub fn arc_pose(x: f32, y: f32, heading: f32, distance: f32, curvature: f32) -> (f32, f32, f32) {
    const STRAIGHT_EPS: f32 = 1e-6;

    if curvature.abs() < STRAIGHT_EPS {
        // What if it's jsut straight?
        (x + distance * heading.cos(), y + distance * heading.sin(), heading)
    } else {
        // Otherwise calc the arc
        let new_heading = heading + curvature * distance;
        let nx = x + (new_heading.sin() - heading.sin()) / curvature;
        let ny = y - (new_heading.cos() - heading.cos()) / curvature;
        (nx, ny, new_heading)
    }
}


pub fn max_steer_at_speed(speed: f32, p: &VehicleParams) -> f32 {
    let v2 = speed * speed;
    if v2 < 1e-6 {
        return p.max_steer;
    }
    p.max_steer.min((p.max_lat_accel * p.wheelbase / v2).atan())
}


/// (lowest, highest) acceleration we can do with our car
pub fn accel_limits(speed: f32, p: &VehicleParams) -> (f32, f32) {
    const EPS: f32 = 1e-3;
    if speed > EPS {
        //Slightly imprecise BC we might break and then accel backwards if that makes sense
        (-p.max_brake, p.max_accel)
    } else if speed < -EPS {
        (-p.max_accel, p.max_brake)
    } else {
        (-p.max_accel, p.max_accel)
    }
}

//Tests are very clauded but proof read oc

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::consts::PI;

    fn close(a: f32, b: f32) -> bool {
        (a - b).abs() < 1e-4
    }

    #[test]
    fn straight_line() {
        let (x, y, h) = arc_pose(0.0, 0.0, 0.0, 1.0, 0.0);
        assert!(close(x, 1.0) && close(y, 0.0) && close(h, 0.0));
    }

    #[test]
    fn quarter_circle() {
        let (x, y, h) = arc_pose(0.0, 0.0, 0.0, PI / 2.0, 1.0);
        assert!(close(x, 1.0) && close(y, 1.0) && close(h, PI / 2.0));
    }

    #[test]
    fn full_circle_returns_home() {
        let (x, y, h) = arc_pose(0.0, 0.0, 0.0, 2.0 * PI, 1.0);
        assert!(close(x, 0.0) && close(y, 0.0) && close(h, 2.0 * PI));
    }

    #[test]
    fn accel_limits_moving_forward() {
        let p = VehicleParams::default();
        let (lo, hi) = accel_limits(1.0, &p);
        assert_eq!(lo, -p.max_brake); // negative accel brakes when moving forward
        assert_eq!(hi, p.max_accel);
    }

    #[test]
    fn accel_limits_while_reversing() {
        let p = VehicleParams::default();
        let (lo, hi) = accel_limits(-1.0, &p);
        assert_eq!(lo, -p.max_accel);
        assert_eq!(hi, p.max_brake); // positive accel brakes when reversing
    }

    #[test]
    fn accel_limits_at_rest_are_symmetric() {
        let p = VehicleParams::default();
        let (lo, hi) = accel_limits(0.0, &p);
        assert_eq!(lo, -p.max_accel);
        assert_eq!(hi, p.max_accel);
    }

    #[test]
    fn accel_limits_mirror_between_forward_and_reverse() {
        let p = VehicleParams::default();
        let (lo_f, hi_f) = accel_limits(0.8, &p);
        let (lo_r, hi_r) = accel_limits(-0.8, &p);
        assert_eq!(lo_f, -hi_r);
        assert_eq!(hi_f, -lo_r);
    }
}