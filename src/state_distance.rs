use std::f32::consts::PI;
use crate::state::State;

#[derive(Debug, Clone, Copy)]
pub struct DistanceWeights{
    pub rotWeight: f32,
    pub positionWeight: f32,
    pub speedWeight:f32,
    pub steeringWeight: f32,
}
//Wrap around for the angles
fn angle_diff(a: f32, b: f32) -> f32 {
    let d = (a - b).rem_euclid(2.0 * PI);
    if d > PI { 2.0 * PI - d } else { d }
}
pub fn state_distance(a: &State, b: &State, w: &DistanceWeights)->f32{
    let mut dist:f32;
    dist = w.rotWeight*angle_diff(a.heading, b.heading);
    dist+= w.speedWeight*(a.speed-b.speed).abs();
    dist+= w.steeringWeight*(a.steering-b.steering).abs();
    dist+= w.positionWeight*((a.x-b.x).powi(2)+(a.y-b.y).powi(2)).sqrt();
    dist
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state(x: f32, y: f32, heading: f32, speed: f32) -> State {
        State { x, y, heading, speed, steering: 0.0 }
    }

    fn weights() -> DistanceWeights {
        DistanceWeights { rotWeight: 1.0, positionWeight: 1.0, speedWeight: 1.0, steeringWeight: 1.0 }
    }

    // only one of position / heading / speed switched on (steering off)
    fn only(pos: f32, rot: f32, speed: f32) -> DistanceWeights {
        DistanceWeights { rotWeight: rot, positionWeight: pos, speedWeight: speed, steeringWeight: 0.0 }
    }

    #[test]
    fn identical_states_are_zero_apart() {
        let s = state(1.0, 2.0, 0.5, 1.0);
        assert_eq!(state_distance(&s, &s, &weights()), 0.0);
    }

    #[test]
    fn distance_is_symmetric() {
        let a = state(0.0, 0.0, 0.3, 1.0);
        let b = state(2.0, -1.0, -2.0, 0.2);
        let w = weights();
        assert!((state_distance(&a, &b, &w) - state_distance(&b, &a, &w)).abs() < 1e-6);
    }

    #[test]
    fn position_term_is_euclidean() {
        let a = state(0.0, 0.0, 0.0, 0.0);
        let b = state(3.0, 4.0, 0.0, 0.0);
        assert!((state_distance(&a, &b, &only(1.0, 0.0, 0.0)) - 5.0).abs() < 1e-5);
    }

    #[test]
    fn speed_term_is_absolute_difference() {
        let a = state(0.0, 0.0, 0.0, 2.0);
        let b = state(0.0, 0.0, 0.0, -1.0);
        assert!((state_distance(&a, &b, &only(0.0, 0.0, 1.0)) - 3.0).abs() < 1e-5);
    }

    #[test]
    fn weights_scale_each_term() {
        let a = state(0.0, 0.0, 0.0, 0.0);
        let b = state(3.0, 4.0, 0.0, 0.0);
        assert!((state_distance(&a, &b, &only(0.5, 0.0, 0.0)) - 2.5).abs() < 1e-5);
    }

    #[test]
    fn heading_difference_wraps_around() {
        // 359 degrees and 1 degree are 2 degrees apart, not 358
        let a = state(0.0, 0.0, 359.0_f32.to_radians(), 0.0);
        let b = state(0.0, 0.0, 1.0_f32.to_radians(), 0.0);
        let d = state_distance(&a, &b, &only(0.0, 1.0, 0.0));
        assert!((d - 2.0_f32.to_radians()).abs() < 1e-4, "distance was {d}");
    }

    #[test]
    fn heading_difference_is_at_most_half_a_turn() {
        let a = state(0.0, 0.0, 0.0, 0.0);
        let b = state(0.0, 0.0, PI, 0.0);
        let d = state_distance(&a, &b, &only(0.0, 1.0, 0.0));
        assert!((d - PI).abs() < 1e-4, "distance was {d}");
    }

    #[test]
    fn unwrapped_headings_compare_correctly() {
        // headings in this crate are not wrapped (a full circle ends at 2*PI)
        let a = state(0.0, 0.0, 2.0 * PI + 0.1, 0.0);
        let b = state(0.0, 0.0, 0.1, 0.0);
        let d = state_distance(&a, &b, &only(0.0, 1.0, 0.0));
        assert!(d < 1e-4, "distance was {d}");
    }

    #[test]
    fn terms_add_up() {
        let a = state(0.0, 0.0, 0.0, 0.0);
        let mut b = state(3.0, 4.0, 1.0, 2.0);
        b.steering = 0.5;
        let d = state_distance(&a, &b, &weights());
        assert!((d - (5.0 + 1.0 + 2.0 + 0.5)).abs() < 1e-5, "distance was {d}");
    }

    #[test]
    fn steering_term_is_absolute_difference() {
        let a = State { x: 0.0, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.4 };
        let b = State { x: 0.0, y: 0.0, heading: 0.0, speed: 0.0, steering: -0.2 };
        let w = DistanceWeights { rotWeight: 0.0, positionWeight: 0.0, speedWeight: 0.0, steeringWeight: 1.0 };
        assert!((state_distance(&a, &b, &w) - 0.6).abs() < 1e-5);
    }

    #[test]
    fn steering_weight_scales_the_term() {
        let a = State { x: 0.0, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.4 };
        let b = State { x: 0.0, y: 0.0, heading: 0.0, speed: 0.0, steering: -0.2 };
        let w = DistanceWeights { rotWeight: 0.0, positionWeight: 0.0, speedWeight: 0.0, steeringWeight: 0.5 };
        assert!((state_distance(&a, &b, &w) - 0.3).abs() < 1e-5);
    }

    #[test]
    fn same_pose_opposite_steering_is_no_longer_zero_apart() {
        // the case that made the steering term worth adding
        let a = State { x: 1.0, y: 1.0, heading: 0.3, speed: 1.0, steering: 0.3 };
        let b = State { x: 1.0, y: 1.0, heading: 0.3, speed: 1.0, steering: -0.3 };
        assert!(state_distance(&a, &b, &weights()) > 0.5);
    }
}
