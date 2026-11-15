use rand::{Rng, RngExt};
use rand_distr::{Distribution, Normal};
use std::f32::consts::PI;
use crate::dynamics::{accel_limits, max_steer_at_speed};
use crate::grid::DistanceField;
use crate::params::VehicleParams;
use crate::propogate::Control;
use crate::state::State;
pub struct SamplerParams {
    pub min_frames: u32,        // Minimum time we can propogate
    pub max_frames: u32,        // Max time we cna propogate
    pub steer_sigma: f32,       // How much we wanna bias towards going straight
    pub warm_steer_sigma: f32,  // How much we wanna bias towards going straight during our warm start
}

impl Default for SamplerParams {
    fn default() -> Self {
        Self { min_frames: 10, max_frames: 40, steer_sigma: 0.5, warm_steer_sigma: 0.2 }
    }
}

pub fn random_control<R: Rng>(s: &State,
p:&VehicleParams,
sp: &SamplerParams,
rng: &mut R,
warm: bool,)-> Control{
    let frames = rng.random_range(sp.min_frames..=sp.max_frames);
    let t = frames as f32 * p.plan_dt; //Amount frames*each frame's duratrion
    //Gets us our absolute accel backwards and forwards
    let (lo, hi) = accel_limits(s.speed, p);
    let a_lo = lo.max( (-p.max_reverse - s.speed) / t);
    let a_hi = hi.min( ( p.max_speed   - s.speed) / t);

    let a_lo = if warm{ a_lo.max(0.0)} else{ a_lo};

    let accel = if a_lo < a_hi { rng.random_range(a_lo..=a_hi) } else { a_hi };

    let lim = max_steer_at_speed(s.speed, p);
    let frac = if warm { sp.warm_steer_sigma } else { sp.steer_sigma };
    let target = Normal::new(0.0, frac * lim).unwrap().sample(rng).clamp(-lim, lim);
    let steer_rate = ((target - s.steering) / t).clamp(-p.max_steer_rate, p.max_steer_rate);

    Control { accel, steer_rate, frames }
}


pub fn random_state<R: Rng>(
    center: (f32, f32),
    radius: f32,
    sdf: &DistanceField,
    p: &VehicleParams,
    rng: &mut R,
) -> State {
    //Draws 20 and then tries to keep first one with valid distance from obnstacle (we cna make this simply that it is not in an obstacle)
    //If no luck it just returns last point
    let need = p.footprint_radius + p.safety_margin;
    let mut x = center.0;
    let mut y = center.1;

    for _ in 0..20 {
        let r = radius * rng.random_range(0.0..1.0_f32).sqrt();
        let a = rng.random_range(0.0..2.0 * PI);
        x = center.0 + r * a.cos();
        y = center.1 + r * a.sin();

        if matches!(sdf.at(x, y), Some(d) if d > need) {
            break;
        }
    }

    State {
        x,
        y,
        heading: rng.random_range(-PI..PI),
        speed: rng.random_range(-p.max_reverse..p.max_speed),
        steering: rng.random_range(-p.max_steer..p.max_steer),
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use crate::propogate::step;
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    fn state_at(speed: f32) -> State {
        State { x: 0.0, y: 0.0, heading: 0.0, speed, steering: 0.0 }
    }

    // speeds spanning reverse, rest and forward, including the legal extremes
    fn speeds(p: &VehicleParams) -> Vec<f32> {
        vec![-p.max_reverse, -0.5, 0.0, 0.5, 1.5, p.max_speed]
    }

    #[test]
    fn frames_stay_in_range() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut rng = StdRng::seed_from_u64(1);
        for v in speeds(&p) {
            for _ in 0..500 {
                let c = random_control(&state_at(v), &p, &sp, &mut rng, false);
                assert!(c.frames >= sp.min_frames && c.frames <= sp.max_frames, "frames {}", c.frames);
            }
        }
    }

    #[test]
    fn accel_stays_within_limits_for_the_starting_speed() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut rng = StdRng::seed_from_u64(2);
        for v in speeds(&p) {
            let (lo, hi) = accel_limits(v, &p);
            for _ in 0..500 {
                let c = random_control(&state_at(v), &p, &sp, &mut rng, false);
                assert!(c.accel >= lo - 1e-5 && c.accel <= hi + 1e-5,
                    "speed {v}: accel {} outside [{lo}, {hi}]", c.accel);
            }
        }
    }

    #[test]
    fn sampled_edge_never_leaves_the_legal_speed_range() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut rng = StdRng::seed_from_u64(3);
        for v in speeds(&p) {
            for _ in 0..200 {
                let start = state_at(v);
                let c = random_control(&start, &p, &sp, &mut rng, false);
                let mut s = start;
                for _ in 0..c.frames {
                    s = step(&s, c.accel, c.steer_rate, &p);
                    assert!(s.speed >= -p.max_reverse - 1e-5 && s.speed <= p.max_speed + 1e-5,
                        "speed {} left the legal range", s.speed);
                }
                // for an edge that doesn't cross zero speed, the end speed should match the
                // plain formula. Edges that cross zero are excluded: past the crossing, the
                // same accel switches from braking to throttle and step clamps it harder
                let t = c.frames as f32 * p.plan_dt;
                let expected = v + c.accel * t;
                let crosses_zero = v * expected < 0.0;
                if !crosses_zero {
                    assert!((s.speed - expected).abs() < 1e-3,
                        "start {v}: ended at {} expected about {expected}", s.speed);
                }
            }
        }
    }

    #[test]
    fn steer_rate_respects_the_limit() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut rng = StdRng::seed_from_u64(4);
        for v in speeds(&p) {
            for _ in 0..500 {
                let c = random_control(&state_at(v), &p, &sp, &mut rng, false);
                assert!(c.steer_rate.abs() <= p.max_steer_rate + 1e-6, "rate {}", c.steer_rate);
            }
        }
    }

    #[test]
    fn warm_start_never_brakes_or_reverses() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut rng = StdRng::seed_from_u64(5);
        for _ in 0..1000 {
            let c = random_control(&state_at(0.0), &p, &sp, &mut rng, true);
            assert!(c.accel >= 0.0, "warm start accel was {}", c.accel);
        }
    }

    #[test]
    fn warm_start_steers_less_than_normal_sampling() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let spread = |warm: bool| {
            let mut rng = StdRng::seed_from_u64(6);
            let n = 4000;
            let sum: f32 = (0..n)
                .map(|_| random_control(&state_at(1.0), &p, &sp, &mut rng, warm).steer_rate.abs())
                .sum();
            sum / n as f32
        };
        assert!(spread(true) < spread(false), "warm {} vs normal {}", spread(true), spread(false));
    }

    #[test]
    fn steering_is_centred_on_straight() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut rng = StdRng::seed_from_u64(7);
        let n = 4000;
        let sum: f32 = (0..n)
            .map(|_| random_control(&state_at(1.0), &p, &sp, &mut rng, false).steer_rate)
            .sum();
        let mean = sum / n as f32;
        assert!(mean.abs() < 0.03, "mean steer rate was {mean} (expected near 0)");
    }

    #[test]
    fn same_seed_gives_the_same_controls() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut a = StdRng::seed_from_u64(42);
        let mut b = StdRng::seed_from_u64(42);
        for _ in 0..50 {
            let ca = random_control(&state_at(1.0), &p, &sp, &mut a, false);
            let cb = random_control(&state_at(1.0), &p, &sp, &mut b, false);
            assert_eq!(ca.frames, cb.frames);
            assert_eq!(ca.accel, cb.accel);
            assert_eq!(ca.steer_rate, cb.steer_rate);
        }
    }

    #[test]
    fn different_seeds_give_different_controls() {
        let p = VehicleParams::default();
        let sp = SamplerParams::default();
        let mut a = StdRng::seed_from_u64(1);
        let mut b = StdRng::seed_from_u64(2);
        let ca = random_control(&state_at(1.0), &p, &sp, &mut a, false);
        let cb = random_control(&state_at(1.0), &p, &sp, &mut b, false);
        assert!(ca.accel != cb.accel || ca.steer_rate != cb.steer_rate || ca.frames != cb.frames);
    }

    // ---- random_state ----

    use crate::grid::{Grid, FREE, OCCUPIED};

    // 20 m x 20 m map, 0.1 m cells, filled with `fill`, as an obstacle distance field
    fn sdf_of_filled(fill: u8, wall_col: Option<usize>) -> DistanceField {
        let mut g = Grid::new(200, 200, 0.1, 0.0, 0.0);
        for r in 0..200 {
            for c in 0..200 {
                g.set(c, r, fill);
            }
        }
        if let Some(col) = wall_col {
            for r in 0..200 {
                g.set(col, r, OCCUPIED);
            }
        }
        DistanceField {
            width: g.width,
            height: g.height,
            resolution: g.resolution,
            origin_x: g.origin_x,
            origin_y: g.origin_y,
            data: g.distance_field(OCCUPIED, 2.0, true),
        }
    }

    fn dist(a: (f32, f32), x: f32, y: f32) -> f32 {
        ((x - a.0).powi(2) + (y - a.1).powi(2)).sqrt()
    }

    #[test]
    fn random_state_position_is_inside_the_disk() {
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(FREE, None);
        let mut rng = StdRng::seed_from_u64(10);
        let center = (10.0, 10.0);
        for _ in 0..1000 {
            let s = random_state(center, 5.0, &sdf, &p, &mut rng);
            assert!(dist(center, s.x, s.y) <= 5.0 + 1e-4, "sample at ({}, {})", s.x, s.y);
        }
    }

    #[test]
    fn random_state_other_fields_stay_in_limits() {
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(FREE, None);
        let mut rng = StdRng::seed_from_u64(11);
        for _ in 0..1000 {
            let s = random_state((10.0, 10.0), 5.0, &sdf, &p, &mut rng);
            assert!(s.heading >= -PI && s.heading < PI, "heading {}", s.heading);
            assert!(s.speed >= -p.max_reverse && s.speed < p.max_speed, "speed {}", s.speed);
            assert!(s.steering >= -p.max_steer && s.steering < p.max_steer, "steering {}", s.steering);
        }
    }

    #[test]
    fn random_state_lands_in_free_space_on_an_open_map() {
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(FREE, None);
        let need = p.footprint_radius + p.safety_margin;
        let mut rng = StdRng::seed_from_u64(12);
        for _ in 0..1000 {
            let s = random_state((10.0, 10.0), 5.0, &sdf, &p, &mut rng);
            let d = sdf.at(s.x, s.y).expect("sample should be on the map");
            assert!(d > need, "clearance {d} not above {need}");
        }
    }

    #[test]
    fn random_state_avoids_the_blocked_half() {
        // wall down the middle at x = 10 m: the left half of the disk is within reach of it,
        // so retries should push samples to the right side
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(FREE, Some(100));
        let need = p.footprint_radius + p.safety_margin;
        let mut rng = StdRng::seed_from_u64(13);
        for _ in 0..1000 {
            let s = random_state((10.0, 10.0), 5.0, &sdf, &p, &mut rng);
            let d = sdf.at(s.x, s.y).expect("sample should be on the map");
            assert!(d > need, "sample ({}, {}) has clearance {d}", s.x, s.y);
        }
    }

    #[test]
    fn random_state_terminates_when_everything_is_blocked() {
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(OCCUPIED, None);
        let mut rng = StdRng::seed_from_u64(14);
        let center = (10.0, 10.0);
        for _ in 0..100 {
            let s = random_state(center, 5.0, &sdf, &p, &mut rng);
            assert!(dist(center, s.x, s.y) <= 5.0 + 1e-4);
        }
    }

    #[test]
    fn random_state_is_uniform_over_the_disk_area() {
        // a disk of radius R/2 holds 25% of the area of a disk of radius R
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(FREE, None);
        let mut rng = StdRng::seed_from_u64(15);
        let center = (10.0, 10.0);
        let n = 6000;
        let inner = (0..n)
            .filter(|_| {
                let s = random_state(center, 6.0, &sdf, &p, &mut rng);
                dist(center, s.x, s.y) <= 3.0
            })
            .count();
        let frac = inner as f32 / n as f32;
        assert!((frac - 0.25).abs() < 0.03, "inner fraction was {frac} (expected about 0.25)");
    }

    #[test]
    fn random_state_is_repeatable_with_a_seed() {
        let p = VehicleParams::default();
        let sdf = sdf_of_filled(FREE, None);
        let mut a = StdRng::seed_from_u64(99);
        let mut b = StdRng::seed_from_u64(99);
        for _ in 0..50 {
            let sa = random_state((10.0, 10.0), 5.0, &sdf, &p, &mut a);
            let sb = random_state((10.0, 10.0), 5.0, &sdf, &p, &mut b);
            assert_eq!(sa, sb);
        }
    }
}
