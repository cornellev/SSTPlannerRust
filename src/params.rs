#[derive(Debug, Clone, Copy)]
pub struct VehicleParams {
    // dynamics
    pub max_accel: f32,        // m/s²
    pub max_speed: f32,        // m/s
    pub max_reverse: f32,      // m/s
    pub max_steer: f32,        // rad
    pub max_steer_rate: f32,   // rad/s
    pub max_lat_accel: f32,    // m/s²
    pub max_brake:f32 ,


    // geometry
    pub wheelbase: f32,        // m
    pub footprint_radius: f32, // m, radius of each collision circle
    pub safety_margin: f32,    // m, extra clearance required from obstacles
    pub footprint_offsets: [f32;3],//Meters of how much the collision circles are ahead rear axle
    pub plan_dt:f32, //how long each frame is
    pub check_spacing:f32, //meters (2 times the safety margin)

}

impl Default for VehicleParams {
    fn default() -> Self {
        Self {
            max_accel: 1.2,
            max_speed: 3.0,
            max_reverse: 1.0,
            max_steer: 35.0_f32.to_radians(),
            max_steer_rate: 20.0_f32.to_radians(),
            max_lat_accel: 2.0,
            wheelbase: 0.6,
            footprint_radius: 0.4,
            safety_margin: 0.2,
            footprint_offsets:[0.1, 0.5, 0.9],
            plan_dt:1.0/80.0,
            check_spacing:0.25,
            max_brake:2.4,
        }
    }
}

#[cfg(test)] //Just my first RUST test (not actually useful icl)
mod tests {
    use super::*;

    #[test]
    fn min_turning_radius_makes_sense() {
        let p = VehicleParams::default();
        let r = p.wheelbase / p.max_steer.tan();
        assert!(r > 0.5 && r < 20.0, "radius was {r}");
    }
}
