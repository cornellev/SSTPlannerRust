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
}