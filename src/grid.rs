use crate::params::VehicleParams;

pub const FREE: u8 = 0;
pub const OCCUPIED: u8 = 254;
pub const UNKNOWN: u8 = 255;

pub struct Grid {
    pub width: usize,      // cells in x
    pub height: usize,     // cells in y
    pub resolution: f32,   // meters per cell
    pub origin_x: f32,     // world x of grid origin
    pub origin_y: f32,
    pub cells: Vec<u8>,    
}


impl Grid {
    pub fn new(width: usize, height: usize, resolution: f32, origin_x: f32, origin_y: f32) -> Self {
        Self{
        width,
        height,
        resolution,
        origin_x,
        origin_y,
        cells: vec![UNKNOWN; width * height],}
    }
    

    /// World position (metres) -> (col, row), or None if outside the grid.
    pub fn world_to_cell(&self, x: f32, y: f32) -> Option<(usize, usize)> {
        
    let fx = (x - self.origin_x) / self.resolution;
    let fy = (y - self.origin_y) / self.resolution;
//We can't have neg coords!!
    if fx < 0.0 || fy < 0.0 {
        return None;
    }

    let col = fx as usize;   
    let row = fy as usize;

    if col >= self.width || row >= self.height {
        return None;
    }
    Some((col, row))
    }

    pub fn get(&self, col: usize, row: usize) -> u8 {
        self.cells[row*self.width+col]
     }
    pub fn set(&mut self, col: usize, row: usize, value: u8) { 
        self.cells[row*self.width+col] = value;
     }

pub fn distance_field_slow_test(&self, kind: u8, cap: f32, border_obstacle: bool) -> Vec<f32> {
    let k = (cap / self.resolution).ceil() as i32;
    let mut out = vec![cap; self.width * self.height];

    for row in 0..self.height {
        for col in 0..self.width {
            let mut best = cap;

            for dr in -k..=k {
                for dc in -k..=k {
                    let nc = col as i32 + dc;
                    let nr = row as i32 + dr;

                    let hit = if nc < 0 || nr < 0 || nc >= self.width as i32 || nr >= self.height as i32 {
                        border_obstacle
                    } else {
                        self.get(nc as usize, nr as usize) == kind
                    };

                    if hit {
                        let d = ((dc * dc + dr * dr) as f32).sqrt() * self.resolution;
                        best = best.min(d);
                    }
                }
            }
            out[row * self.width + col] = best;
        }
    }
    out
}

   pub fn distance_field(&self, kind: u8, cap: f32, border_obstacle: bool) -> Vec<f32> {
    let w = self.width;
    let ht = self.height;
    let k = (cap / self.resolution).ceil() as i32;
    let inf = k + 1; // "there is no target cell within k cells"

    // Get horiz distance of eahc cell (only within its row)
    let mut h = vec![inf; w * ht];
    for row in 0..ht {
        let line = &mut h[row * w..(row + 1) * w];
        let cells = &self.cells[row * w..(row + 1) * w];

        // left -> right. Col -1 and Col length are counted as hits if border_obstacle is true.
        let mut dist = if border_obstacle { 0 } else { inf };
        for col in 0..w {
            dist = if cells[col] == kind { 0 } else { (dist + 1).min(inf) };
            line[col] = line[col].min(dist);
        }

        // right -> left.
        let mut dist = if border_obstacle { 0 } else { inf };
        for col in (0..w).rev() {
            dist = if cells[col] == kind { 0 } else { (dist + 1).min(inf) };
            line[col] = line[col].min(dist);
        }
    }

    // Combining vertically
    let mut out = vec![cap; w * ht];
    let mut acc = vec![i32::MAX; w];

    for row in 0..ht {
        acc.fill(i32::MAX);

        for dr in -k..=k {
            let nr = row as i32 + dr;

            if nr < 0 || nr >= ht as i32 {
                if border_obstacle {//Horiz distance of 0
                    for a in acc.iter_mut() {
                        *a = (*a).min(dr * dr);
                    }
                }
                continue;
            }

            let h_row = &h[nr as usize * w..(nr as usize + 1) * w];
            for (a, &hv) in acc.iter_mut().zip(h_row) {
                *a = (*a).min(dr * dr + hv * hv);
            }
        }

        for col in 0..w {
            let d = (acc[col] as f32).sqrt() * self.resolution;
            out[row * w +col] = d.min(cap);
        }
    }
    out
}


pub fn to_distance_field(&self, kind: u8, cap: f32, border_obstacle: bool) -> DistanceField {
    DistanceField {
        width: self.width,
        height: self.height,
        resolution: self.resolution,
        origin_x: self.origin_x,
        origin_y: self.origin_y,
        data: self.distance_field(kind, cap, border_obstacle),
    }
}
}

pub struct DistanceField {
    pub width: usize,
    pub height: usize,
    pub resolution: f32,
    pub origin_x: f32,
    pub origin_y: f32,
    pub data: Vec<f32>,
}

impl DistanceField {
    /// Clearance in metres at a world point, or None if the point is off the grid.
    pub fn at(&self, x: f32, y: f32) -> Option<f32> {
        
    let fx = (x - self.origin_x) / self.resolution;
    let fy = (y - self.origin_y) / self.resolution;
//We can't have neg coords!!
    if fx < 0.0 || fy < 0.0 {
        return None;
    }

    let col = fx as usize;   
    let row = fy as usize;

    if col >= self.width || row >= self.height {
        return None;
    }
    Some(self.data[row * self.width + col])
    }


    pub fn pose_clear(&self, params: &VehicleParams, x: f32, y: f32, heading: f32) -> bool {
        let (s, c) = heading.sin_cos();                    // getting sin and cos of the ehading to find how body is oriented
        let need = params.footprint_radius + params.safety_margin;

        for &off in &params.footprint_offsets {
            let cx = x + off * c;
            let cy = y + off * s;
            match self.at(cx, cy) {
                Some(d) if d > need => {}          // Circle is safe
                _ => return false,                 // Too close to an obstacle
        }
    }
    true
}
}




#[cfg(test)]
mod tests {
    use super::*;

    fn grid() -> Grid {
        Grid::new(10, 10, 0.5, 0.0, 0.0)
    }

    #[test]
    fn inside_point_maps_to_cell() {
        assert_eq!(grid().world_to_cell(0.6, 0.6), Some((1, 1)));
    }

    #[test]
    fn negative_is_outside() {
        assert_eq!(grid().world_to_cell(-0.1, 0.0), None);
    }

    #[test]
    fn upper_edge_is_outside() {
        // x = 5.0 would be column 10, but valid columns are 0..=9
        assert_eq!(grid().world_to_cell(5.0, 0.0), None);
    }

    #[test]
    fn set_then_get_round_trips() {
        let mut g = grid();
        g.set(3, 4, OCCUPIED);
        assert_eq!(g.get(3, 4), OCCUPIED);
        assert_eq!(g.get(0, 0), UNKNOWN); // untouched cells stay unknown
    }


#[test]
fn distance_field_basic() {
    let mut g = Grid::new(5, 5, 1.0, 0.0, 0.0);
    for r in 0..5 { for c in 0..5 { g.set(c, r, FREE); } }
    g.set(2, 2, OCCUPIED);

    let f = g.distance_field_slow_test(OCCUPIED, 10.0, false);
    assert_eq!(f[2 * 5 + 2], 0.0);                                   // the obstacle itself
    assert!((f[2 * 5 + 3] - 1.0).abs() < 1e-5);                      // beside it
    assert!((f[3 * 5 + 3] - 2.0_f32.sqrt()).abs() < 1e-5);           // diagonal
}

#[test]
fn distance_field_cap() {
    let mut g = Grid::new(9, 1, 1.0, 0.0, 0.0);
    for c in 0..9 { g.set(c, 0, FREE); }
    g.set(0, 0, OCCUPIED);

    let f = g.distance_field(OCCUPIED, 3.0, false);
    assert!((f[2] - 2.0).abs() < 1e-5);   // within the cap
    assert!((f[8] - 3.0).abs() < 1e-5);   // 8 away, so clamped to the cap
}

#[test]
fn distance_field_border() {
    let mut g = Grid::new(5, 5, 1.0, 0.0, 0.0);
    for r in 0..5 { for c in 0..5 { g.set(c, r, FREE); } }

    let with = g.distance_field(OCCUPIED, 10.0, true);
    let without = g.distance_field(OCCUPIED, 10.0, false);
    assert!((with[0] - 1.0).abs() < 1e-5);      // corner is 1 m from the border wall
    assert!((without[0] - 10.0).abs() < 1e-5);  // no obstacles anywhere -> cap
}

#[test]
fn separable_matches_slow() {
    let mut seed: u32 = 12345;
    let mut next = || { seed = seed.wrapping_mul(1664525).wrapping_add(1013904223); seed >> 16 };

    for &border in &[false, true] {
        let mut g = Grid::new(23, 17, 0.5, 0.0, 0.0);
        for r in 0..17 {
            for c in 0..23 {
                let v = match next() % 10 { 0 => OCCUPIED, 1 => UNKNOWN, _ => FREE };
                g.set(c, r, v);
            }
        }
        for &kind in &[OCCUPIED, UNKNOWN] {
            let a = g.distance_field_slow_test(kind, 3.0, border);
            let b = g.distance_field(kind, 3.0, border);
            for i in 0..a.len() {
                assert!((a[i] - b[i]).abs() < 1e-4,
                    "cell {i} kind {kind} border {border}: {} vs {}", a[i], b[i]);
            }
        }
    }
}
}
