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
}
