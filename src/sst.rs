use rand::{Rng, RngExt};
use crate::graph::{Node, Tree};
use crate::grid::DistanceField;
use crate::params::VehicleParams;
use crate::propogate::{propagate, Control};
use crate::sampler::{random_control, random_state, SamplerParams};
use crate::state::State;
use crate::state_distance::DistanceWeights;
use crate::witness::WitnessMap;

//Are we sampling the entire map or just a disk
#[derive(Debug, Clone, Copy)]
pub enum SampleRegion {
    WholeMap,
    Disk { center: (f32, f32), radius: f32 },
}
pub struct Planner {
    pub tree: Tree,
    pub witnesses: WitnessMap,
    pub params: VehicleParams,
    pub sampler: SamplerParams,
    pub goal: (f32, f32),
    pub goal_radius: f32,   // epsilon ball, we are almsot there
    pub goal_bias: f32,     // We want to make it a bit more likely that the sample is literally in the target
    pub region: SampleRegion
}

impl Planner {
    pub fn new(start: State, goal: (f32, f32), params: VehicleParams) -> Self {
        let root = Node {
            state: start,
            parent: None,
            cost: 0.0,//Starting state
            time: 0.0,
            control: Control { accel: 0.0, steer_rate: 0.0, frames: 1 },//lowkirk not useful 
            active: true,
        };
        let tree = Tree::new(root);
        let weights = DistanceWeights {
            rotWeight: 0.5,
            positionWeight: 1.0,
            speedWeight: 0.5,
            steeringWeight: 0.1,
        };
        let delta_s = 0.3;    // Read sst paper 
        let delta_bn = 0.6;   // read sst

        let mut witnesses = WitnessMap::new(delta_bn / weights.positionWeight, delta_s, delta_bn, weights);
        witnesses.registerRoot(&tree);
        Planner {
            tree,
            witnesses,
            params,
            sampler: SamplerParams::default(),
            goal,
            goal_radius: 0.5,
            goal_bias: 0.1,
            region: SampleRegion::WholeMap, 

        }
    }
    // a single iteration of SST which returns true if the tree grew
    pub fn iterate<R: Rng>(&mut self, sdf: &DistanceField, rng: &mut R) -> bool {
        let sample = if rng.random::<f32>() < self.goal_bias {
            State { x: self.goal.0, y: self.goal.1, heading: 0.0, speed: 0.0, steering: 0.0 }//Only x and y matter for this sample
        } else {
            let (center, radius) = match self.region {
                SampleRegion::WholeMap => map_disk(sdf),
                SampleRegion::Disk { center, radius } => (center, radius),
            };
            random_state(center, radius, sdf, &self.params, rng)
        };

        let parent = self.witnesses.best_near(&self.tree, &sample)
            .unwrap_or_else(|| self.witnesses.nearest_active(&sample).unwrap());
        let chosenNode = self.tree.nodes[parent];

        let warm = chosenNode.state.speed.abs() < 0.1;//If nearly stopped we wanna accelerate
        let ctrl = random_control(&chosenNode.state, &self.params, &self.sampler, rng, warm);

        // check if safe path
        let edge = match propagate(&chosenNode.state, &ctrl, sdf, &self.params) {
            Some(e) => e,
            None => return false,
        };

        // Create the new node from the resul
        let node = Node {
            state: edge.end,
            parent: Some(parent),
            cost: chosenNode.cost + edge.cost,
            time: chosenNode.time + edge.time,
            control: ctrl,
            active: true,
        };
        //Try to add to witness map
         self.witnesses.try_add(&mut self.tree, node).is_some()
    }

    // Cheapest active node within goal_radius of the goal.
    pub fn best_goal_node(&self) -> Option<usize> {
        let mut best: Option<usize> = None;
        for (i, n) in self.tree.nodes.iter().enumerate() {
            if !n.active { continue; }
            let dx = n.state.x - self.goal.0;
            let dy = n.state.y - self.goal.1;
            if (dx * dx + dy * dy).sqrt() > self.goal_radius { continue; }
            let better = match best {
                None => true,
                Some(b) => n.cost < self.tree.nodes[b].cost,
            };
            if better { best = Some(i); }
        }
        best
}

    // Node indices from the root to `idx`.
    pub fn extract_path(&self, idx: usize) -> Vec<usize> {
        let mut path = self.tree.path_to_root(idx);
        path.reverse();
        path
    }

    pub fn focus(&mut self, center: (f32, f32), radius: f32) {
        self.region = SampleRegion::Disk { center, radius };
    }

    pub fn sample_whole_map(&mut self) {
        self.region = SampleRegion::WholeMap;
    }


    
}

//The most brain dead way to make sure the disk covers entire map
fn map_disk(sdf: &DistanceField) -> ((f32, f32), f32) {
    let w = sdf.width as f32 * sdf.resolution;
    let h = sdf.height as f32 * sdf.resolution;
    ((sdf.origin_x + w / 2.0, sdf.origin_y + h / 2.0), (w * w + h * h).sqrt() / 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{Grid, FREE, OCCUPIED};
    use rand::rngs::StdRng;
    use rand::SeedableRng;

    // 20 m x 20 m, 0.1 m cells, all free (the map border still counts as a wall)
    fn open_grid() -> Grid {
        let mut g = Grid::new(200, 200, 0.1, 0.0, 0.0);
        for r in 0..200 {
            for c in 0..200 {
                g.set(c, r, FREE);
            }
        }
        g
    }

    fn start() -> State {
        State { x: 2.0, y: 10.0, heading: 0.0, speed: 0.0, steering: 0.0 }
    }

    fn planner() -> Planner {
        Planner::new(start(), (17.0, 10.0), VehicleParams::default())
    }

    fn run(p: &mut Planner, sdf: &DistanceField, iters: usize, seed: u64) {
        let mut rng = StdRng::seed_from_u64(seed);
        for _ in 0..iters {
            p.iterate(sdf, &mut rng);
        }
    }

    #[test]
    fn new_planner_has_only_the_root() {
        let p = planner();
        assert_eq!(p.tree.len(), 1);
        assert_eq!(p.tree.nodes[0].parent, None);
        assert_eq!(p.witnesses.witnesses.len(), 1);
    }

    #[test]
    fn no_goal_node_before_any_search() {
        assert_eq!(planner().best_goal_node(), None);
    }

    #[test]
    fn iterate_grows_the_tree_on_an_open_map() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        run(&mut p, &sdf, 500, 1);
        assert!(p.tree.len() > 1, "tree never grew");
    }

    #[test]
    fn iterate_returns_true_exactly_when_the_tree_grows() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        let mut rng = StdRng::seed_from_u64(2);
        for _ in 0..300 {
            let before = p.tree.len();
            let grew = p.iterate(&sdf, &mut rng);
            assert_eq!(grew, p.tree.len() == before + 1);
        }
    }

    #[test]
    fn every_node_is_collision_free() {
        let mut g = open_grid();
        for r in 60..140 {
            g.set(100, r, OCCUPIED);
        }
        let sdf = g.to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        run(&mut p, &sdf, 2000, 3);
        for (i, n) in p.tree.nodes.iter().enumerate() {
            assert!(
                sdf.pose_clear(&p.params, n.state.x, n.state.y, n.state.heading).is_some(),
                "node {i} at ({}, {}) collides", n.state.x, n.state.y
            );
        }
    }

    #[test]
    fn parents_come_before_children_and_costs_never_decrease() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        run(&mut p, &sdf, 1000, 4);
        for (i, n) in p.tree.nodes.iter().enumerate().skip(1) {
            let parent = n.parent.expect("non-root node without a parent");
            assert!(parent < i, "node {i} has parent {parent}");
            assert!(n.cost >= p.tree.nodes[parent].cost);
            assert!(n.time > p.tree.nodes[parent].time);
        }
    }

    #[test]
    fn same_seed_gives_the_same_tree() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut a = planner();
        let mut b = planner();
        run(&mut a, &sdf, 500, 5);
        run(&mut b, &sdf, 500, 5);
        assert_eq!(a.tree.len(), b.tree.len());
        assert_eq!(a.tree.nodes.last().unwrap().state, b.tree.nodes.last().unwrap().state);
    }

    #[test]
    fn finds_the_goal_on_an_open_map() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        let mut rng = StdRng::seed_from_u64(6);
        for _ in 0..30000 {
            p.iterate(&sdf, &mut rng);
            if p.best_goal_node().is_some() {
                break;
            }
        }
        let goal = p.best_goal_node().expect("goal not reached in 30000 iterations");
        let n = p.tree.nodes[goal];
        assert!(((n.state.x - 17.0).powi(2) + (n.state.y - 10.0).powi(2)).sqrt() <= p.goal_radius);
        println!("reached goal with {} nodes, cost {}", p.tree.len(), n.cost);
    }

    #[test]
    fn extract_path_runs_from_the_root_to_the_node() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        run(&mut p, &sdf, 500, 7);
        let last = p.tree.len() - 1;
        let path = p.extract_path(last);
        assert_eq!(path[0], 0);
        assert_eq!(*path.last().unwrap(), last);
        for pair in path.windows(2) {
            assert_eq!(p.tree.nodes[pair[1]].parent, Some(pair[0]));
        }
    }

    #[test]
    fn finds_a_way_around_a_wall() {
        // wall at x = 10 m from y = 6 to 14: it blocks the straight line but not the detour
        let mut g = open_grid();
        for r in 60..140 {
            g.set(100, r, OCCUPIED);
        }
        let sdf = g.to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        let mut rng = StdRng::seed_from_u64(8);
        for _ in 0..100000 {
            p.iterate(&sdf, &mut rng);
            if p.best_goal_node().is_some() {
                break;
            }
        }
        let goal = p.best_goal_node().expect("no path around the wall in 100000 iterations");
        // the path must pass the wall's x without ever being inside its y range
        let path = p.extract_path(goal);
        for &i in &path {
            let s = p.tree.nodes[i].state;
            if (s.x - 10.0).abs() < 0.3 {
                assert!(s.y < 6.0 || s.y > 14.0, "path went through the wall at y = {}", s.y);
            }
        }
    }

    #[test]
    fn focus_and_whole_map_switch_the_region() {
        let mut p = planner();
        p.focus((5.0, 5.0), 2.0);
        assert!(matches!(p.region, SampleRegion::Disk { .. }));
        p.sample_whole_map();
        assert!(matches!(p.region, SampleRegion::WholeMap));
    }

    #[test]
    fn a_focused_planner_still_grows() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let mut p = planner();
        p.focus((6.0, 10.0), 3.0);
        run(&mut p, &sdf, 500, 9);
        assert!(p.tree.len() > 1);
    }

    #[test]
    fn map_disk_covers_the_map() {
        let sdf = open_grid().to_distance_field(OCCUPIED, 2.0, true);
        let ((cx, cy), r) = map_disk(&sdf);
        assert!((cx - 10.0).abs() < 1e-4 && (cy - 10.0).abs() < 1e-4);
        // every corner is inside
        for (x, y) in [(0.0, 0.0), (20.0, 0.0), (0.0, 20.0), (20.0, 20.0)] {
            assert!(((x - cx).powi(2) + (y - cy as f32).powi(2)).sqrt() <= r + 1e-3);
        }
    }
}
