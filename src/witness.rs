use std::collections::HashMap;
use crate::state::State;
use crate::graph::{Node, Tree};
use crate::state_distance::{state_distance, DistanceWeights};

pub struct Witness{
    pub state: State,
    pub rep: usize,
}



pub struct WitnessMap{
    pub witnesses: Vec<Witness>,
    hash: SpatialHash,
    pub delta_s:f32,
    pub delta_bn:f32,
    pub weights: DistanceWeights,
}

pub struct SpatialHash{//Satial hashmap to get witness indices
    cell_size: f32,
    cells: HashMap<(i32, i32), Vec<usize>>,
}


impl SpatialHash{
    pub fn new(cell_size: f32) -> Self{
        SpatialHash{cell_size, cells: HashMap::new()}
    }


   fn cell_of(&self, x: f32, y: f32) -> (i32, i32) {
        ((x / self.cell_size).floor() as i32, (y / self.cell_size).floor() as i32)
    }

    pub fn insert(&mut self, id: usize, x: f32, y: f32) {
        let key = self.cell_of(x, y);
        self.cells.entry(key).or_default().push(id);//Mamny nodes in each state
    }

    pub fn nearby(&self, x: f32, y: f32, r: f32) -> Vec<usize> {
        //Get target cell
        let (cx, cy) = self.cell_of(x, y);
        //Finds how many cells in each dir we msut search
        let reach = (r / self.cell_size).ceil() as i32;
        let mut out = Vec::new();
        for dx in -reach..=reach {
            for dy in -reach..=reach {
                if let Some(ids) = self.cells.get(&(cx + dx, cy + dy)) {
                    out.extend_from_slice(ids);
                }
            }
        }
        out
    }

}

impl WitnessMap{
    pub fn new(cell_size: f32, delta_s:f32, delta_bn:f32, weights: DistanceWeights)-> Self{
        WitnessMap {
            witnesses: Vec::new(),
            hash: SpatialHash::new(cell_size),
            delta_s,
            delta_bn,
            weights,
        }
    }

    // returns the tree indez of the newly added node if allowed and none if not
    pub fn try_add(&mut self, tree: &mut Tree, node: Node) -> Option<usize>{
        let r = self.delta_s / self.weights.positionWeight;         // than delta_s must be within delta_s / positionWeight m or else the nodes can't possibly work


        let mut nearest: Option<(usize, f32)> = None;
        for id in self.hash.nearby(node.state.x, node.state.y, r) {
            let d = state_distance(&self.witnesses[id].state, &node.state, &self.weights);
            if d < self.delta_s && nearest.map_or(true, |(_, best_d)| d < best_d) {
                nearest = Some((id, d));
            }
        }

        match nearest {
            // If there are no nodes in range then this is the new rep
            None => {
                let idx = tree.add_node(node);
                let wid = self.witnesses.len();
                self.witnesses.push(Witness { state: node.state, rep: idx });
                self.hash.insert(wid, node.state.x, node.state.y);
                Some(idx)
            }
            // otherwise replace as rep only if cheaper
            Some((wid, _)) => {
                let old = self.witnesses[wid].rep;
                if node.cost < tree.nodes[old].cost {
                    tree.nodes[old].active = false;
                    let idx = tree.add_node(node);
                    self.witnesses[wid].rep = idx;
                    Some(idx)
                } else {
                    None
                }
            }
        }
    }
//Uses similar logic to nearby to check all near by nodes and return lowest cost one
    pub fn best_near(&self, tree: &Tree, sample: &State) -> Option<usize> {
        let r  = self.delta_bn/self.weights.positionWeight;//Again, this is the most physical distance we need to search
        let mut best: Option<usize> = None;

        for id in self.hash.nearby(sample.x, sample.y, r){
            let witness = &self.witnesses[id];
            if state_distance(&witness.state, sample, &self.weights)>= self.delta_bn{
                continue;
            }

            let better = match best{
                None => true,
                Some(b) => tree.nodes[witness.rep].cost < tree.nodes[b].cost,
            };

            if better{
                best = Some(witness.rep);
            }
        }
        best
    }
    
    pub fn nearest_active(&self, sample: &State) -> Option<usize> {
        let mut closest: Option<(usize, f32)> = None;
        for w in &self.witnesses {
            let d = state_distance(&w.state, sample, &self.weights);
            let closer = match closest {
                None => true,                       
                Some((_, bd)) => d < bd,            
            };
            if closer {
                closest = Some((w.rep, d));
            }

        }
        //Only return the index!
        closest.map(|(rep, _)| rep)
}



    pub fn registerRoot(&mut self, tree: &Tree){
        let state = tree.nodes[0].state;
        let wId = self.witnesses.len();
        self.witnesses.push(Witness{state, rep: 0});
        self.hash.insert(wId, state.x, state.y);
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_hash_finds_nothing() {
        let h = SpatialHash::new(1.0);
        assert!(h.nearby(0.0, 0.0, 5.0).is_empty());
    }

    #[test]
    fn finds_item_in_same_cell() {
        let mut h = SpatialHash::new(1.0);
        h.insert(7, 0.5, 0.5);
        assert_eq!(h.nearby(0.2, 0.9, 0.5), vec![7]);
    }

    #[test]
    fn finds_item_in_neighbouring_cell() {
        let mut h = SpatialHash::new(1.0);
        h.insert(1, 1.2, 0.5); // cell (1, 0)
        assert_eq!(h.nearby(0.9, 0.5, 0.5), vec![1]); // query sits in cell (0, 0)
    }

    #[test]
    fn excludes_far_items() {
        let mut h = SpatialHash::new(1.0);
        h.insert(1, 10.0, 10.0);
        assert!(h.nearby(0.0, 0.0, 1.0).is_empty());
    }

    #[test]
    fn negative_coordinates_use_floor() {
        // -0.3 belongs to cell -1, not cell 0. With truncation both would share cell 0.
        let mut h = SpatialHash::new(1.0);
        h.insert(1, -0.3, 0.5);
        h.insert(2, 0.3, 0.5);
        assert_eq!(h.cell_of(-0.3, 0.5), (-1, 0));
        assert_eq!(h.cell_of(0.3, 0.5), (0, 0));
        // radius smaller than a cell still reaches the neighbour (reach = 1)
        let mut found = h.nearby(0.1, 0.5, 0.5);
        found.sort();
        assert_eq!(found, vec![1, 2]);
    }

    #[test]
    fn many_items_in_one_cell_all_returned() {
        let mut h = SpatialHash::new(2.0);
        for id in 0..5 {
            h.insert(id, 0.1 * id as f32, 0.0);
        }
        let mut found = h.nearby(0.0, 0.0, 1.0);
        found.sort();
        assert_eq!(found, vec![0, 1, 2, 3, 4]);
    }

    #[test]
    fn larger_radius_reaches_more_cells() {
        let mut h = SpatialHash::new(1.0);
        h.insert(1, 3.5, 0.5); // 3 cells away
        assert!(h.nearby(0.5, 0.5, 1.0).is_empty());
        assert_eq!(h.nearby(0.5, 0.5, 3.0), vec![1]);
    }

    // ---- WitnessMap ----
    use crate::propogate::Control;

    fn node_at(x: f32, cost: f32) -> Node {
        Node {
            state: State { x, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.0 },
            parent: Some(0),
            cost,
            time: cost,
            control: Control { accel: 0.0, steer_rate: 0.0, frames: 1 },
            active: true,
        }
    }

    fn w() -> DistanceWeights {
        DistanceWeights { rotWeight: 1.0, positionWeight: 1.0, speedWeight: 1.0, steeringWeight: 1.0 }
    }

    // root at x = 0, delta_s = 1.0
    fn setup() -> (Tree, WitnessMap) {
        let mut root = node_at(0.0, 0.0);
        root.parent = None;
        let tree = Tree::new(root);
        let mut map = WitnessMap::new(1.0, 1.0, 2.0, w());
        map.registerRoot(&tree);
        (tree, map)
    }

    #[test]
    fn register_root_makes_one_witness_pointing_at_node_zero() {
        let (_, map) = setup();
        assert_eq!(map.witnesses.len(), 1);
        assert_eq!(map.witnesses[0].rep, 0);
    }

    #[test]
    fn far_node_creates_a_new_witness() {
        let (mut tree, mut map) = setup();
        let idx = map.try_add(&mut tree, node_at(5.0, 1.0));
        assert_eq!(idx, Some(1));
        assert_eq!(tree.len(), 2);
        assert_eq!(map.witnesses.len(), 2);
        assert_eq!(map.witnesses[1].rep, 1);
    }

    #[test]
    fn worse_node_near_the_root_is_rejected() {
        let (mut tree, mut map) = setup();
        // root has cost 0, so any positive cost in its ball loses
        assert_eq!(map.try_add(&mut tree, node_at(0.3, 1.0)), None);
        assert_eq!(tree.len(), 1);
        assert_eq!(map.witnesses.len(), 1);
        assert!(tree.nodes[0].active);
    }

    #[test]
    fn better_node_replaces_and_deactivates_old_rep() {
        let (mut tree, mut map) = setup();
        let a = map.try_add(&mut tree, node_at(5.0, 4.0)).unwrap();
        let b = map.try_add(&mut tree, node_at(5.2, 2.0)).unwrap(); // same ball, cheaper
        assert!(!tree.nodes[a].active, "old rep should be deactivated");
        assert!(tree.nodes[b].active);
        assert_eq!(map.witnesses.len(), 2); // root + one at x=5, no new witness
        assert_eq!(map.witnesses[1].rep, b);
    }

    #[test]
    fn equal_cost_does_not_replace() {
        let (mut tree, mut map) = setup();
        map.try_add(&mut tree, node_at(5.0, 2.0)).unwrap();
        assert_eq!(map.try_add(&mut tree, node_at(5.1, 2.0)), None);
    }

    #[test]
    fn two_witnesses_keep_separate_representatives() {
        let (mut tree, mut map) = setup();
        let a = map.try_add(&mut tree, node_at(5.0, 1.0)).unwrap();
        let b = map.try_add(&mut tree, node_at(9.0, 1.0)).unwrap();
        assert_ne!(a, b);
        // improving one ball leaves the other alone
        map.try_add(&mut tree, node_at(5.1, 0.5)).unwrap();
        assert!(!tree.nodes[a].active);
        assert!(tree.nodes[b].active);
    }

    #[test]
    fn nearest_witness_wins_when_two_overlap() {
        // witnesses at 0 (root) and 1.5 with delta_s = 1.0; a node at 1.2 is within 1.0 of 1.5 only
        let (mut tree, mut map) = setup();
        let a = map.try_add(&mut tree, node_at(1.5, 5.0)).unwrap();
        // cost 3 beats the rep at 1.5 (cost 5), but would lose to the root (cost 0) if matched to it
        let b = map.try_add(&mut tree, node_at(1.2, 3.0));
        assert!(b.is_some());
        assert!(!tree.nodes[a].active);
    }

    #[test]
    fn heading_difference_keeps_nodes_in_separate_balls() {
        // same position, headings far apart: state_distance exceeds delta_s, so a new witness
        let (mut tree, mut map) = setup();
        let mut n = node_at(0.0, 1.0);
        n.state.heading = 2.0;
        assert!(map.try_add(&mut tree, n).is_some());
        assert_eq!(map.witnesses.len(), 2);
    }

    // ---- best_near / nearest_active ----
    fn probe(x: f32) -> State {
        State { x, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.0 }
    }

    // delta_s = 0.5 keeps witnesses at 0, 2, 4 separate; delta_bn = 1.5
    fn spread() -> (Tree, WitnessMap, usize, usize) {
        let mut root = node_at(0.0, 0.0);
        root.parent = None;
        let mut tree = Tree::new(root);
        let mut map = WitnessMap::new(1.0, 0.5, 1.5, w());
        map.registerRoot(&tree);
        let a = map.try_add(&mut tree, node_at(2.0, 5.0)).unwrap();
        let b = map.try_add(&mut tree, node_at(4.0, 1.0)).unwrap();
        (tree, map, a, b)
    }

    #[test]
    fn best_near_is_none_when_nothing_in_range() {
        let (tree, map, _, _) = spread();
        assert_eq!(map.best_near(&tree, &probe(10.0)), None);
    }

    #[test]
    fn best_near_returns_the_only_candidate() {
        let (tree, map, a, _) = spread();
        // x = 2.0 is within 1.5 of the witness at 2.0, but 2.0 away from both others
        assert_eq!(map.best_near(&tree, &probe(2.0)), Some(a));
    }

    #[test]
    fn best_near_picks_the_cheapest_in_range() {
        let (tree, map, a, b) = spread();
        // x = 3.0 is 1.0 from both: costs are a = 5, b = 1
        assert_eq!(map.best_near(&tree, &probe(3.0)), Some(b));
        assert_ne!(map.best_near(&tree, &probe(3.0)), Some(a));
    }

    #[test]
    fn best_near_ignores_witnesses_outside_delta_bn() {
        let (tree, map, a, _) = spread();
        // b (cost 1) is at 4.0, 1.9 away from 2.1: out of range, so the costlier a wins
        assert_eq!(map.best_near(&tree, &probe(2.1)), Some(a));
    }

    #[test]
    fn best_near_follows_a_replaced_representative() {
        let (mut tree, mut map, a, _) = spread();
        let better = map.try_add(&mut tree, node_at(2.1, 0.5)).unwrap();
        assert_eq!(map.best_near(&tree, &probe(2.0)), Some(better));
        assert!(!tree.nodes[a].active);
    }

    #[test]
    fn nearest_active_is_the_root_on_a_fresh_map() {
        let mut root = node_at(0.0, 0.0);
        root.parent = None;
        let tree = Tree::new(root);
        let mut map = WitnessMap::new(1.0, 0.5, 1.5, w());
        map.registerRoot(&tree);
        assert_eq!(map.nearest_active(&probe(99.0)), Some(0));
    }

    #[test]
    fn nearest_active_is_none_without_witnesses() {
        let map = WitnessMap::new(1.0, 0.5, 1.5, w());
        assert_eq!(map.nearest_active(&probe(0.0)), None);
    }

    #[test]
    fn nearest_active_picks_the_closest_at_any_distance() {
        let (_, map, a, b) = spread();
        assert_eq!(map.nearest_active(&probe(1.8)), Some(a));
        assert_eq!(map.nearest_active(&probe(50.0)), Some(b));
        assert_eq!(map.nearest_active(&probe(-7.0)), Some(0));
    }

    #[test]
    fn nearest_active_ignores_cost() {
        // a is much more expensive than b, but nearer to the probe
        let (_, map, a, _) = spread();
        assert_eq!(map.nearest_active(&probe(2.4)), Some(a));
    }
}
