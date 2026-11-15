use crate::propogate::Control;
use crate::state::State;

#[derive(Debug, Clone, Copy)]
pub struct Node {
    pub state: State,
    pub parent: Option<usize>,   // index iof the parent in the tree vector
    pub cost: f32,               
    pub time: f32,
    pub control: Control,        // how to reach uiuf going from parent (sometimes not fully accurate)
    pub active: bool,           
}

pub struct Tree { pub nodes: Vec<Node> }


impl Tree{

    pub fn new(root:Node) -> Self{
        Tree{nodes: vec![root]}
    }

    pub fn len(&self)-> usize{
        self.nodes.len()
    }
//returns new node's index
    pub fn add_node(&mut self, node: Node)-> usize{
        self.nodes.push(node);
        self.nodes.len()-1
    }

    pub fn path_to_root(&self, mut i:usize) -> Vec<usize>{
        let mut path = vec![i];
        while let Some(parent) = self.nodes[i].parent{
            path.push(parent);
            i = parent;
        }

        path 
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn node(parent: Option<usize>, cost: f32) -> Node {
        Node {
            state: State { x: cost, y: 0.0, heading: 0.0, speed: 0.0, steering: 0.0 },
            parent,
            cost,
            time: cost,
            control: Control { accel: 0.0, steer_rate: 0.0, frames: 1 },
            active: true,
        }
    }

    #[test]
    fn new_tree_has_only_the_root() {
        let t = Tree::new(node(None, 0.0));
        assert_eq!(t.len(), 1);
        assert_eq!(t.nodes[0].parent, None);
    }

    #[test]
    fn add_node_returns_sequential_indices() {
        let mut t = Tree::new(node(None, 0.0));
        assert_eq!(t.add_node(node(Some(0), 1.0)), 1);
        assert_eq!(t.add_node(node(Some(1), 2.0)), 2);
        assert_eq!(t.add_node(node(Some(0), 3.0)), 3);
        assert_eq!(t.len(), 4);
    }

    #[test]
    fn added_nodes_keep_their_contents() {
        let mut t = Tree::new(node(None, 0.0));
        let i = t.add_node(node(Some(0), 7.5));
        assert_eq!(t.nodes[i].parent, Some(0));
        assert_eq!(t.nodes[i].cost, 7.5);
        assert!(t.nodes[i].active);
    }

    #[test]
    fn path_to_root_of_the_root_is_just_the_root() {
        let t = Tree::new(node(None, 0.0));
        assert_eq!(t.path_to_root(0), vec![0]);
    }

    #[test]
    fn path_to_root_follows_the_chain_upwards() {
        let mut t = Tree::new(node(None, 0.0));
        let a = t.add_node(node(Some(0), 1.0));
        let b = t.add_node(node(Some(a), 2.0));
        let c = t.add_node(node(Some(b), 3.0));
        assert_eq!(t.path_to_root(c), vec![3, 2, 1, 0]);
    }

    #[test]
    fn branches_have_independent_paths() {
        let mut t = Tree::new(node(None, 0.0));
        let a = t.add_node(node(Some(0), 1.0));
        let left = t.add_node(node(Some(a), 2.0));
        let right = t.add_node(node(Some(a), 2.5));
        let deep = t.add_node(node(Some(left), 3.0));
        assert_eq!(t.path_to_root(deep), vec![deep, left, a, 0]);
        assert_eq!(t.path_to_root(right), vec![right, a, 0]);
    }

    #[test]
    fn path_to_root_does_not_change_the_tree() {
        let mut t = Tree::new(node(None, 0.0));
        let a = t.add_node(node(Some(0), 1.0));
        let _ = t.path_to_root(a);
        assert_eq!(t.len(), 2);
        assert_eq!(t.nodes[a].parent, Some(0));
    }
}
