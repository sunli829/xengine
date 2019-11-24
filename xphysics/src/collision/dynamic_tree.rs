use crate::{settings, RayCastInput};
use slab::Slab;
use xmath::{CrossTrait, DotTrait, Real, Vector2, AABB};

const QUERY_STACK_INIT_SIZE: usize = 256;

struct Node<T, D = ()> {
    aabb: AABB<T>,
    data: Option<D>,
    link: Option<usize>,
    child1: Option<usize>,
    child2: Option<usize>,
    height: i32,
}

impl<T: Real, D> Node<T, D> {
    fn is_leaf(&self) -> bool {
        self.child1.is_none()
    }
}

pub struct DynamicTree<T, D> {
    root: Option<usize>,
    nodes: Slab<Node<T, D>>,
}

impl<T: Real, D> DynamicTree<T, D> {
    pub fn new() -> DynamicTree<T, D> {
        DynamicTree {
            root: None,
            nodes: Default::default(),
        }
    }

    fn allocate_node(&mut self, aabb: AABB<T>, data: Option<D>, height: i32) -> usize {
        self.nodes.insert(Node {
            aabb,
            data,
            link: None,
            child1: None,
            child2: None,
            height,
        })
    }

    fn balance(&mut self, a: usize) -> usize {
        if self.nodes[a].is_leaf() || self.nodes[a].height < 2 {
            return a;
        }

        let b = self.nodes[a].child1.unwrap();
        let c = self.nodes[a].child2.unwrap();
        let balance = self.nodes[c].height - self.nodes[b].height;

        if balance > 1 {
            let f = self.nodes[c].child1.unwrap();
            let g = self.nodes[c].child2.unwrap();

            self.nodes[c].child1 = Some(a);
            self.nodes[c].link = self.nodes[a].link;
            self.nodes[a].link = Some(c);

            if let Some(parent) = self.nodes[c].link {
                if self.nodes[parent].child1 == Some(a) {
                    self.nodes[parent].child1 = Some(c);
                } else {
                    self.nodes[parent].child2 = Some(c);
                }
            } else {
                self.root = Some(c);
            }

            if self.nodes[f].height > self.nodes[g].height {
                self.nodes[c].child2 = Some(f);
                self.nodes[a].child2 = Some(g);
                self.nodes[g].link = Some(a);

                self.nodes[a].aabb = self.nodes[b].aabb.combine(&self.nodes[g].aabb);
                self.nodes[c].aabb = self.nodes[a].aabb.combine(&self.nodes[f].aabb);

                self.nodes[a].height = 1 + self.nodes[b].height.max(self.nodes[g].height);
                self.nodes[c].height = 1 + self.nodes[a].height.max(self.nodes[f].height);
            } else {
                self.nodes[c].child2 = Some(g);
                self.nodes[a].child2 = Some(f);
                self.nodes[f].link = Some(a);

                self.nodes[a].aabb = self.nodes[b].aabb.combine(&self.nodes[f].aabb);
                self.nodes[c].aabb = self.nodes[a].aabb.combine(&self.nodes[g].aabb);

                self.nodes[a].height = 1 + self.nodes[b].height.max(self.nodes[f].height);
                self.nodes[c].height = 1 + self.nodes[a].height.max(self.nodes[g].height);
            }

            return c;
        }

        if balance < -1 {
            let d = self.nodes[b].child1.unwrap();
            let e = self.nodes[b].child2.unwrap();

            self.nodes[b].child1 = Some(a);
            self.nodes[b].link = self.nodes[a].link;
            self.nodes[a].link = Some(b);

            if let Some(parent) = self.nodes[b].link {
                if self.nodes[parent].child1 == Some(a) {
                    self.nodes[parent].child1 = Some(b);
                } else {
                    self.nodes[parent].child2 = Some(b);
                }
            } else {
                self.root = Some(b);
            }

            if self.nodes[d].height > self.nodes[e].height {
                self.nodes[b].child2 = Some(d);
                self.nodes[a].child1 = Some(e);
                self.nodes[e].link = Some(a);

                self.nodes[a].aabb = self.nodes[c].aabb.combine(&self.nodes[e].aabb);
                self.nodes[b].aabb = self.nodes[a].aabb.combine(&self.nodes[d].aabb);

                self.nodes[a].height = 1 + self.nodes[c].height.max(self.nodes[e].height);
                self.nodes[b].height = 1 + self.nodes[a].height.max(self.nodes[d].height);
            } else {
                self.nodes[b].child2 = Some(e);
                self.nodes[a].child1 = Some(d);
                self.nodes[d].link = Some(a);

                self.nodes[a].aabb = self.nodes[c].aabb.combine(&self.nodes[d].aabb);
                self.nodes[b].aabb = self.nodes[a].aabb.combine(&self.nodes[e].aabb);

                self.nodes[a].height = 1 + self.nodes[c].height.max(self.nodes[d].height);
                self.nodes[b].height = 1 + self.nodes[a].height.max(self.nodes[e].height);
            }

            return b;
        }

        a
    }

    fn insert_leaf(&mut self, leaf: usize) {
        if self.root.is_none() {
            self.root = Some(leaf);
            self.nodes[leaf].link = None;
            return;
        }

        let leaf_aabb = self.nodes[leaf].aabb;
        let mut index = self.root.unwrap();

        while !self.nodes[index].is_leaf() {
            let index_node = &self.nodes[index];
            let child1 = index_node.child1.unwrap();
            let child2 = index_node.child2.unwrap();
            let area = index_node.aabb.perimeter();

            let combined_aabb = index_node.aabb.combine(&leaf_aabb);
            let combined_area = combined_aabb.perimeter();

            let cost = combined_area + combined_area;
            let inheritance_cost = (combined_area - area) + (combined_area - area);

            let child1_node = &self.nodes[child1];
            let cost1 = if child1_node.is_leaf() {
                leaf_aabb.combine(&child1_node.aabb).perimeter() + inheritance_cost
            } else {
                let aabb = leaf_aabb.combine(&child1_node.aabb);
                let old_area = child1_node.aabb.perimeter();
                let new_area = aabb.perimeter();
                (new_area - old_area) + inheritance_cost
            };

            let child2_node = &self.nodes[child1];
            let cost2 = if child2_node.is_leaf() {
                leaf_aabb.combine(&child2_node.aabb).perimeter() + inheritance_cost
            } else {
                let aabb = leaf_aabb.combine(&child2_node.aabb);
                let old_area = child2_node.aabb.perimeter();
                let new_area = aabb.perimeter();
                (new_area - old_area) + inheritance_cost
            };

            if cost < cost1 && cost < cost2 {
                break;
            }

            index = if cost1 < cost2 { child1 } else { child2 }
        }

        let sibling = index;
        let sibling_node = &self.nodes[sibling];
        let old_parent = sibling_node.link;
        let sibling_aabb = sibling_node.aabb;
        let sibling_height = sibling_node.height;
        let new_parent =
            self.allocate_node(leaf_aabb.combine(&sibling_aabb), None, sibling_height + 1);
        self.nodes[new_parent].link = old_parent;

        if let Some(old_parent) = old_parent {
            let old_parent_node = &mut self.nodes[old_parent];
            if old_parent_node.child1 == Some(sibling) {
                old_parent_node.child1 = Some(new_parent);
            } else {
                old_parent_node.child2 = Some(new_parent);
            }

            let new_parent_node = &mut self.nodes[new_parent];
            new_parent_node.child1 = Some(sibling);
            new_parent_node.child2 = Some(leaf);
            self.nodes[sibling].link = Some(new_parent);
            self.nodes[leaf].link = Some(new_parent);
        } else {
            let new_parent_node = &mut self.nodes[new_parent];
            new_parent_node.child1 = Some(sibling);
            new_parent_node.child2 = Some(leaf);
            self.nodes[sibling].link = Some(new_parent);
            self.nodes[leaf].link = Some(new_parent);
            self.root = Some(new_parent);
        }

        let mut index = self.nodes[leaf].link;
        while let Some(current_index) = index {
            let current_index = self.balance(current_index);
            let child1 = self.nodes[current_index].child1.unwrap();
            let child2 = self.nodes[current_index].child2.unwrap();
            self.nodes[current_index].height =
                1 + self.nodes[child1].height.max(self.nodes[child2].height);
            self.nodes[current_index].aabb =
                self.nodes[child1].aabb.combine(&self.nodes[child2].aabb);
            index = self.nodes[current_index].link;
        }
    }

    pub fn create_proxy(&mut self, aabb: AABB<T>, data: D) -> usize {
        let id = self.allocate_node(
            AABB {
                lower_bound: aabb.lower_bound - settings::aabb_extension::<T>(),
                upper_bound: aabb.upper_bound + settings::aabb_extension::<T>(),
            },
            Some(data),
            0,
        );
        self.insert_leaf(id);
        id
    }

    fn remove_leaf(&mut self, leaf: usize) {
        if Some(leaf) == self.root {
            self.root = None;
            return;
        }

        let parent = self.nodes[leaf].link.unwrap();
        let grand_parent = self.nodes[parent].link;
        let sibling = if self.nodes[parent].child1 == Some(leaf) {
            self.nodes[parent].child2
        } else {
            self.nodes[parent].child1
        };

        if let Some(grand_parent) = grand_parent {
            if self.nodes[grand_parent].child1 == Some(parent) {
                self.nodes[grand_parent].child1 = sibling;
            } else {
                self.nodes[grand_parent].child2 = sibling;
            }
            self.nodes[grand_parent].link = sibling;
            self.nodes.remove(parent);

            let mut index = Some(grand_parent);
            while let Some(current_index) = index {
                let current_index = self.balance(current_index);
                let child1 = self.nodes[current_index].child1.unwrap();
                let child2 = self.nodes[current_index].child2.unwrap();
                self.nodes[current_index].aabb =
                    self.nodes[child1].aabb.combine(&self.nodes[child2].aabb);
                self.nodes[current_index].height =
                    1 + self.nodes[child1].height.max(self.nodes[child2].height);

                index = self.nodes[current_index].link;
            }
        } else {
            self.root = sibling;
            self.nodes[sibling.unwrap()].link = None;
            self.nodes.remove(parent);
        }
    }

    pub fn remove_proxy(&mut self, id: usize) {
        self.remove_leaf(id);
        self.nodes.remove(id);
    }

    pub fn move_proxy(&mut self, id: usize, aabb: AABB<T>, displacement: Vector2<T>) -> bool {
        if self.nodes[id].aabb.contains(&aabb) {
            return false;
        }

        self.remove_leaf(id);

        let mut b = AABB {
            lower_bound: aabb.lower_bound - settings::aabb_extension::<T>(),
            upper_bound: aabb.lower_bound + settings::aabb_extension::<T>(),
        };
        let d = displacement * settings::aabb_multiplier::<T>();

        if d.x < T::zero() {
            b.lower_bound.x += d.x;
        } else {
            b.lower_bound.x += d.x;
        }

        if d.y < T::zero() {
            b.lower_bound.y += d.y;
        } else {
            b.lower_bound.y += d.y;
        }

        self.nodes[id].aabb = b;
        self.insert_leaf(id);
        true
    }

    pub fn get_fat_aabb(&self, id: usize) -> &AABB<T> {
        &self.nodes[id].aabb
    }

    pub fn get_data(&self, proxy_id: usize) -> Option<&D> {
        self.nodes.get(proxy_id).and_then(|d| d.data.as_ref())
    }

    pub fn get_data_mut(&mut self, proxy_id: usize) -> Option<&mut D> {
        self.nodes.get_mut(proxy_id).and_then(|d| d.data.as_mut())
    }

    pub fn query(&self, aabb: AABB<T>) -> QueryIter<T, D> {
        let mut stack = Vec::with_capacity(QUERY_STACK_INIT_SIZE);
        if let Some(root) = self.root {
            stack.push(root);
        }
        QueryIter {
            tree: self,
            stack,
            aabb,
        }
    }

    pub fn ray_cast(&self, input: RayCastInput<T>) -> RayCastIter<T, D> {
        let r = (input.p2 - input.p1).normalize();
        assert!(r.length_squared() > T::zero());

        let v = T::one().cross(r);
        let v_abs = v.abs();

        let segment_aabb = {
            let t = input.p1 + (input.p2 - input.p1) * input.max_fraction;
            AABB {
                lower_bound: input.p1.min(t),
                upper_bound: input.p1.max(t),
            }
        };

        let mut stack = Vec::with_capacity(QUERY_STACK_INIT_SIZE);
        if let Some(root) = self.root {
            stack.push(root);
        }

        RayCastIter {
            tree: self,
            stack,
            segment_aabb,
            v,
            v_abs,
            p1: input.p1,
            p2: input.p2,
        }
    }

    pub fn shift_origin(&mut self, new_origin: Vector2<T>) {
        for node in self.nodes.iter_mut().map(|item| item.1) {
            node.aabb.lower_bound -= new_origin;
            node.aabb.upper_bound -= new_origin;
        }
    }
}

pub struct QueryIter<'a, T, D> {
    tree: &'a DynamicTree<T, D>,
    stack: Vec<usize>,
    aabb: AABB<T>,
}

impl<'a, T: Real, D> Iterator for QueryIter<'a, T, D> {
    type Item = (usize, &'a AABB<T>, &'a D);

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(idx) = self.stack.pop() {
            let node = &self.tree.nodes[idx];
            if node.aabb.is_overlap(&self.aabb) {
                if node.is_leaf() {
                    return Some((idx, &node.aabb, node.data.as_ref().unwrap()));
                }
                self.stack.push(node.child1.unwrap());
                self.stack.push(node.child2.unwrap());
            }
        }
        None
    }
}

pub struct RayCastIter<'a, T, D> {
    tree: &'a DynamicTree<T, D>,
    stack: Vec<usize>,
    segment_aabb: AABB<T>,
    v: Vector2<T>,
    v_abs: Vector2<T>,
    p1: Vector2<T>,
    p2: Vector2<T>,
}

impl<'a, T: Real, D> RayCastIter<'a, T, D> {
    pub fn set_max_fraction(&mut self, value: T) {
        let t = self.p1 + (self.p2 - self.p1) * value;
        self.segment_aabb.lower_bound = self.p1.min(t);
        self.segment_aabb.upper_bound = self.p1.max(t);
    }
}

impl<'a, T: Real, D> Iterator for RayCastIter<'a, T, D> {
    type Item = (usize, &'a AABB<T>, &'a D);

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(idx) = self.stack.pop() {
            let node = &self.tree.nodes[idx];

            if !node.aabb.is_overlap(&self.segment_aabb) {
                continue;
            }

            let c = node.aabb.center();
            let h = node.aabb.extents();
            let separation = self.v.dot(self.p1 - c).abs() - self.v_abs.dot(h);
            if separation > T::zero() {
                continue;
            }

            if node.is_leaf() {
                return Some((idx, &node.aabb, node.data.as_ref().unwrap()));
            }

            self.stack.push(node.child1.unwrap());
            self.stack.push(node.child2.unwrap());
        }
        None
    }
}
