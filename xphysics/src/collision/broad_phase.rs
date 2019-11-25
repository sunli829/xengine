use crate::collision::dynamic_tree::DynamicTree;
use xmath::{Real, Vector2, AABB};

#[derive(Ord, PartialOrd, Eq, PartialEq)]
struct Pair {
    proxy_id_a: usize,
    proxy_id_b: usize,
}

pub(crate) struct BroadPhase<T, D> {
    tree: DynamicTree<T, D>,
    move_buffer: Vec<Option<usize>>,
    pair_buffer: Vec<Pair>,
}

impl<T: Real, D> BroadPhase<T, D> {
    pub fn create_proxy(&mut self, aabb: AABB<T>, data: D) -> usize {
        let proxy_id = self.tree.create_proxy(aabb, data);
        self.buffer_move(proxy_id);
        proxy_id
    }

    pub fn destroy_proxy(&mut self, proxy_id: usize) {
        self.unbuffer_move(proxy_id);
        self.tree.remove_proxy(proxy_id);
    }

    pub fn move_proxy(&mut self, proxy_id: usize, aabb: AABB<T>, displacement: Vector2<T>) {
        if self.tree.move_proxy(proxy_id, aabb, displacement) {
            self.buffer_move(proxy_id);
        }
    }

    pub fn touch_proxy(&mut self, proxy_id: usize) {
        self.buffer_move(proxy_id);
    }

    pub fn get_fat_aabb(&self, proxy_id: usize) -> &AABB<T> {
        self.tree.get_fat_aabb(proxy_id)
    }

    pub fn get_data(&self, proxy_id: usize) -> Option<&D> {
        self.tree.get_data(proxy_id)
    }

    pub fn get_data_mut(&mut self, proxy_id: usize) -> Option<&mut D> {
        self.tree.get_data_mut(proxy_id)
    }

    pub fn test_overlap(&self, proxy_id_a: usize, proxy_id_b: usize) -> bool {
        let aabb_a = self.tree.get_fat_aabb(proxy_id_a);
        let aabb_b = self.tree.get_fat_aabb(proxy_id_b);
        aabb_a.is_overlap(aabb_b)
    }

    pub fn update_pairs<F: Fn(&D, &D)>(&mut self, cb: F) {
        self.pair_buffer.clear();

        for id in &self.move_buffer {
            if let Some(id) = id {
                let fat_aabb = self.tree.get_fat_aabb(*id);
                for other_id in self
                    .tree
                    .query(*fat_aabb)
                    .map(|item| item.0)
                    .filter(|other_id| *other_id != *id)
                {
                    self.pair_buffer.push(Pair {
                        proxy_id_a: other_id.min(*id),
                        proxy_id_b: other_id.max(*id),
                    });
                }
            }
        }

        self.move_buffer.clear();
        self.pair_buffer.sort();

        let mut i = 0;
        while i < self.pair_buffer.len() {
            let primary_pair = &self.pair_buffer[i];
            let data_a = self.tree.get_data(primary_pair.proxy_id_a).unwrap();
            let data_b = self.tree.get_data(primary_pair.proxy_id_b).unwrap();
            cb(data_a, data_b);
            i += 1;

            while i < self.pair_buffer.len() {
                let pair = &self.pair_buffer[i];
                if pair.proxy_id_a != primary_pair.proxy_id_a
                    || pair.proxy_id_b != primary_pair.proxy_id_b
                {
                    break;
                }
                i += 1;
            }
        }
    }

    pub fn shift_origin(&mut self, new_origin: Vector2<T>) {
        self.tree.shift_origin(new_origin);
    }

    fn buffer_move(&mut self, proxy_id: usize) {
        self.move_buffer.push(Some(proxy_id));
    }

    fn unbuffer_move(&mut self, proxy_id: usize) {
        for i in 0..self.move_buffer.len() {
            if self.move_buffer[i] == Some(proxy_id) {
                self.move_buffer[i] = None;
            }
        }
    }
}
