use std::borrow::Cow;
use xmath::{DotTrait, Real, Vector2};

pub struct DistanceProxy<'a, T: Real> {
    pub vertices: Cow<'a, [Vector2<T>]>,
    pub radius: T,
}

impl<'a, T: Real> DistanceProxy<'a, T> {
    pub fn get_support(&self, d: &Vector2<T>) -> usize {
        let mut best_index = 0;
        let mut best_value = self.vertices[0].dot(*d);
        for i in 1..self.vertices.len() {
            let value = self.vertices[i].dot(*d);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
        }
        best_index
    }

    pub fn get_support_vertex(&self, d: &Vector2<T>) -> &Vector2<T> {
        &self.vertices[self.get_support(d)]
    }
}
