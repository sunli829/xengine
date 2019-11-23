use crate::{Real, Vector2};

#[derive(Debug, Clone, Copy, Default)]
pub struct AABB<T> {
    pub lower_bound: Vector2<T>,
    pub upper_bound: Vector2<T>,
}

impl<T: Real> AABB<T> {
    pub fn new(lower_bound: Vector2<T>, upper_bound: Vector2<T>) -> AABB<T> {
        AABB {
            lower_bound,
            upper_bound,
        }
    }

    pub fn is_valid(&self) -> bool {
        let d = self.upper_bound - self.lower_bound;
        self.lower_bound.is_valid()
            && self.upper_bound.is_valid()
            && d.x > T::zero()
            && d.y > T::zero()
    }

    pub fn center(&self) -> Vector2<T> {
        (self.lower_bound + self.upper_bound) * T::half()
    }

    pub fn extents(&self) -> Vector2<T> {
        (self.upper_bound - self.lower_bound) * T::half()
    }

    pub fn perimeter(&self) -> T {
        let wx = self.upper_bound.x - self.lower_bound.x;
        let wy = self.upper_bound.y - self.lower_bound.y;
        wx + wx + wy + wy
    }

    pub fn combine(self, aabb: &AABB<T>) -> AABB<T> {
        AABB {
            lower_bound: self.lower_bound.min(aabb.lower_bound),
            upper_bound: self.upper_bound.max(aabb.upper_bound),
        }
    }

    pub fn contains(&self, aabb: &AABB<T>) -> bool {
        let mut result = true;
        result = result && self.lower_bound.x <= aabb.lower_bound.x;
        result = result && self.lower_bound.y <= aabb.lower_bound.y;
        result = result && aabb.upper_bound.x <= self.upper_bound.x;
        result = result && aabb.upper_bound.y <= self.upper_bound.y;
        result
    }

    pub fn is_overlap(&self, aabb: &AABB<T>) -> bool {
        let d1 = aabb.lower_bound - self.upper_bound;
        let d2 = self.lower_bound - aabb.upper_bound;
        if d1.x > T::zero() || d1.y > T::zero() {
            return false;
        }
        if d2.x > T::zero() || d2.y > T::zero() {
            return false;
        }
        true
    }
}
