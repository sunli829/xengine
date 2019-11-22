use crate::{Multiply, Real, Rotation, Transform, Vector2};

#[derive(Debug, Copy, Clone)]
pub struct Sweep<T> {
    pub local_center: Vector2<T>,
    pub c0: Vector2<T>,
    pub c: Vector2<T>,
    pub a0: T,
    pub a: T,
    pub alpha0: T,
}

impl<T: Real> Sweep<T> {
    pub fn transform(&self, beta: T) -> Transform<T> {
        let mut p = self.c0 * (T::one() - beta) + self.c * beta;
        let angle = (T::one() - beta) * self.a0 + beta * self.a;
        let q = Rotation::new(angle);
        p -= q.multiply(self.local_center);
        Transform { p, q }
    }
}
