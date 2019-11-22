use crate::{Multiply, Real, TransposeMultiply, Vector2};
use std::ops::Mul;

#[derive(Debug, Copy, Clone)]
pub struct Rotation<T> {
    pub s: T,
    pub c: T,
}

impl<T: Real> Rotation<T> {
    pub fn new(angle: T) -> Rotation<T> {
        Rotation {
            s: angle.sin(),
            c: angle.cos(),
        }
    }

    pub fn identity() -> Rotation<T> {
        Rotation {
            s: T::zero(),
            c: T::one(),
        }
    }

    pub fn angle(&self) -> T {
        self.s.atan2(self.c)
    }

    pub fn x_axis(&self) -> Vector2<T> {
        Vector2::new(self.c, self.s)
    }

    pub fn y_axis(&self) -> Vector2<T> {
        Vector2::new(-self.s, self.c)
    }
}

impl<T: Real> Multiply<Rotation<T>> for Rotation<T> {
    type Output = Rotation<T>;

    fn multiply(self, rhs: Rotation<T>) -> Self::Output {
        let q = self;
        let r = rhs;
        let s = q.c * r.s - q.s * r.c;
        let c = q.c * r.c + q.s * r.s;
        Rotation { s, c }
    }
}

impl<T: Real> TransposeMultiply<Rotation<T>> for Rotation<T> {
    type Output = Rotation<T>;

    fn transpose_multiply(self, rhs: Rotation<T>) -> Self::Output {
        let q = self;
        let r = rhs;
        let s = q.c * r.s - q.s * r.c;
        let c = q.c * r.c + q.s * r.s;
        Rotation { s, c }
    }
}

impl<T: Real> Multiply<Vector2<T>> for Rotation<T> {
    type Output = Vector2<T>;

    fn multiply(self, rhs: Vector2<T>) -> Self::Output {
        let q = self;
        let v = rhs;
        Vector2::new(q.c * v.x - q.s * v.y, q.s * v.x + q.c * v.y)
    }
}

impl<T: Real> TransposeMultiply<Vector2<T>> for Rotation<T> {
    type Output = Vector2<T>;

    fn transpose_multiply(self, rhs: Vector2<T>) -> Self::Output {
        let q = self;
        let v = rhs;
        Vector2::new(q.c * v.x + q.s * v.y, -q.s * v.x + q.c * v.y)
    }
}
