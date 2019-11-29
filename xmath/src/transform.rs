use crate::{Multiply, Real, Rotation, TransposeMultiply, Vector2};

#[derive(Debug, Copy, Clone, Default)]
pub struct Transform<T> {
    pub p: Vector2<T>,
    pub q: Rotation<T>,
}

impl<T: Real> Transform<T> {
    pub fn new(position: Vector2<T>, rotation: Rotation<T>) -> Transform<T> {
        Transform {
            p: position,
            q: rotation,
        }
    }

    pub fn identity() -> Transform<T> {
        Transform {
            p: Vector2::zero(),
            q: Rotation::identity(),
        }
    }
}

impl<T: Real> Multiply<Vector2<T>> for Transform<T> {
    type Output = Vector2<T>;

    fn multiply(self, rhs: Vector2<T>) -> Self::Output {
        let t = self;
        let v = rhs;
        let x = (t.q.c * v.x - t.q.s * v.y) + t.p.x;
        let y = (t.q.s * v.x + t.q.c * v.y) + t.p.y;
        Vector2 { x, y }
    }
}

impl<T: Real> TransposeMultiply<Vector2<T>> for Transform<T> {
    type Output = Vector2<T>;

    fn transpose_multiply(self, rhs: Vector2<T>) -> Self::Output {
        let t = self;
        let v = rhs;
        let px = v.x - t.p.x;
        let py = v.y - t.p.y;
        let x = t.q.c * px + t.q.s * py;
        let y = -t.q.s * px + t.q.c * py;
        Vector2 { x, y }
    }
}

impl<T: Real> Multiply<Transform<T>> for Transform<T> {
    type Output = Transform<T>;

    fn multiply(self, rhs: Transform<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        Transform {
            p: a.q.multiply(b.p) + a.p,
            q: a.q.multiply(b.q),
        }
    }
}

impl<T: Real> TransposeMultiply<Transform<T>> for Transform<T> {
    type Output = Transform<T>;

    fn transpose_multiply(self, rhs: Transform<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        Transform {
            p: a.q.transpose_multiply(b.p - a.p),
            q: a.q.transpose_multiply(b.q),
        }
    }
}
