use crate::{Multiply, Real, Vector2};
use std::ops::{Mul, MulAssign};

#[derive(Debug, Copy, Clone, Default)]
pub struct AffineTransform<T>(pub [T; 6]);

impl<T: Real> AffineTransform<T> {
    pub fn identity() -> AffineTransform<T> {
        AffineTransform([
            T::f32(1.0),
            T::f32(0.0),
            T::f32(0.0),
            T::f32(1.0),
            T::f32(0.0),
            T::f32(0.0),
        ])
    }

    pub fn translate(tx: T, ty: T) -> AffineTransform<T> {
        AffineTransform([T::f32(1.0), T::f32(0.0), T::f32(0.0), T::f32(1.0), tx, ty])
    }

    pub fn scale(sx: T, sy: T) -> AffineTransform<T> {
        AffineTransform([sx, T::f32(0.0), T::f32(0.0), sy, T::f32(0.0), T::f32(0.0)])
    }

    pub fn rotate(a: T) -> AffineTransform<T> {
        let cs = a.cos();
        let sn = a.sin();
        AffineTransform([cs, sn, -sn, cs, T::f32(0.0), T::f32(0.0)])
    }

    pub fn skew_x(a: T) -> AffineTransform<T> {
        AffineTransform([
            T::f32(1.0),
            T::f32(0.0),
            a.tan(),
            T::f32(1.0),
            T::f32(0.0),
            T::f32(0.0),
        ])
    }

    pub fn skew_y(a: T) -> AffineTransform<T> {
        AffineTransform([
            T::f32(1.0),
            a.tan(),
            T::f32(0.0),
            T::f32(1.0),
            T::f32(0.0),
            T::f32(0.0),
        ])
    }

    pub fn inverse(self) -> AffineTransform<T> {
        let t = &self.0;
        let det = t[0] * t[3] - t[2] * t[1];
        if det > T::epsilon() && det < T::epsilon() {
            return AffineTransform::identity();
        }
        let invdet = T::one() / det;
        let mut inv = [T::zero(); 6];
        inv[0] = t[3] * invdet;
        inv[2] = -t[2] * invdet;
        inv[4] = (t[2] * t[5] - t[3] * t[4]) * invdet;
        inv[1] = -t[1] * invdet;
        inv[3] = t[0] * invdet;
        inv[5] = (t[1] * t[4] - t[0] * t[5]) * invdet;
        AffineTransform(inv)
    }
}

impl<T: Real> Multiply<Vector2<T>> for AffineTransform<T> {
    type Output = Vector2<T>;

    fn multiply(self, pt: Vector2<T>) -> Self::Output {
        let t = &self.0;
        Vector2::new(
            pt.x * t[0] + pt.y * t[2] + t[4],
            pt.x * t[1] + pt.y * t[3] + t[5],
        )
    }
}

impl<T: Real> Mul for AffineTransform<T> {
    type Output = AffineTransform<T>;

    fn mul(mut self, rhs: Self) -> Self::Output {
        let t = &mut self.0;
        let s = &rhs.0;
        let t0 = t[0] * s[0] + t[1] * s[2];
        let t2 = t[2] * s[0] + t[3] * s[2];
        let t4 = t[4] * s[0] + t[5] * s[2] + s[4];
        t[1] = t[0] * s[1] + t[1] * s[3];
        t[3] = t[2] * s[1] + t[3] * s[3];
        t[5] = t[4] * s[1] + t[5] * s[3] + s[5];
        t[0] = t0;
        t[2] = t2;
        t[4] = t4;
        self
    }
}

impl<T: Real> MulAssign for AffineTransform<T> {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}
