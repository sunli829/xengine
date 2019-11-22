use crate::{CrossTrait, DotTrait, Real, Vector2, Vector3};

#[derive(Debug, Copy, Clone)]
pub struct Matrix33<T> {
    pub ex: Vector3<T>,
    pub ey: Vector3<T>,
    pub ez: Vector3<T>,
}

impl<T: Real> Matrix33<T> {
    pub fn new(ex: Vector3<T>, ey: Vector3<T>, ez: Vector3<T>) -> Matrix33<T> {
        Matrix33 { ex, ey, ez }
    }

    pub fn zero() -> Matrix33<T> {
        Matrix33 {
            ex: Vector3::zero(),
            ey: Vector3::zero(),
            ez: Vector3::zero(),
        }
    }

    pub fn solve33(&self, b: Vector3<T>) -> Vector3<T> {
        let mut det = self.ex.dot(self.ey.cross(self.ez));
        if det != T::zero() {
            det = T::one() / det;
        }
        Vector3 {
            x: det * b.dot(self.ey.cross(self.ez)),
            y: det * self.ex.dot(b.cross(self.ez)),
            z: det * self.ex.dot(self.ey.cross(b)),
        }
    }

    pub fn solve22(&self, b: Vector2<T>) -> Vector2<T> {
        let a11 = self.ex.x;
        let a12 = self.ey.x;
        let a21 = self.ex.y;
        let a22 = self.ey.y;
        let mut det = a11 * a22 - a12 * a21;
        if det != T::zero() {
            det = T::one() / det;
        }
        Vector2 {
            x: det * (a22 * b.x - a12 * b.y),
            y: det * (a11 * b.y - a21 * b.x),
        }
    }

    pub fn inverse22(&self) -> Matrix33<T> {
        let a = self.ex.x;
        let b = self.ey.x;
        let c = self.ex.y;
        let d = self.ey.y;
        let mut det = a * d - b * c;
        if det != T::zero() {
            det = T::one() / det;
        }

        Matrix33 {
            ex: Vector3::new(det * d, -det * c, T::zero()),
            ey: Vector3::new(-det * b, det * a, T::zero()),
            ez: Vector3::new(T::zero(), T::zero(), T::zero()),
        }
    }

    pub fn symmetric_inverse(&self) -> Matrix33<T> {
        let mut det = self.ex.dot(self.ey.cross(self.ez));
        if det != T::zero() {
            det = T::one() / det;
        }

        let a11 = self.ex.x;
        let a12 = self.ey.x;
        let a13 = self.ez.x;
        let a22 = self.ey.y;
        let a23 = self.ez.y;
        let a33 = self.ez.z;

        Matrix33 {
            ex: Vector3::new(
                det * (a22 * a33 - a23 * a23),
                det * (a13 * a23 - a12 * a33),
                det * (a12 * a23 - a13 * a22),
            ),
            ey: Vector3::new(
                det * (a13 * a23 - a12 * a33),
                det * (a11 * a33 - a13 * a13),
                det * (a13 * a12 - a11 * a23),
            ),
            ez: Vector3::new(
                det * (a12 * a23 - a13 * a22),
                det * (a13 * a12 - a11 * a23),
                det * (a11 * a22 - a12 * a12),
            ),
        }
    }
}
