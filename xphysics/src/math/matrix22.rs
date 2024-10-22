use xmath::{DotTrait, Multiply, Real, TransposeMultiply, Vector2};

#[derive(Debug, Copy, Clone, Default)]
pub struct Matrix22<T> {
    pub ex: Vector2<T>,
    pub ey: Vector2<T>,
}

impl<T: Real> Matrix22<T> {
    #[allow(dead_code)]
    pub fn new(ex: Vector2<T>, ey: Vector2<T>) -> Matrix22<T> {
        Matrix22 { ex, ey }
    }

    #[allow(dead_code)]
    pub fn identity() -> Matrix22<T> {
        Matrix22 {
            ex: Vector2 {
                x: T::one(),
                y: T::zero(),
            },
            ey: Vector2 {
                x: T::zero(),
                y: T::one(),
            },
        }
    }

    pub fn zero() -> Matrix22<T> {
        Matrix22 {
            ex: Vector2::zero(),
            ey: Vector2::zero(),
        }
    }

    pub fn inverse(&self) -> Matrix22<T> {
        let a = self.ex.x;
        let b = self.ey.x;
        let c = self.ex.y;
        let d = self.ey.y;
        let mut det = a * d - b * c;
        if det != T::zero() {
            det = T::one() / det;
        }
        Matrix22 {
            ex: Vector2 {
                x: det * d,
                y: -det * c,
            },
            ey: Vector2 {
                x: -det * b,
                y: det * a,
            },
        }
    }

    #[allow(dead_code)]
    pub fn solve(&self, b: Vector2<T>) -> Vector2<T> {
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
}

impl<T: Real> Multiply<Vector2<T>> for Matrix22<T> {
    type Output = Vector2<T>;

    fn multiply(self, rhs: Vector2<T>) -> Self::Output {
        let a = self;
        let v = rhs;
        Vector2::new(a.ex.x * v.x + a.ey.x * v.y, a.ex.y * v.x + a.ey.y * v.y)
    }
}

impl<T: Real> TransposeMultiply<Vector2<T>> for Matrix22<T> {
    type Output = Vector2<T>;

    fn transpose_multiply(self, rhs: Vector2<T>) -> Self::Output {
        let a = self;
        let v = rhs;
        Vector2::new(v.dot(a.ex), v.dot(a.ey))
    }
}

impl<T: Real> Multiply<Matrix22<T>> for Matrix22<T> {
    type Output = Matrix22<T>;

    fn multiply(self, rhs: Matrix22<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        Matrix22::new(a.multiply(b.ex), a.multiply(b.ey))
    }
}

impl<T: Real> TransposeMultiply<Matrix22<T>> for Matrix22<T> {
    type Output = Matrix22<T>;

    fn transpose_multiply(self, rhs: Matrix22<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        let c1 = (a.ex.dot(b.ex), a.ey.dot(b.ex)).into();
        let c2 = (a.ex.dot(b.ey), a.ey.dot(b.ey)).into();
        Matrix22::new(c1, c2)
    }
}
