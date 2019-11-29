use xmath::{Multiply, Real, Rotation, Transform, Vector2};

#[derive(Debug, Copy, Clone, Default)]
pub struct Sweep<T> {
    pub local_center: Vector2<T>,
    pub c0: Vector2<T>,
    pub c: Vector2<T>,
    pub a0: T,
    pub a: T,
    pub alpha0: T,
}

impl<T: Real> Sweep<T> {
    pub fn get_transform(&self, beta: T) -> Transform<T> {
        let mut p = self.c0 * (T::one() - beta) + self.c * beta;
        let angle = (T::one() - beta) * self.a0 + beta * self.a;
        let q = Rotation::new(angle);
        p -= q.multiply(self.local_center);
        Transform { p, q }
    }

    pub fn advance(&mut self, alpha: T) {
        let beta = (alpha - self.alpha0) / (T::one() - self.alpha0);
        self.c0 += (self.c - self.c0) * beta;
        self.a0 += (self.a - self.a0) * beta;
        self.alpha0 = alpha;
    }

    pub fn normalize(&self) -> Sweep<T> {
        let d = T::pi_times_2() * (self.a0 / T::pi_times_2()).floor();
        let a0 = self.a0 - d;
        let a = self.a - d;
        Sweep {
            local_center: self.local_center,
            c0: self.c0,
            c: self.c,
            a0,
            a,
            alpha0: self.alpha0,
        }
    }
}
