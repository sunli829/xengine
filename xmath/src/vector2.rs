use crate::real::Real;
use crate::{CrossTrait, DotTrait};
use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub struct Vector2<T> {
    pub x: T,
    pub y: T,
}

impl<T: Real> Vector2<T> {
    pub fn new(x: T, y: T) -> Vector2<T> {
        Vector2 { x, y }
    }

    pub fn zero() -> Vector2<T> {
        Vector2 {
            x: T::zero(),
            y: T::zero(),
        }
    }

    pub fn length(&self) -> T {
        (self.x * self.x + self.y * self.y).sqrt()
    }

    pub fn length_squared(&self) -> T {
        self.x * self.x + self.y * self.y
    }

    pub fn normalize(&self) -> Vector2<T> {
        let len = self.length();
        if len >= T::epsilon() {
            let inv_length = T::one() / len;
            Vector2 {
                x: self.x * inv_length,
                y: self.y * inv_length,
            }
        } else {
            *self
        }
    }

    pub fn is_valid(&self) -> bool {
        self.x.is_valid() && self.y.is_valid()
    }

    pub fn skew(&self) -> Vector2<T> {
        Vector2 {
            x: -self.y,
            y: self.x,
        }
    }

    pub fn max(self, rhs: Vector2<T>) -> Vector2<T> {
        Vector2 {
            x: self.x.max(rhs.x),
            y: self.y.max(rhs.y),
        }
    }

    pub fn min(self, rhs: Vector2<T>) -> Vector2<T> {
        Vector2 {
            x: self.x.min(rhs.x),
            y: self.y.min(rhs.y),
        }
    }

    pub fn abs(&self) -> Vector2<T> {
        Vector2::new(self.x.abs(), self.y.abs())
    }

    pub fn distance(&self, other: &Vector2<T>) -> T {
        (*self - *other).length()
    }

    pub fn distance_squared(&self, other: &Vector2<T>) -> T {
        let c = *self - *other;
        c.dot(c)
    }
}

impl<T: Real> Neg for Vector2<T> {
    type Output = Vector2<T>;

    fn neg(self) -> Self::Output {
        Vector2 {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl<T: Real> Add for Vector2<T> {
    type Output = Vector2<T>;

    fn add(self, rhs: Self) -> Self::Output {
        Vector2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: Real> Add<T> for Vector2<T> {
    type Output = Vector2<T>;

    fn add(self, rhs: T) -> Self::Output {
        Vector2 {
            x: self.x + rhs,
            y: self.y + rhs,
        }
    }
}

impl<T: Real> AddAssign for Vector2<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T: Real> AddAssign<T> for Vector2<T> {
    fn add_assign(&mut self, rhs: T) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl<T: Real> Sub for Vector2<T> {
    type Output = Vector2<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Real> Sub<T> for Vector2<T> {
    type Output = Vector2<T>;

    fn sub(self, rhs: T) -> Self::Output {
        Vector2 {
            x: self.x - rhs,
            y: self.y - rhs,
        }
    }
}

impl<T: Real> SubAssign for Vector2<T> {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl<T: Real> SubAssign<T> for Vector2<T> {
    fn sub_assign(&mut self, rhs: T) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl<T: Real> Mul<T> for Vector2<T> {
    type Output = Vector2<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Vector2 {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl<T: Real> Mul for Vector2<T> {
    type Output = Vector2<T>;

    fn mul(self, rhs: Vector2<T>) -> Self::Output {
        Vector2 {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
        }
    }
}

impl<T: Real> MulAssign<T> for Vector2<T> {
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

impl<T: Real> MulAssign for Vector2<T> {
    fn mul_assign(&mut self, rhs: Vector2<T>) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}

impl<T: Real> CrossTrait<Vector2<T>> for Vector2<T> {
    type Output = T;

    fn cross(self, rhs: Vector2<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        a.x * b.y - a.y * b.x
    }
}

impl<T: Real> CrossTrait<T> for Vector2<T> {
    type Output = Vector2<T>;

    fn cross(self, rhs: T) -> Self::Output {
        let a = self;
        let s = rhs;
        Vector2::new(s * a.y, -s * a.x)
    }
}

impl<T: Real> CrossTrait<Vector2<T>> for T {
    type Output = Vector2<T>;

    fn cross(self, rhs: Vector2<T>) -> Self::Output {
        let s = self;
        let a = rhs;
        Vector2::new(-s * a.y, s * a.x)
    }
}

impl<T: Real> DotTrait<Vector2<T>> for Vector2<T> {
    type Output = T;

    fn dot(self, rhs: Vector2<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        a.x * b.x + a.y * b.y
    }
}

impl<T> From<(T, T)> for Vector2<T> {
    fn from((x, y): (T, T)) -> Self {
        Vector2 { x, y }
    }
}
