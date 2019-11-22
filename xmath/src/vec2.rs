use crate::real::Real;
use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Default, Debug, Copy, Clone)]
pub struct Vec2<T> {
    pub x: T,
    pub y: T,
}

impl<T: Real> Vec2<T> {
    pub fn new(x: T, y: T) -> Vec2<T> {
        Vec2 { x, y }
    }

    pub fn zero() -> Vec2<T> {
        Vec2 {
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

    pub fn normalize(&mut self) {
        let len = self.length();
        if len >= T::epsilon() {
            let inv_length = T::one() / len;
            self.x *= inv_length;
            self.y *= inv_length;
        }
    }

    pub fn is_valid(&self) -> bool {
        self.x.is_valid() && self.y.is_valid()
    }

    pub fn skew(&self) -> Vec2<T> {
        Vec2 {
            x: -self.y,
            y: self.x,
        }
    }

    pub fn max(self, rhs: Vec2<T>) -> Vec2<T> {
        Vec2 {
            x: self.x.max(rhs.x),
            y: self.y.max(rhs.y),
        }
    }

    pub fn min(self, rhs: Vec2<T>) -> Vec2<T> {
        Vec2 {
            x: self.x.min(rhs.x),
            y: self.y.min(rhs.y),
        }
    }

    pub fn abs(&self) -> Vec2<T> {
        Vec2::new(self.x.abs(), self.y.abs())
    }

    pub fn dot(&self, rhs: &Vec2<T>) -> T {
        let a = self;
        let b = rhs;
        a.x * b.x + a.y * b.y
    }
}

impl<T: Real> Neg for Vec2<T> {
    type Output = Vec2<T>;

    fn neg(self) -> Self::Output {
        Vec2 {
            x: -self.x,
            y: -self.y,
        }
    }
}

impl<T: Real> Add for Vec2<T> {
    type Output = Vec2<T>;

    fn add(self, rhs: Self) -> Self::Output {
        Vec2 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T: Real> Add<T> for Vec2<T> {
    type Output = Vec2<T>;

    fn add(self, rhs: T) -> Self::Output {
        Vec2 {
            x: self.x + rhs,
            y: self.y + rhs,
        }
    }
}

impl<T: Real> AddAssign for Vec2<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T: Real> AddAssign<T> for Vec2<T> {
    fn add_assign(&mut self, rhs: T) {
        self.x += rhs;
        self.y += rhs;
    }
}

impl<T: Real> Sub for Vec2<T> {
    type Output = Vec2<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        Vec2 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T: Real> Sub<T> for Vec2<T> {
    type Output = Vec2<T>;

    fn sub(self, rhs: T) -> Self::Output {
        Vec2 {
            x: self.x - rhs,
            y: self.y - rhs,
        }
    }
}

impl<T: Real> SubAssign for Vec2<T> {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
    }
}

impl<T: Real> SubAssign<T> for Vec2<T> {
    fn sub_assign(&mut self, rhs: T) {
        self.x -= rhs;
        self.y -= rhs;
    }
}

impl<T: Real> Mul<T> for Vec2<T> {
    type Output = Vec2<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Vec2 {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl<T: Real> MulAssign<T> for Vec2<T> {
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
    }
}

pub trait CrossTrait<Rhs> {
    type Output;
    fn cross(self, rhs: Rhs) -> Self::Output;
}

impl<T: Real> CrossTrait<Vec2<T>> for Vec2<T> {
    type Output = T;

    fn cross(self, rhs: Vec2<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        a.x * b.y - a.y * b.x
    }
}

impl<T: Real> CrossTrait<T> for Vec2<T> {
    type Output = Vec2<T>;

    fn cross(self, rhs: T) -> Self::Output {
        let a = self;
        let s = rhs;
        Vec2::new(s * a.y, -s * a.x)
    }
}

impl<T: Real> CrossTrait<Vec2<T>> for T {
    type Output = Vec2<T>;

    fn cross(self, rhs: Vec2<T>) -> Self::Output {
        let s = self;
        let a = rhs;
        Vec2::new(-s * a.y, s * a.x)
    }
}
