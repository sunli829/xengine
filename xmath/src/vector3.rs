use crate::real::Real;
use crate::{CrossTrait, DotTrait};
use std::ops::{Add, AddAssign, Mul, MulAssign, Neg, Sub, SubAssign};

#[derive(Debug, Copy, Clone, Default)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Real> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Vector3<T> {
        Vector3 { x, y, z }
    }

    pub fn zero() -> Vector3<T> {
        Vector3 {
            x: T::zero(),
            y: T::zero(),
            z: T::zero(),
        }
    }
}

impl<T: Real> Neg for Vector3<T> {
    type Output = Vector3<T>;

    fn neg(self) -> Self::Output {
        Vector3 {
            x: -self.x,
            y: -self.y,
            z: -self.z,
        }
    }
}

impl<T: Real> Add for Vector3<T> {
    type Output = Vector3<T>;

    fn add(self, rhs: Self) -> Self::Output {
        Vector3 {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<T: Real> Add<T> for Vector3<T> {
    type Output = Vector3<T>;

    fn add(self, rhs: T) -> Self::Output {
        Vector3 {
            x: self.x + rhs,
            y: self.y + rhs,
            z: self.z + rhs,
        }
    }
}

impl<T: Real> AddAssign for Vector3<T> {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl<T: Real> AddAssign<T> for Vector3<T> {
    fn add_assign(&mut self, rhs: T) {
        self.x += rhs;
        self.y += rhs;
        self.z += rhs;
    }
}

impl<T: Real> Sub for Vector3<T> {
    type Output = Vector3<T>;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector3 {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

impl<T: Real> Sub<T> for Vector3<T> {
    type Output = Vector3<T>;

    fn sub(self, rhs: T) -> Self::Output {
        Vector3 {
            x: self.x - rhs,
            y: self.y - rhs,
            z: self.z - rhs,
        }
    }
}

impl<T: Real> SubAssign for Vector3<T> {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl<T: Real> SubAssign<T> for Vector3<T> {
    fn sub_assign(&mut self, rhs: T) {
        self.x -= rhs;
        self.y -= rhs;
        self.z -= rhs;
    }
}

impl<T: Real> Mul<T> for Vector3<T> {
    type Output = Vector3<T>;

    fn mul(self, rhs: T) -> Self::Output {
        Vector3 {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs,
        }
    }
}

impl<T: Real> MulAssign<T> for Vector3<T> {
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl<T: Real> DotTrait<Vector3<T>> for Vector3<T> {
    type Output = T;

    fn dot(self, rhs: Vector3<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        a.x * b.x + a.y * b.y + a.z * b.z
    }
}

impl<T: Real> CrossTrait<Vector3<T>> for Vector3<T> {
    type Output = Vector3<T>;

    fn cross(self, rhs: Vector3<T>) -> Self::Output {
        let a = self;
        let b = rhs;
        Vector3::new(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        )
    }
}

impl<T> From<(T, T, T)> for Vector3<T> {
    fn from((x, y, z): (T, T, T)) -> Self {
        Vector3 { x, y, z }
    }
}
