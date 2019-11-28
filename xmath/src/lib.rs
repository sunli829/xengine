mod aabb;
mod float32;
mod matrix33;
mod real;
mod rotation;
mod transform;
mod vector2;
mod vector3;

pub use aabb::*;
pub use matrix33::*;
pub use real::*;
pub use rotation::*;
pub use transform::*;
pub use vector2::*;
pub use vector3::*;

pub trait CrossTrait<Rhs> {
    type Output;
    fn cross(self, rhs: Rhs) -> Self::Output;
}

pub trait DotTrait<Rhs> {
    type Output;
    fn dot(self, rhs: Rhs) -> Self::Output;
}

pub trait Multiply<Rhs> {
    type Output;
    fn multiply(self, rhs: Rhs) -> Self::Output;
}

pub trait TransposeMultiply<Rhs> {
    type Output;
    fn transpose_multiply(self, rhs: Rhs) -> Self::Output;
}
