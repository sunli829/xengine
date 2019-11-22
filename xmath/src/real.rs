use std::fmt::Debug;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

pub trait Real:
    Debug
    + Copy
    + Clone
    + RealConstants
    + Neg<Output = Self>
    + Add<Output = Self>
    + Sub<Output = Self>
    + Mul<Output = Self>
    + Div<Output = Self>
    + AddAssign
    + SubAssign
    + MulAssign
    + DivAssign
    + PartialOrd
{
    fn is_valid(&self) -> bool;

    fn sqrt(&self) -> Self;

    fn abs(&self) -> Self;

    fn max(&self, other: Self) -> Self;

    fn min(&self, other: Self) -> Self;
}

pub trait RealConstants {
    fn zero() -> Self;

    fn one() -> Self;

    fn epsilon() -> Self;

    fn half() -> Self;

    fn en1() -> Self;
}
