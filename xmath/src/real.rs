use std::fmt::Debug;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

pub trait Real:
    'static
    + Debug
    + Copy
    + Clone
    + ToOwned
    + Default
    + RealConstants
    + RealConverter
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

    fn is_less_epsilon(&self) -> bool {
        *self < Self::epsilon()
    }

    fn abs(self) -> Self;

    fn sqrt(self) -> Self;

    fn atan(self) -> Self;

    fn atan2(self, x: Self) -> Self;

    fn floor(self) -> Self;

    fn ceiling(self) -> Self;

    fn round(self) -> Self;

    fn signum(self) -> Self;

    fn sin(self) -> Self;

    fn cos(self) -> Self;

    fn tan(self) -> Self;

    fn asin(self) -> Self;

    fn acos(self) -> Self;

    fn integral(self) -> Self {
        self.floor()
    }

    fn fractional(self) -> Self {
        self - self.integral()
    }

    fn log2(self) -> Self;

    fn ln(self) -> Self;

    fn pow(self, exp: Self) -> Self;

    fn max(self, other: Self) -> Self;

    fn min(self, other: Self) -> Self;

    fn clamp(self, min: Self, max: Self) -> Self {
        if self < min {
            min
        } else if self > max {
            max
        } else {
            self
        }
    }
}

pub trait RealConstants {
    fn max_value() -> Self;

    fn min_value() -> Self;

    fn one() -> Self;

    fn ten() -> Self;

    fn half() -> Self;

    fn two() -> Self;

    fn zero() -> Self;

    fn positive_infinity() -> Self;

    fn negative_infinity() -> Self;

    fn nan() -> Self;

    fn epsilon() -> Self;

    fn en1() -> Self;

    fn en2() -> Self;

    fn en3() -> Self;

    fn pi_times_2() -> Self;

    fn pi() -> Self;

    fn pi_over_2() -> Self;

    fn deg_2_rad() -> Self;

    fn rad_2_deg() -> Self;

    fn e() -> Self;

    fn ln_2() -> Self;

    fn ln_10() -> Self;

    fn log2_e() -> Self;

    fn log10_e() -> Self;

    fn frac_1_pi() -> Self;

    fn frac_2_pi() -> Self;

    fn frac_2_sqrt_pi() -> Self;

    fn frac_1_sqrt_2() -> Self;

    fn frac_pi_2() -> Self;

    fn frac_pi_3() -> Self;

    fn frac_pi_4() -> Self;

    fn frac_pi_6() -> Self;

    fn frac_pi_8() -> Self;

    fn sqrt_2() -> Self;
}

pub trait RealConverter {
    fn to_f32(&self) -> f32;

    fn to_i32(&self) -> i32;

    fn f32(value: f32) -> Self;

    fn i32(value: i32) -> Self;
}
