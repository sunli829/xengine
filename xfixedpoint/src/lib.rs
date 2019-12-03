mod acos_lut;
mod sin_lut;
mod tan_lut;

use crate::acos_lut::ACOS_LUT;
use crate::sin_lut::SIN_LUT;
use crate::tan_lut::TAN_LUT;
use std::cmp::Ordering;
use std::fmt::{Debug, Display, Formatter};
use std::ops::{
    Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Rem, RemAssign, Sub, SubAssign,
};
use xmath::{Real, RealConstants, RealConverter};

const MAX_VALUE: i64 = std::i64::MAX;
const MIN_VALUE: i64 = std::i64::MIN;
const NUM_BITS: usize = 64;
const FRACTIONAL_PLACES: usize = 32;
const MIN_POSITIVE_VALUE: i64 = 1;
const ONE: i64 = 1i64 << FRACTIONAL_PLACES;
const TWO: i64 = 2i64 << FRACTIONAL_PLACES;
const TEN: i64 = 10i64 << FRACTIONAL_PLACES;
const HALF: i64 = 1i64 << (FRACTIONAL_PLACES - 1);
const PI_TIMES_2: i64 = 0x6487ED511;
const PI: i64 = 0x3243F6A88;
const PI_OVER_2: i64 = 0x1921FB544;
const LUT_SIZE: usize = (PI_OVER_2 >> 15) as usize;
const LUT_INTERVAL: FP = FP(0x1ffff19349798);

#[derive(Copy, Clone, Default)]
pub struct FP(i64);

pub mod consts {
    use super::FP;

    pub const MAX_VALUE: FP = FP(super::MAX_VALUE - 1);
    pub const MIN_VALUE: FP = FP(super::MAX_VALUE - 1);
    pub const MIN_POSITIVE_VALUE: FP = FP(super::MIN_POSITIVE_VALUE);
    pub const ONE: FP = FP(super::ONE);
    pub const TWO: FP = FP(super::TWO);
    pub const TEN: FP = FP(super::TEN);
    pub const HALF: FP = FP(super::HALF);
    pub const ZERO: FP = FP(0);
    pub const POSITIVE_INFINITY: FP = FP(super::MAX_VALUE);
    pub const NEGATIVE_INFINITY: FP = FP(super::MIN_VALUE + 1);
    pub const NAN: FP = FP(super::MIN_VALUE);
    pub const EPSILON: FP = FP(0x418937);
    pub const EN1: FP = FP(0x1999999a);
    pub const EN2: FP = FP(0x28f5c29);
    pub const EN3: FP = FP(0x418937);
    pub const PI_TIMES_2: FP = FP(super::PI_TIMES_2);
    pub const PI: FP = FP(super::PI);
    pub const PI_OVER_2: FP = FP(super::PI_OVER_2);
    pub const DEG_2_RAD: FP = FP(0x477d1a9);
    pub const RAD_2_DEG: FP = FP(0x394bb834d1);
    pub const E: FP = FP(0x2b7e15000);
    pub const LN_2: FP = FP(0xb17217f7);
    pub const LN_10: FP = FP(0x24d763800);
    pub const LOG2_E: FP = FP(0x171547600);
    pub const LOG10_E: FP = FP(0x6f2dec80);
    pub const FRAC_1_PI: FP = FP(0x517cc180);
    pub const FRAC_2_PI: FP = FP(0xa2f98300);
    pub const FRAC_2_SQRT_PI: FP = FP(0x120dd7600);
    pub const FRAC_1_SQRT_2: FP = FP(0xb504f300);
    pub const FRAC_PI_2: FP = FP(0x1921fb600);
    pub const FRAC_PI_3: FP = FP(0x10c152400);
    pub const FRAC_PI_4: FP = FP(0xc90fdb00);
    pub const FRAC_PI_6: FP = FP(0x860a9200);
    pub const FRAC_PI_8: FP = FP(0x6487ed80);
    pub const SQRT_2: FP = FP(0x16a09e600);
    pub const LOG2MAX: FP = FP(0x1F00000000);
    pub const LOG2MIN: FP = FP(-0x2000000000);
}

impl Real for FP {
    fn is_valid(&self) -> bool {
        self.0 > MIN_VALUE + 1 && self.0 < MAX_VALUE
    }

    fn abs(self) -> Self {
        if self.0 == MIN_VALUE {
            return consts::MAX_VALUE;
        }

        let mask = self.0 >> 63;
        FP((self.0 + mask) ^ mask)
    }

    fn sqrt(self) -> Self {
        let xl = self.0;
        if xl < 0 {
            return consts::NAN;
        };

        let mut num = xl as u64;
        let mut result = 0u64;
        let mut bit = 1u64 << (NUM_BITS - 2);

        while bit > num {
            bit >>= 2;
        }

        for i in 0..2 {
            while bit != 0 {
                if num >= result + bit {
                    num -= result + bit;
                    result = (result >> 1) + bit;
                } else {
                    result = result >> 1;
                }
                bit >>= 2;
            }

            if i == 0 {
                if num > (1u64 << (NUM_BITS / 2)) - 1 {
                    num -= result;
                    num = (num << (NUM_BITS / 2)) - 0x80000000;
                    result = (result << (NUM_BITS / 2)) + 0x80000000;
                } else {
                    num <<= NUM_BITS / 2;
                    result <<= NUM_BITS / 2;
                }

                bit = 1 << (NUM_BITS / 2 - 2);
            }
        }
        if num > result {
            result += 1;
        }
        FP(result as i64)
    }

    fn atan(self) -> Self {
        self.atan2(consts::ONE)
    }

    fn atan2(self, x: Self) -> Self {
        let yl = self.0;
        let xl = x.0;
        if xl == 0 {
            if yl > 0 {
                return consts::PI_OVER_2;
            }
            if yl == 0 {
                return consts::ZERO;
            }
            return -consts::PI_OVER_2;
        }

        let z = self / x;

        let sm = consts::EN2 * FP(28);
        if consts::ONE + sm * z * z == consts::MAX_VALUE {
            return if self < consts::ZERO {
                -consts::PI_OVER_2
            } else {
                consts::PI_OVER_2
            };
        }

        let atan = if z.abs() < consts::ONE {
            let atan = z / (consts::ONE + sm * z * z);
            if xl < 0 {
                if yl < 0 {
                    return atan - consts::PI;
                }
                return atan + consts::PI;
            }
            atan
        } else {
            let atan = consts::PI_OVER_2 - z / (z * z + sm);
            if yl < 0 {
                return atan - consts::PI;
            }
            atan
        };
        atan
    }

    fn floor(self) -> Self {
        FP(((self.0 as u64) & 0xFFFFFFFF00000000) as i64)
    }

    fn ceiling(self) -> Self {
        let has_fractional_part = (self.0 & 0x00000000FFFFFFFF) != 0;
        if has_fractional_part {
            self.floor() + consts::ONE
        } else {
            self
        }
    }

    fn round(self) -> Self {
        let fractional_part = self.0 & 0x00000000FFFFFFFF;
        let integral_part = self.floor();
        if fractional_part < 0x80000000 {
            return integral_part;
        }
        if fractional_part > 0x80000000 {
            return integral_part + consts::ONE;
        }
        if (integral_part.0 & ONE) == 0 {
            integral_part
        } else {
            integral_part + consts::ONE
        }
    }

    fn signum(self) -> Self {
        if self == consts::POSITIVE_INFINITY || self >= consts::ZERO {
            return FP::i32(1);
        } else if self == consts::NEGATIVE_INFINITY || self < consts::ZERO {
            return FP::i32(-1);
        } else if self == consts::NAN {
            return consts::NAN;
        } else {
            return FP::i32(0);
        }
    }

    fn sin(self) -> Self {
        let (clamped_l, flip_horizontal, flip_vertical) = clamp_sin_value(self.0);
        let clamped = FP(clamped_l);

        let raw_index = clamped * LUT_INTERVAL;
        let rounded_index = raw_index.round();
        let index_error = consts::ZERO;

        let nearest_value = FP(SIN_LUT[if flip_horizontal {
            SIN_LUT.len() - 1 - rounded_index.to_i32() as usize
        } else {
            rounded_index.to_i32() as usize
        }]);
        let second_nearest_value = FP(SIN_LUT[if flip_horizontal {
            SIN_LUT.len() - 1 - (rounded_index.to_i32() - index_error.signum().to_i32()) as usize
        } else {
            (rounded_index.to_i32() + index_error.signum().to_i32()) as usize
        }]);

        let delta = (index_error * (nearest_value - second_nearest_value).abs()).0;
        let interpolated_value = nearest_value.0 + (if flip_horizontal { -delta } else { delta });
        let final_value = if flip_vertical {
            -interpolated_value
        } else {
            interpolated_value
        };
        FP(final_value)
    }

    fn cos(self) -> Self {
        let xl = self.0;
        let raw_angle = xl + (if xl > 0 { -PI - PI_OVER_2 } else { PI_OVER_2 });
        FP(raw_angle).sin()
    }

    fn tan(self) -> Self {
        let mut clamped_pi = self.0 % PI;
        let mut flip = false;
        if clamped_pi < 0 {
            clamped_pi = -clamped_pi;
            flip = true;
        }
        if clamped_pi > PI_OVER_2 {
            flip = !flip;
            clamped_pi = PI_OVER_2 - (clamped_pi - PI_OVER_2);
        }

        let clamped = FP(clamped_pi);

        let raw_index = clamped * LUT_INTERVAL;
        let rounded_index = raw_index.round();
        let index_error = raw_index - rounded_index;

        let nearest_value = FP(TAN_LUT[rounded_index.to_i32() as usize]);
        let second_nearest_value =
            FP(TAN_LUT[(rounded_index + FP::i32(index_error.signum().to_i32())).to_i32() as usize]);

        let delta = (index_error * (nearest_value - second_nearest_value).abs()).0;
        let interpolated_value = nearest_value.0 + delta;
        let final_value = if flip {
            -interpolated_value
        } else {
            interpolated_value
        };
        FP(final_value)
    }

    fn asin(self) -> Self {
        consts::PI_OVER_2 - self.acos()
    }

    fn acos(self) -> Self {
        let mut value = self;

        if value == consts::ZERO {
            return consts::PI_OVER_2;
        }

        let mut flip = false;
        if value < consts::ZERO {
            value = -value;
            flip = true;
        }

        let raw_index = value * FP::i32(LUT_SIZE as i32);
        let mut rounded_index = raw_index.round();
        if rounded_index >= FP::i32(LUT_SIZE as i32) {
            rounded_index = FP::i32(LUT_SIZE as i32 - 1);
        }

        let index_error = raw_index - rounded_index;
        let nearest_value = FP(ACOS_LUT[rounded_index.to_i32() as usize]);

        let mut next_index = rounded_index.to_i32() + index_error.signum().to_i32();
        if next_index >= LUT_SIZE as i32 {
            next_index = LUT_SIZE as i32 - 1;
        }

        let second_nearest_value = FP(ACOS_LUT[next_index as usize]);

        let delta = (index_error * (nearest_value - second_nearest_value).abs()).0;
        let interpolated_value = FP(nearest_value.0 + delta);
        if flip {
            (consts::PI - interpolated_value)
        } else {
            interpolated_value
        }
    }

    fn log2(self) -> Self {
        let x = self;
        if x.0 <= 0 {
            return consts::NAN;
        }

        let mut b = 1i64 << (FRACTIONAL_PLACES - 1);
        let mut y = 0;

        let mut raw_x = x.0;
        while raw_x < ONE {
            raw_x <<= 1;
            y -= ONE;
        }

        while raw_x >= (ONE << 1) {
            raw_x >>= 1;
            y += ONE;
        }

        let mut z = FP(raw_x);

        for _ in 0..FRACTIONAL_PLACES {
            z = z * z;
            if z.0 >= (ONE << 1) {
                z = FP(z.0 >> 1);
                y += b;
            }
            b >>= 1;
        }

        FP(y)
    }

    fn ln(self) -> Self {
        self.log2() * consts::LN_2
    }

    fn pow(self, exp: Self) -> Self {
        let b = self;
        if b == consts::ONE {
            return consts::ONE;
        }
        if exp.0 == 0 {
            return consts::ONE;
        }

        if b.0 == 0 {
            if exp.0 < 0 {
                return consts::NAN;
            }
            return consts::ZERO;
        }

        (exp * b.log2()).pow2()
    }

    fn max(self, rhs: Self) -> Self {
        if self > rhs {
            self
        } else {
            rhs
        }
    }

    fn min(self, rhs: Self) -> Self {
        if self < rhs {
            self
        } else {
            rhs
        }
    }
}

impl Add for FP {
    type Output = FP;

    fn add(self, rhs: Self) -> Self::Output {
        FP(self.0 + rhs.0)
    }
}

impl Sub for FP {
    type Output = FP;

    fn sub(self, rhs: Self) -> Self::Output {
        FP(self.0 - rhs.0)
    }
}

impl Mul for FP {
    type Output = FP;

    fn mul(self, rhs: Self) -> Self::Output {
        let xl = self.0;
        let yl = rhs.0;

        let xlo = (xl & 0x00000000FFFFFFFF) as u64;
        let xhi = xl >> FRACTIONAL_PLACES;
        let ylo = (yl & 0x00000000FFFFFFFF) as u64;
        let yhi = yl >> FRACTIONAL_PLACES;

        let lolo = xlo * ylo;
        let lohi = xlo as i64 * yhi;
        let hilo = xhi * (ylo as i64);
        let hihi = xhi * yhi;

        let lo_result = lolo >> FRACTIONAL_PLACES;
        let mid_result1 = lohi;
        let mid_result2 = hilo;
        let hi_result = hihi << FRACTIONAL_PLACES;

        let sum = (lo_result as i64) + mid_result1 + mid_result2 + hi_result;
        FP(sum)
    }
}

impl Div for FP {
    type Output = FP;

    fn div(self, rhs: Self) -> Self::Output {
        let xl = self.0;
        let yl = rhs.0;

        if yl == 0 {
            return consts::POSITIVE_INFINITY;
        }

        let mut remainder = (if xl >= 0 { xl } else { -xl }) as u64;
        let mut divider = (if yl >= 0 { yl } else { -yl }) as u64;
        let mut quotient = 0u64;
        let mut bit_pos = (NUM_BITS / 2 + 1) as i32;

        while (divider & 0xF) == 0 && bit_pos >= 4 {
            divider >>= 4;
            bit_pos -= 4;
        }

        while remainder != 0 && bit_pos >= 0 {
            let mut shift = remainder.leading_zeros() as i32;
            if shift > bit_pos {
                shift = bit_pos;
            }
            remainder <<= shift;
            bit_pos -= shift;

            let div = remainder / divider;
            remainder = remainder % divider;
            quotient += div << bit_pos;

            if (div & !(0xFFFFFFFFFFFFFFFF >> bit_pos)) != 0 {
                return if ((xl ^ yl) & MIN_VALUE) == 0 {
                    consts::MAX_VALUE
                } else {
                    consts::MIN_VALUE
                };
            }

            remainder <<= 1;
            bit_pos -= 1;
        }

        quotient += 1;
        let mut result = (quotient >> 1) as i64;
        if ((xl ^ yl) & MIN_VALUE) != 0 {
            result = -result;
        }

        FP(result)
    }
}

impl Rem for FP {
    type Output = FP;

    fn rem(self, rhs: Self) -> Self::Output {
        FP(if self.0 == MIN_VALUE && rhs.0 == -1 {
            0
        } else {
            self.0 % rhs.0
        })
    }
}

impl PartialEq for FP {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl PartialOrd for FP {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.0.partial_cmp(&other.0)
    }
}

impl Neg for FP {
    type Output = FP;

    fn neg(self) -> Self::Output {
        if self.0 == MIN_VALUE {
            consts::MAX_VALUE
        } else {
            FP(-self.0)
        }
    }
}

impl AddAssign for FP {
    fn add_assign(&mut self, rhs: Self) {
        *self = *self + rhs;
    }
}

impl SubAssign for FP {
    fn sub_assign(&mut self, rhs: Self) {
        *self = *self - rhs;
    }
}

impl MulAssign for FP {
    fn mul_assign(&mut self, rhs: Self) {
        *self = *self * rhs;
    }
}

impl DivAssign for FP {
    fn div_assign(&mut self, rhs: Self) {
        *self = *self / rhs;
    }
}

impl RemAssign for FP {
    fn rem_assign(&mut self, rhs: Self) {
        *self = *self % rhs;
    }
}

impl Display for FP {
    fn fmt(&self, f: &mut Formatter) -> std::fmt::Result {
        write!(f, "{}", self.to_f32())
    }
}

impl Debug for FP {
    fn fmt(&self, f: &mut Formatter) -> std::fmt::Result {
        Display::fmt(self, f)
    }
}

impl RealConverter for FP {
    fn to_f32(&self) -> f32 {
        (self.0 as f32) / (ONE as f32)
    }

    fn to_i32(&self) -> i32 {
        (self.0 >> FRACTIONAL_PLACES) as i32
    }

    fn f32(value: f32) -> Self {
        FP((value * (ONE as f32)) as i64)
    }

    fn i32(value: i32) -> Self {
        FP(value as i64 * ONE)
    }
}

impl RealConstants for FP {
    fn max_value() -> Self {
        consts::MAX_VALUE
    }

    fn min_value() -> Self {
        consts::MIN_VALUE
    }

    fn one() -> Self {
        consts::ONE
    }

    fn ten() -> Self {
        consts::TEN
    }

    fn half() -> Self {
        consts::HALF
    }

    fn two() -> Self {
        consts::TWO
    }

    fn zero() -> Self {
        consts::ZERO
    }

    fn positive_infinity() -> Self {
        consts::POSITIVE_INFINITY
    }

    fn negative_infinity() -> Self {
        consts::NEGATIVE_INFINITY
    }

    fn nan() -> Self {
        consts::NAN
    }

    fn epsilon() -> Self {
        consts::EPSILON
    }

    fn en1() -> Self {
        consts::EN1
    }

    fn en2() -> Self {
        consts::EN2
    }

    fn en3() -> Self {
        consts::EN3
    }

    fn pi_times_2() -> Self {
        consts::PI_TIMES_2
    }

    fn pi() -> Self {
        consts::PI
    }

    fn pi_over_2() -> Self {
        consts::PI_OVER_2
    }

    fn deg_2_rad() -> Self {
        consts::DEG_2_RAD
    }

    fn rad_2_deg() -> Self {
        consts::RAD_2_DEG
    }

    fn e() -> Self {
        consts::E
    }

    fn ln_2() -> Self {
        consts::LN_2
    }

    fn ln_10() -> Self {
        consts::LN_10
    }

    fn log2_e() -> Self {
        consts::LOG2_E
    }

    fn log10_e() -> Self {
        consts::LOG10_E
    }

    fn frac_1_pi() -> Self {
        consts::FRAC_1_PI
    }

    fn frac_2_pi() -> Self {
        consts::FRAC_2_PI
    }

    fn frac_2_sqrt_pi() -> Self {
        consts::FRAC_2_SQRT_PI
    }

    fn frac_1_sqrt_2() -> Self {
        consts::FRAC_1_SQRT_2
    }

    fn frac_pi_2() -> Self {
        consts::FRAC_PI_2
    }

    fn frac_pi_3() -> Self {
        consts::FRAC_PI_3
    }

    fn frac_pi_4() -> Self {
        consts::FRAC_PI_4
    }

    fn frac_pi_6() -> Self {
        consts::FRAC_PI_6
    }

    fn frac_pi_8() -> Self {
        consts::FRAC_PI_8
    }

    fn sqrt_2() -> Self {
        consts::SQRT_2
    }
}

impl FP {
    fn pow2(self) -> FP {
        let mut x = self;
        if x.0 == 0 {
            return consts::ONE;
        }

        let neg = x.0 < 0;
        if neg {
            x = -x;
        }

        if x == consts::ONE {
            if neg {
                return consts::ONE / FP::i32(2);
            } else {
                return FP::i32(2);
            }
        }
        if x >= consts::LOG2MAX {
            if neg {
                return consts::ONE / consts::MAX_VALUE;
            } else {
                return consts::MAX_VALUE;
            }
        }
        if x <= consts::LOG2MIN {
            if neg {
                return consts::MAX_VALUE;
            } else {
                return consts::ZERO;
            }
        }

        let integer_part = x.floor().to_i32();
        let x = FP(x.0 & 0x00000000FFFFFFFF);

        let mut result = consts::ONE;
        let mut term = consts::ONE;
        let mut i = 1;
        while term.0 != 0 {
            term = ((x * term) * consts::LN_2) / FP::i32(i);
            result += term;
            i += 1;
        }

        let mut result = FP(result.0 << integer_part);
        if neg {
            result = consts::ONE / result;
        }
        result
    }
}

fn clamp_sin_value(angle: i64) -> (i64, bool, bool) {
    let mut clamped2pi = angle % PI_TIMES_2;
    if angle < 0 {
        clamped2pi += PI_TIMES_2;
    }

    let flip_vertical = clamped2pi >= PI;
    let mut clamped_pi = clamped2pi;
    while clamped_pi >= PI {
        clamped_pi -= PI;
    }

    let flip_horizontal = clamped_pi >= PI_OVER_2;
    let mut clamped_pi_over2 = clamped_pi;
    if clamped_pi_over2 >= PI_OVER_2 {
        clamped_pi_over2 -= PI_OVER_2;
    }
    (clamped_pi_over2, flip_horizontal, flip_vertical)
}
