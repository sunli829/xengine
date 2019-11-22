use crate::{Real, RealConstants, RealConverter};

impl Real for f32 {
    fn is_valid(&self) -> bool {
        f32::is_normal(*self)
    }

    fn abs(self) -> Self {
        f32::abs(self)
    }

    fn sqrt(self) -> Self {
        f32::sqrt(self)
    }

    fn atan(self) -> Self {
        f32::atan(self)
    }

    fn atan2(self, x: Self) -> Self {
        f32::atan2(self, x)
    }

    fn floor(self) -> Self {
        f32::floor(self)
    }

    fn ceiling(self) -> Self {
        f32::ceil(self)
    }

    fn round(self) -> Self {
        f32::round(self)
    }

    fn signum(self) -> Self {
        f32::signum(self)
    }

    fn sin(self) -> Self {
        f32::sin(self)
    }

    fn cos(self) -> Self {
        f32::cos(self)
    }

    fn tan(self) -> Self {
        f32::tan(self)
    }

    fn asin(self) -> Self {
        f32::asin(self)
    }

    fn acos(self) -> Self {
        f32::acos(self)
    }

    fn log2(self) -> Self {
        f32::log2(self)
    }

    fn ln(self) -> Self {
        f32::ln(self)
    }

    fn pow(self, exp: Self) -> Self {
        f32::powf(self, exp)
    }

    fn max(self, other: Self) -> Self {
        f32::max(self, other)
    }

    fn min(self, other: Self) -> Self {
        f32::min(self, other)
    }
}

impl RealConstants for f32 {
    fn max_value() -> Self {
        std::f32::MAX
    }

    fn min_value() -> Self {
        std::f32::MIN
    }

    fn one() -> Self {
        1.0
    }

    fn ten() -> Self {
        10.0
    }

    fn half() -> Self {
        0.5
    }

    fn zero() -> Self {
        0.0
    }

    fn positive_infinity() -> Self {
        std::f32::INFINITY
    }

    fn negative_infinity() -> Self {
        std::f32::NEG_INFINITY
    }

    fn nan() -> Self {
        std::f32::NAN
    }

    fn epsilon() -> Self {
        std::f32::EPSILON
    }

    fn en1() -> Self {
        0.1
    }

    fn en2() -> Self {
        0.01
    }

    fn en3() -> Self {
        0.001
    }

    fn pi_times_2() -> Self {
        std::f32::consts::PI * 2.0
    }

    fn pi() -> Self {
        std::f32::consts::PI
    }

    fn pi_over_2() -> Self {
        std::f32::consts::PI * 0.5
    }

    fn deg_2_rad() -> Self {
        std::f32::consts::PI / 180.0
    }

    fn rad_2_deg() -> Self {
        57.2957795130823208767981548141051703
    }

    fn e() -> Self {
        std::f32::consts::E
    }

    fn ln_2() -> Self {
        std::f32::consts::LN_2
    }

    fn ln_10() -> Self {
        std::f32::consts::LN_10
    }

    fn log2_e() -> Self {
        std::f32::consts::LOG2_E
    }

    fn log10_e() -> Self {
        std::f32::consts::LOG10_E
    }

    fn frac_1_pi() -> Self {
        std::f32::consts::FRAC_1_PI
    }

    fn frac_2_pi() -> Self {
        std::f32::consts::FRAC_2_PI
    }

    fn frac_2_sqrt_pi() -> Self {
        std::f32::consts::FRAC_2_SQRT_PI
    }

    fn frac_1_sqrt_2() -> Self {
        std::f32::consts::FRAC_1_SQRT_2
    }

    fn frac_pi_2() -> Self {
        std::f32::consts::FRAC_PI_2
    }

    fn frac_pi_3() -> Self {
        std::f32::consts::FRAC_PI_3
    }

    fn frac_pi_4() -> Self {
        std::f32::consts::FRAC_PI_4
    }

    fn frac_pi_6() -> Self {
        std::f32::consts::FRAC_PI_6
    }

    fn frac_pi_8() -> Self {
        std::f32::consts::FRAC_PI_8
    }

    fn sqrt_2() -> Self {
        std::f32::consts::SQRT_2
    }
}

impl RealConverter for f32 {
    fn to_f32(&self) -> f32 {
        *self
    }

    fn to_i32(&self) -> i32 {
        *self as i32
    }

    fn from_f32(value: f32) -> Self {
        value
    }

    fn from_i32(value: i32) -> Self {
        value as f32
    }
}
