use crate::{Real, RealConstants};

impl Real for f32 {
    fn is_valid(&self) -> bool {
        f32::is_normal(*self)
    }

    fn sqrt(&self) -> Self {
        f32::sqrt(*self)
    }

    fn abs(&self) -> Self {
        f32::abs(*self)
    }

    fn max(&self, other: Self) -> Self {
        f32::max(*self, other)
    }

    fn min(&self, other: Self) -> Self {
        f32::min(*self, other)
    }
}

impl RealConstants for f32 {
    fn zero() -> Self {
        0.0
    }

    fn one() -> Self {
        1.0
    }

    fn epsilon() -> Self {
        std::f32::EPSILON
    }

    fn half() -> Self {
        0.5
    }

    fn en1() -> Self {
        0.1
    }
}
