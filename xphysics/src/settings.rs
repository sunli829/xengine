use xmath::Real;

pub const MAX_MANIFOLD_POINTS: usize = 2;
pub const MAX_POLYGON_VERTICES: usize = 8;
pub const MAX_SUB_STEPS: usize = 8;
pub const MAX_TOI_CONTACTS: usize = 32;

#[inline]
pub fn aabb_extension<T: Real>() -> T {
    T::en1()
}

#[inline]
pub fn aabb_multiplier<T: Real>() -> T {
    T::from_i32(2)
}

#[inline]
pub fn linear_slop<T: Real>() -> T {
    T::en3() * T::from_i32(5)
}

#[inline]
pub fn angular_slop<T: Real>() -> T {
    (T::from_i32(2) / T::from_i32(180) * T::pi())
}

#[inline]
pub fn polygon_radius<T: Real>() -> T {
    linear_slop::<T>() * T::from_i32(2)
}

#[inline]
pub fn velocity_threshold<T: Real>() -> T {
    T::one()
}

#[inline]
pub fn max_linear_correction<T: Real>() -> T {
    T::from_f32(0.2)
}

#[inline]
pub fn max_angular_correction<T: Real>() -> T {
    T::from_i32(8) / T::from_i32(180) * T::pi()
}

#[inline]
pub fn max_translation<T: Real>() -> T {
    T::two()
}

#[inline]
pub fn max_translation_squared<T: Real>() -> T {
    max_translation::<T>() * max_translation::<T>()
}

#[inline]
pub fn max_rotation<T: Real>() -> T {
    T::pi_over_2()
}

#[inline]
pub fn max_rotation_squared<T: Real>() -> T {
    max_rotation::<T>() * max_rotation::<T>()
}

#[inline]
pub fn baumgarte<T: Real>() -> T {
    T::from_f32(0.2)
}

#[inline]
pub fn toi_baugarte<T: Real>() -> T {
    T::from_f32(0.75)
}

#[inline]
pub fn time_to_sleep<T: Real>() -> T {
    T::half()
}

#[inline]
pub fn linear_sleep_tolerance<T: Real>() -> T {
    T::en1()
}

#[inline]
pub fn angular_sleep_tolerance<T: Real>() -> T {
    T::from_i32(2) / T::from_i32(180) * T::pi()
}
