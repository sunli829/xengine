use xmath::Real;

pub const MAX_MANIFOLD_POINTS: usize = 2;
pub const MAX_POLYGON_VERTICES: usize = 8;
pub const MAX_SUB_STEPS: usize = 8;

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
