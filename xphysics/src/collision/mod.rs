use xmath::Vector2;

mod distance;
mod dyntree;
pub mod shapes;
mod time_of_impact;

pub struct MassData<T> {
    pub mass: T,
    pub center: Vector2<T>,
    pub i: T,
}

pub struct RayCastInput<T> {
    pub p1: Vector2<T>,
    pub p2: Vector2<T>,
    pub max_fraction: T,
}

pub struct RayCastOutput<T> {
    pub normal: Vector2<T>,
    pub fraction: T,
}
