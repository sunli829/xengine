use crate::{MassData, RayCastInput, RayCastOutput};
use xmath::{Real, Transform, Vector2, AABB};

mod circle;
mod edge;

use crate::collision::distance::DistanceProxy;
pub use circle::Circle;
pub use edge::Edge;

pub enum ShapeType {
    Circle,
    Edge,
    Polygon,
    Chain,
    TypeCount,
}

pub trait Shape<T: Real> {
    fn shape_type(&self) -> ShapeType;

    fn radius(&self) -> T;

    fn child_count(&self) -> usize;

    fn test_point(&self, xf: &Transform<T>, p: &Vector2<T>) -> bool;

    fn ray_cast(
        &self,
        input: &RayCastInput<T>,
        xf: &Transform<T>,
        child_index: usize,
    ) -> Option<RayCastOutput<T>>;

    fn compute_aabb(&self, xf: &Transform<T>, child_index: usize) -> AABB<T>;

    fn compute_mass(&self, density: T) -> MassData<T>;

    fn distance_proxy(&self, index: usize) -> DistanceProxy<'_, T>;
}
