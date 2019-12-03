use crate::{MassData, RayCastInput, RayCastOutput};
use xmath::{Real, Transform, Vector2, AABB};

mod chain;
mod circle;
mod edge;
mod polygon;

use crate::collision::distance::DistanceProxy;
pub use chain::ShapeChain;
pub use circle::ShapeCircle;
pub use edge::ShapeEdge;
pub use polygon::ShapePolygon;

pub enum ShapeType {
    Circle,
    Edge,
    Polygon,
    Chain,
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

pub trait IntoBoxedShape<T: Real>: Shape<T> {
    fn into_boxed(self) -> Box<dyn Shape<T>>;
}

impl<T: Real, S: Shape<T> + Sized + 'static> IntoBoxedShape<T> for S {
    fn into_boxed(self) -> Box<dyn Shape<T>> {
        Box::new(self)
    }
}
