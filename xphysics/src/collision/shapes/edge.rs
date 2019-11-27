use crate::collision::distance::DistanceProxy;
use crate::{settings, MassData, RayCastInput, RayCastOutput, Shape, ShapeType};
use std::borrow::Cow;
use xmath::{DotTrait, Multiply, Real, Transform, TransposeMultiply, Vector2, AABB};

pub struct ShapeEdge<T> {
    pub(crate) vertex1: Vector2<T>,
    pub(crate) vertex2: Vector2<T>,
    pub(crate) vertex0: Option<Vector2<T>>,
    pub(crate) vertex3: Option<Vector2<T>>,
}

impl<T: Real> ShapeEdge<T> {
    pub fn new(v1: Vector2<T>, v2: Vector2<T>) -> ShapeEdge<T> {
        ShapeEdge {
            vertex1: v1,
            vertex2: v2,
            vertex0: None,
            vertex3: None,
        }
    }

    pub fn new_with_adjacent(
        v1: Vector2<T>,
        v2: Vector2<T>,
        v0: Vector2<T>,
        v3: Vector2<T>,
    ) -> ShapeEdge<T> {
        ShapeEdge {
            vertex1: v1,
            vertex2: v2,
            vertex0: Some(v0),
            vertex3: Some(v3),
        }
    }
}

impl<T: Real> Shape<T> for ShapeEdge<T> {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Edge
    }

    fn radius(&self) -> T {
        settings::polygon_radius()
    }

    fn child_count(&self) -> usize {
        1
    }

    fn test_point(&self, _xf: &Transform<T>, _p: &Vector2<T>) -> bool {
        false
    }

    fn ray_cast(
        &self,
        input: &RayCastInput<T>,
        xf: &Transform<T>,
        _child_index: usize,
    ) -> Option<RayCastOutput<T>> {
        let p1 = xf.q.transpose_multiply(input.p1 - xf.p);
        let p2 = xf.q.transpose_multiply(input.p2 - xf.p);
        let d = p2 - p1;

        let v1 = self.vertex1;
        let v2 = self.vertex2;
        let e = v2 - v1;
        let normal = Vector2::new(e.y, -e.x).normalize();

        let numerator = normal.dot(v1 - p1);
        let denominator = normal.dot(d);

        if denominator == T::zero() {
            return None;
        }

        let t = numerator / denominator;
        if t < T::zero() || input.max_fraction < t {
            return None;
        }

        let q = p1 + d * t;

        let r = v2 - v1;
        let rr = r.dot(r);
        if rr == T::zero() {
            return None;
        }

        let s = (q - v1).dot(r) / rr;
        if s < T::zero() || T::one() < s {
            return None;
        }

        Some(RayCastOutput {
            normal: if numerator > T::zero() {
                -xf.q.multiply(normal)
            } else {
                xf.q.multiply(normal)
            },
            fraction: t,
        })
    }

    fn compute_aabb(&self, xf: &Transform<T>, _child_index: usize) -> AABB<T> {
        let v1 = xf.multiply(self.vertex1);
        let v2 = xf.multiply(self.vertex2);

        let lower = v1.min(v2);
        let upper = v1.max(v2);

        let r = Vector2::new(self.radius(), self.radius());
        AABB {
            lower_bound: lower - r,
            upper_bound: upper + r,
        }
    }

    fn compute_mass(&self, _density: T) -> MassData<T> {
        MassData {
            mass: T::zero(),
            center: (self.vertex1 + self.vertex2) * T::half(),
            i: T::zero(),
        }
    }

    fn distance_proxy(&self, _index: usize) -> DistanceProxy<'_, T> {
        DistanceProxy {
            vertices: Cow::Borrowed(unsafe {
                std::slice::from_raw_parts(&self.vertex1 as *const Vector2<T>, 2)
            }),
            radius: self.radius(),
        }
    }
}
