use crate::collision::distance::DistanceProxy;
use crate::{MassData, RayCastInput, RayCastOutput, Shape, ShapeType};
use std::borrow::Cow;
use xmath::{DotTrait, Multiply, Real, Transform, Vector2, AABB};

pub struct ShapeCircle<T> {
    pub(crate) radius: T,
    pub(crate) position: Vector2<T>,
}

impl<T: Real> ShapeCircle<T> {
    pub fn new<P, R>(position: P, radius: R) -> ShapeCircle<T>
    where
        P: Into<Vector2<T>>,
        R: Into<T>,
    {
        ShapeCircle {
            position: position.into(),
            radius: radius.into(),
        }
    }
}

impl<T: Real> Shape<T> for ShapeCircle<T> {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Circle
    }

    fn radius(&self) -> T {
        self.radius
    }

    fn child_count(&self) -> usize {
        1
    }

    fn test_point(&self, xf: &Transform<T>, p: &Vector2<T>) -> bool {
        let center = xf.p + xf.q.multiply(*p);
        let d = *p - center;
        d.dot(d) <= self.radius * self.radius
    }

    fn ray_cast(
        &self,
        input: &RayCastInput<T>,
        xf: &Transform<T>,
        _child_index: usize,
    ) -> Option<RayCastOutput<T>> {
        let position = xf.p + xf.q.multiply(self.position);
        let s = input.p1 - position;
        let b = s.dot(s) - self.radius * self.radius;

        let r = input.p2 - input.p1;
        let c = s.dot(r);
        let rr = r.dot(r);
        let sigma = c * c - rr * b;

        if sigma < T::zero() || rr < T::epsilon() {
            return None;
        }

        let mut a = -(c + sigma.sqrt());

        if T::zero() <= a && a <= input.max_fraction * rr {
            a /= rr;
            Some(RayCastOutput {
                fraction: a,
                normal: (s + r * a).normalize(),
            })
        } else {
            None
        }
    }

    fn compute_aabb(&self, xf: &Transform<T>, _child_index: usize) -> AABB<T> {
        let p = xf.p + xf.q.multiply(self.position);
        AABB {
            lower_bound: Vector2::new(p.x - self.radius, p.y - self.radius),
            upper_bound: Vector2::new(p.x + self.radius, p.y + self.radius),
        }
    }

    fn compute_mass(&self, density: T) -> MassData<T> {
        let mass = density * T::pi() * self.radius * self.radius;
        let center = self.position;
        let i = mass * (T::half() * self.radius * self.radius + self.position.dot(self.position));
        MassData { mass, center, i }
    }

    fn distance_proxy(&self, _index: usize) -> DistanceProxy<'_, T> {
        DistanceProxy {
            vertices: Cow::Borrowed(std::slice::from_ref(&self.position)),
            radius: self.radius,
        }
    }
}
