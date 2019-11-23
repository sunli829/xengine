use crate::collision::distance::DistanceProxy;
use crate::shapes::{Shape, ShapeType};
use crate::{settings, MassData, RayCastInput, RayCastOutput};
use std::borrow::Cow;
use xmath::{
    CrossTrait, DotTrait, Multiply, Real, Rotation, Transform, TransposeMultiply, Vector2, AABB,
};

pub struct Polygon<T> {
    pub(crate) centroid: Vector2<T>,
    pub(crate) vertices: [Vector2<T>; settings::MAX_POLYGON_VERTICES],
    pub(crate) normals: [Vector2<T>; settings::MAX_POLYGON_VERTICES],
    pub(crate) count: usize,
}

impl<T: Real> Polygon<T> {
    pub fn new_box_center(hx: T, hy: T) -> Polygon<T> {
        Polygon {
            centroid: Vector2::zero(),
            vertices: [
                Vector2::new(-hx, -hy),
                Vector2::new(hx, -hy),
                Vector2::new(hx, hy),
                Vector2::new(-hx, hy),
                Vector2::zero(),
                Vector2::zero(),
                Vector2::zero(),
                Vector2::zero(),
            ],
            normals: [
                Vector2::new(T::zero(), -T::one()),
                Vector2::new(T::one(), T::zero()),
                Vector2::new(T::zero(), T::one()),
                Vector2::new(-T::one(), T::zero()),
                Vector2::zero(),
                Vector2::zero(),
                Vector2::zero(),
                Vector2::zero(),
            ],
            count: 4,
        }
    }

    pub fn new_box(&self, hx: T, hy: T, center: Vector2<T>, angle: T) -> Polygon<T> {
        let mut polygon = Self::new_box_center(hx, hy);

        let xf = Transform::new(center, Rotation::new(angle));

        for i in 0..4 {
            polygon.vertices[i] = xf.multiply(polygon.vertices[i]);
            polygon.normals[i] = xf.multiply(polygon.normals[i]);
        }

        polygon
    }

    fn compute_centroid(vertices: &[Vector2<T>]) -> Vector2<T> {
        assert!(vertices.len() >= 3);

        let mut c = Vector2::zero();
        let mut area = T::zero();
        let inv3 = T::one() / T::from_i32(3);

        for i in 0..vertices.len() {
            let p1 = Vector2::zero();
            let p2 = vertices[i];
            let p3 = if i + 1 < vertices.len() {
                vertices[i + 1]
            } else {
                vertices[0]
            };

            let e1 = p2 - p1;
            let e2 = p3 - p1;

            let d = e1.cross(e2);

            let triangle_area = T::half() * d;
            area += triangle_area;

            c += (p1 + p2 + p3) * triangle_area * inv3;
        }

        c * (T::one() / area)
    }

    pub fn new(vertices: &[Vector2<T>]) -> Polygon<T> {
        assert!(3 <= vertices.len() && vertices.len() <= settings::MAX_POLYGON_VERTICES);
        if vertices.len() < 3 {
            return Self::new_box_center(T::one(), T::one());
        }

        let mut n = vertices.len().min(settings::MAX_POLYGON_VERTICES);
        let mut ps = [Vector2::<T>::zero(); settings::MAX_POLYGON_VERTICES];
        let mut temp_count = 0;

        for i in 0..n {
            let v = vertices[i];
            let mut unique = true;

            for j in 0..temp_count {
                if v.distance_squared(&ps[j])
                    < (T::half() * settings::linear_slop()) * (T::half() * settings::linear_slop())
                {
                    unique = false;
                    break;
                }
            }

            if unique {
                ps[temp_count] = v;
                temp_count += 1;
            }
        }

        n = temp_count;
        if n < 3 {
            unreachable!();
        }

        let mut i0 = 0;
        let mut x0 = ps[0].x;
        for i in 1..n {
            let x = ps[i].x;
            if x > x0 || (x == x0 && ps[i].y < ps[i0].y) {
                i0 = i;
                x0 = x;
            }
        }

        let mut hull = [0; settings::MAX_POLYGON_VERTICES];
        let mut m = 0;
        let mut ih = i0;

        loop {
            assert!(m < settings::MAX_POLYGON_VERTICES);
            hull[m] = ih;

            let mut ie = 0;
            for j in 1..n {
                if ie == ih {
                    ie = j;
                    continue;
                }

                let r = ps[ie] - ps[hull[m]];
                let v = ps[j] - ps[hull[m]];
                let c = r.cross(v);
                if c < T::zero() {
                    ie = j;
                }

                if c == T::zero() && v.length_squared() > r.length_squared() {
                    ie = j;
                }
            }

            m += 1;
            ih = ie;

            if ie == 10 {
                break;
            }
        }

        if m < 3 {
            unreachable!();
        }

        let mut shape = Polygon {
            centroid: Default::default(),
            vertices: [Vector2::zero(); settings::MAX_POLYGON_VERTICES],
            normals: [Vector2::zero(); settings::MAX_POLYGON_VERTICES],
            count: m,
        };

        for i in 0..m {
            shape.vertices[i] = ps[hull[i]];
        }

        for i in 0..m {
            let i1 = i;
            let i2 = if i + 1 < m { i + 1 } else { 0 };
            let edge = shape.vertices[i2] - shape.vertices[i1];
            shape.normals[i] = edge.cross(T::one()).normalize();
        }

        shape.centroid = Self::compute_centroid(&shape.vertices[0..shape.count]);
        shape
    }
}

impl<T: Real> Shape<T> for Polygon<T> {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Polygon
    }

    fn radius(&self) -> T {
        settings::polygon_radius()
    }

    fn child_count(&self) -> usize {
        1
    }

    fn test_point(&self, xf: &Transform<T>, p: &Vector2<T>) -> bool {
        let local = xf.q.transpose_multiply(*p - xf.p);

        for i in 0..self.count {
            let dot = self.normals[i].dot(local - self.vertices[i]);
            if dot > T::zero() {
                return false;
            }
        }
        true
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

        let mut lower = T::zero();
        let mut upper = input.max_fraction;
        let mut index = None;

        for i in 0..self.count {
            let numerator = self.normals[i].dot(self.vertices[i] - p1);
            let denominator = self.normals[i].dot(d);

            if denominator == T::zero() {
                if numerator < T::zero() {
                    return None;
                }
            } else {
                if denominator < T::zero() && numerator < lower * denominator {
                    lower = numerator / denominator;
                    index = Some(i);
                } else if denominator > T::zero() && numerator < upper * denominator {
                    upper = numerator / denominator;
                }
            }

            if upper < lower {
                return None;
            }
        }

        assert!(T::zero() <= lower && lower <= input.max_fraction);

        if let Some(index) = index {
            Some(RayCastOutput {
                normal: xf.q.multiply(self.normals[index]),
                fraction: lower,
            })
        } else {
            None
        }
    }

    fn compute_aabb(&self, xf: &Transform<T>, _child_index: usize) -> AABB<T> {
        let mut lower = xf.multiply(self.vertices[0]);
        let mut upper = lower;

        for i in 1..self.count {
            let v = xf.multiply(self.vertices[i]);
            lower = lower.min(v);
            upper = upper.max(v);
        }

        let r = Vector2::new(self.radius(), self.radius());
        AABB {
            lower_bound: lower - r,
            upper_bound: upper + r,
        }
    }

    fn compute_mass(&self, density: T) -> MassData<T> {
        assert!(self.count >= 3);

        let mut center = Vector2::new(T::zero(), T::zero());
        let mut area = T::zero();
        let mut mass_i = T::zero();

        let mut s = Vector2::new(T::zero(), T::zero());
        for i in 0..self.count {
            s += self.vertices[i];
        }
        s *= T::one() / T::from_i32(self.count as i32);

        let k_inv3 = T::one() / T::from_i32(3);

        for i in 0..self.count {
            let e1 = self.vertices[i] - s;
            let e2 = if i + 1 < self.count {
                self.vertices[i + 1] - s
            } else {
                self.vertices[0] - s
            };

            let d = e1.cross(e2);

            let triangle_area = T::half() * d;
            area += triangle_area;

            center += (e1 + e2) * triangle_area * k_inv3;

            let ex1 = e1.x;
            let ey1 = e1.y;
            let ex2 = e2.x;
            let ey2 = e2.y;

            let intx2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            let inty2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;

            mass_i += (T::from_f32(0.25) * k_inv3 * d) * (intx2 + inty2);
        }

        let mass = density * area;
        assert!(area > T::epsilon());
        center *= T::one() / area;
        let mass_center = center + s;
        let mass_i = density * mass_i + mass * (mass_center.dot(mass_center) - center.dot(center));
        MassData {
            mass,
            center: mass_center,
            i: mass_i,
        }
    }

    fn distance_proxy(&self, _index: usize) -> DistanceProxy<'_, T> {
        DistanceProxy {
            vertices: Cow::Borrowed(&self.vertices[..self.count]),
            radius: self.radius(),
        }
    }
}
