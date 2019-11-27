use crate::{ContactId, Manifold, ManifoldPoint, ManifoldType, Shape, ShapeCircle, ShapePolygon};
use xmath::{DotTrait, Multiply, Real, Transform, TransposeMultiply, Vector2};

pub fn collide_circles<T: Real>(
    manifold: &mut Manifold<T>,
    circle_a: &ShapeCircle<T>,
    xf_a: &Transform<T>,
    circle_b: &ShapeCircle<T>,
    xf_b: &Transform<T>,
) {
    let pa = xf_a.multiply(circle_a.position);
    let pb = xf_b.multiply(circle_b.position);

    let d = pb - pa;
    let dist_sqr = d.dot(d);
    let ra = circle_a.radius;
    let rb = circle_b.radius;
    let radius = ra + rb;
    if dist_sqr > radius * radius {
        return;
    }

    manifold.type_ = ManifoldType::Circles;
    manifold.local_point = circle_a.position;
    manifold.local_normal = Vector2::zero();

    manifold.point_count = 1;
    manifold.points[0] = ManifoldPoint {
        local_point: circle_b.position,
        normal_impulse: T::zero(),
        tangent_impulse: T::zero(),
        id: ContactId::zero(),
    };
}

pub fn collide_polygon_and_circle<T: Real>(
    manifold: &mut Manifold<T>,
    polygon_a: &ShapePolygon<T>,
    xf_a: &Transform<T>,
    circle_b: &ShapeCircle<T>,
    xf_b: &Transform<T>,
) {
    let c = xf_b.multiply(circle_b.position);
    let c_local = xf_a.transpose_multiply(c);

    let mut normal_index = 0;
    let mut separation = -T::max_value();
    let radius = polygon_a.radius() + circle_b.radius();
    let vertex_count = polygon_a.count;
    let vertices = &polygon_a.vertices;
    let normals = &polygon_a.normals;

    for i in 0..vertex_count {
        let s = normals[i].dot(c_local - vertices[i]);
        if s > radius {
            return;
        }
        if s > separation {
            separation = s;
            normal_index = i;
        }
    }

    let vert_index1 = normal_index;
    let vert_index2 = if vert_index1 + 1 < vertex_count {
        vert_index1 + 1
    } else {
        0
    };
    let v1 = vertices[vert_index1];
    let v2 = vertices[vert_index2];

    if separation < T::epsilon() {
        manifold.point_count = 1;
        manifold.type_ = ManifoldType::FaceA;
        manifold.local_normal = normals[normal_index];
        manifold.local_point = (v1 + v2) * T::half();
        manifold.points[0].local_point = circle_b.position;
        manifold.points[0].id = ContactId::zero();
        return;
    }

    let u1 = (c_local - v1).dot(v2 - v1);
    let u2 = (c_local - v2).dot(v1 - v2);
    if u1 <= T::zero() {
        if c_local.distance_squared(&v1) > radius * radius {
            return;
        }

        manifold.point_count = 1;
        manifold.type_ = ManifoldType::FaceA;
        manifold.local_normal = (c_local - v1).normalize();
        manifold.local_point = v1;
        manifold.points[0].local_point = circle_b.position;
        manifold.points[0].id = ContactId::zero();
    } else if u2 <= T::zero() {
        if c_local.distance_squared(&v2) > radius * radius {
            return;
        }

        manifold.point_count = 1;
        manifold.type_ = ManifoldType::FaceA;
        manifold.local_normal = (c_local - v2).normalize();
        manifold.local_point = v2;
        manifold.points[0].local_point = circle_b.position;
        manifold.points[0].id = ContactId::zero();
    } else {
        let face_center = (v1 + v2) * T::half();
        let s = (c_local - face_center).dot(normals[vert_index1]);
        if s > radius {
            return;
        }

        manifold.point_count = 1;
        manifold.type_ = ManifoldType::FaceA;
        manifold.local_normal = normals[vert_index1];
        manifold.local_point = face_center;
        manifold.points[0].local_point = circle_b.position;
        manifold.points[0].id = ContactId::zero();
    }
}
