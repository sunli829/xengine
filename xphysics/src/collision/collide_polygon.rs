use crate::settings;
use crate::shapes::{Polygon, Shape};
use crate::{clip_segment_to_line, ClipVertex, ContactFeatureType, Manifold, ManifoldType};
use xmath::{CrossTrait, DotTrait, Multiply, Real, Transform, TransposeMultiply};

fn find_max_separation<T: Real>(
    edge_index: &mut usize,
    poly1: &Polygon<T>,
    xf1: &Transform<T>,
    poly2: &Polygon<T>,
    xf2: &Transform<T>,
) -> T {
    let count1 = poly1.count;
    let count2 = poly2.count;
    let n1s = &poly1.normals;
    let v1s = &poly1.vertices;
    let v2s = &poly2.vertices;
    let xf = xf2.transpose_multiply(*xf1);

    let mut best_index = 0;
    let mut max_separation = -T::max_value();
    for i in 0..count1 {
        let n = xf.q.multiply(n1s[i]);
        let v1 = xf.multiply(v1s[i]);
        let mut si = T::max_value();

        for j in 0..count2 {
            let sij = n.dot(v2s[j] - v1);
            if sij < si {
                si = sij;
            }
        }

        if si > max_separation {
            max_separation = si;
            best_index = i;
        }
    }

    *edge_index = best_index;
    max_separation
}

fn find_incident_edge<T: Real>(
    c: &mut [ClipVertex<T>; 2],
    poly1: &Polygon<T>,
    xf1: &Transform<T>,
    edge1: usize,
    poly2: &Polygon<T>,
    xf2: &Transform<T>,
) {
    let normals1 = poly1.normals;

    let count2 = poly2.count;
    let vertices2 = &poly2.vertices;
    let normals2 = &poly2.normals;

    assert!(edge1 < poly1.count);

    let normal1 = xf2.q.transpose_multiply(xf1.q.multiply(normals1[edge1]));

    let mut index = 0;
    let mut min_dot = T::max_value();
    for i in 0..count2 {
        let dot = normal1.dot(normals2[i]);
        if dot < min_dot {
            min_dot = dot;
            index = i;
        }
    }

    let i1 = index;
    let i2 = if i1 + 1 < count2 { i1 + 1 } else { 0 };

    c[0].v = xf2.multiply(vertices2[i1]);
    c[0].id.0.index_a = edge1 as u8;
    c[0].id.0.index_b = i1 as u8;
    c[0].id.0.type_a = ContactFeatureType::Face;
    c[0].id.0.type_b = ContactFeatureType::Vertex;

    c[1].v = xf2.multiply(vertices2[i2]);
    c[1].id.0.index_a = edge1 as u8;
    c[1].id.0.index_b = i2 as u8;
    c[1].id.0.type_a = ContactFeatureType::Face;
    c[1].id.0.type_b = ContactFeatureType::Vertex;
}

pub fn collide_polygons<T: Real>(
    manifold: &mut Manifold<T>,
    poly_a: &Polygon<T>,
    xf_a: &Transform<T>,
    poly_b: &Polygon<T>,
    xf_b: &Transform<T>,
) {
    manifold.point_count = 0;
    let total_radius = poly_a.radius() + poly_b.radius();

    let mut edge_a = 0;
    let separation_a = find_max_separation(&mut edge_a, poly_a, xf_a, poly_b, xf_b);
    if separation_a > total_radius {
        return;
    }

    let mut edge_b = 0;
    let separation_b = find_max_separation(&mut edge_b, poly_b, xf_b, poly_a, xf_a);
    if separation_b > total_radius {
        return;
    }

    let poly1;
    let poly2;
    let xf1;
    let xf2;
    let edge1;
    let flip;
    let k_tol = T::en1() * settings::linear_slop();

    if separation_b > separation_a + k_tol {
        poly1 = poly_b;
        poly2 = poly_a;
        xf1 = xf_b;
        xf2 = xf_a;
        edge1 = edge_b;
        manifold.type_ = ManifoldType::FaceB;
        flip = true;
    } else {
        poly1 = poly_a;
        poly2 = poly_b;
        xf1 = xf_a;
        xf2 = xf_b;
        edge1 = edge_a;
        manifold.type_ = ManifoldType::FaceA;
        flip = false;
    }

    let mut incident_edge: [ClipVertex<T>; 2] = unsafe { std::mem::zeroed() };
    find_incident_edge(&mut incident_edge, poly1, xf1, edge1, poly2, xf2);

    let count1 = poly1.count;
    let vertices1 = poly1.vertices;

    let iv1 = edge1;
    let iv2 = if edge1 + 1 < count1 { edge1 + 1 } else { 0 };

    let mut v11 = vertices1[iv1];
    let mut v12 = vertices1[iv2];

    let local_tangent = (v12 - v11).normalize();

    let local_normal = local_tangent.cross(T::one());
    let plane_point = (v11 + v12) * T::half();

    let tangent = xf1.q.multiply(local_tangent);
    let normal = tangent.cross(T::one());

    v11 = xf1.multiply(v11);
    v12 = xf1.multiply(v12);

    let front_offset = normal.dot(v11);

    let side_offset1 = -tangent.dot(v11) + total_radius;
    let side_offset2 = tangent.dot(v12) + total_radius;

    let mut clip_points1: [ClipVertex<T>; 2] = unsafe { std::mem::zeroed() };
    let mut clip_points2: [ClipVertex<T>; 2] = unsafe { std::mem::zeroed() };
    let mut np = clip_segment_to_line(
        &mut clip_points1,
        &incident_edge,
        &-tangent,
        side_offset1,
        iv1,
    );

    if np < 2 {
        return;
    }

    np = clip_segment_to_line(
        &mut clip_points2,
        &clip_points1,
        &tangent,
        side_offset2,
        iv2,
    );

    if np < 2 {
        return;
    }

    manifold.local_normal = local_normal;
    manifold.local_point = plane_point;

    let mut point_count = 0;
    for i in 0..settings::MAX_MANIFOLD_POINTS {
        let separation = normal.dot(clip_points2[i].v) - front_offset;

        if separation <= total_radius {
            let cp = &mut manifold.points[point_count];
            cp.local_point = xf2.transpose_multiply(clip_points2[i].v);
            cp.id = clip_points2[i].id;
            if flip {
                let cf = cp.id.0;
                cp.id.0.index_a = cf.index_b;
                cp.id.0.index_b = cf.index_a;
                cp.id.0.type_a = cf.type_b;
                cp.id.0.type_b = cf.type_a;
            }
            point_count += 1;
        }
    }

    manifold.point_count = point_count;
}
