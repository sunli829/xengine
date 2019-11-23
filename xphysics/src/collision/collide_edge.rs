use crate::collision::shapes::Polygon;
use crate::shapes::{Circle, Edge, Shape};
use crate::{
    clip_segment_to_line, settings, ClipVertex, ContactFeature, ContactFeatureType, Manifold,
    ManifoldType,
};
use xmath::{CrossTrait, DotTrait, Multiply, Real, Transform, TransposeMultiply, Vector2};

pub fn collide_edge_and_circle<T: Real>(
    manifold: &mut Manifold<T>,
    edge_a: &Edge<T>,
    xf_a: &Transform<T>,
    circle_b: &Circle<T>,
    xf_b: &Transform<T>,
) {
    manifold.point_count = 0;

    let q = xf_a.transpose_multiply(xf_b.multiply(circle_b.position));

    let a = edge_a.vertex1;
    let b = edge_a.vertex2;
    let e = b - a;

    let u = e.dot(b - q);
    let v = e.dot(q - a);

    let radius = edge_a.radius() + circle_b.radius();

    let mut cf = ContactFeature {
        index_a: 0,
        index_b: 0,
        type_a: ContactFeatureType::Vertex,
        type_b: ContactFeatureType::Vertex,
    };

    // Region A
    if v <= T::zero() {
        let p = a;
        let d = q - p;
        let dd = d.dot(d);
        if dd > radius * radius {
            return;
        }

        if let Some(vertex0) = edge_a.vertex0 {
            let a1 = vertex0;
            let b1 = a;
            let e1 = b1 - a1;
            let u1 = e1.dot(b1 - q);

            if u1 > T::zero() {
                return;
            }
        }

        cf.index_a = 0;
        cf.type_a = ContactFeatureType::Vertex;
        manifold.point_count = 1;
        manifold.type_ = ManifoldType::Circles;
        manifold.local_normal = Vector2::zero();
        manifold.local_point = p;
        manifold.points[0].id.0 = cf;
        manifold.points[0].local_point = circle_b.position;
        return;
    }

    if u <= T::zero() {
        let p = b;
        let d = q - p;
        let dd = d.dot(d);
        if dd > radius * radius {
            return;
        }

        if let Some(vertex3) = edge_a.vertex3 {
            let b2 = vertex3;
            let a2 = b;
            let e2 = b2 - a2;
            let v2 = e2.dot(q - a2);

            if v2 > T::zero() {
                return;
            }
        }

        cf.index_a = 1;
        cf.type_a = ContactFeatureType::Vertex;
        manifold.point_count = 1;
        manifold.type_ = ManifoldType::Circles;
        manifold.local_normal = Vector2::zero();
        manifold.local_point = p;
        manifold.points[0].id.0 = cf;
        manifold.points[0].local_point = circle_b.position;
        return;
    }

    let den = e.dot(e);
    assert!(den > T::zero());
    let p = (a * u + b * v) * (T::one() / den);
    let d = q - p;
    let dd = d.dot(d);
    if dd > radius * radius {
        return;
    }

    let mut n = Vector2::new(-e.y, e.x);
    if n.dot(q - a) < T::zero() {
        n = -n;
    }
    n = n.normalize();

    cf.index_a = 0;
    cf.type_a = ContactFeatureType::Face;
    manifold.point_count = 1;
    manifold.type_ = ManifoldType::FaceA;
    manifold.local_normal = n;
    manifold.local_point = a;
    manifold.points[0].id.0 = cf;
    manifold.points[0].local_point = circle_b.position;
}

#[derive(Debug, PartialEq, Eq)]
enum EPAxisType {
    Unknown,
    EdgeA,
    EdgeB,
}

struct EPAxis<T> {
    type_: EPAxisType,
    index: usize,
    separation: T,
}

struct TempPolygon<T: Real> {
    vertices: [Vector2<T>; settings::MAX_POLYGON_VERTICES],
    normals: [Vector2<T>; settings::MAX_POLYGON_VERTICES],
    count: usize,
}

struct ReferenceFace<T> {
    i1: usize,
    i2: usize,
    v1: Vector2<T>,
    v2: Vector2<T>,
    normal: Vector2<T>,
    side_normal1: Vector2<T>,
    side_offset1: T,
    side_normal2: Vector2<T>,
    side_offset2: T,
}

enum VertexType {
    ISOLATED,
    CONCAVE,
    CONVEX,
}

struct EPCollider<T: Real> {
    polygon_b: TempPolygon<T>,
    xf: Transform<T>,
    centroid_b: Vector2<T>,
    v0: Option<Vector2<T>>,
    v1: Vector2<T>,
    v2: Vector2<T>,
    v3: Option<Vector2<T>>,
    normal0: Vector2<T>,
    normal1: Vector2<T>,
    normal2: Vector2<T>,
    normal: Vector2<T>,
    type1: VertexType,
    type2: VertexType,
    lower_limit: Vector2<T>,
    upper_limit: Vector2<T>,
    radius: T,
    front: bool,
}

impl<T: Real> EPCollider<T> {
    fn compute_edge_separation(&mut self) -> EPAxis<T> {
        let mut axis = EPAxis {
            type_: EPAxisType::EdgeA,
            index: if self.front { 0 } else { 1 },
            separation: T::max_value(),
        };

        for i in 0..self.polygon_b.count {
            let s = self.normal.dot(self.polygon_b.vertices[i] - self.v1);
            if s < axis.separation {
                axis.separation = s;
            }
        }
        axis
    }

    fn compute_polygon_separation(&mut self) -> EPAxis<T> {
        unimplemented!()
    }

    fn collide(
        &mut self,
        manifold: &mut Manifold<T>,
        edge_a: &Edge<T>,
        xf_a: &Transform<T>,
        polygon_b: &Polygon<T>,
        xf_b: &Transform<T>,
    ) {
        self.xf = xf_a.transpose_multiply(*xf_b);
        self.centroid_b = self.xf.multiply(polygon_b.centroid);

        self.v0 = edge_a.vertex0;
        self.v1 = edge_a.vertex1;
        self.v2 = edge_a.vertex2;
        self.v3 = edge_a.vertex3;

        let edge1 = (self.v2 - self.v1).normalize();
        self.normal1 = Vector2::new(edge1.y, -edge1.x);
        let offset1 = self.normal1.dot(self.centroid_b - self.v1);
        let mut offset0 = T::zero();
        let mut offset2 = T::zero();
        let mut convex1 = false;
        let mut convex2 = false;

        if let Some(v0) = self.v0 {
            let edge0 = (self.v1 - v0).normalize();
            self.normal0 = Vector2::new(edge0.y, -edge0.x);
            convex1 = edge0.cross(edge1) >= T::zero();
            offset0 = self.normal0.dot(self.centroid_b - v0);
        }

        if let Some(v3) = self.v3 {
            let edge2 = (v3 - self.v2).normalize();
            self.normal2 = Vector2::new(edge2.y, -edge2.x);
            convex2 = edge1.cross(edge2) > T::zero();
            offset2 = self.normal2.dot(self.centroid_b - self.v2);
        }

        if self.v0.is_some() && self.v3.is_some() {
            if convex1 && convex2 {
                self.front = offset0 >= T::zero() || offset1 >= T::zero() || offset2 >= T::zero();
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = self.normal0;
                    self.upper_limit = self.normal2;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = -self.normal1;
                    self.upper_limit = -self.normal1;
                }
            } else if convex1 {
                self.front = offset0 >= T::zero() || (offset1 >= T::zero() && offset2 >= T::zero());
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = self.normal0;
                    self.upper_limit = self.normal1;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = -self.normal2;
                    self.upper_limit = -self.normal1;
                }
            } else if convex2 {
                self.front = offset2 >= T::zero() || (offset0 >= T::zero() && offset1 >= T::zero());
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = self.normal1;
                    self.upper_limit = self.normal2;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = -self.normal1;
                    self.upper_limit = -self.normal0;
                }
            } else {
                self.front = offset0 >= T::zero() && offset1 >= T::zero() && offset2 >= T::zero();
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = self.normal1;
                    self.upper_limit = self.normal1;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = -self.normal2;
                    self.upper_limit = -self.normal0;
                }
            }
        } else if self.v0.is_some() {
            if convex1 {
                self.front = offset0 >= T::zero() || offset1 >= T::zero();
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = self.normal0;
                    self.upper_limit = -self.normal1;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = self.normal1;
                    self.upper_limit = -self.normal1;
                }
            } else {
                self.front = offset0 >= T::zero() && offset1 >= T::zero();
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = self.normal1;
                    self.upper_limit = -self.normal1;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = self.normal1;
                    self.upper_limit = -self.normal0;
                }
            }
        } else if self.v3.is_some() {
            if convex2 {
                self.front = offset1 >= T::zero() || offset2 >= T::zero();
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = -self.normal1;
                    self.upper_limit = self.normal2;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = -self.normal1;
                    self.upper_limit = self.normal1;
                }
            } else {
                self.front = offset1 >= T::zero() && offset2 >= T::zero();
                if self.front {
                    self.normal = self.normal1;
                    self.lower_limit = -self.normal1;
                    self.upper_limit = self.normal1;
                } else {
                    self.normal = -self.normal1;
                    self.lower_limit = -self.normal2;
                    self.upper_limit = self.normal1;
                }
            }
        } else {
            self.front = offset1 >= T::zero();
            if self.front {
                self.normal = self.normal1;
                self.lower_limit = -self.normal1;
                self.upper_limit = -self.normal1;
            } else {
                self.normal = -self.normal1;
                self.lower_limit = self.normal1;
                self.upper_limit = self.normal1;
            }
        }

        self.polygon_b.count = polygon_b.count;
        for i in 0..polygon_b.count {
            self.polygon_b.vertices[i] = self.xf.multiply(polygon_b.vertices[i]);
            self.polygon_b.normals[i] = self.xf.q.multiply(polygon_b.normals[i]);
        }

        self.radius = polygon_b.radius() + edge_a.radius();

        manifold.point_count = 0;

        let edge_axis = self.compute_edge_separation();

        if edge_axis.type_ == EPAxisType::Unknown {
            return;
        }

        if edge_axis.separation > self.radius {
            return;
        }

        let polygon_axis = self.compute_polygon_separation();
        if polygon_axis.type_ != EPAxisType::Unknown && polygon_axis.separation > self.radius {
            return;
        }

        let k_relative_tol = T::one() - (T::two() * T::en2());
        let k_absolute_tol = T::en3();

        let primary_axis = if polygon_axis.type_ == EPAxisType::Unknown {
            edge_axis
        } else if polygon_axis.separation > k_relative_tol * edge_axis.separation + k_absolute_tol {
            polygon_axis
        } else {
            edge_axis
        };

        let mut ie: [ClipVertex<T>; 2] = unsafe { std::mem::zeroed() };
        let mut rf: ReferenceFace<T> = unsafe { std::mem::zeroed() };
        if primary_axis.type_ == EPAxisType::EdgeA {
            manifold.type_ = ManifoldType::FaceA;

            let mut best_index = 0;
            let mut best_value = self.normal.dot(self.polygon_b.normals[0]);
            for i in 1..self.polygon_b.count {
                let value = self.normal.dot(self.polygon_b.normals[i]);
                if value < best_value {
                    best_value = value;
                    best_index = i;
                }
            }

            let i1 = best_index;
            let i2 = if i1 + 1 < self.polygon_b.count {
                i1 + 1
            } else {
                0
            };

            ie[0].v = self.polygon_b.vertices[i1];
            ie[0].id.0.index_a = 0;
            ie[0].id.0.index_b = i1 as u8;
            ie[0].id.0.type_a = ContactFeatureType::Face;
            ie[0].id.0.type_b = ContactFeatureType::Vertex;

            ie[1].v = self.polygon_b.vertices[i2];
            ie[1].id.0.index_a = 0;
            ie[1].id.0.index_b = i2 as u8;
            ie[1].id.0.type_a = ContactFeatureType::Face;
            ie[1].id.0.type_b = ContactFeatureType::Vertex;

            if self.front {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1 = self.v1;
                rf.v2 = self.v2;
                rf.normal = self.normal1;
            } else {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1 = self.v2;
                rf.v2 = self.v1;
                rf.normal = -self.normal1;
            }
        } else {
            manifold.type_ = ManifoldType::FaceB;

            ie[0].v = self.v1;
            ie[0].id.0.index_a = 0;
            ie[0].id.0.index_b = primary_axis.index as u8;
            ie[0].id.0.type_a = ContactFeatureType::Vertex;
            ie[0].id.0.type_b = ContactFeatureType::Face;

            ie[1].v = self.v2;
            ie[1].id.0.index_a = 0;
            ie[1].id.0.index_b = primary_axis.index as u8;
            ie[1].id.0.type_a = ContactFeatureType::Vertex;
            ie[1].id.0.type_b = ContactFeatureType::Face;

            rf.i1 = primary_axis.index;
            rf.i2 = if rf.i1 + 1 < self.polygon_b.count {
                rf.i1 + 1
            } else {
                0
            };
            rf.v1 = self.polygon_b.vertices[rf.i1];
            rf.v2 = self.polygon_b.vertices[rf.i2];
            rf.normal = self.polygon_b.normals[rf.i1];
        }

        rf.side_normal1 = Vector2::new(rf.normal.y, -rf.normal.x);
        rf.side_normal2 = -rf.side_normal1;
        rf.side_offset1 = rf.side_normal1.dot(rf.v1);
        rf.side_offset2 = rf.side_normal2.dot(rf.v2);

        let mut clip_points1: [ClipVertex<T>; 2] = unsafe { std::mem::zeroed() };
        let mut clip_points2: [ClipVertex<T>; 2] = unsafe { std::mem::zeroed() };
        let np = clip_segment_to_line(
            &mut clip_points1,
            &ie,
            &rf.side_normal1,
            rf.side_offset1,
            rf.i1,
        );

        if np < settings::MAX_MANIFOLD_POINTS {
            return;
        }

        let np = clip_segment_to_line(
            &mut clip_points2,
            &clip_points1,
            &rf.side_normal2,
            rf.side_offset2,
            rf.i2,
        );

        if np < settings::MAX_MANIFOLD_POINTS {
            return;
        }

        if primary_axis.type_ == EPAxisType::EdgeA {
            manifold.local_normal = rf.normal;
            manifold.local_point = rf.v1;
        } else {
            manifold.local_normal = polygon_b.normals[rf.i1];
            manifold.local_point = polygon_b.vertices[rf.i1];
        }

        let mut point_count = 0;
        for i in 0..settings::MAX_MANIFOLD_POINTS {
            let separation = rf.normal.dot(clip_points2[i].v - rf.v1);

            if separation <= self.radius {
                let cp = &mut manifold.points[point_count];

                if primary_axis.type_ == EPAxisType::EdgeA {
                    cp.local_point = self.xf.transpose_multiply(clip_points2[i].v);
                    cp.id = clip_points2[i].id;
                } else {
                    cp.local_point = clip_points2[i].v;
                    cp.id.0.type_a = clip_points2[i].id.0.type_b;
                    cp.id.0.type_b = clip_points2[i].id.0.type_a;
                    cp.id.0.index_a = clip_points2[i].id.0.index_b;
                    cp.id.0.index_b = clip_points2[i].id.0.index_a;
                }

                point_count += 1;
            }
        }

        manifold.point_count = point_count;
    }
}
