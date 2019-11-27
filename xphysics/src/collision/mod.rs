use crate::collision::distance::{DistanceInput, SimpleCache};
use crate::settings;
use xmath::{DotTrait, Multiply, Real, Transform, Vector2};

mod broad_phase;
mod collide_circle;
mod collide_edge;
mod collide_polygon;
mod distance;
pub(crate) mod dynamic_tree;
mod shapes;
pub(crate) mod time_of_impact;

pub(crate) use broad_phase::BroadPhase;
pub(crate) use collide_circle::{collide_circles, collide_polygon_and_circle};
pub(crate) use collide_edge::{collide_edge_and_circle, collide_edge_and_polygon};
pub(crate) use collide_polygon::collide_polygons;
pub use shapes::*;

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

#[derive(Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Debug)]
pub(crate) enum ContactFeatureType {
    Vertex,
    Face,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub(crate) struct ContactFeature {
    index_a: u8,
    index_b: u8,
    type_a: ContactFeatureType,
    type_b: ContactFeatureType,
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ContactId(ContactFeature);

impl ContactId {
    pub(crate) fn zero() -> ContactId {
        ContactId(ContactFeature {
            index_a: 0,
            index_b: 0,
            type_a: ContactFeatureType::Vertex,
            type_b: ContactFeatureType::Vertex,
        })
    }
}

#[derive(Copy, Clone, Eq, PartialEq, Debug)]
pub struct ManifoldPoint<T> {
    pub local_point: Vector2<T>,
    pub normal_impulse: T,
    pub tangent_impulse: T,
    pub id: ContactId,
}

#[derive(Debug, Copy, Clone)]
pub enum ManifoldType {
    Circles,
    FaceA,
    FaceB,
}

#[derive(Debug, Copy, Clone)]
pub struct Manifold<T> {
    pub points: [ManifoldPoint<T>; settings::MAX_MANIFOLD_POINTS],
    pub local_normal: Vector2<T>,
    pub local_point: Vector2<T>,
    pub type_: ManifoldType,
    pub point_count: usize,
}

impl<T> Default for Manifold<T> {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[derive(Debug, Default)]
pub struct WorldManifold<T> {
    pub normal: Vector2<T>,
    pub points: [Vector2<T>; settings::MAX_MANIFOLD_POINTS],
    pub separations: [T; settings::MAX_MANIFOLD_POINTS],
}

impl<T: Real> WorldManifold<T> {
    pub(crate) fn new(
        manifold: &Manifold<T>,
        xf_a: &Transform<T>,
        radius_a: T,
        xf_b: &Transform<T>,
        radius_b: T,
    ) -> WorldManifold<T> {
        let mut world_manifold = WorldManifold::<T>::default();

        if manifold.point_count == 0 {
            return world_manifold;
        }

        match manifold.type_ {
            ManifoldType::Circles => {
                world_manifold.normal = Vector2::new(T::one(), T::zero());
                let point_a = xf_a.multiply(manifold.local_point);
                let point_b = xf_b.multiply(manifold.points[0].local_point);
                if point_a.distance_squared(&point_b) > T::epsilon() * T::epsilon() {
                    world_manifold.normal = (point_b - point_a).normalize();
                }

                let c_a = point_a + world_manifold.normal * radius_a;
                let c_b = point_b - world_manifold.normal * radius_b;
                world_manifold.points[0] = (c_a + c_b) * T::half();
                world_manifold.separations[0] = (c_b - c_a).dot(world_manifold.normal);
            }
            ManifoldType::FaceA => {
                world_manifold.normal = xf_a.q.multiply(manifold.local_normal);
                let plane_point = xf_a.multiply(manifold.local_point);

                for i in 0..manifold.point_count {
                    let clip_point = xf_b.multiply(manifold.points[i].local_point);
                    let c_a = clip_point
                        + world_manifold.normal
                            * (radius_a - (clip_point - plane_point).dot(world_manifold.normal));
                    let c_b = clip_point - world_manifold.normal * radius_b;
                    world_manifold.points[i] = (c_a + c_b) * T::half();
                    world_manifold.separations[i] = (c_b - c_a).dot(world_manifold.normal);
                }
            }
            ManifoldType::FaceB => {
                world_manifold.normal = xf_b.q.multiply(manifold.local_normal);
                let plane_point = xf_b.multiply(manifold.local_point);

                for i in 0..manifold.point_count {
                    let clip_point = xf_a.multiply(manifold.points[i].local_point);
                    let c_b = clip_point
                        + world_manifold.normal
                            * (radius_b - (clip_point - plane_point).dot(world_manifold.normal));
                    let c_a = clip_point - world_manifold.normal * radius_a;
                    world_manifold.points[i] = (c_a + c_b) * T::half();
                    world_manifold.separations[i] = (c_a - c_b).dot(world_manifold.normal);
                }

                world_manifold.normal = -world_manifold.normal;
            }
        }

        world_manifold
    }
}

#[derive(Debug, Copy, Clone)]
pub enum PointState {
    Null,
    Add,
    Persist,
    Remove,
}

#[allow(dead_code)]
pub(crate) fn get_point_states<T: Real>(
    manifold1: &Manifold<T>,
    manifold2: &Manifold<T>,
) -> (
    [PointState; settings::MAX_MANIFOLD_POINTS],
    [PointState; settings::MAX_MANIFOLD_POINTS],
) {
    let mut state1 = [PointState::Null; settings::MAX_MANIFOLD_POINTS];
    let mut state2 = [PointState::Null; settings::MAX_MANIFOLD_POINTS];

    for i in 0..manifold1.point_count {
        let id = manifold1.points[i].id;
        state1[i] = PointState::Remove;
        for j in 0..manifold2.point_count {
            if manifold2.points[j].id == id {
                state1[i] = PointState::Persist;
                break;
            }
        }
    }

    for i in 0..manifold2.point_count {
        let id = manifold2.points[i].id;
        state2[i] = PointState::Add;
        for j in 0..manifold1.point_count {
            if manifold1.points[j].id == id {
                state2[i] = PointState::Persist;
                break;
            }
        }
    }

    (state1, state2)
}

#[derive(Debug, Copy, Clone)]
pub(crate) struct ClipVertex<T> {
    v: Vector2<T>,
    id: ContactId,
}

pub(crate) fn clip_segment_to_line<T: Real>(
    vout: &mut [ClipVertex<T>; 2],
    vin: &[ClipVertex<T>; 2],
    normal: &Vector2<T>,
    offset: T,
    vertex_index_a: usize,
) -> usize {
    let mut num_out = 0;

    let distance0 = normal.dot(vin[0].v) - offset;
    let distance1 = normal.dot(vin[1].v) - offset;

    if distance0 <= T::zero() {
        vout[num_out] = vin[0];
        num_out += 1;
    };
    if distance1 <= T::zero() {
        vout[num_out] = vin[1];
        num_out += 1;
    }

    if distance0 * distance1 < T::zero() {
        let interp = distance0 / (distance0 - distance1);
        vout[num_out].v = vin[0].v + (vin[1].v - vin[0].v) * interp;

        vout[num_out].id.0.index_a = vertex_index_a as u8;
        vout[num_out].id.0.index_b = vin[0].id.0.index_b;
        vout[num_out].id.0.type_a = ContactFeatureType::Vertex;
        vout[num_out].id.0.type_b = ContactFeatureType::Face;
        num_out += 1;
    }

    num_out
}

pub(crate) fn test_overlap<T: Real>(
    shape_a: &dyn Shape<T>,
    index_a: usize,
    shape_b: &dyn Shape<T>,
    index_b: usize,
    xf_a: Transform<T>,
    xf_b: Transform<T>,
) -> bool {
    let input = DistanceInput {
        proxy_a: &shape_a.distance_proxy(index_a),
        proxy_b: &shape_b.distance_proxy(index_b),
        transform_a: xf_a,
        transform_b: xf_b,
        use_radii: true,
    };

    let mut cache: SimpleCache<T> = unsafe { std::mem::zeroed() };
    let output = distance::distance(&input, &mut cache);
    output.distance < T::ten() * T::epsilon()
}
