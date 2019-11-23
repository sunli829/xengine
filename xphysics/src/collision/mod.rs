use crate::settings;
use xmath::{DotTrait, Real, Vector2};

mod collide_circle;
mod collide_edge;
mod collide_polygon;
mod distance;
mod dynamic_tree;
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

#[derive(Debug)]
pub struct Manifold<T> {
    points: [ManifoldPoint<T>; settings::MAX_MANIFOLD_POINTS],
    local_normal: Vector2<T>,
    local_point: Vector2<T>,
    type_: ManifoldType,
    point_count: usize,
}

impl<T> Default for Manifold<T> {
    fn default() -> Self {
        unsafe { std::mem::zeroed() }
    }
}

#[derive(Debug)]
pub struct WorldManifold<T> {
    normal: Vector2<T>,
    points: [Vector2<T>; settings::MAX_MANIFOLD_POINTS],
    separations: [T; settings::MAX_MANIFOLD_POINTS],
}

#[derive(Debug, Copy, Clone)]
pub enum PointState {
    Null,
    Add,
    Persist,
    Remove,
}

pub fn get_point_states<T: Real>(
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
