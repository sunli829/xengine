use std::collections::hash_map::DefaultHasher;
use std::hash::Hasher;
use std::ops::Deref;
use xecs::Component;
use xmath::Vector2;
use xphysics::Filter;

pub struct Fixture {
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
    pub is_sensor: bool,
    pub filter: Filter,
}

fn hash_f32<H: Hasher>(value: f32, state: &mut H) {
    state.write_u32(unsafe { std::mem::transmute(value) });
}

fn hash_vector2<H: Hasher>(value: Vector2<f32>, state: &mut H) {
    hash_f32(value.x, state);
    hash_f32(value.y, state);
}

pub(crate) trait ShapeHash {
    fn hash(&self) -> u64;
}

pub struct ComponentColliderCircle {
    pub fixture: Fixture,
    pub position: Vector2<f32>,
    pub radius: f32,
}

impl Component for ComponentColliderCircle {
    fn name() -> &'static str {
        "ColliderCircle"
    }
}

impl Deref for ComponentColliderCircle {
    type Target = Fixture;

    fn deref(&self) -> &Self::Target {
        &self.fixture
    }
}

impl ShapeHash for ComponentColliderCircle {
    fn hash(&self) -> u64 {
        let mut state = DefaultHasher::new();
        hash_vector2(self.position, &mut state);
        hash_f32(self.radius, &mut state);
        state.finish()
    }
}

pub struct ComponentColliderBox {
    pub fixture: Fixture,
    pub position: Vector2<f32>,
    pub half_size: Vector2<f32>,
}

impl Component for ComponentColliderBox {
    fn name() -> &'static str {
        "ColliderBox"
    }
}

impl Deref for ComponentColliderBox {
    type Target = Fixture;

    fn deref(&self) -> &Self::Target {
        &self.fixture
    }
}

impl ShapeHash for ComponentColliderBox {
    fn hash(&self) -> u64 {
        let mut state = DefaultHasher::new();
        hash_vector2(self.position, &mut state);
        hash_vector2(self.half_size, &mut state);
        state.finish()
    }
}

pub struct ComponentColliderEdge {
    pub fixture: Fixture,
    pub vertex1: Vector2<f32>,
    pub vertex2: Vector2<f32>,
}

impl Component for ComponentColliderEdge {
    fn name() -> &'static str {
        "ColliderEdge"
    }
}

impl Deref for ComponentColliderEdge {
    type Target = Fixture;

    fn deref(&self) -> &Self::Target {
        &self.fixture
    }
}

impl ShapeHash for ComponentColliderEdge {
    fn hash(&self) -> u64 {
        let mut state = DefaultHasher::new();
        hash_vector2(self.vertex1, &mut state);
        hash_vector2(self.vertex2, &mut state);
        state.finish()
    }
}

pub struct ComponentColliderChain {
    pub fixture: Fixture,
    pub is_loop: bool,
    pub vertices: Vec<Vector2<f32>>,
}

impl Component for ComponentColliderChain {
    fn name() -> &'static str {
        "ColliderChain"
    }
}

impl Deref for ComponentColliderChain {
    type Target = Fixture;

    fn deref(&self) -> &Self::Target {
        &self.fixture
    }
}

impl ShapeHash for ComponentColliderChain {
    fn hash(&self) -> u64 {
        let mut state = DefaultHasher::new();
        state.write_u8(self.is_loop as u8);
        for v in &self.vertices {
            hash_vector2(*v, &mut state);
        }
        state.finish()
    }
}

pub struct ComponentColliderPolygon {
    pub fixture: Fixture,
    pub vertices: Vec<Vector2<f32>>,
}

impl Component for ComponentColliderPolygon {
    fn name() -> &'static str {
        "ColliderPolygon"
    }
}

impl Deref for ComponentColliderPolygon {
    type Target = Fixture;

    fn deref(&self) -> &Self::Target {
        &self.fixture
    }
}

impl ShapeHash for ComponentColliderPolygon {
    fn hash(&self) -> u64 {
        let mut state = DefaultHasher::new();
        for v in &self.vertices {
            hash_vector2(*v, &mut state);
        }
        state.finish()
    }
}
