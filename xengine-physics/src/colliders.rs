use xmath::Vector2;

pub struct ComponentColliderCircle<T> {
    pub position: Vector2<T>,
    pub radius: T,
}

pub struct ComponentColliderBox<T> {
    pub position: Vector2<T>,
    pub half_size: Vector2<T>,
}

pub struct ComponentColliderEdge<T> {
    pub vertex1: Vector2<T>,
    pub vertex2: Vector2<T>,
}

pub struct ComponentColliderChain<T> {
    pub vertices: Vec<Vector2<T>>,
}

pub struct ComponentColliderPolygon<T> {
    pub vertices: Vec<Vector2<T>>,
}
