use xecs::EntityId;
use xmath::Transform;

pub struct ComponentTransform(pub Transform<f32>);

pub struct ComponentChildren(pub Vec<EntityId>);

