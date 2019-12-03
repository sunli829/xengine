use xecs::{EntityId, Component};
use xmath::Transform;

pub struct ComponentTransform(pub Transform<f32>);

impl Component for ComponentTransform {
    fn name() -> &'static str {
        "Transform"
    }
}

pub struct ComponentChildren(pub Vec<EntityId>);

