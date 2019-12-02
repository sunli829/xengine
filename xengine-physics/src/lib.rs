mod colliders;

use std::time::Duration;
use xecs::{EntityId, Event, System, ECS};
use xmath::{Real, Vector2};
use xphysics::{Body, ShapeCircle, World};

pub struct ComponentRigidBody<T>(Body<T, EntityId>);

pub struct SystemPhysics<T> {
    world: World<T, EntityId>,
}

impl<T: Real> System for SystemPhysics<T> {
    fn update(&mut self, ecs: &mut ECS, delta: Duration) {}

    fn handle_event(&mut self, ecs: &mut ECS, event: &Event) {
        match event {
            Event::CreateEntity(id) => {
                let entity = ecs.entity(*id).unwrap();
                if let Some(collider) = entity.get::<colliders::ComponentColliderBox<T>>() {
                }
            }
        }
    }
}
