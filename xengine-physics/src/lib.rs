mod colliders;

use crate::colliders::ShapeHash;
pub use crate::colliders::{
    ComponentColliderBox, ComponentColliderChain, ComponentColliderCircle, ComponentColliderEdge,
    ComponentColliderPolygon,
};
use std::any::TypeId;
use std::collections::HashMap;
use std::ops::Deref;
use std::time::Duration;
use xecs::{Component, EntityId, EntityRef, Event, System, ECS};
use xengine_basecomponents::ComponentTransform;
use xmath::Vector2;
use xphysics::{
    BodyDef, BodyId, BodyType, FixtureDef, FixtureId, IntoBoxedShape, Shape, ShapeChain,
    ShapeCircle, ShapeEdge, ShapePolygon, World,
};

/// 刚体
pub struct ComponentRigidBody {
    pub type_: BodyType,
    pub linear_velocity: Vector2<f32>,
    pub angular_velocity: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub allow_sleep: bool,
    pub awake: bool,
    pub fixed_rotation: bool,
    pub bullet: bool,
    pub active: bool,
    pub gravity_scale: f32,
}

impl Default for ComponentRigidBody {
    fn default() -> Self {
        ComponentRigidBody {
            type_: BodyType::Static,
            linear_velocity: (0.0, 0.0).into(),
            angular_velocity: 0.0,
            linear_damping: 0.0,
            angular_damping: 0.0,
            allow_sleep: true,
            awake: true,
            fixed_rotation: false,
            bullet: false,
            active: true,
            gravity_scale: 1.0,
        }
    }
}

impl Component for ComponentRigidBody {
    fn name() -> &'static str {
        "RigidBody"
    }
}

struct FixtureInfo {
    fixture_id: FixtureId,
    shape_hash: u64,
}

struct BodyInfo {
    body_id: BodyId,
    fixtures: HashMap<TypeId, FixtureInfo>,
}

/// 刚体物理系统
pub struct SystemPhysics {
    world: World<f32, EntityId>,
    bodies: HashMap<EntityId, BodyInfo>,
}

impl System for SystemPhysics {
    fn update(&mut self, ecs: &mut ECS, delta: Duration) {
        // 同步component的属性到world
        for (entity_id, body_info) in &mut self.bodies {
            let entity = ecs.entity(*entity_id).unwrap();
            let component_body = entity.get::<ComponentRigidBody>().unwrap();
            let body = self.world.body_mut(body_info.body_id).unwrap();

            // 同步body属性
            body.set_body_type(component_body.type_);
            body.set_linear_velocity(component_body.linear_velocity);
            body.set_angular_velocity(component_body.angular_velocity);
            body.set_linear_damping(component_body.linear_damping);
            body.set_angular_damping(component_body.angular_damping);
            body.set_sleeping_allowed(component_body.allow_sleep);
            body.set_awake(component_body.awake);
            body.set_fixed_rotation(component_body.fixed_rotation);
            body.set_bullet(component_body.bullet);
            body.set_active(component_body.active);
            body.set_gravity_scale(component_body.gravity_scale);

            // 删除shape已经改变的fixtures
            check_remove_fixtures(&entity, &mut self.world, body_info);

            // 创建或者删除fixtures
            create_or_remove_fixtures(&entity, &mut self.world, body_info);

            // 通过fixtures的属性
            sync_fixtures(&entity, &mut self.world, body_info);
        }

        // 步进世界
        self.world.step(delta.as_secs_f32(), 8, 3);

        // 同步body的位置和旋转属性到ComponentTransform
        for (entity_id, body_info) in &mut self.bodies {
            if let Some(transform) = ecs
                .entity_mut(*entity_id)
                .unwrap()
                .get_mut::<ComponentTransform>()
            {
                let body = self.world.body(body_info.body_id).unwrap();
                transform.0 = *body.transform();
            }
        }
    }

    fn handle_event(&mut self, ecs: &mut ECS, event: &Event) {
        match event {
            Event::CreateEntity(id) => {
                let entity = ecs.entity(*id).unwrap();
                if let (Some(component_body), Some(component_transform)) = (
                    entity.get::<ComponentRigidBody>(),
                    entity.get::<ComponentTransform>(),
                ) {
                    let body_id = self.world.create_body(BodyDef {
                        type_: BodyType::Static,
                        position: component_transform.0.p,
                        angle: component_transform.0.q.angle(),
                        linear_velocity: component_body.linear_velocity,
                        angular_velocity: component_body.angular_velocity,
                        linear_damping: component_body.linear_damping,
                        angular_damping: component_body.angular_damping,
                        allow_sleep: component_body.allow_sleep,
                        awake: component_body.awake,
                        fixed_rotation: component_body.fixed_rotation,
                        bullet: component_body.bullet,
                        active: component_body.active,
                        gravity_scale: component_body.gravity_scale,
                        data: Some(*id),
                    });
                    self.bodies.insert(
                        *id,
                        BodyInfo {
                            body_id,
                            fixtures: Default::default(),
                        },
                    );

                    create_or_remove_fixtures(
                        &entity,
                        &mut self.world,
                        self.bodies.get_mut(id).unwrap(),
                    );
                }
            }
            Event::RemoveEntity(id) => {
                if let Some(body_info) = self.bodies.remove(id) {
                    self.world.destroy_body(body_info.body_id);
                }
            }
            Event::CreateComponent(id, _) | Event::RemoveComponent(id, _) => {
                if let Some(body_info) = self.bodies.get_mut(id) {
                    create_or_remove_fixtures(
                        &ecs.entity(*id).unwrap(),
                        &mut self.world,
                        body_info,
                    );
                }
            }
            _ => {}
        }
    }
}

fn check_remove_fixture<C>(
    entity: &EntityRef,
    world: &mut World<f32, EntityId>,
    body_info: &mut BodyInfo,
) where
    C: Component + ShapeHash,
{
    if let Some(collider) = entity.get::<C>() {
        let remove_fixture_id =
            if let Some(fixture_info) = body_info.fixtures.get(&TypeId::of::<C>()) {
                let new_hash = collider.hash();
                if new_hash != fixture_info.shape_hash {
                    Some(fixture_info.fixture_id)
                } else {
                    None
                }
            } else {
                None
            };

        if let Some(fixture_id) = remove_fixture_id {
            // shape改变了，删除这个fixture
            body_info.fixtures.remove(&TypeId::of::<C>());
            world
                .body_mut(body_info.body_id)
                .unwrap()
                .destroy_fixture(fixture_id);
        }
    }
}

fn check_remove_fixtures(
    entity: &EntityRef,
    world: &mut World<f32, EntityId>,
    body_info: &mut BodyInfo,
) {
    check_remove_fixture::<ComponentColliderCircle>(entity, world, body_info);
    check_remove_fixture::<ComponentColliderBox>(entity, world, body_info);
    check_remove_fixture::<ComponentColliderEdge>(entity, world, body_info);
    check_remove_fixture::<ComponentColliderChain>(entity, world, body_info);
    check_remove_fixture::<ComponentColliderPolygon>(entity, world, body_info);
}

fn create_or_remove_fixture<C, F, S: Shape<f32> + 'static>(
    entity: &EntityRef,
    world: &mut World<f32, EntityId>,
    body_info: &mut BodyInfo,
    create_shape: F,
) where
    C: Component + Deref<Target = colliders::Fixture> + ShapeHash,
    F: FnOnce(&C) -> S,
{
    if let Some(collider) = entity.get::<C>() {
        if !body_info.fixtures.contains_key(&TypeId::of::<C>()) {
            // 创建
            let fixture_def = FixtureDef {
                shape: create_shape(collider).into_boxed(),
                friction: collider.friction,
                restitution: collider.restitution,
                density: collider.density,
                is_sensor: collider.is_sensor,
                filter: collider.filter,
                data: Some(entity.id()),
            };
            let fixture_id = world
                .body_mut(body_info.body_id)
                .unwrap()
                .create_fixture(fixture_def);

            body_info.fixtures.insert(
                TypeId::of::<C>(),
                FixtureInfo {
                    fixture_id,
                    shape_hash: collider.hash(),
                },
            );
        }
    } else {
        if let Some(fixture_info) = body_info.fixtures.remove(&TypeId::of::<C>()) {
            // 删除
            world
                .body_mut(body_info.body_id)
                .unwrap()
                .destroy_fixture(fixture_info.fixture_id);
        }
    }
}

fn create_or_remove_fixtures(
    entity: &EntityRef,
    world: &mut World<f32, EntityId>,
    body_info: &mut BodyInfo,
) {
    create_or_remove_fixture::<ComponentColliderCircle, _, _>(
        &entity,
        world,
        body_info,
        |collider| ShapeCircle::new(collider.position, collider.radius),
    );

    create_or_remove_fixture::<ComponentColliderBox, _, _>(&entity, world, body_info, |collider| {
        ShapePolygon::new_box(
            collider.half_size.x,
            collider.half_size.y,
            collider.position,
            0.0,
        )
    });

    create_or_remove_fixture::<ComponentColliderEdge, _, _>(
        &entity,
        world,
        body_info,
        |collider| ShapeEdge::new(collider.vertex1, collider.vertex2),
    );

    create_or_remove_fixture::<ComponentColliderChain, _, _>(
        &entity,
        world,
        body_info,
        |collider| match collider.is_loop {
            true => ShapeChain::create_loop(collider.vertices.clone()),
            false => ShapeChain::create_chain(collider.vertices.clone()),
        },
    );

    create_or_remove_fixture::<ComponentColliderPolygon, _, _>(
        &entity,
        world,
        body_info,
        |collider| ShapePolygon::new(collider.vertices.clone()),
    );
}

fn sync_fixture<C>(entity: &EntityRef, world: &mut World<f32, EntityId>, body_info: &BodyInfo)
where
    C: Component + Deref<Target = colliders::Fixture>,
{
    if let Some(component) = entity.get::<C>() {
        let component_fixture = component.deref();
        let body = world.body_mut(body_info.body_id).unwrap();
        for (_, fixture) in body.fixture_list_mut() {
            fixture.set_friction(component_fixture.friction);
            fixture.set_restitution(component_fixture.restitution);
            fixture.set_density(component_fixture.density);
            fixture.set_sensor(component_fixture.is_sensor);
            fixture.set_filter(component_fixture.filter);
        }
    }
}
fn sync_fixtures(entity: &EntityRef, world: &mut World<f32, EntityId>, body_info: &BodyInfo) {
    sync_fixture::<ComponentColliderCircle>(entity, world, body_info);
    sync_fixture::<ComponentColliderBox>(entity, world, body_info);
    sync_fixture::<ComponentColliderEdge>(entity, world, body_info);
    sync_fixture::<ComponentColliderChain>(entity, world, body_info);
    sync_fixture::<ComponentColliderPolygon>(entity, world, body_info);
}
