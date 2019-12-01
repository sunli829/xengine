use slab::Slab;
use std::any::{Any, TypeId};
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;

pub struct EntityId(usize);

pub struct Entity {
    components: Vec<Option<Box<dyn Any>>>,
}

pub trait System {}

pub enum Event {
    CreateEntity(EntityId),
    RemoveEntity(EntityId),
    CreateComponent(EntityId, TypeId),
    RemoveComponent(EntityId, TypeId),
}

struct ECSInner {
    entities: Slab<Entity>,
    components_idx: HashMap<TypeId, u8>,
    systems: HashMap<TypeId, (usize, Rc<RefCell<dyn Any>>)>,
    events: Vec<Event>,
}

pub struct EntityBuilder<'a> {
    ecs_inner: &'a mut ECSInner,
    entity: Entity,
}

impl<'a> EntityBuilder<'a> {
    pub fn component<C: 'static>(mut self, c: C) -> Self {
        let tid = TypeId::of::<C>();
        let idx = self
            .ecs_inner
            .components_idx
            .get(&tid)
            .map(|idx| *idx)
            .unwrap_or_else(|| {
                let count = self.ecs_inner.components_idx.len();
                let idx = count as u8;
                self.ecs_inner.components_idx.insert(tid, idx);
                idx
            });
        self.entity.components[idx as usize] = Some(Box::new(c));
        self
    }
}

pub struct ECS {
    inner: ECSInner,
}

impl ECS {
    pub fn new() -> ECS {
        ECS {
            inner: ECSInner {
                entities: Default::default(),
                components_idx: Default::default(),
                systems: Default::default(),
                events: Default::default(),
            },
        }
    }

    pub fn create_entity(&mut self) -> EntityBuilder<'_> {
        EntityBuilder {
            ecs_inner: &mut self.inner,
            entity: Entity {
                components: Vec::with_capacity(32),
            },
        }
    }
}
