use slab::Slab;
use std::any::{Any, TypeId};
use std::collections::HashMap;
use std::time::Duration;

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub struct EntityId(usize);

#[derive(Default)]
struct ComponentRegistry(HashMap<TypeId, u8>);

pub trait Component: 'static {
    fn name() -> &'static str;
}

impl ComponentRegistry {
    fn get_idx<C: Component>(&self) -> Option<u8> {
        self.0.get(&TypeId::of::<C>()).map(ToOwned::to_owned)
    }

    fn get_or_create_idx<C: Component>(&mut self) -> u8 {
        let tid = TypeId::of::<C>();
        self.0.get(&tid).map(|idx| *idx).unwrap_or_else(move || {
            let count = self.0.len();
            let idx = count as u8;
            self.0.insert(tid, idx);
            idx
        })
    }
}

struct Entity {
    components: Vec<Option<Box<dyn Any>>>,
}

pub struct EntityRef<'a> {
    components_registry: &'a ComponentRegistry,
    entity: &'a Entity,
}

impl<'a> EntityRef<'a> {
    pub fn contains<C: Component>(&self) -> bool {
        let entity = &self.entity;
        match self
            .components_registry
            .get_idx::<C>()
            .and_then(|idx| entity.components.get(idx as usize))
        {
            Some(Some(_)) => true,
            _ => false,
        }
    }

    pub fn get<C: Component>(&self) -> Option<&C> {
        let entity = &self.entity;
        match self
            .components_registry
            .get_idx::<C>()
            .and_then(|idx| entity.components.get(idx as usize))
        {
            Some(Some(ref c)) => Some(c.downcast_ref::<C>().unwrap()),
            _ => None,
        }
    }
}

pub struct EntityMut<'a> {
    id: EntityId,
    components_registry: &'a mut ComponentRegistry,
    events: &'a mut Vec<Event>,
    entity: &'a mut Entity,
}

impl<'a> EntityMut<'a> {
    pub fn contains<C: Component>(&self) -> bool {
        let entity = &self.entity;
        match self
            .components_registry
            .get_idx::<C>()
            .and_then(|idx| entity.components.get(idx as usize))
        {
            Some(Some(_)) => true,
            _ => false,
        }
    }

    pub fn add<C: Component>(&mut self, c: C) {
        let tid = TypeId::of::<C>();
        let idx = self.components_registry.get_or_create_idx::<C>();
        self.entity.components[idx as usize] = Some(Box::new(c));
        self.events.push(Event::CreateComponent(self.id, tid));
    }

    pub fn remove<C: Component>(&mut self) {
        let tid = TypeId::of::<C>();
        let idx = self.components_registry.get_or_create_idx::<C>();
        self.entity.components[idx as usize] = None;
        self.events.push(Event::RemoveComponent(self.id, tid));
    }

    pub fn get<C: Component>(&self) -> Option<&C> {
        let entity = &self.entity;
        match self
            .components_registry
            .get_idx::<C>()
            .and_then(|idx| entity.components.get(idx as usize))
        {
            Some(Some(ref c)) => Some(c.downcast_ref::<C>().unwrap()),
            _ => None,
        }
    }

    pub fn get_mut<C: Component>(&mut self) -> Option<&mut C> {
        let entity = &mut self.entity;
        match self
            .components_registry
            .get_idx::<C>()
            .and_then(move |idx| entity.components.get_mut(idx as usize))
        {
            Some(Some(ref mut c)) => Some(c.downcast_mut::<C>().unwrap()),
            _ => None,
        }
    }
}

pub trait System {
    fn update(&mut self, ecs: &mut ECS, delta: Duration) {}
    fn handle_event(&mut self, ecs: &mut ECS, event: &Event) {}
}

pub enum Event {
    CreateEntity(EntityId),
    RemoveEntity(EntityId),
    CreateComponent(EntityId, TypeId),
    RemoveComponent(EntityId, TypeId),
    Custom(Box<dyn Any>),
}

struct ECSInner {
    entities: Slab<Entity>,
    components_registry: ComponentRegistry,
    systems: HashMap<TypeId, (usize, Box<dyn System>)>,
    events: Vec<Event>,
}

pub struct EntityBuilder<'a> {
    ecs_inner: &'a mut ECSInner,
    entity: Entity,
}

impl<'a> EntityBuilder<'a> {
    pub fn component<C: Component>(mut self, c: C) -> Self {
        let idx = self.ecs_inner.components_registry.get_or_create_idx::<C>();
        self.entity.components[idx as usize] = Some(Box::new(c));
        self
    }

    pub fn finish(self) -> EntityId {
        let id = EntityId(self.ecs_inner.entities.insert(self.entity));
        self.ecs_inner.events.push(Event::CreateEntity(id));
        id
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
                components_registry: Default::default(),
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

    pub fn remove_entity(&mut self, id: EntityId) {
        self.inner.entities.remove(id.0);
        self.inner.events.push(Event::RemoveEntity(id));
    }

    pub fn entity(&self, id: EntityId) -> Option<EntityRef<'_>> {
        let components_registry = &self.inner.components_registry;
        self.inner.entities.get(id.0).map(|entity| EntityRef {
            components_registry,
            entity,
        })
    }

    pub fn entity_mut(&mut self, id: EntityId) -> Option<EntityMut<'_>> {
        let events = &mut self.inner.events;
        let components_registry = &mut self.inner.components_registry;
        self.inner
            .entities
            .get_mut(id.0)
            .map(move |entity| EntityMut {
                id,
                components_registry,
                events,
                entity,
            })
    }

    pub fn entities_count(&self) -> usize {
        self.inner.entities.len()
    }

    pub fn system<T: 'static>(&self) -> Option<&T> {
        unsafe {
            self.inner
                .systems
                .get(&TypeId::of::<T>())
                .map(|item| (&*(item.1.as_ref() as *const dyn System as *const T)))
        }
    }

    pub fn system_mut<T: 'static>(&mut self) -> Option<&mut T> {
        unsafe {
            self.inner
                .systems
                .get_mut(&TypeId::of::<T>())
                .map(|item| (&mut *(item.1.as_mut() as *mut dyn System as *mut T)))
        }
    }

    pub fn update(&mut self, delta: Duration) {
        unsafe {
            let mut systems = Vec::with_capacity(self.inner.systems.len());
            for (_, (ord, system)) in &mut self.inner.systems {
                systems.push((*ord, system.as_mut() as *mut dyn System));
            }
            systems.sort_by(|a, b| a.0.cmp(&b.0));

            for (_, system) in &systems {
                system.as_mut().unwrap().update(self, delta);
            }

            let events = std::mem::replace(&mut self.inner.events, Default::default());
            for (_, system) in &systems {
                for event in &events {
                    system.as_mut().unwrap().handle_event(self, event);
                }
            }

            self.inner.events.clear();
        }
    }
}
