use crate::collision::dynamic_tree;
use crate::collision::time_of_impact::{time_of_impact, TOIInput, TOIOutputState};
use crate::dynamic::body::{BodyDef, BodyFlags};
use crate::dynamic::contact_manager::ContactManager;
use crate::dynamic::contacts::{ContactFilter, ContactFlags, ContactListener};
use crate::dynamic::fixture::FixtureProxy;
use crate::dynamic::island::Island;
use crate::dynamic::time_step::{Profile, TimeStep};
use crate::timer::Timer;
use crate::{
    settings, Body, BodyType, Fixture, RayCastInput, Shape, ShapeChain, ShapeCircle, ShapeEdge,
    ShapePolygon, ShapeType,
};
use slab::Slab;
use xmath::{Multiply, Real, Rotation, Transform, Vector2, AABB};

#[derive(Debug, Copy, Clone)]
pub struct BodyId(usize);

bitflags! {
    pub struct WorldFlags: u32 {
        const NEW_FIXTURE = 0x0001;
        const LOCKED = 0x0002;
        const CLEAR_FORCES = 0x0004;
    }
}

pub trait DestructionListener<T, D> {
    fn fixture_destroyed(&self, fixture: &Fixture<T, D>);
}

#[derive(Copy, Clone, Debug)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub(crate) fn rgb(r: f32, g: f32, b: f32) -> Color {
        Color { r, g, b, a: 1.0 }
    }

    pub(crate) fn rgba(r: f32, g: f32, b: f32, a: f32) -> Color {
        Color { r, g, b, a }
    }
}

bitflags! {
    pub struct DebugDrawFlags: u32 {
        const SHAPE = 0x0001;
        const AABB = 0x0002;
        const CENTER_OF_MASS_BIT = 0x0004;
    }
}

pub trait DebugDraw {
    fn draw_polygon(&self, vertices: &[Vector2<f32>], color: Color);
    fn draw_solid_polygon(&self, vertices: &[Vector2<f32>], color: Color);
    fn draw_circle(&self, center: &Vector2<f32>, radius: f32, color: Color);
    fn draw_solid_circle(
        &self,
        center: &Vector2<f32>,
        radius: f32,
        axis: &Vector2<f32>,
        color: Color,
    );
    fn draw_segment(&self, p1: &Vector2<f32>, p2: &Vector2<f32>, color: Color);
    fn draw_transform(&self, xf: &Transform<f32>);
    fn draw_point(&self, p: &Vector2<f32>, size: f32, color: Color);
}

pub(crate) struct WorldInner<T, D> {
    pub(crate) bodies_slab: Slab<Body<T, D>>,
    pub(crate) flags: WorldFlags,
    pub(crate) contact_manager: ContactManager<T, D>,
    pub(crate) body_list: *mut Body<T, D>,
    pub(crate) body_count: usize,
    pub(crate) gravity: Vector2<T>,
    pub(crate) allow_sleep: bool,
    pub(crate) destruction_listener: Option<Box<dyn DestructionListener<T, D>>>,
    pub(crate) debug_draw_flags: DebugDrawFlags,
    pub(crate) debug_draw: Option<Box<dyn DebugDraw>>,
    pub(crate) inv_dt0: T,
    pub(crate) warm_starting: bool,
    pub(crate) continuous_physics: bool,
    pub(crate) sub_stepping: bool,
    pub(crate) step_complete: bool,
    pub(crate) profile: Profile,
}

pub struct World<T, D>(Box<WorldInner<T, D>>);

impl<T: Real, D> World<T, D> {
    pub fn new(gravity: Vector2<T>) -> World<T, D> {
        World(Box::new(WorldInner {
            bodies_slab: Default::default(),
            flags: WorldFlags::CLEAR_FORCES,
            contact_manager: ContactManager::new(),
            body_list: std::ptr::null_mut(),
            body_count: 0,
            gravity,
            allow_sleep: true,
            destruction_listener: None,
            debug_draw_flags: DebugDrawFlags::all(),
            debug_draw: None,
            inv_dt0: T::zero(),
            warm_starting: true,
            continuous_physics: true,
            sub_stepping: false,
            step_complete: true,
            profile: Profile::default(),
        }))
    }

    pub fn set_destruction_listener<L: DestructionListener<T, D> + 'static>(
        &mut self,
        listener: L,
    ) {
        self.0.destruction_listener = Some(Box::new(listener));
    }

    pub fn set_debug_draw_flags(&mut self, flags: DebugDrawFlags) {
        self.0.debug_draw_flags = flags;
    }

    pub fn debug_draw_flags(&self) -> DebugDrawFlags {
        self.0.debug_draw_flags
    }

    pub fn set_debug_draw<DD: DebugDraw + 'static>(&mut self, debug_draw: DD) {
        self.0.debug_draw = Some(Box::new(debug_draw));
    }

    pub fn set_contact_filter<F: ContactFilter<T, D> + 'static>(&mut self, filter: F) {
        self.0.contact_manager.contact_filter = Box::new(filter);
    }

    pub fn set_contact_listener<L: ContactListener<T, D> + 'static>(&mut self, listener: L) {
        self.0.contact_manager.contact_listener = Some(Box::new(listener));
    }

    pub fn is_locked(&self) -> bool {
        self.0.flags.contains(WorldFlags::LOCKED)
    }

    pub fn set_auto_clear_forces(&mut self, flag: bool) {
        self.0.flags.set(WorldFlags::CLEAR_FORCES, flag);
    }

    pub fn auto_clear_forces(&mut self) -> bool {
        self.0.flags.contains(WorldFlags::CLEAR_FORCES)
    }

    pub fn shift_origin(&mut self, new_origin: Vector2<T>) {
        assert!(!self.0.flags.contains(WorldFlags::LOCKED));

        unsafe {
            let mut b = self.0.body_list;
            while b.is_null() {
                (*b).xf.p -= new_origin;
                (*b).sweep.c0 -= new_origin;
                (*b).sweep.c -= new_origin;
                b = (*b).next_ptr;
            }
            self.0.contact_manager.broad_phase.shift_origin(new_origin);
        }
    }

    pub fn clear_forces(&mut self) {
        unsafe {
            let mut b = self.0.body_list;
            while !b.is_null() {
                (*b).force = Vector2::zero();
                (*b).torque = T::zero();
                b = (*b).next_ptr;
            }
        }
    }

    pub fn body(&self, id: BodyId) -> Option<&Body<T, D>> {
        self.0.bodies_slab.get(id.0)
    }

    pub fn body_mut(&mut self, id: BodyId) -> Option<&mut Body<T, D>> {
        self.0.bodies_slab.get_mut(id.0)
    }

    pub fn create_body(&mut self, def: BodyDef<T, D>) -> BodyId {
        unsafe {
            let world_ptr = self.0.as_mut() as *mut WorldInner<T, D>;
            let id = self.0.bodies_slab.insert(Body::new(world_ptr, def));
            let body = self.0.bodies_slab.get_unchecked_mut(id);

            body.prev_ptr = std::ptr::null_mut();
            body.next_ptr = self.0.body_list;
            if !self.0.body_list.is_null() {
                (*self.0.body_list).prev_ptr = body;
            }
            self.0.body_list = body;
            self.0.body_count += 1;
            BodyId(id)
        }
    }

    pub fn destroy_body(&mut self, id: BodyId) {
        unsafe {
            assert!(self.0.flags.contains(WorldFlags::LOCKED));
            let body = self.0.bodies_slab.get_unchecked_mut(id.0);

            let mut ce = body.contact_list_ptr;
            while !ce.is_null() {
                let ce0 = ce;
                ce = (*ce).next_ptr;
                self.0.contact_manager.destroy((*ce0).contact_ptr);
            }

            for (_, f) in &mut body.fixture_list {
                if let Some(l) = &(*body.world_ptr).destruction_listener {
                    l.fixture_destroyed(f.as_ref());
                }
                f.destroy_proxies(&mut (*body.world_ptr).contact_manager.broad_phase);
            }

            if !body.prev_ptr.is_null() {
                (*body.prev_ptr).next_ptr = body.next_ptr;
            }
            if !body.next_ptr.is_null() {
                (*body.next_ptr).prev_ptr = body.prev_ptr;
            }
            if body as *mut Body<T, D> == self.0.body_list {
                self.0.body_list = body.next_ptr;
            }

            self.0.bodies_slab.remove(id.0);
            self.0.body_count -= 1;
        }
    }

    pub fn warm_starting(&self) -> bool {
        self.0.warm_starting
    }

    pub fn set_warm_starting(&mut self, flag: bool) {
        self.0.warm_starting = flag;
    }

    pub fn continuous_physics(&self) -> bool {
        self.0.continuous_physics
    }

    pub fn set_continuous_physics(&mut self, flag: bool) {
        self.0.continuous_physics = flag;
    }

    pub fn sub_stepping(&self) -> bool {
        self.0.sub_stepping
    }

    pub fn set_sub_stepping(&mut self, flag: bool) {
        self.0.sub_stepping = flag;
    }

    pub fn gravity(&self) -> Vector2<T> {
        self.0.gravity
    }

    pub fn set_gravity(&mut self, gravity: Vector2<T>) {
        self.0.gravity = gravity;
    }

    pub fn set_allow_sleeping(&mut self, flag: bool) {
        if flag == self.0.allow_sleep {
            return;
        }

        self.0.allow_sleep = flag;
        if !self.0.allow_sleep {
            unsafe {
                let mut b = self.0.body_list;
                while !b.is_null() {
                    (*b).set_awake(true);
                    b = (*b).next_ptr;
                }
            }
        }
    }

    fn solve(&mut self, step: &TimeStep<T>) {
        unsafe {
            self.0.profile.solve_init = Default::default();
            self.0.profile.solve_velocity = Default::default();
            self.0.profile.solve_position = Default::default();

            let mut island = Island::new(
                self.0.body_count,
                self.0.contact_manager.contact_count,
                &self.0.contact_manager.contact_listener,
            );

            let mut b = self.0.body_list;
            while !b.is_null() {
                (*b).flags.remove(BodyFlags::ISLAND);
                b = (*b).next_ptr;
            }

            let mut c = self.0.contact_manager.contact_list;
            while !c.is_null() {
                (*c).flags.remove(ContactFlags::ISLAND);
                c = (*c).next_ptr;
            }

            let stack_size = self.0.body_count;
            let mut stack = Vec::with_capacity(stack_size);
            let mut seed = self.0.body_list;
            while !seed.is_null() {
                if (*seed).flags.contains(BodyFlags::ISLAND) {
                    seed = (*seed).next_ptr;
                    continue;
                }
                if !(*seed).is_awake() || !(*seed).is_active() {
                    seed = (*seed).next_ptr;
                    continue;
                }
                if (*seed).body_type() == BodyType::Static {
                    seed = (*seed).next_ptr;
                    continue;
                }

                island.clear();

                stack.push(seed);
                (*seed).flags.insert(BodyFlags::ISLAND);

                while let Some(b) = stack.pop() {
                    assert!((*b).is_active());
                    island.add_body(b);

                    (*b).flags.insert(BodyFlags::AWAKE);
                    if (*b).body_type() == BodyType::Static {
                        continue;
                    }

                    let ce = (*b).contact_list_ptr;
                    while !ce.is_null() {
                        let contact = (*ce).contact_ptr;
                        if !(*contact).is_enable() || !(*contact).is_touching() {
                            continue;
                        }

                        let sensor_a = (*(*contact).fixture_a_ptr).is_sensor;
                        let sensor_b = (*(*contact).fixture_b_ptr).is_sensor;
                        if sensor_a || sensor_b {
                            continue;
                        }

                        island.add_contact(contact);
                        (*contact).flags.insert(ContactFlags::ISLAND);

                        let other = (*ce).other_ptr;
                        if (*other).flags.contains(BodyFlags::ISLAND) {
                            continue;
                        }

                        stack.push(other);
                        (*other).flags.insert(BodyFlags::ISLAND);
                    }

                    seed = (*seed).next_ptr;
                }

                let mut profile = Profile::default();
                island.solve(&mut profile, step, self.0.gravity, self.0.allow_sleep);
                self.0.profile.solve_init += profile.solve_init;
                self.0.profile.solve_velocity += profile.solve_velocity;
                self.0.profile.solve_position += profile.solve_position;

                for i in 0..island.bodies.len() {
                    let b = island.bodies[i];
                    if (*b).body_type() == BodyType::Static {
                        (*b).flags.remove(BodyFlags::ISLAND);
                    }
                }
            }

            let timer = Timer::new();
            let mut b = self.0.body_list;
            while !b.is_null() {
                if !(*b).flags.contains(BodyFlags::ISLAND) {
                    b = (*b).next_ptr;
                    continue;
                }
                if (*b).body_type() == BodyType::Static {
                    b = (*b).next_ptr;
                    continue;
                }
                (*b).synchronize_fixtures();
                b = (*b).next_ptr;
            }

            self.0.contact_manager.find_new_contacts();
            self.0.profile.broad_phase = timer.get_duration();
        }
    }

    fn solve_toi(&mut self, step: &TimeStep<T>) {
        unsafe {
            let mut island = Island::new(
                2 * settings::MAX_TOI_CONTACTS,
                settings::MAX_TOI_CONTACTS,
                (&self.0.contact_manager.contact_listener
                    as *const Option<Box<dyn ContactListener<T, D>>>)
                    .as_ref()
                    .unwrap(),
            );

            if self.0.step_complete {
                let mut b = self.0.body_list;
                while !b.is_null() {
                    (*b).flags.remove(BodyFlags::ISLAND);
                    b = (*b).next_ptr;
                }

                let mut c = self.0.contact_manager.contact_list;
                while !c.is_null() {
                    (*c).flags.remove(ContactFlags::TOI | ContactFlags::ISLAND);
                    (*c).toi_count = 0;
                    c = (*c).next_ptr;
                }
            }

            loop {
                let mut min_contact = std::ptr::null_mut();
                let mut min_alpha = T::one();

                let mut c = self.0.contact_manager.contact_list;
                while !c.is_null() {
                    if !(*c).is_enable() {
                        c = (*c).next_ptr;
                        continue;
                    }

                    if (*c).toi_count > settings::MAX_SUB_STEPS {
                        c = (*c).next_ptr;
                        continue;
                    }

                    let alpha;
                    if (*c).flags.contains(ContactFlags::TOI) {
                        alpha = (*c).toi;
                    } else {
                        let fa = (*c).fixture_a_ptr;
                        let fb = (*c).fixture_b_ptr;

                        if (*fa).is_sensor || (*fb).is_sensor {
                            c = (*c).next_ptr;
                            continue;
                        }

                        let ba = (*fa).body_ptr;
                        let bb = (*fb).body_ptr;

                        let type_a = (*ba).type_;
                        let type_b = (*bb).type_;
                        assert!(type_a == BodyType::Dynamic || type_b == BodyType::Dynamic);

                        let active_a = (*ba).is_awake() && type_a != BodyType::Static;
                        let active_b = (*bb).is_awake() && type_b != BodyType::Static;

                        if !active_a && !active_b {
                            c = (*c).next_ptr;
                            continue;
                        }

                        let collide_a = (*ba).is_bullet() || type_a != BodyType::Dynamic;
                        let collide_b = (*bb).is_bullet() || type_b != BodyType::Dynamic;

                        if !collide_a && !collide_b {
                            c = (*c).next_ptr;
                            continue;
                        }

                        let mut alpha0 = (*ba).sweep.alpha0;

                        if (*ba).sweep.alpha0 < (*bb).sweep.alpha0 {
                            alpha0 = (*bb).sweep.alpha0;
                            (*ba).sweep.advance(alpha0);
                        } else if (*bb).sweep.alpha0 < (*ba).sweep.alpha0 {
                            alpha0 = (*ba).sweep.alpha0;
                            (*bb).sweep.advance(alpha0);
                        }

                        assert!(alpha0 < T::one());

                        let index_a = (*c).child_index_a();
                        let index_b = (*c).child_index_b();

                        let input = TOIInput {
                            proxy_a: &(*fa).shape.distance_proxy(index_a),
                            proxy_b: &(*fb).shape.distance_proxy(index_b),
                            sweep_a: (*ba).sweep,
                            sweep_b: (*bb).sweep,
                            max: T::one(),
                        };

                        let output = time_of_impact(input);

                        let beta = output.t;
                        if output.state == TOIOutputState::Touching {
                            alpha = T::min(alpha0 + (T::one() - alpha0) * beta, T::one());
                        } else {
                            alpha = T::one();
                        }

                        (*c).toi = alpha;
                        (*c).flags.insert(ContactFlags::TOI);
                    }

                    if alpha < min_alpha {
                        min_contact = c;
                        min_alpha = alpha;
                    }
                }

                if min_contact.is_null() || T::one() - T::ten() * T::epsilon() < min_alpha {
                    self.0.step_complete = true;
                    break;
                }

                let fa = (*min_contact).fixture_a_ptr;
                let fb = (*min_contact).fixture_b_ptr;
                let mut ba = (*fa).body_ptr;
                let mut bb = (*fb).body_ptr;

                let backup1 = (*ba).sweep;
                let backup2 = (*bb).sweep;

                (*ba).advance(min_alpha);
                (*bb).advance(min_alpha);

                (*min_contact).update(&self.0.contact_manager.contact_listener);
                (*min_contact).flags.remove(ContactFlags::TOI);
                (*min_contact).toi_count += 1;

                if !(*min_contact).is_enable() || !(*min_contact).is_touching() {
                    (*min_contact).set_enable(false);
                    (*ba).sweep = backup1;
                    (*bb).sweep = backup2;
                    (*ba).synchronize_transform();
                    (*bb).synchronize_transform();
                    continue;
                }

                (*ba).set_awake(true);
                (*bb).set_awake(true);

                island.clear();
                island.add_body(ba);
                island.add_body(bb);
                island.add_contact(min_contact);

                (*ba).flags.insert(BodyFlags::ISLAND);
                (*bb).flags.insert(BodyFlags::ISLAND);
                (*min_contact).flags.insert(ContactFlags::ISLAND);

                let bodies = [ba, bb];
                for i in 0..2 {
                    let body = bodies[i];
                    if (*body).type_ == BodyType::Dynamic {
                        let mut ce = (*body).contact_list_ptr;
                        while !ce.is_null() {
                            if island.bodies.len() == island.bodies.capacity() {
                                break;
                            }

                            if island.contacts.len() == island.contacts.capacity() {
                                break;
                            }

                            let contact = (*ce).contact_ptr;
                            if (*contact).flags.contains(ContactFlags::ISLAND) {
                                ce = (*ce).next_ptr;
                                continue;
                            }

                            let other = (*ce).other_ptr;
                            if (*other).type_ == BodyType::Dynamic
                                && !(*body).is_bullet()
                                && !(*body).is_bullet()
                            {
                                ce = (*ce).next_ptr;
                                continue;
                            }

                            let sensor_a = (*(*contact).fixture_a_ptr).is_sensor;
                            let sensor_b = (*(*contact).fixture_b_ptr).is_sensor;
                            if sensor_a || sensor_b {
                                ce = (*ce).next_ptr;
                                continue;
                            }

                            let backup = (*other).sweep;
                            if !(*other).flags.contains(BodyFlags::ISLAND) {
                                (*other).advance(min_alpha);
                            }

                            (*contact).update(&self.0.contact_manager.contact_listener);

                            if !(*contact).is_enable() {
                                (*other).sweep = backup;
                                (*other).synchronize_transform();
                                ce = (*ce).next_ptr;
                                continue;
                            }

                            (*contact).flags.insert(ContactFlags::ISLAND);
                            island.add_contact(contact);

                            if (*other).flags.contains(BodyFlags::ISLAND) {
                                ce = (*ce).next_ptr;
                                continue;
                            }

                            (*other).flags.insert(BodyFlags::ISLAND);

                            if (*other).type_ != BodyType::Static {
                                (*other).set_awake(true);
                            }

                            island.add_body(other);
                        }
                    }
                }

                let sub_step_dt = (T::one() - min_alpha) * step.dt;
                let sub_step = TimeStep {
                    dt: sub_step_dt,
                    inv_dt: T::one() / sub_step_dt,
                    dt_ratio: T::one(),
                    velocity_iterations: step.velocity_iterations,
                    position_iterations: 20,
                    warm_starting: false,
                };
                island.solve_toi(&sub_step, (*ba).island_index, (*bb).island_index);

                for i in 0..island.bodies.len() {
                    let body = island.bodies[i];
                    (*body).flags.remove(BodyFlags::ISLAND);
                    if (*body).type_ != BodyType::Dynamic {
                        continue;
                    }
                    (*body).synchronize_fixtures();

                    let mut ce = (*body).contact_list_ptr;
                    while !ce.is_null() {
                        (*(*ce).contact_ptr)
                            .flags
                            .remove(ContactFlags::TOI | ContactFlags::ISLAND);
                        ce = (*ce).next_ptr;
                    }
                }

                self.0.contact_manager.find_new_contacts();

                if self.0.sub_stepping {
                    self.0.step_complete = false;
                    break;
                }
            }
        }
    }

    pub fn step(&mut self, dt: T, velocity_iterations: usize, position_iterations: usize) {
        let timer = Timer::new();

        if self.0.flags.contains(WorldFlags::NEW_FIXTURE) {
            self.0.contact_manager.find_new_contacts();
            self.0.flags.remove(WorldFlags::NEW_FIXTURE)
        }

        self.0.flags.insert(WorldFlags::LOCKED);

        let step = TimeStep {
            dt,
            inv_dt: if dt > T::zero() {
                T::one() / dt
            } else {
                T::zero()
            },
            dt_ratio: self.0.inv_dt0 * dt,
            velocity_iterations,
            position_iterations,
            warm_starting: self.0.warm_starting,
        };

        {
            let timer = Timer::new();
            self.0.contact_manager.collide();
            self.0.profile.collide = timer.get_duration();
        }

        if self.0.step_complete && step.dt > T::zero() {
            let timer = Timer::new();
            self.solve(&step);
            self.0.profile.solve = timer.get_duration();
        }

        if self.0.continuous_physics && step.dt > T::zero() {
            let timer = Timer::new();
            self.solve_toi(&step);
            self.0.profile.solve_toi = timer.get_duration();
        }

        if step.dt > T::zero() {
            self.0.inv_dt0 = step.inv_dt;
        }

        if self.0.flags.contains(WorldFlags::CLEAR_FORCES) {
            self.clear_forces();
        }

        self.0.flags.remove(WorldFlags::LOCKED);
        self.0.profile.step = timer.get_duration();
    }

    pub fn draw_debug_data(&self) {
        if let Some(dd) = &self.0.debug_draw {
            unsafe {
                if self.0.debug_draw_flags.contains(DebugDrawFlags::SHAPE) {
                    let mut b = self.0.body_list;
                    while !b.is_null() {
                        if (*b).is_debug_draw() {
                            b = (*b).next_ptr;
                            continue;
                        }

                        let xf = (*b).transform();
                        for (_, f) in &(*b).fixture_list {
                            if !(*b).is_active() {
                                self.draw_shape(f, xf, Color::rgb(0.5, 0.5, 0.3));
                            } else if (*b).body_type() == BodyType::Static {
                                self.draw_shape(f, xf, Color::rgb(0.5, 0.9, 0.5));
                            } else if (*b).body_type() == BodyType::Kinematic {
                                self.draw_shape(f, xf, Color::rgb(0.5, 0.5, 0.9));
                            } else if !(*b).is_awake() {
                                self.draw_shape(f, xf, Color::rgb(0.6, 0.6, 0.6));
                            } else {
                                self.draw_shape(f, xf, Color::rgb(0.9, 0.7, 0.7));
                            }
                        }

                        b = (*b).next_ptr;
                    }
                }

                if self.0.debug_draw_flags.contains(DebugDrawFlags::AABB) {
                    let color = Color::rgb(0.9, 0.3, 0.9);
                    let bp = &self.0.contact_manager.broad_phase;

                    let mut b = self.0.body_list;
                    while !b.is_null() {
                        if !(*b).is_debug_draw() {
                            b = (*b).next_ptr;
                            continue;
                        }

                        if !(*b).is_active() {
                            b = (*b).next_ptr;
                            continue;
                        }

                        for (_, f) in &(*b).fixture_list {
                            for i in 0..f.proxies.len() {
                                let proxy = &f.proxies[i];
                                let aabb = bp.tree.get_fat_aabb(proxy.proxy_id);
                                let vs = [
                                    Vector2::new(
                                        aabb.lower_bound.x.to_f32(),
                                        aabb.lower_bound.y.to_f32(),
                                    ),
                                    Vector2::new(
                                        aabb.upper_bound.x.to_f32(),
                                        aabb.lower_bound.y.to_f32(),
                                    ),
                                    Vector2::new(
                                        aabb.upper_bound.x.to_f32(),
                                        aabb.upper_bound.y.to_f32(),
                                    ),
                                    Vector2::new(
                                        aabb.lower_bound.x.to_f32(),
                                        aabb.upper_bound.y.to_f32(),
                                    ),
                                ];
                                dd.draw_polygon(&vs, color);
                            }
                        }

                        b = (*b).next_ptr;
                    }
                }

                if self
                    .0
                    .debug_draw_flags
                    .contains(DebugDrawFlags::CENTER_OF_MASS_BIT)
                {
                    let mut b = self.0.body_list;
                    while !b.is_null() {
                        if (*b).is_debug_draw() {
                            b = (*b).next_ptr;
                            continue;
                        }

                        let mut xf = *((*b).transform());
                        xf.p = *((*b).world_center());

                        let xf_f32 = {
                            Transform::<f32> {
                                p: Vector2::new(xf.p.x.to_f32(), xf.p.y.to_f32()),
                                q: Rotation::new(xf.q.angle().to_f32()),
                            }
                        };
                        dd.draw_transform(&xf_f32);
                        b = (*b).next_ptr;
                    }
                }
            }
        }
    }

    fn draw_shape(&self, f: &Fixture<T, D>, xf: &Transform<T>, color: Color) {
        unsafe {
            if let Some(dd) = &self.0.debug_draw {
                match f.shape.shape_type() {
                    ShapeType::Circle => {
                        let circle = (f.shape.as_ref() as *const dyn Shape<T>
                            as *const ShapeCircle<T>)
                            .as_ref()
                            .unwrap();
                        let center = xf.multiply(circle.position);
                        let radius = circle.radius;
                        let axis = xf.q.multiply(Vector2::new(T::one(), T::zero()));
                        dd.draw_solid_circle(
                            &Vector2 {
                                x: center.x.to_f32(),
                                y: center.y.to_f32(),
                            },
                            radius.to_f32(),
                            &Vector2::new(axis.x.to_f32(), axis.y.to_f32()),
                            color,
                        );
                    }
                    ShapeType::Edge => {
                        let edge = (f.shape.as_ref() as *const dyn Shape<T> as *const ShapeEdge<T>)
                            .as_ref()
                            .unwrap();
                        let v1 = xf.multiply(edge.vertex1);
                        let v2 = xf.multiply(edge.vertex2);
                        dd.draw_segment(
                            &Vector2 {
                                x: v1.x.to_f32(),
                                y: v1.y.to_f32(),
                            },
                            &Vector2 {
                                x: v2.x.to_f32(),
                                y: v2.y.to_f32(),
                            },
                            color,
                        );
                    }
                    ShapeType::Polygon => {
                        let polygon = (f.shape.as_ref() as *const dyn Shape<T>
                            as *const ShapePolygon<T>)
                            .as_ref()
                            .unwrap();
                        let mut vertices = [Vector2::<f32>::zero(); settings::MAX_POLYGON_VERTICES];
                        for (i, vertex) in polygon.vertices.iter().enumerate() {
                            let v = xf.multiply(*vertex);
                            vertices[i] = Vector2::new(v.x.to_f32(), v.y.to_f32());
                        }
                        dd.draw_polygon(&vertices[0..polygon.vertices.len()], color);
                    }
                    ShapeType::Chain => {
                        let chain = (f.shape.as_ref() as *const dyn Shape<T>
                            as *const ShapeChain<T>)
                            .as_ref()
                            .unwrap();
                        let count = chain.vertices.len();
                        let vertices = &chain.vertices;
                        let ghost_color =
                            Color::rgba(0.75 * color.r, 0.75 * color.g, 0.75 * color.b, color.a);

                        let mut v1 = xf.multiply(vertices[0]);
                        dd.draw_point(
                            &Vector2 {
                                x: v1.x.to_f32(),
                                y: v1.y.to_f32(),
                            },
                            4.0,
                            color,
                        );

                        if let Some(prev_vertex) = &chain.prev_vertex {
                            let vp = xf.multiply(*prev_vertex);
                            dd.draw_segment(
                                &Vector2::new(vp.x.to_f32(), vp.y.to_f32()),
                                &Vector2::new(v1.x.to_f32(), v1.y.to_f32()),
                                ghost_color,
                            );
                            dd.draw_circle(
                                &Vector2::new(vp.x.to_f32(), vp.y.to_f32()),
                                0.1,
                                ghost_color,
                            );
                        }

                        for i in 1..count {
                            let v2 = xf.multiply(vertices[i]);
                            dd.draw_segment(
                                &Vector2::new(v1.x.to_f32(), v1.y.to_f32()),
                                &Vector2::new(v2.x.to_f32(), v2.y.to_f32()),
                                ghost_color,
                            );
                            dd.draw_point(
                                &Vector2 {
                                    x: v2.x.to_f32(),
                                    y: v2.y.to_f32(),
                                },
                                4.0,
                                color,
                            );
                            v1 = v2;
                        }

                        if let Some(next_vertex) = &chain.next_vertex {
                            let vn = xf.multiply(*next_vertex);
                            dd.draw_segment(
                                &Vector2::new(v1.x.to_f32(), v1.y.to_f32()),
                                &Vector2::new(vn.x.to_f32(), vn.y.to_f32()),
                                ghost_color,
                            );
                            dd.draw_circle(
                                &Vector2::new(vn.x.to_f32(), vn.y.to_f32()),
                                0.1,
                                ghost_color,
                            );
                        }
                    }
                }
            }
        }
    }

    pub fn query_aabb(&self, aabb: AABB<T>) -> impl Iterator<Item = &Fixture<T, D>> {
        self.0
            .contact_manager
            .broad_phase
            .tree
            .query(aabb)
            .map(|item| unsafe { ((*(*item.2)).fixture_ptr).as_ref().unwrap() })
    }

    pub fn ray_cast(&self, input: RayCastInput<T>) -> RayCastIter<T, D> {
        RayCastIter {
            iter: self.0.contact_manager.broad_phase.tree.ray_cast(input),
        }
    }
}

pub struct RayCastIter<'a, T, D> {
    iter: dynamic_tree::RayCastIter<'a, T, *mut FixtureProxy<T, D>>,
}

impl<'a, T: Real, D> RayCastIter<'a, T, D> {
    pub fn set_max_fraction(&mut self, value: T) {
        self.iter.set_max_fraction(value);
    }
}

impl<'a, T: Real, D> Iterator for RayCastIter<'a, T, D> {
    type Item = &'a Fixture<T, D>;

    fn next(&mut self) -> Option<Self::Item> {
        Iterator::next(&mut self.iter)
            .map(|item| unsafe { ((*(*item.2)).fixture_ptr).as_ref().unwrap() })
    }
}
