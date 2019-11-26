use crate::collision::shapes::Shape;
use crate::dynamic::contacts::ContactEdge;
use crate::dynamic::fixture::FixtureDef;
use crate::dynamic::world::{WorldFlags, WorldInner};
use crate::{Fixture, MassData};
use std::alloc::Layout;
use xmath::{
    CrossTrait, DotTrait, Multiply, Real, Rotation, Sweep, Transform, TransposeMultiply, Vector2,
};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum BodyType {
    Static,
    Kinematic,
    Dynamic,
}

pub struct BodyDef<T, D> {
    pub type_: BodyType,
    pub position: Vector2<T>,
    pub angle: T,
    pub linear_velocity: Vector2<T>,
    pub angular_velocity: T,
    pub linear_damping: T,
    pub angular_damping: T,
    pub allow_sleep: bool,
    pub awake: bool,
    pub fixed_rotation: bool,
    pub bullet: bool,
    pub active: bool,
    pub data: D,
    pub gravity_scale: T,
}

impl<T: Real, D: Default> Default for BodyDef<T, D> {
    fn default() -> Self {
        BodyDef {
            type_: BodyType::Static,
            position: Vector2::zero(),
            angle: T::zero(),
            linear_velocity: Vector2::zero(),
            angular_velocity: T::zero(),
            linear_damping: T::zero(),
            angular_damping: T::zero(),
            allow_sleep: true,
            awake: true,
            fixed_rotation: false,
            bullet: false,
            active: true,
            data: Default::default(),
            gravity_scale: T::zero(),
        }
    }
}

bitflags! {
    pub struct BodyFlags: u32 {
        const ISLAND = 0x0001;
        const AWAKE = 0x0002;
        const AUTO_SLEEP = 0x0004;
        const BULLET = 0x0008;
        const FIXED_ROTATION = 0x0010;
        const ACTIVE = 0x0020;
        const TOI = 0x0040;
        const DEBUG_DRAW = 0x0080;
    }
}

pub struct Body<T, D> {
    pub(crate) world_ptr: *mut WorldInner<T, D>,
    pub(crate) type_: BodyType,
    pub(crate) flags: BodyFlags,
    pub(crate) island_index: usize,
    pub(crate) xf: Transform<T>,
    pub(crate) sweep: Sweep<T>,
    pub(crate) linear_velocity_: Vector2<T>,
    pub(crate) angular_velocity_: T,
    pub(crate) force: Vector2<T>,
    pub(crate) torque: T,
    pub(crate) prev_ptr: *mut Body<T, D>,
    pub(crate) next_ptr: *mut Body<T, D>,
    pub(crate) fixture_list: Vec<Box<Fixture<T, D>>>,
    pub(crate) contact_list_ptr: *mut ContactEdge<T, D>,
    pub(crate) mass: T,
    pub(crate) inv_mass: T,
    pub(crate) i: T,
    pub(crate) inv_i: T,
    pub(crate) linear_damping: T,
    pub(crate) angular_damping: T,
    pub(crate) gravity_scale: T,
    pub(crate) sleep_time: T,
    pub(crate) data: D,
}

impl<T: Real, D> Body<T, D> {
    pub(crate) fn new(world_ptr: *mut WorldInner<T, D>, def: BodyDef<T, D>) -> Body<T, D> {
        assert!(def.position.is_valid());
        assert!(def.linear_velocity.is_valid());
        assert!(def.angle.is_valid());
        assert!(def.angular_velocity.is_valid());
        assert!(def.angular_damping.is_valid() && def.angular_damping >= T::zero());
        assert!(def.linear_damping.is_valid() && def.linear_damping >= T::zero());

        let mut flags = BodyFlags::DEBUG_DRAW;

        if def.bullet {
            flags.insert(BodyFlags::BULLET);
        }

        if def.fixed_rotation {
            flags.insert(BodyFlags::FIXED_ROTATION);
        }

        if def.allow_sleep {
            flags.insert(BodyFlags::AUTO_SLEEP);
        }

        if def.awake {
            flags.insert(BodyFlags::AWAKE);
        }

        if def.active {
            flags.insert(BodyFlags::ACTIVE);
        }

        let (mass, inv_mass) = if def.type_ == BodyType::Dynamic {
            (T::one(), T::one())
        } else {
            (T::zero(), T::zero())
        };

        Body {
            world_ptr,
            type_: def.type_,
            flags,
            island_index: 0,
            xf: Transform::new(def.position, Rotation::new(def.angle)),
            sweep: Sweep {
                local_center: Vector2::zero(),
                c0: def.position,
                c: def.position,
                a0: def.angle,
                a: def.angle,
                alpha0: T::zero(),
            },
            linear_velocity_: def.linear_velocity,
            angular_velocity_: def.angular_velocity,
            force: Vector2::zero(),
            torque: T::zero(),
            prev_ptr: std::ptr::null_mut(),
            next_ptr: std::ptr::null_mut(),
            fixture_list: Vec::new(),
            contact_list_ptr: std::ptr::null_mut(),
            mass,
            inv_mass,
            i: T::zero(),
            inv_i: T::zero(),
            linear_damping: def.linear_damping,
            angular_damping: def.angular_damping,
            gravity_scale: def.gravity_scale,
            sleep_time: T::zero(),
            data: def.data,
        }
    }

    pub fn destroy(&mut self) {
        unsafe {
            assert!(!(*self.world_ptr).flags.contains(WorldFlags::LOCKED));

            let mut ce = self.contact_list_ptr;
            while !ce.is_null() {
                let ce0 = ce;
                ce = (*ce).next_ptr;
                (*self.world_ptr)
                    .contact_manager
                    .destroy((*ce0).contact_ptr);
            }

            for f in &mut self.fixture_list {
                if let Some(l) = &(*self.world_ptr).destruction_listener {
                    l.fixture_destroyed(f.as_ref());
                }
                f.destroy_proxies(&mut (*self.world_ptr).contact_manager.broad_phase);
            }

            if !self.prev_ptr.is_null() {
                (*self.prev_ptr).next_ptr = self.next_ptr;
            }
            if !self.next_ptr.is_null() {
                (*self.next_ptr).prev_ptr = self.prev_ptr;
            }
            if self as *mut Body<T, D> == (*self.world_ptr).body_list {
                (*self.world_ptr).body_list = self.next_ptr;
            }

            std::ptr::drop_in_place(self);
            std::alloc::dealloc(
                self as *mut Body<T, D> as *mut u8,
                Layout::new::<Body<T, D>>(),
            );

            (*self.world_ptr).body_count -= 1;
        }
    }

    pub fn body_type(&self) -> BodyType {
        self.type_
    }

    pub fn transform(&self) -> &Transform<T> {
        &self.xf
    }

    pub fn position(&self) -> &Vector2<T> {
        &self.xf.p
    }

    pub fn angle(&self) -> T {
        self.sweep.a
    }

    pub fn world_center(&self) -> &Vector2<T> {
        &self.sweep.c
    }

    pub fn local_center(&self) -> &Vector2<T> {
        &self.sweep.local_center
    }

    pub fn set_linear_velocity(&mut self, v: Vector2<T>) {
        if self.type_ == BodyType::Static {
            return;
        }
        if v.dot(v) > T::zero() {
            self.set_awake(true);
        }
        self.linear_velocity_ = v;
    }

    pub fn linear_velocity(&self) -> &Vector2<T> {
        &self.linear_velocity_
    }

    pub fn set_angular_velocity(&mut self, w: T) {
        if self.type_ == BodyType::Static {
            return;
        }
        if w * w > T::zero() {
            self.set_awake(true);
        }
        self.angular_velocity_ = w;
    }

    pub fn angular_velocity(&self) -> T {
        self.angular_velocity_
    }

    pub fn apply_force(&mut self, force: Vector2<T>, point: Vector2<T>, wake: bool) {
        if self.type_ != BodyType::Dynamic {
            return;
        }

        if wake && !self.flags.contains(BodyFlags::AWAKE) {
            self.set_awake(true);
        }

        if self.flags.contains(BodyFlags::AWAKE) {
            self.force += force;
            self.torque += (point - self.sweep.c).cross(force);
        }
    }

    pub fn apply_force_to_center(&mut self, force: Vector2<T>, wake: bool) {
        if self.type_ != BodyType::Dynamic {
            return;
        }

        if wake && !self.flags.contains(BodyFlags::AWAKE) {
            self.set_awake(true);
        }

        if self.flags.contains(BodyFlags::AWAKE) {
            self.force += force;
        }
    }

    pub fn apply_torque(&mut self, torque: T, wake: bool) {
        if self.type_ != BodyType::Dynamic {
            return;
        }

        if wake && !self.flags.contains(BodyFlags::AWAKE) {
            self.set_awake(true);
        }

        if self.flags.contains(BodyFlags::AWAKE) {
            self.torque += torque;
        }
    }

    pub fn apply_linear_impulse(&mut self, impulse: Vector2<T>, point: Vector2<T>, wake: bool) {
        if self.type_ != BodyType::Dynamic {
            return;
        }

        if wake && !self.flags.contains(BodyFlags::AWAKE) {
            self.set_awake(true);
        }

        if self.flags.contains(BodyFlags::AWAKE) {
            self.linear_velocity_ += impulse * self.inv_mass;
            self.angular_velocity_ += self.inv_i * (point - self.sweep.c).cross(impulse);
        }
    }

    pub fn apply_linear_impulse_to_center(&mut self, impulse: Vector2<T>, wake: bool) {
        if self.type_ != BodyType::Dynamic {
            return;
        }

        if wake && !self.flags.contains(BodyFlags::AWAKE) {
            self.set_awake(true);
        }

        if self.flags.contains(BodyFlags::AWAKE) {
            self.linear_velocity_ += impulse * self.inv_mass;
        }
    }

    pub fn apply_angular_impulse(&mut self, impulse: T, wake: bool) {
        if self.type_ != BodyType::Dynamic {
            return;
        }

        if wake && !self.flags.contains(BodyFlags::AWAKE) {
            self.set_awake(true);
        }

        if self.flags.contains(BodyFlags::AWAKE) {
            self.angular_velocity_ += impulse * self.inv_i;
        }
    }

    pub(crate) fn synchronize_transform(&mut self) {
        self.xf.q = Rotation::new(self.sweep.a);
        self.xf.p = self.sweep.c - self.xf.q.multiply(self.sweep.local_center);
    }

    pub(crate) fn advance(&mut self, alpha: T) {
        self.sweep.advance(alpha);
        self.sweep.c = self.sweep.c0;
        self.sweep.a = self.sweep.a0;
        self.xf.q = Rotation::new(self.sweep.a);
        self.xf.p = self.sweep.c - self.xf.q.multiply(self.sweep.local_center);
    }

    pub(crate) fn synchronize_fixtures(&mut self) {
        let r = Rotation::new(self.sweep.a0);
        let xf1 = Transform {
            q: r,
            p: self.sweep.c0 - r.multiply(self.sweep.local_center),
        };

        unsafe {
            let broad_phase = &mut (*self.world_ptr).contact_manager.broad_phase;
            for f in &mut self.fixture_list {
                f.synchronize(broad_phase, &xf1, &self.xf);
            }
        }
    }

    pub fn mass(&self) -> T {
        self.mass
    }

    pub fn inertia(&self) -> T {
        self.i + self.mass * self.sweep.local_center.dot(self.sweep.local_center)
    }

    pub fn mass_data(&self) -> MassData<T> {
        MassData {
            mass: self.mass,
            center: self.sweep.local_center,
            i: self.inertia(),
        }
    }

    pub fn world_point(&self, local_point: Vector2<T>) -> Vector2<T> {
        self.xf.multiply(local_point)
    }

    pub fn world_vector(&self, local_vector: Vector2<T>) -> Vector2<T> {
        self.xf.q.multiply(local_vector)
    }

    pub fn local_point(&self, world_point: Vector2<T>) -> Vector2<T> {
        self.xf.transpose_multiply(world_point)
    }

    pub fn local_vector(&self, world_vector: Vector2<T>) -> Vector2<T> {
        self.xf.q.transpose_multiply(world_vector)
    }

    pub fn linear_velocity_from_world_point(&self, world_point: Vector2<T>) -> Vector2<T> {
        self.linear_velocity_ + self.angular_velocity_.cross(world_point - self.sweep.c)
    }

    pub fn linear_velocity_from_local_point(&self, local_point: Vector2<T>) -> Vector2<T> {
        self.linear_velocity_from_world_point(self.world_point(local_point))
    }

    pub fn linear_damping(&self) -> T {
        self.linear_damping
    }

    pub fn set_linear_damping(&mut self, linear_damping: T) {
        self.linear_damping = linear_damping;
    }

    pub fn angular_damping(&self) -> T {
        self.angular_damping
    }

    pub fn set_angular_damping(&mut self, angular_damping: T) {
        self.angular_damping = angular_damping;
    }

    pub fn gravity_scale(&self) -> T {
        self.gravity_scale
    }

    pub fn set_gravity_scale(&mut self, scale: T) {
        self.gravity_scale = scale;
    }

    pub fn set_debug_draw(&mut self, flag: bool) {
        self.flags.set(BodyFlags::DEBUG_DRAW, flag);
    }

    pub fn is_debug_draw(&self) -> bool {
        self.flags.contains(BodyFlags::DEBUG_DRAW)
    }

    pub fn set_body_type(&mut self, type_: BodyType) {
        unsafe {
            assert!(!(*self.world_ptr).flags.contains(WorldFlags::LOCKED));
            if self.type_ == type_ {
                return;
            }
            self.type_ = type_;
            self.reset_mass();

            if self.type_ == BodyType::Static {
                self.linear_velocity_ = Vector2::zero();
                self.angular_velocity_ = T::zero();
                self.sweep.a0 = self.sweep.a;
                self.sweep.c0 = self.sweep.c;
                self.synchronize_fixtures();
            }

            self.set_awake(true);

            self.force = Vector2::zero();
            self.torque = T::zero();

            let mut ce = self.contact_list_ptr;
            while !ce.is_null() {
                let ce0 = ce;
                ce = (*ce).next_ptr;
                (*self.world_ptr)
                    .contact_manager
                    .destroy((*ce0).contact_ptr);
            }
            self.contact_list_ptr = std::ptr::null_mut();

            let broad_phase = &mut (*self.world_ptr).contact_manager.broad_phase;
            for f in &mut self.fixture_list {
                for proxy in &f.proxies {
                    broad_phase.touch_proxy(proxy.proxy_id);
                }
            }
        }
    }

    pub fn set_bullet(&mut self, flag: bool) {
        self.flags.set(BodyFlags::BULLET, flag);
    }

    pub fn is_bullet(&self) -> bool {
        self.flags.contains(BodyFlags::BULLET)
    }

    pub fn set_awake(&mut self, flag: bool) {
        if flag {
            self.flags.insert(BodyFlags::AWAKE);
            self.sleep_time = T::zero();
        } else {
            self.flags.remove(BodyFlags::AWAKE);
            self.sleep_time = T::zero();
            self.linear_velocity_ = Vector2::zero();
            self.angular_velocity_ = T::zero();
            self.force = Vector2::zero();
            self.torque = T::zero();
        }
    }

    pub fn is_awake(&self) -> bool {
        self.flags.contains(BodyFlags::AWAKE)
    }

    pub fn is_active(&self) -> bool {
        self.flags.contains(BodyFlags::ACTIVE)
    }

    pub fn set_fixed_rotation(&mut self, flag: bool) {
        self.flags.set(BodyFlags::FIXED_ROTATION, flag);
    }

    pub fn is_fixed_rotation(&self) -> bool {
        self.flags.contains(BodyFlags::FIXED_ROTATION)
    }

    pub fn set_sleeping_allowed(&mut self, flag: bool) {
        if flag {
            self.flags.insert(BodyFlags::AUTO_SLEEP);
        } else {
            self.flags.remove(BodyFlags::AUTO_SLEEP);
            self.set_awake(true);
        }
    }

    pub fn is_sleeping_allowed(&self) -> bool {
        self.flags.contains(BodyFlags::AUTO_SLEEP)
    }

    pub fn get_fixture_list(&self) -> &[Box<Fixture<T, D>>] {
        &self.fixture_list
    }

    fn reset_mass(&mut self) {
        self.mass = T::zero();
        self.inv_mass = T::zero();
        self.i = T::zero();
        self.inv_i = T::zero();
        self.sweep.local_center = Vector2::zero();

        if self.type_ == BodyType::Static || self.type_ == BodyType::Kinematic {
            self.sweep.c0 = self.xf.p;
            self.sweep.c = self.xf.p;
            self.sweep.a0 = self.sweep.a;
            return;
        }

        assert_eq!(self.type_, BodyType::Dynamic);

        let mut local_center = Vector2::zero();
        for f in &self.fixture_list {
            if f.density() == T::zero() {
                continue;
            }

            let mass_data = f.mass_data();
            self.mass += mass_data.mass;
            local_center += mass_data.center * mass_data.mass;
            self.i += mass_data.i;
        }

        if self.mass > T::zero() {
            self.inv_mass = T::one() / self.mass;
            local_center *= self.inv_mass;
        } else {
            self.mass = T::one();
            self.inv_mass = T::one();
        }

        if self.i > T::zero() && !self.flags.contains(BodyFlags::FIXED_ROTATION) {
            self.i -= self.mass * local_center.dot(local_center);
            assert!(self.i > T::zero());
            self.inv_i = T::one() / self.i;
        } else {
            self.i = T::zero();
            self.inv_i = T::zero();
        }

        let old_center = self.sweep.c;
        self.sweep.local_center = local_center;
        self.sweep.c0 = self.xf.multiply(self.sweep.local_center);
        self.sweep.c = self.sweep.c0;

        self.linear_velocity_ += self.angular_velocity_.cross(self.sweep.c - old_center);
    }

    pub(crate) fn should_collide(&self, other: &Body<T, D>) -> bool {
        match (self.type_, other.type_) {
            (BodyType::Dynamic, BodyType::Dynamic) => true,
            _ => false,
        }
    }

    pub fn create_fixture<S: Shape<T> + 'static>(
        &mut self,
        def: FixtureDef<T, D, S>,
    ) -> &mut Fixture<T, D> {
        unsafe {
            let shape = Box::new(def.shape);
            let child_count = shape.child_count();
            let fixture = Box::new(Fixture {
                density: def.density,
                body_ptr: self as *mut Body<T, D>,
                shape,
                friction: def.friction,
                restitution: def.restitution,
                proxies: Vec::with_capacity(child_count),
                filter: def.filter,
                is_sensor: def.is_sensor,
                data: def.data,
            });
            self.fixture_list.push(fixture);

            let fixture = self.fixture_list.last_mut().unwrap().as_mut() as *mut Fixture<T, D>;
            if self.flags.contains(BodyFlags::ACTIVE) {
                (*fixture)
                    .create_proxies(&mut (*self.world_ptr).contact_manager.broad_phase, self.xf);
            }

            if (*fixture).density > T::zero() {
                self.reset_mass();
            }

            (*self.world_ptr).flags.insert(WorldFlags::NEW_FIXTURE);
            fixture.as_mut().unwrap()
        }
    }
}
