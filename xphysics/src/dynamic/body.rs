use crate::shapes::Shape;
use crate::{Fixture, FixtureBuilder, MassData, World};
use xmath::{CrossTrait, DotTrait, Multiply, Real, Sweep, Transform, TransposeMultiply, Vector2};

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

bitflags! {
    struct BodyFlags: u32 {
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
    type_: BodyType,
    flags: BodyFlags,
    island_index: usize,
    xf: Transform<T>,
    sweep: Sweep<T>,
    linear_velocity: Vector2<T>,
    angular_velocity: T,
    force: Vector2<T>,
    torque: T,
    world: *mut World,
    prev: *mut Body<T, D>,
    next: *mut Body<T, D>,
    fixture_list: Vec<Box<Fixture<T, D>>>,
    mass: T,
    inv_mass: T,
    i: T,
    inv_i: T,
    linear_damping: T,
    angular_damping: T,
    gravity_scale: T,
    sleep_time: T,
    data: D,
}

impl<T: Real, D> Body<T, D> {
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
        self.linear_velocity = v;
    }

    pub fn linear_velocity(&self) -> &Vector2<T> {
        &self.linear_velocity
    }

    pub fn set_angular_velocity(&mut self, w: T) {
        if self.type_ == BodyType::Static {
            return;
        }
        if w * w > T::zero() {
            self.set_awake(true);
        }
        self.angular_velocity = w;
    }

    pub fn angular_velocity(&self) -> T {
        self.angular_velocity
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
        self.linear_velocity + self.angular_velocity.cross(world_point - self.sweep.c)
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
            self.linear_velocity = Vector2::zero();
            self.angular_velocity = T::zero();
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

        self.linear_velocity += self.angular_velocity.cross(self.sweep.c - old_center);
    }

    pub fn create_fixture<S: Shape<T> + 'static>(
        &mut self,
        shape: S,
        density: T,
    ) -> FixtureBuilder<T, D> {
        FixtureBuilder::new(self, shape, density)
    }

    pub(crate) fn add_fixture(&mut self, mut fixture: Box<Fixture<T, D>>) -> &mut Fixture<T, D> {
        self.fixture_list.push(fixture);
        self.fixture_list.last_mut().unwrap()
    }
}
