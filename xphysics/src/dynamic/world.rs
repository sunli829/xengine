use crate::collision::dynamic_tree;
use crate::dynamic::body::{BodyDef, BodyFlags};
use crate::dynamic::contact_manager::ContactManager;
use crate::dynamic::contacts::{ContactFilter, ContactFlags, ContactListener};
use crate::dynamic::fixture::FixtureProxy;
use crate::dynamic::island::Island;
use crate::dynamic::time_step::{Profile, TimeStep};
use crate::timer::Timer;
use crate::{Body, BodyType, Fixture, RayCastInput};
use std::alloc::Layout;
use xmath::{Real, Vector2, AABB};

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

pub(crate) struct WorldInner<T, D> {
    pub(crate) flags: WorldFlags,
    pub(crate) contact_manager: ContactManager<T, D>,
    pub(crate) body_list: *mut Body<T, D>,
    pub(crate) body_count: usize,
    pub(crate) gravity: Vector2<T>,
    pub(crate) allow_sleep: bool,
    pub(crate) destruction_listener: Option<Box<dyn DestructionListener<T, D>>>,
    pub(crate) inv_dt0: T,
    pub(crate) warm_starting: bool,
    pub(crate) continuous_physics: bool,
    pub(crate) sub_stepping: bool,
    pub(crate) step_complete: bool,
    pub(crate) profile: Profile,
}

impl<T, D> Drop for WorldInner<T, D> {
    fn drop(&mut self) {
        unsafe {
            let body_layout = Layout::new::<Body<T, D>>();
            let mut b = self.body_list;
            while !b.is_null() {
                let next = (*b).next_ptr;
                std::ptr::drop_in_place::<Body<T, D>>(b);
                std::alloc::dealloc(b as *mut u8, body_layout);
                b = next;
            }
        }
    }
}

pub struct World<T, D>(Box<WorldInner<T, D>>);

impl<T: Real, D> World<T, D> {
    pub fn new(gravity: Vector2<T>) -> World<T, D> {
        World(Box::new(WorldInner {
            flags: WorldFlags::CLEAR_FORCES,
            contact_manager: ContactManager::new(),
            body_list: std::ptr::null_mut(),
            body_count: 0,
            gravity,
            allow_sleep: true,
            destruction_listener: None,
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

    pub fn create_body(&mut self, def: BodyDef<T, D>) -> &mut Body<T, D> {
        unsafe {
            let data = std::alloc::alloc(Layout::new::<Body<T, D>>());
            let body = Body::new(self.0.as_mut(), def);
            *(data as *mut Body<T, D>) = body;

            let body = (data as *mut Body<T, D>).as_mut().unwrap();
            body.prev_ptr = std::ptr::null_mut();
            body.next_ptr = self.0.body_list;
            if !self.0.body_list.is_null() {
                (*self.0.body_list).prev_ptr = body;
            }
            self.0.body_list = body;
            self.0.body_count += 1;
            body
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
        unimplemented!()
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

    pub fn query_aabb(&self, aabb: AABB<T>) -> impl Iterator<Item = &mut Fixture<T, D>> {
        self.0
            .contact_manager
            .broad_phase
            .tree
            .query(aabb)
            .map(|item| unsafe { ((*(*item.2)).fixture_ptr).as_mut().unwrap() })
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
    type Item = &'a mut Fixture<T, D>;

    fn next(&mut self) -> Option<Self::Item> {
        Iterator::next(&mut self.iter)
            .map(|item| unsafe { ((*(*item.2)).fixture_ptr).as_mut().unwrap() })
    }
}
