use crate::dynamic::body::BodyFlags;
use crate::dynamic::contacts::{
    Contact, ContactImpulse, ContactListener, ContactSolver, ContactSolverDef,
    ContactVelocityConstraint,
};
use crate::dynamic::time_step::{Position, Profile, TimeStep, Velocity};
use crate::timer::Timer;
use crate::{settings, Body, BodyType};
use xmath::{DotTrait, Real, Vector2};

pub struct Island<'a, T, D> {
    contact_listener: &'a Option<Box<dyn ContactListener<T, D>>>,
    pub(crate) bodies: Vec<*mut Body<T, D>>,
    contacts: Vec<*mut Contact<T, D>>,
    positions: Vec<Position<T>>,
    velocities: Vec<Velocity<T>>,
}

impl<'a, T: Real, D> Island<'a, T, D> {
    pub fn new(
        body_capacity: usize,
        contact_capacity: usize,
        listener: &Option<Box<dyn ContactListener<T, D>>>,
    ) -> Island<T, D> {
        let mut bodies = Vec::with_capacity(body_capacity);
        bodies.resize(body_capacity, std::ptr::null_mut());

        let mut contacts = Vec::with_capacity(contact_capacity);
        contacts.resize(contact_capacity, std::ptr::null_mut());

        let mut positions = Vec::with_capacity(body_capacity);
        positions.resize(body_capacity, unsafe { std::mem::zeroed() });

        let mut velocities = Vec::with_capacity(body_capacity);
        velocities.resize(body_capacity, unsafe { std::mem::zeroed() });

        Island {
            contact_listener: listener,
            bodies,
            contacts,
            positions,
            velocities,
        }
    }

    pub fn solve(
        &mut self,
        profile: &mut Profile,
        step: &TimeStep<T>,
        gravity: Vector2<T>,
        allow_sleep: bool,
    ) {
        unsafe {
            let mut timer = Timer::new();
            let h = step.dt;

            for i in 0..self.bodies.len() {
                let b = self.bodies[i];

                let c = (*b).sweep.c;
                let a = (*b).sweep.a;
                let mut v = *(*b).linear_velocity();
                let mut w = (*b).angular_velocity();

                if (*b).body_type() == BodyType::Dynamic {
                    v += ((*b).force * (*b).inv_mass + gravity * (*b).gravity_scale()) * h;
                    w += (*b).inv_i * (*b).torque * h;

                    v *= T::one() / (T::one() + h * (*b).linear_damping());
                    w *= T::one() / (T::one() + h * (*b).angular_damping());
                }

                self.positions[i].c = c;
                self.positions[i].a = a;
                self.velocities[i].v = v;
                self.velocities[i].w = w;
            }

            timer.reset();

            // TODO: joints
            //            let solver_data = SolverData {
            //                step: *step,
            //                positions: &self.positions,
            //                velocities: &self.velocities,
            //            };

            let self_positions = self.positions.as_mut_ptr();
            let self_velocities = self.velocities.as_mut_ptr();
            let self_contacts = self.contacts.as_ptr();
            let mut contact_solver = ContactSolver::new(ContactSolverDef {
                step: *step,
                contacts: std::slice::from_raw_parts(self_contacts, self.contacts.len()),
                positions: std::slice::from_raw_parts_mut(self_positions, self.positions.len()),
                velocities: std::slice::from_raw_parts_mut(self_velocities, self.velocities.len()),
            });
            contact_solver.initialize_velocity_constraints();
            if step.warm_starting {
                contact_solver.warm_start();
            }

            profile.solve_init = timer.get_duration();

            timer.reset();
            for _ in 0..step.velocity_iterations {
                contact_solver.solve_velocity_constraints();
            }

            contact_solver.store_impulses();
            profile.solve_velocity = timer.get_duration();

            for i in 0..self.bodies.len() {
                let mut c = (*self_positions.add(i)).c;
                let mut a = (*self_positions.add(i)).a;
                let mut v = (*self_velocities.add(i)).v;
                let mut w = (*self_velocities.add(i)).w;

                let translation = v * h;
                if translation.dot(translation) > settings::max_translation_squared() {
                    let ratio = settings::max_translation::<T>() / translation.length();
                    v *= ratio;
                }

                let rotation = w * h;
                if rotation * rotation > settings::max_rotation_squared::<T>() {
                    let ratio = settings::max_rotation::<T>() / rotation.abs();
                    w *= ratio;
                }

                c += v * h;
                a += w * h;

                (*self_positions.add(i)).c = c;
                (*self_positions.add(i)).a = a;
                (*self_velocities.add(i)).v = v;
                (*self_velocities.add(i)).w = w;
            }

            timer.reset();
            let mut position_solved = false;
            for _ in 0..step.position_iterations {
                let contacts_okay = contact_solver.solve_position_constraints();
                if contacts_okay {
                    position_solved = true;
                    break;
                }
            }

            for i in 0..self.bodies.len() {
                let body = self.bodies[i];
                (*body).sweep.c = (*self_positions.add(i)).c;
                (*body).sweep.a = (*self_positions.add(i)).a;
                (*body).linear_velocity_ = (*self_velocities.add(i)).v;
                (*body).angular_velocity_ = (*self_velocities.add(i)).w;
                (*body).synchronize_transform();
            }

            profile.solve_position = timer.get_duration();
            self.report(contact_solver.velocity_constraints.as_ptr());

            if allow_sleep {
                let mut min_sleep_time = T::max_value();
                let lin_tol_sqr = settings::linear_sleep_tolerance::<T>()
                    * settings::linear_sleep_tolerance::<T>();
                let ang_tol_sqr = settings::angular_sleep_tolerance::<T>()
                    * settings::angular_sleep_tolerance::<T>();

                for i in 0..self.bodies.len() {
                    let b = self.bodies[i];
                    if (*b).body_type() == BodyType::Static {
                        continue;
                    }

                    if !(*b).flags.contains(BodyFlags::AUTO_SLEEP)
                        || (*b).angular_velocity_ * (*b).angular_velocity_ > ang_tol_sqr
                        || (*b).linear_velocity_.dot((*b).linear_velocity_) > lin_tol_sqr
                    {
                        (*b).sleep_time = T::zero();
                        min_sleep_time = T::zero();
                    } else {
                        (*b).sleep_time += h;
                        min_sleep_time = min_sleep_time.min((*b).sleep_time);
                    }
                }

                if min_sleep_time >= settings::time_to_sleep() && position_solved {
                    for i in 0..self.bodies.len() {
                        let b = self.bodies[i];
                        (*b).set_awake(false);
                    }
                }
            }
        }
    }

    unsafe fn report(&mut self, constraints: *const ContactVelocityConstraint<T>) {
        if let Some(listener) = &self.contact_listener {
            for i in 0..self.contacts.len() {
                let c = self.contacts[i];
                let vc = constraints.add(i);
                let mut impulse = ContactImpulse {
                    normal_impulses: Default::default(),
                    tangent_impulses: Default::default(),
                    count: (*vc).point_count,
                };
                for j in 0..(*vc).point_count {
                    impulse.normal_impulses[j] = (*vc).points[j].normal_impulse;
                    impulse.tangent_impulses[j] = (*vc).points[j].tangent_impulse;
                }
                listener.post_solve(c.as_mut().unwrap(), &impulse);
            }
        }
    }

    pub fn solve_toi(&mut self, sub_step: &TimeStep<T>, toi_index_a: usize, toi_index_b: usize) {
        assert!(toi_index_a < self.bodies.len());
        assert!(toi_index_b < self.bodies.len());

        unsafe {
            for i in 0..self.bodies.len() {
                let b = self.bodies[i];
                self.positions[i].c = (*b).sweep.c;
                self.positions[i].a = (*b).sweep.a;
                self.velocities[i].v = (*b).linear_velocity_;
                self.velocities[i].w = (*b).angular_velocity_;
            }

            let mut contact_solver = ContactSolver::new(ContactSolverDef {
                step: *sub_step,
                contacts: std::slice::from_raw_parts(self.contacts.as_ptr(), self.contacts.len()),
                positions: std::slice::from_raw_parts_mut(
                    self.positions.as_mut_ptr(),
                    self.positions.len(),
                ),
                velocities: std::slice::from_raw_parts_mut(
                    self.velocities.as_mut_ptr(),
                    self.velocities.len(),
                ),
            });

            for _ in 0..sub_step.position_iterations {
                let contacts_okay =
                    contact_solver.solve_toi_position_constraints(toi_index_a, toi_index_b);
                if contacts_okay {
                    break;
                }
            }

            (*self.bodies[toi_index_a]).sweep.c0 = self.positions[toi_index_a].c;
            (*self.bodies[toi_index_a]).sweep.a0 = self.positions[toi_index_a].a;
            (*self.bodies[toi_index_b]).sweep.c0 = self.positions[toi_index_b].c;
            (*self.bodies[toi_index_b]).sweep.a0 = self.positions[toi_index_b].a;

            contact_solver.initialize_velocity_constraints();

            for _ in 0..sub_step.velocity_iterations {
                contact_solver.solve_velocity_constraints();
            }

            let h = sub_step.dt;

            for i in 0..self.bodies.len() {
                let mut c = self.positions[i].c;
                let mut a = self.positions[i].a;
                let mut v = self.velocities[i].v;
                let mut w = self.velocities[i].w;

                let translation = v * h;
                if translation.dot(translation) > settings::max_translation_squared() {
                    let ratio = settings::max_translation::<T>() / translation.length();
                    v *= ratio;
                }

                let rotation = w * h;
                if rotation * rotation > settings::max_rotation_squared::<T>() {
                    let ratio = settings::max_rotation::<T>() / rotation.abs();
                    w *= ratio;
                }

                c += v * h;
                a += w * h;

                self.positions[i].c = c;
                self.positions[i].a = a;
                self.velocities[i].v = v;
                self.velocities[i].w = w;

                let body = self.bodies[i];
                (*body).sweep.c = c;
                (*body).sweep.a = a;
                (*body).linear_velocity_ = v;
                (*body).angular_velocity_ = w;
                (*body).synchronize_transform();
            }

            self.report(contact_solver.velocity_constraints.as_ptr());
        }
    }

    pub fn add_body(&mut self, body: *mut Body<T, D>) {
        self.bodies.push(body);
    }

    pub fn add_contact(&mut self, contact: *mut Contact<T, D>) {
        self.contacts.push(contact);
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.contacts.clear();
    }
}
