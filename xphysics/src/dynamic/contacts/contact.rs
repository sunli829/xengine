use crate::{
    collision, settings, test_overlap, Body, Fixture, Manifold, Shape, ShapeChain, ShapeCircle,
    ShapeEdge, ShapePolygon, ShapeType, WorldManifold,
};
use std::alloc::Layout;
use xmath::{Real, Transform};

#[inline]
fn mix_friction<T: Real>(friction1: T, friction2: T) -> T {
    (friction1 * friction2).sqrt()
}

#[inline]
fn mix_restitution<T: Real>(restitution1: T, restitution2: T) -> T {
    restitution1.max(restitution2)
}

pub struct ContactEdge<T, D> {
    pub other_ptr: *mut Body<T, D>,
    pub contact_ptr: *mut Contact<T, D>,
    pub prev_ptr: *mut ContactEdge<T, D>,
    pub next_ptr: *mut ContactEdge<T, D>,
}

bitflags! {
    pub struct ContactFlags: u32 {
        const ISLAND = 0x0001;
        const TOUCHING = 0x0002;
        const ENABLED = 0x0004;
        const FILTER = 0x0008;
        const BULLET_HIT = 0x0010;
        const TOI = 0x0020;
    }
}

pub struct ContactImpulse<T> {
    pub normal_impulses: [T; settings::MAX_MANIFOLD_POINTS],
    pub tangent_impulses: [T; settings::MAX_MANIFOLD_POINTS],
    pub count: usize,
}

pub trait ContactListener<T, D> {
    fn begin_contact(&self, _contact: &mut Contact<T, D>) {}
    fn end_contact(&self, _contact: &mut Contact<T, D>) {}
    fn pre_solve(&self, _contact: &mut Contact<T, D>, _old_manifold: &Manifold<T>) {}
    fn post_solve(&self, _contact: &mut Contact<T, D>, _impulse: &ContactImpulse<T>) {}
}

pub trait ContactFilter<T, D> {
    fn should_collide(&self, fixture_a: &Fixture<T, D>, fixture_b: &Fixture<T, D>) -> bool;
}

pub struct DefaultContactFilter;

impl<T: Real, D> ContactFilter<T, D> for DefaultContactFilter {
    fn should_collide(&self, fixture_a: &Fixture<T, D>, fixture_b: &Fixture<T, D>) -> bool {
        let filter_a = fixture_a.filter();
        let filter_b = fixture_b.filter();

        if filter_a.group_index == filter_b.group_index && filter_a.group_index != 0 {
            return filter_a.group_index > 0;
        }

        (filter_a.mask_bits & filter_b.category_bits) != 0
            && (filter_a.category_bits & filter_b.mask_bits) != 0
    }
}

pub struct Contact<T, D> {
    pub(crate) flags: ContactFlags,
    pub(crate) prev_ptr: *mut Contact<T, D>,
    pub(crate) next_ptr: *mut Contact<T, D>,
    pub(crate) node_a: ContactEdge<T, D>,
    pub(crate) node_b: ContactEdge<T, D>,
    pub(crate) fixture_a_ptr: *mut Fixture<T, D>,
    pub(crate) fixture_b_ptr: *mut Fixture<T, D>,
    pub(crate) index_a: usize,
    pub(crate) index_b: usize,
    pub(crate) manifold: Manifold<T>,
    pub(crate) toi_count: usize,
    pub(crate) toi: T,
    pub(crate) friction: T,
    pub(crate) restitution: T,
    pub(crate) tangent_speed: T,
    pub(crate) evaluate_fn: fn(
        manifold: &mut Manifold<T>,
        shape_a: &dyn Shape<T>,
        shape_b: &dyn Shape<T>,
        xf_a: &Transform<T>,
        xf_b: &Transform<T>,
        index_a: usize,
        index_b: usize,
    ),
}

impl<T: Real, D> Contact<T, D> {
    pub(crate) fn new(
        fixture_a: *mut Fixture<T, D>,
        index_a: usize,
        fixture_b: *mut Fixture<T, D>,
        index_b: usize,
    ) -> *mut Contact<T, D> {
        let contact = Contact {
            flags: ContactFlags::ENABLED,
            prev_ptr: std::ptr::null_mut(),
            next_ptr: std::ptr::null_mut(),
            node_a: ContactEdge {
                other_ptr: std::ptr::null_mut(),
                contact_ptr: std::ptr::null_mut(),
                prev_ptr: std::ptr::null_mut(),
                next_ptr: std::ptr::null_mut(),
            },
            node_b: ContactEdge {
                other_ptr: std::ptr::null_mut(),
                contact_ptr: std::ptr::null_mut(),
                prev_ptr: std::ptr::null_mut(),
                next_ptr: std::ptr::null_mut(),
            },
            fixture_a_ptr: fixture_a,
            fixture_b_ptr: fixture_b,
            index_a,
            index_b,
            manifold: unsafe { std::mem::zeroed() },
            toi_count: 0,
            toi: T::zero(),
            friction: unsafe { mix_friction((*fixture_a).friction(), (*fixture_b).friction()) },
            restitution: unsafe {
                mix_restitution((*fixture_a).friction(), (*fixture_b).friction())
            },
            tangent_speed: T::zero(),
            evaluate_fn: unsafe {
                match (
                    (*fixture_a).shape().shape_type(),
                    (*fixture_b).shape().shape_type(),
                ) {
                    (ShapeType::Circle, ShapeType::Circle) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, _index_a, _index_b| {
                            collision::collide_circles(
                                manifold,
                                (shape_a as *const dyn Shape<T> as *const ShapeCircle<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapeCircle<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    (ShapeType::Polygon, ShapeType::Circle) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, _index_a, _index_b| {
                            collision::collide_polygon_and_circle(
                                manifold,
                                (shape_a as *const dyn Shape<T> as *const ShapePolygon<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapeCircle<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    (ShapeType::Polygon, ShapeType::Polygon) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, _index_a, _index_b| {
                            collision::collide_polygons(
                                manifold,
                                (shape_a as *const dyn Shape<T> as *const ShapePolygon<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapePolygon<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    (ShapeType::Edge, ShapeType::Circle) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, _index_a, _index_b| {
                            collision::collide_edge_and_circle(
                                manifold,
                                (shape_a as *const dyn Shape<T> as *const ShapeEdge<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapeCircle<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    (ShapeType::Edge, ShapeType::Polygon) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, _index_a, _index_b| {
                            collision::collide_edge_and_polygon(
                                manifold,
                                (shape_a as *const dyn Shape<T> as *const ShapeEdge<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapePolygon<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    (ShapeType::Chain, ShapeType::Circle) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, index_a, _index_b| {
                            collision::collide_edge_and_circle(
                                manifold,
                                &(shape_a as *const dyn Shape<T> as *const ShapeChain<T>)
                                    .as_ref()
                                    .unwrap()
                                    .get_child_edge(index_a),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapeCircle<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    (ShapeType::Chain, ShapeType::Polygon) => {
                        |manifold, shape_a, shape_b, xf_a, xf_b, index_a, _index_b| {
                            collision::collide_edge_and_polygon(
                                manifold,
                                &(shape_a as *const dyn Shape<T> as *const ShapeChain<T>)
                                    .as_ref()
                                    .unwrap()
                                    .get_child_edge(index_a),
                                xf_a,
                                (shape_b as *const dyn Shape<T> as *const ShapePolygon<T>)
                                    .as_ref()
                                    .unwrap(),
                                xf_b,
                            )
                        }
                    }
                    _ => return std::ptr::null_mut(),
                }
            },
        };

        unsafe {
            let data = std::alloc::alloc(Layout::new::<Contact<T, D>>());
            std::ptr::copy_nonoverlapping(
                &contact as *const Contact<T, D>,
                data as *mut Contact<T, D>,
                std::mem::size_of::<Contact<T, D>>(),
            );
            data as *mut Contact<T, D>
        }
    }

    pub(crate) fn destroy(contact: *mut Contact<T, D>) {
        unsafe {
            std::ptr::drop_in_place::<Contact<T, D>>(contact);
            std::alloc::dealloc(contact as *mut u8, Layout::new::<Contact<T, D>>());
        }
    }

    pub fn manifold(&self) -> &Manifold<T> {
        &self.manifold
    }

    pub(crate) fn manifold_mut(&mut self) -> &mut Manifold<T> {
        &mut self.manifold
    }

    pub fn world_manifold(&self) -> WorldManifold<T> {
        unsafe {
            let body_a = (*self.fixture_a_ptr).body_ptr;
            let body_b = (*self.fixture_b_ptr).body_ptr;
            let shape_a = (*self.fixture_a_ptr).shape();
            let shape_b = (*self.fixture_b_ptr).shape();
            WorldManifold::new(
                &self.manifold,
                (*body_a).transform(),
                shape_a.radius(),
                (*body_b).transform(),
                shape_b.radius(),
            )
        }
    }

    pub fn is_touching(&self) -> bool {
        self.flags.contains(ContactFlags::TOUCHING)
    }

    pub fn set_enable(&mut self, flag: bool) {
        self.flags.set(ContactFlags::ENABLED, flag);
    }

    pub fn is_enable(&self) -> bool {
        self.flags.contains(ContactFlags::ENABLED)
    }

    pub fn fixture_a(&self) -> &Fixture<T, D> {
        unsafe { self.fixture_a_ptr.as_ref().unwrap() }
    }

    pub fn fixture_a_mut(&self) -> &mut Fixture<T, D> {
        unsafe { self.fixture_a_ptr.as_mut().unwrap() }
    }

    pub fn child_index_a(&self) -> usize {
        self.index_a
    }

    pub fn fixture_b(&self) -> &Fixture<T, D> {
        unsafe { self.fixture_b_ptr.as_ref().unwrap() }
    }

    pub fn fixture_b_mut(&self) -> &mut Fixture<T, D> {
        unsafe { self.fixture_b_ptr.as_mut().unwrap() }
    }

    pub fn child_index_b(&self) -> usize {
        self.index_b
    }

    pub fn set_friction(&mut self, friction: T) {
        self.friction = friction;
    }

    pub fn friction(&self) -> T {
        self.friction
    }

    pub fn reset_friction(&mut self) {
        self.friction = unsafe {
            mix_friction(
                (*self.fixture_a_ptr).friction(),
                (*self.fixture_b_ptr).friction(),
            )
        };
    }

    pub fn set_restitution(&mut self, restitution: T) {
        self.restitution = restitution;
    }

    pub fn restitution(&self) -> T {
        self.restitution
    }

    pub(crate) fn flag_for_filtering(&mut self) {
        self.flags.insert(ContactFlags::FILTER)
    }

    pub fn reset_restitution(&mut self) {
        self.friction = unsafe {
            mix_restitution(
                (*self.fixture_a_ptr).restitution(),
                (*self.fixture_b_ptr).restitution(),
            )
        };
    }

    pub fn set_tangent_speed(&mut self, speed: T) {
        self.tangent_speed = speed;
    }

    pub fn tangent_speed(&self) -> T {
        self.tangent_speed
    }

    pub(crate) fn evaluate(&mut self, xf_a: &Transform<T>, xf_b: &Transform<T>) {
        unsafe {
            (self.evaluate_fn)(
                &mut self.manifold,
                (*self.fixture_a_ptr).shape(),
                (*self.fixture_b_ptr).shape(),
                xf_a,
                xf_b,
                self.index_a,
                self.index_b,
            )
        }
    }

    pub(crate) fn update(&mut self, mut listener: &Option<Box<dyn ContactListener<T, D>>>) {
        unsafe {
            let old_manifold = self.manifold.clone();

            self.flags.insert(ContactFlags::ENABLED);

            let touching;
            let was_touching = self.flags.contains(ContactFlags::TOUCHING);

            let sensor_a = (*self.fixture_a_ptr).is_sensor();
            let sensor_b = (*self.fixture_b_ptr).is_sensor();
            let sensor = sensor_a || sensor_b;

            let body_a = (*self.fixture_a_ptr).body_ptr;
            let body_b = (*self.fixture_b_ptr).body_ptr;
            let xf_a = (*body_a).transform();
            let xf_b = (*body_b).transform();

            if sensor {
                let shape_a = (*self.fixture_a_ptr).shape();
                let shape_b = (*self.fixture_b_ptr).shape();
                touching = test_overlap(shape_a, self.index_a, shape_b, self.index_b, *xf_a, *xf_b);
                self.manifold.point_count = 0;
            } else {
                self.evaluate(xf_a, xf_b);
                touching = self.manifold.point_count > 0;

                for i in 0..self.manifold.point_count {
                    let mp2 = &mut self.manifold.points[i];
                    mp2.normal_impulse = T::zero();
                    mp2.tangent_impulse = T::zero();
                    let id2 = mp2.id;

                    for j in 0..old_manifold.point_count {
                        let mp1 = &old_manifold.points[j];

                        if mp1.id == id2 {
                            mp2.normal_impulse = mp1.normal_impulse;
                            mp2.tangent_impulse = mp1.tangent_impulse;
                            break;
                        }
                    }
                }

                if touching != was_touching {
                    (*body_a).set_awake(true);
                    (*body_b).set_awake(true);
                }
            }

            if touching {
                self.flags.insert(ContactFlags::TOUCHING);
            } else {
                self.flags.remove(ContactFlags::TOUCHING);
            }

            if !was_touching && touching {
                if let Some(l) = &mut listener {
                    l.begin_contact(self);
                }
            }

            if was_touching && !touching {
                if let Some(l) = &mut listener {
                    l.end_contact(self);
                }
            }

            if !sensor && touching {
                if let Some(l) = &mut listener {
                    l.pre_solve(self, &old_manifold);
                }
            }
        }
    }
}
