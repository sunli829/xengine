use crate::shapes::Shape;
use crate::{Body, BroadPhase, MassData, RayCastInput, RayCastOutput};
use xmath::{Real, Transform, Vector2, AABB};

#[derive(Copy, Clone)]
pub struct Filter {
    pub category_bits: u16,
    pub mask_bits: u16,
    pub group_index: i16,
}

impl Default for Filter {
    fn default() -> Self {
        Filter {
            category_bits: 0x0001,
            mask_bits: 0xFFFF,
            group_index: 0,
        }
    }
}

pub struct FixtureDef<T, D, S> {
    pub shape: S,
    pub data: D,
    pub friction: T,
    pub restitution: T,
    pub density: T,
    pub is_sensor: bool,
    pub filter: Filter,
}

impl<T: Real, D: Default, S: Shape<T> + 'static> FixtureDef<T, D, S> {
    pub fn new(shape: S) -> Self {
        Self {
            shape,
            data: Default::default(),
            friction: T::two() * T::en1(),
            restitution: T::zero(),
            density: T::zero(),
            is_sensor: false,
            filter: Default::default(),
        }
    }
}

pub(crate) struct FixtureProxy<T, D> {
    pub(crate) aabb: AABB<T>,
    pub(crate) fixture_ptr: *mut Fixture<T, D>,
    pub(crate) child_index: usize,
    pub(crate) proxy_id: usize,
}

pub struct Fixture<T, D> {
    pub(crate) density: T,
    pub(crate) body_ptr: *mut Body<T, D>,
    pub(crate) shape: Box<dyn Shape<T> + 'static>,
    pub(crate) friction: T,
    pub(crate) restitution: T,
    pub(crate) proxies: Vec<Box<FixtureProxy<T, D>>>,
    pub(crate) filter: Filter,
    pub(crate) is_sensor: bool,
    pub(crate) data: D,
}

impl<T: Real, D> Fixture<T, D> {
    pub fn body_mut(&mut self) -> &mut Body<T, D> {
        unsafe { self.body_ptr.as_mut().unwrap() }
    }

    pub fn body(&self) -> &Body<T, D> {
        unsafe { self.body_ptr.as_ref().unwrap() }
    }

    pub fn shape(&self) -> &dyn Shape<T> {
        self.shape.as_ref()
    }

    pub fn set_sensor(&mut self, sensor: bool) {
        if self.is_sensor != sensor {
            self.body_mut().set_awake(true);
            self.is_sensor = sensor;
        }
    }

    pub fn is_sensor(&self) -> bool {
        self.is_sensor
    }

    pub fn set_filter(&mut self, filter: Filter) {
        self.filter = filter;
        self.re_filter();
    }

    fn re_filter(&mut self) {
        unimplemented!();
    }

    pub fn filter(&self) -> &Filter {
        &self.filter
    }

    pub fn data(&self) -> &D {
        &self.data
    }

    pub fn data_mut(&mut self) -> &mut D {
        &mut self.data
    }

    pub fn set_data(&mut self, data: D) {
        self.data = data;
    }

    pub fn density(&self) -> T {
        self.density
    }

    pub fn set_density(&mut self, density: T) {
        assert!(density.is_valid() && density >= T::zero());
        self.density = density;
    }

    pub fn friction(&self) -> T {
        self.friction
    }

    pub fn set_friction(&mut self, friction: T) {
        self.friction = friction;
    }

    pub fn restitution(&self) -> T {
        self.restitution
    }

    pub fn set_restitution(&mut self, restitution: T) {
        self.restitution = restitution;
    }

    pub fn test_point(&self, pt: &Vector2<T>) -> bool {
        self.shape.test_point(self.body().transform(), pt)
    }

    pub fn ray_cast(
        &self,
        input: &RayCastInput<T>,
        child_index: usize,
    ) -> Option<RayCastOutput<T>> {
        self.shape
            .ray_cast(input, self.body().transform(), child_index)
    }

    pub fn mass_data(&self) -> MassData<T> {
        self.shape.compute_mass(self.density)
    }

    pub fn aabb(&self, child_index: usize) -> &AABB<T> {
        &self.proxies[child_index].aabb
    }

    pub(crate) fn create_proxies(
        &mut self,
        broad_phase: &mut BroadPhase<T, *mut FixtureProxy<T, D>>,
        xf: Transform<T>,
    ) {
        self.proxies.clear();
        for i in 0..self.shape.child_count() {
            let aabb = self.shape.compute_aabb(&xf, i);
            let fixture_ptr = self as *mut Fixture<T, D>;
            let mut proxy = Box::new(FixtureProxy {
                aabb,
                fixture_ptr,
                child_index: i,
                proxy_id: 0,
            });
            proxy.proxy_id =
                broad_phase.create_proxy(aabb, proxy.as_mut() as *mut FixtureProxy<T, D>);
            self.proxies.push(proxy);
        }
    }

    pub(crate) fn destroy_proxies(
        &mut self,
        broad_phase: &mut BroadPhase<T, *mut FixtureProxy<T, D>>,
    ) {
        for proxy in self.proxies.drain(..) {
            broad_phase.destroy_proxy(proxy.proxy_id);
        }
    }

    pub(crate) fn synchronize(
        &mut self,
        broad_phase: &mut BroadPhase<T, *mut FixtureProxy<T, D>>,
        xf1: &Transform<T>,
        xf2: &Transform<T>,
    ) {
        if self.proxies.is_empty() {
            return;
        }

        for proxy in &mut self.proxies {
            let aabb1 = self.shape.compute_aabb(xf1, proxy.child_index);
            let aabb2 = self.shape.compute_aabb(xf2, proxy.child_index);
            proxy.aabb = aabb1.combine(&aabb2);
            let displacement = xf2.p - xf1.p;
            broad_phase.move_proxy(proxy.proxy_id, proxy.aabb, displacement);
        }
    }
}
