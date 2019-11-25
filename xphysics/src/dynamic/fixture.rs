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

pub struct FixtureBuilder<'a, T, D> {
    body: &'a mut Body<T, D>,
    pub shape: Box<dyn Shape<T> + 'static>,
    pub data: Option<D>,
    pub friction: T,
    pub restitution: T,
    pub density: T,
    pub is_sensor: bool,
    pub filter: Filter,
}

impl<'a, T: Real, D> FixtureBuilder<'a, T, D> {
    pub(crate) fn new<S: Shape<T> + 'static>(
        body: &'a mut Body<T, D>,
        shape: S,
        density: T,
    ) -> FixtureBuilder<'a, T, D> {
        FixtureBuilder {
            body,
            shape: Box::new(shape),
            data: None,
            friction: T::two() * T::en1(),
            restitution: T::zero(),
            density,
            is_sensor: false,
            filter: Default::default(),
        }
    }

    pub fn data(self, data: D) -> FixtureBuilder<'a, T, D> {
        FixtureBuilder {
            data: Some(data),
            ..self
        }
    }

    pub fn friction(self, friction: T) -> FixtureBuilder<'a, T, D> {
        FixtureBuilder { friction, ..self }
    }

    pub fn restitution(self, restitution: T) -> FixtureBuilder<'a, T, D> {
        FixtureBuilder {
            restitution,
            ..self
        }
    }

    pub fn sensor(self, is_sensor: bool) -> FixtureBuilder<'a, T, D> {
        FixtureBuilder { is_sensor, ..self }
    }

    pub fn filter(self, filter: Filter) -> FixtureBuilder<'a, T, D> {
        FixtureBuilder { filter, ..self }
    }

    pub fn finish(self) -> &'a Fixture<T, D> {
        let child_count = self.shape.child_count();
        let fixture = Box::new(Fixture {
            density: self.density,
            body: self.body as *mut Body<T, D>,
            shape: self.shape,
            friction: self.friction,
            restitution: self.restitution,
            proxies: Vec::with_capacity(child_count),
            filter: self.filter,
            is_sensor: self.is_sensor,
            data: self.data,
        });
        self.body.add_fixture(fixture)
    }
}

pub(crate) struct FixtureProxy<T, D> {
    aabb: AABB<T>,
    fixture: *mut Fixture<T, D>,
    child_index: usize,
    proxy_id: usize,
}

pub struct Fixture<T, D> {
    density: T,
    body: *mut Body<T, D>,
    shape: Box<dyn Shape<T> + 'static>,
    friction: T,
    restitution: T,
    proxies: Vec<Box<FixtureProxy<T, D>>>,
    filter: Filter,
    is_sensor: bool,
    data: Option<D>,
}

impl<T: Real, D> Fixture<T, D> {
    pub fn body_mut(&mut self) -> &mut Body<T, D> {
        unsafe { self.body.as_mut().unwrap() }
    }

    pub fn body(&self) -> &Body<T, D> {
        unsafe { self.body.as_ref().unwrap() }
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

    pub fn data(&self) -> Option<&D> {
        self.data.as_ref()
    }

    pub fn data_mut(&mut self) -> Option<&mut D> {
        self.data.as_mut()
    }

    pub fn set_data(&mut self, data: D) {
        self.data = Some(data);
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
        xf: &Transform<T>,
    ) {
        self.proxies.clear();
        for i in 0..self.shape.child_count() {
            let aabb = self.shape.compute_aabb(xf, i);
            let fixture_ptr = self as *mut Fixture<T, D>;
            let mut proxy = Box::new(FixtureProxy {
                aabb,
                fixture: fixture_ptr,
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
