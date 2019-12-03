use crate::camera::Camera;
use crate::debug_draw::NvgDebugDraw;
use std::cell::RefCell;
use std::rc::Rc;
use xmath::{Real, Vector2};
use xphysics::{DebugDrawFlags, World};

pub struct TestSetting<T> {
    hz: T,
    velocity_iterations: usize,
    position_iterations: usize,
    draw_shapes: bool,
    draw_aabbs: bool,
    draw_contact_points: bool,
    draw_contact_normals: bool,
    draw_contact_impulse: bool,
    draw_friction_impulse: bool,
    draw_center_of_mass: bool,
    draw_stats: bool,
    enable_warm_starting: bool,
    enable_continuous: bool,
    enable_sub_stepping: bool,
    enable_sleep: bool,
    pause: bool,
    single_step: bool,
}

impl<T: Real> Default for TestSetting<T> {
    fn default() -> Self {
        Self {
            hz: T::f32(360.0),
            velocity_iterations: 8,
            position_iterations: 3,
            draw_shapes: true,
            draw_aabbs: false,
            draw_contact_points: true,
            draw_contact_normals: true,
            draw_contact_impulse: false,
            draw_friction_impulse: false,
            draw_center_of_mass: false,
            draw_stats: false,
            enable_warm_starting: true,
            enable_continuous: true,
            enable_sub_stepping: false,
            enable_sleep: true,
            pause: false,
            single_step: false,
        }
    }
}

pub trait TestImpl<T: Real> {
    fn new(world: &mut World<T, ()>) -> Self;

    fn step(
        &self,
        world: &mut World<T, ()>,
        time_step: T,
        velocity_iterations: usize,
        position_iterations: usize,
    ) {
        world.step(time_step, velocity_iterations, position_iterations);
    }
}

pub struct Test<T, I> {
    ctx: Rc<RefCell<nvg::Context<nvg_gl::Renderer>>>,
    camera: Rc<RefCell<Camera>>,
    test_impl: I,
    pub world: World<T, ()>,
}

impl<T: Real, I: TestImpl<T>> Test<T, I> {
    pub fn new() -> Test<T, I> {
        let mut world = World::new(Vector2::new(T::f32(0.0), T::f32(-10.0)));
        let test_impl = I::new(&mut world);
        let ctx = Rc::new(RefCell::new(
            nvg::Context::create(nvg_gl::Renderer::create().unwrap()).unwrap(),
        ));
        let camera = Rc::new(RefCell::new(Camera::default()));
        world.set_debug_draw(NvgDebugDraw {
            ctx: ctx.clone(),
            camera: camera.clone(),
        });
        Test {
            ctx,
            camera,
            test_impl,
            world,
        }
    }

    pub fn step(
        &mut self,
        settings: &mut TestSetting<T>,
        window_width: f32,
        window_height: f32,
        device_pixel_ratio: f32,
    ) {
        self.ctx
            .borrow_mut()
            .begin_frame((window_width, window_height), device_pixel_ratio)
            .unwrap();
        self.camera.borrow_mut().window_size = (window_width, window_height).into();

        let mut time_step = if settings.hz > T::zero() {
            T::one() / settings.hz
        } else {
            T::zero()
        };

        if settings.pause {
            if settings.single_step {
                settings.single_step = false;
            } else {
                time_step = T::zero();
            }
        }

        self.world.set_debug_draw_flags({
            let mut flags = DebugDrawFlags::empty();
            flags.set(DebugDrawFlags::SHAPE, settings.draw_shapes);
            flags.set(DebugDrawFlags::AABB, settings.draw_aabbs);
            flags.set(DebugDrawFlags::CENTER_OF_MASS, settings.draw_center_of_mass);
            flags
        });

        self.world.set_allow_sleeping(settings.enable_sleep);
        self.world.set_warm_starting(settings.enable_warm_starting);
        self.world
            .set_continuous_physics(settings.enable_continuous);
        self.world.set_sub_stepping(settings.enable_sub_stepping);

        self.test_impl.step(
            &mut self.world,
            time_step,
            settings.velocity_iterations,
            settings.position_iterations,
        );
        self.world.draw_debug_data();

        self.ctx.borrow_mut().end_frame().unwrap();
    }
}
