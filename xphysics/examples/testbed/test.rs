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
            hz: T::from_f32(60.0),
            velocity_iterations: 8,
            position_iterations: 3,
            draw_shapes: true,
            draw_aabbs: false,
            draw_contact_points: false,
            draw_contact_normals: false,
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

pub trait TestImpl<T> {
    fn create(&mut self, world: &mut World<T, ()>);
}

pub struct Test<T> {
    ctx: Rc<RefCell<nvg::Context<nvg_gl::Renderer>>>,
    test_impl: Box<dyn TestImpl<T> + 'static>,
    world: World<T, ()>,
}

impl<T: Real> Test<T> {
    pub fn new(mut test_impl: Box<dyn TestImpl<T> + 'static>) -> Test<T> {
        let mut world = World::new(Vector2::new(T::from_f32(0.0), T::from_f32(-10.0)));
        let ctx = Rc::new(RefCell::new(
            nvg::Context::create(nvg_gl::Renderer::create().unwrap()).unwrap(),
        ));
        world.set_debug_draw(NvgDebugDraw { ctx: ctx.clone() });
        test_impl.create(&mut world);
        Test {
            ctx,
            test_impl,
            world,
        }
    }

    pub fn step(&mut self, settings: &mut TestSetting<T>) {
        self.ctx.borrow_mut().begin_frame((1280, 800), 1.0).unwrap();

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

        self.world.step(
            time_step,
            settings.velocity_iterations,
            settings.position_iterations,
        );
        self.world.draw_debug_data();

        self.ctx.borrow_mut().end_frame().unwrap();
    }
}
