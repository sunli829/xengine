use crate::test::{Test, TestSetting};
use crate::tests::CharacterCollision;

mod camera;
mod debug_draw;
mod test;
mod tests;

fn main() {
    let el = glutin::event_loop::EventLoop::new();
    let wb = glutin::window::WindowBuilder::new()
        .with_inner_size(glutin::dpi::LogicalSize::new(1024.0, 768.0));
    let windowed_context = glutin::ContextBuilder::new()
        .build_windowed(wb, &el)
        .unwrap();
    let windowed_context = unsafe { windowed_context.make_current().unwrap() };
    gl::load_with(|p| windowed_context.get_proc_address(p) as *const _);

    let mut test = Test::<f32, CharacterCollision>::new();
    let mut settings = TestSetting::default();

    el.run(move |event, _, control_flow| {
        let size = windowed_context.window().inner_size();
        let device_pixel_ratio = windowed_context.window().hidpi_factor() as f32;

        unsafe {
            gl::Viewport(
                0,
                0,
                (size.width as f32 * device_pixel_ratio) as i32,
                (size.height as f32 * device_pixel_ratio) as i32,
            );
            gl::ClearColor(0.0, 0.0, 0.0, 1.0);
            gl::Clear(gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT | gl::STENCIL_BUFFER_BIT);
        }

        test.step(
            &mut settings,
            size.width as f32,
            size.height as f32,
            device_pixel_ratio,
        );
        windowed_context.swap_buffers().unwrap();

        match event {
            glutin::event::Event::WindowEvent { event, .. } => match event {
                glutin::event::WindowEvent::CloseRequested => {
                    *control_flow = glutin::event_loop::ControlFlow::Exit;
                }
                glutin::event::WindowEvent::Resized(sz) => {
                    windowed_context.resize(glutin::dpi::PhysicalSize {
                        width: sz.width,
                        height: sz.height,
                    });
                }
                _ => {}
            },
            _ => {}
        }
    });
}
