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

    el.run(move |event, _, control_flow| match event {
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
    });
}
