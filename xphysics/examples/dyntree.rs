use rand::Rng;
use std::collections::HashSet;
use xmath::{Vector2, AABB};
use xphysics::DynamicTree;

fn main() {
    let el = glutin::event_loop::EventLoop::new();
    let wb = glutin::window::WindowBuilder::new()
        .with_inner_size(glutin::dpi::LogicalSize::new(1024.0, 768.0));
    let windowed_context = glutin::ContextBuilder::new()
        .build_windowed(wb, &el)
        .unwrap();
    let windowed_context = unsafe { windowed_context.make_current().unwrap() };
    gl::load_with(|p| windowed_context.get_proc_address(p) as *const _);

    let renderer = nvg_gl::Renderer::create().unwrap();
    let mut context = nvg::Context::create(renderer).unwrap();
    let mut tree = DynamicTree::new();

    let mut rng = rand::thread_rng();
    let mut rects = Vec::new();
    let mut selected = HashSet::new();

    for _ in 0..1000 {
        let w = rng.gen_range(10.0, 20.0);
        let h = rng.gen_range(10.0, 20.0);
        let x = rng.gen_range(-500.0, 500.0);
        let y = rng.gen_range(-500.0, 500.0);
        let bounds = AABB {
            lower_bound: Vector2::new(x, y),
            upper_bound: Vector2::new(x + w, y + h),
        };
        let id = tree.create_proxy(bounds, ());
        rects.push((id, bounds));
    }

    let mut drag_start = None;
    let mut cursor_position = nvg::Point::new(0.0, 0.0);
    let mut select_rect = None;

    el.run(move |event, _, control_flow| {
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
                glutin::event::WindowEvent::CursorMoved { position, .. } => {
                    let window_size = windowed_context.window().inner_size();
                    cursor_position = to_view_pt(
                        (position.x, position.y).into(),
                        nvg::Extent::new(window_size.width as f32, window_size.height as f32),
                        nvg::Extent::new(1000.0, 1000.0),
                    );

                    if let Some(drag_start) = drag_start {
                        let sr = nvg::Bounds {
                            min: drag_start,
                            max: cursor_position,
                        };
                        select_rect = Some(sr);
                        windowed_context.window().request_redraw();
                    }
                }
                glutin::event::WindowEvent::MouseInput { state, button, .. }
                    if button == glutin::event::MouseButton::Left
                        && state == glutin::event::ElementState::Pressed =>
                {
                    // 鼠标左键按下
                    drag_start = Some(cursor_position);
                }
                glutin::event::WindowEvent::MouseInput { state, button, .. }
                    if button == glutin::event::MouseButton::Left
                        && state == glutin::event::ElementState::Released =>
                {
                    // 鼠标左键松开
                    if let Some(sr) = select_rect {
                        selected.clear();
                        let aabb = AABB {
                            lower_bound: Vector2::new(sr.min.x, sr.min.y),
                            upper_bound: Vector2::new(sr.max.x, sr.max.y),
                        };
                        selected.extend(tree.query(aabb).map(|item| item.0));
                        windowed_context.window().request_redraw();
                    }

                    drag_start = None;
                }
                glutin::event::WindowEvent::RedrawRequested => {
                    let size = windowed_context.window().inner_size();
                    let device_pixel_ratio = windowed_context.window().hidpi_factor() as f32;

                    unsafe {
                        gl::Viewport(
                            0,
                            0,
                            (size.width as f32 * device_pixel_ratio) as i32,
                            (size.height as f32 * device_pixel_ratio) as i32,
                        );
                        gl::ClearColor(0.3, 0.3, 0.3, 1.0);
                        gl::Clear(
                            gl::COLOR_BUFFER_BIT | gl::DEPTH_BUFFER_BIT | gl::STENCIL_BUFFER_BIT,
                        );
                    }

                    context
                        .begin_frame(
                            nvg::Extent {
                                width: 1000.0,
                                height: 1000.0,
                            },
                            device_pixel_ratio,
                        )
                        .unwrap();
                    draw(&mut context, &rects, &select_rect, &selected);
                    context.end_frame().unwrap();

                    windowed_context.swap_buffers().unwrap();
                }
                _ => {}
            },
            _ => {}
        }
    });
}

fn to_view_pt(pt: nvg::Point, window_size: nvg::Extent, view_size: nvg::Extent) -> nvg::Point {
    let window_transform = nvg::Transform::scale(
        view_size.width / window_size.width,
        view_size.height / window_size.height,
    );
    let view_transform = nvg::Transform::scale(0.9, 0.9) * nvg::Transform::translate(500.0, 500.0);
    let transform = window_transform * view_transform.inverse();
    transform.transform_point(pt)
}

fn draw<R: nvg::Renderer>(
    ctx: &mut nvg::Context<R>,
    rects: &[(usize, AABB<f32>)],
    select_rect: &Option<nvg::Bounds>,
    selects: &HashSet<usize>,
) {
    ctx.translate(500.0, 500.0);
    ctx.scale(0.9, 0.9);

    for (id, rt) in rects {
        if selects.contains(id) {
            ctx.fill_paint(nvg::Color::rgba(0.0, 0.0, 1.0, 0.5));
        } else {
            ctx.fill_paint(nvg::Color::rgba(1.0, 1.0, 1.0, 0.3));
        }

        ctx.begin_path();
        ctx.rect((
            rt.lower_bound.x,
            rt.lower_bound.y,
            rt.upper_bound.x - rt.lower_bound.x,
            rt.upper_bound.y - rt.lower_bound.y,
        ));
        ctx.fill().unwrap();
    }

    ctx.begin_path();
    ctx.stroke_paint(nvg::Color::rgba(1.0, 0.0, 0.0, 0.3));
    ctx.rect((-500, -500, 1000, 1000));
    ctx.stroke().unwrap();

    if let Some(select_rect) = select_rect {
        ctx.begin_path();
        ctx.stroke_paint(nvg::Color::rgb(0.0, 1.0, 1.0));
        ctx.stroke_width(3.0);
        ctx.rect(nvg::Rect::new(
            select_rect.left_top(),
            (select_rect.width(), select_rect.height()).into(),
        ));
        ctx.stroke().unwrap();
    }
}
