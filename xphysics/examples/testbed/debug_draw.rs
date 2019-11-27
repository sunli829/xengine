use std::cell::RefCell;
use std::rc::Rc;
use xmath::{Transform, Vector2};

fn to_nvg_color(color: xphysics::Color) -> nvg::Color {
    nvg::Color {
        r: color.r,
        g: color.g,
        b: color.b,
        a: color.a,
    }
}

pub struct NvgDebugDraw {
    pub ctx: Rc<RefCell<nvg::Context<nvg_gl::Renderer>>>,
}

impl xphysics::DebugDraw for NvgDebugDraw {
    fn draw_polygon(&mut self, vertices: &[Vector2<f32>], color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.move_to((vertices[0].x, vertices[0].y));
        for v in &vertices[1..] {
            ctx.line_to((v.x, v.y));
        }
        ctx.close_path();
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke_width(1.0);
        ctx.stroke().unwrap();
    }

    fn draw_solid_polygon(&mut self, vertices: &[Vector2<f32>], color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.move_to((vertices[0].x, vertices[0].y));
        for v in &vertices[1..] {
            ctx.line_to((v.x, v.y));
        }
        ctx.close_path();
        ctx.fill_paint(to_nvg_color(color));
        ctx.fill().unwrap();
    }

    fn draw_circle(&mut self, center: &Vector2<f32>, radius: f32, color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.circle((center.x, center.y), radius);
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke_width(1.0);
        ctx.stroke().unwrap();
    }

    fn draw_solid_circle(
        &mut self,
        center: &Vector2<f32>,
        radius: f32,
        axis: &Vector2<f32>,
        color: xphysics::Color,
    ) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.circle((center.x, center.y), radius);
        ctx.fill_paint(to_nvg_color(color));
        ctx.fill().unwrap();

        let p = *center + *axis * radius;
        ctx.begin_path();
        ctx.move_to((center.x, center.y));
        ctx.line_to((p.x, p.y));
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke_width(1.0);
        ctx.stroke().unwrap();
    }

    fn draw_segment(&mut self, p1: &Vector2<f32>, p2: &Vector2<f32>, color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.move_to((p1.x, p1.y));
        ctx.line_to((p2.x, p2.y));
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke_width(1.0);
        ctx.stroke().unwrap();
    }

    fn draw_transform(&mut self, _xf: &Transform<f32>) {
        unimplemented!()
    }

    fn draw_point(&mut self, p: &Vector2<f32>, size: f32, color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.rect((p.x - size / 2.0, p.y - size / 2.0, size, size));
        ctx.fill_paint(to_nvg_color(color));
        ctx.fill().unwrap();
    }
}
