use crate::camera::Camera;
use std::cell::RefCell;
use std::rc::Rc;
use xmath::{DotTrait, Multiply, Transform, Vector2};
use xphysics::Color;

fn to_nvg_color(color: xphysics::Color) -> nvg::Color {
    nvg::Color {
        r: color.r,
        g: color.g,
        b: color.b,
        a: color.a,
    }
}

fn to_nvg_fill_color(color: xphysics::Color) -> nvg::Color {
    nvg::Color {
        r: color.r * 0.5,
        g: color.g * 0.5,
        b: color.b * 0.5,
        a: 0.5,
    }
}

pub struct NvgDebugDraw {
    pub ctx: Rc<RefCell<nvg::Context<nvg_gl::Renderer>>>,
    pub camera: Rc<RefCell<Camera>>,
}

impl NvgDebugDraw {
    fn transform_pt(&self, pt: Vector2<f32>) -> nvg::Point {
        let Vector2 { x, y } = self.camera.borrow().transform_pt(pt);
        (x, y).into()
    }

    pub fn transform_width(&self, width: f32) -> f32 {
        self.camera.borrow().transform_width(width)
    }

    pub fn transform_height(&self, height: f32) -> f32 {
        self.camera.borrow().transform_height(height)
    }

    pub fn transform_size(&self, size: Vector2<f32>) -> Vector2<f32> {
        let Vector2 { x, y } = self.camera.borrow().transform_size(size);
        (x, y).into()
    }
}

impl xphysics::DebugDraw for NvgDebugDraw {
    fn draw_polygon(&mut self, vertices: &[Vector2<f32>], color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.move_to(self.transform_pt((vertices[0].x, vertices[0].y).into()));
        for v in &vertices[1..] {
            ctx.line_to(self.transform_pt((v.x, v.y).into()));
        }
        ctx.close_path();
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke().unwrap();
    }

    fn draw_solid_polygon(&mut self, vertices: &[Vector2<f32>], color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.move_to(self.transform_pt((vertices[0].x, vertices[0].y).into()));
        for v in &vertices[1..] {
            ctx.line_to(self.transform_pt((v.x, v.y).into()));
        }
        ctx.close_path();
        ctx.fill_paint(to_nvg_fill_color(color));
        ctx.fill().unwrap();
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke().unwrap();
    }

    fn draw_circle(&mut self, center: &Vector2<f32>, radius: f32, color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.ellipse(
            self.transform_pt((center.x, center.y).into()),
            self.transform_width(radius),
            self.transform_height(radius),
        );
        ctx.stroke_paint(to_nvg_color(color));
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
        ctx.ellipse(
            self.transform_pt((center.x, center.y).into()),
            self.transform_width(radius),
            self.transform_height(radius),
        );
        ctx.fill_paint(to_nvg_fill_color(color));
        ctx.fill().unwrap();
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke().unwrap();

        let p = *center + *axis * Vector2::new(radius, radius);
        ctx.begin_path();
        ctx.move_to(self.transform_pt((center.x, center.y).into()));
        ctx.line_to(self.transform_pt((p.x, p.y).into()));
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke().unwrap();
    }

    fn draw_segment(&mut self, p1: &Vector2<f32>, p2: &Vector2<f32>, color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.move_to(self.transform_pt(*p1));
        ctx.line_to(self.transform_pt(*p2));
        ctx.stroke_paint(to_nvg_color(color));
        ctx.stroke().unwrap();
    }

    fn draw_transform(&mut self, xf: &Transform<f32>) {
        let axis_scale = 0.4;
        let red = nvg::Color::rgb(1.0, 0.0, 0.0);
        let green = nvg::Color::rgb(0.0, 1.0, 0.0);
        let p1 = xf.p;

        let mut ctx = self.ctx.borrow_mut();

        ctx.begin_path();
        ctx.move_to(self.transform_pt(p1));
        let p2 = p1 + xf.q.x_axis() * axis_scale;
        ctx.line_to(self.transform_pt(p2));
        ctx.stroke_paint(red);
        ctx.stroke();

        ctx.begin_path();
        ctx.move_to(self.transform_pt(p1));
        let ya = xf.q.y_axis();
        let p2 = p1 + xf.q.y_axis() * axis_scale;
        ctx.line_to(self.transform_pt(p2));
        ctx.stroke_paint(green);
        ctx.stroke();
    }

    fn draw_point(&mut self, p: &Vector2<f32>, color: xphysics::Color) {
        let mut ctx = self.ctx.borrow_mut();
        ctx.begin_path();
        ctx.ellipse(self.transform_pt(*p), 2.0, 2.0);
        ctx.fill_paint(to_nvg_color(color));
        ctx.fill().unwrap();
    }
}
