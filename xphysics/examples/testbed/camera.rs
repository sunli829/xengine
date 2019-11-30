use xmath::{AffineTransform, Real, Vector2};

const WORLD_WIDTH: f32 = 60.0;
const WORLD_HEIGHT: f32 = 50.0;

#[derive(Copy, Clone)]
pub struct Camera {
    pub offset: Vector2<f32>,
    pub zoom: f32,
    pub window_size: Vector2<f32>,
}

impl Default for Camera {
    fn default() -> Self {
        Camera {
            offset: Vector2::zero(),
            zoom: 1.5,
            window_size: Vector2::zero(),
        }
    }
}

impl Camera {
    pub fn transform_pt(&self, pt: Vector2<f32>) -> Vector2<f32> {
        let rx = self.window_size.x / WORLD_WIDTH;
        let ry = self.window_size.y / WORLD_HEIGHT;
        (Vector2::new(pt.x, -pt.y) * self.zoom
            + Vector2::new(WORLD_WIDTH / 2.0, WORLD_HEIGHT - 5.0))
            * Vector2::new(rx, ry)
    }

    pub fn transform_width(&self, width: f32) -> f32 {
        let rx = self.window_size.x / WORLD_WIDTH * self.zoom;
        width * rx
    }

    pub fn transform_height(&self, height: f32) -> f32 {
        let ry = self.window_size.y / WORLD_HEIGHT * self.zoom;
        height * ry
    }

    pub fn transform_size(&self, size: Vector2<f32>) -> Vector2<f32> {
        (self.transform_width(size.x), self.transform_height(size.y)).into()
    }
}
