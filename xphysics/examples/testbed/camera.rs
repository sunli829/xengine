use xmath::{Matrix33, Vector2};

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
            zoom: 1.0,
            window_size: Vector2::zero(),
        }
    }
}

impl Camera {
    pub fn matrix(&self) -> Matrix33<f32> {
        let extent_width = 50.0;
        let extent_height = 50.0;
        Matrix33::scale(1.0, -1.0)
            * Matrix33::translate(0.0, 18.0)
            * Matrix33::scale(
                self.window_size.x / extent_width,
                self.window_size.y / extent_height,
            )
            * Matrix33::translate(self.window_size.x / 2.0, self.window_size.y / 2.0)
    }
}
