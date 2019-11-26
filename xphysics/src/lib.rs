#[macro_use]
extern crate bitflags;

mod collision;
mod dynamic;
mod settings;
mod timer;

pub use collision::*;
pub use dynamic::*;

#[cfg(test)]
mod tests {
    use super::*;
    use xmath::Vector2;

    #[test]
    fn test() {
        let mut world = World::<f32, _>::new(Vector2::zero());
        let body = world.create_body(BodyDef {
            data: 10,
            ..BodyDef::default()
        });
        let fixture = body.create_fixture(FixtureDef {
            ..FixtureDef::new(shapes::Circle::new(Vector2::new(100.0, 100.0), 50.0))
        });
        world.step(0.1, 5, 20);
    }
}
