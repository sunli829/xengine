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

    fn create_body(world: &mut World<f32, i32>, position: Vector2<f32>) {
        let body = world.create_body(BodyDef {
            data: 10,
            type_: BodyType::Dynamic,
            ..BodyDef::default()
        });
        body.create_fixture(FixtureDef {
            ..FixtureDef::new(ShapeCircle::new(position, 50.0))
        });
    }

    #[test]
    fn test() {
        let mut world = World::<f32, _>::new(Vector2::zero());
        create_body(&mut world, Vector2::new(100.0, 100.0));
        create_body(&mut world, Vector2::new(130.0, 100.0));
        world.step(0.1, 5, 20);
        for _ in 0..10000 {
            world.step(0.1, 5, 20);
        }
    }
}
