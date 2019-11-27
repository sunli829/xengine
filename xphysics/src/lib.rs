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

    fn create_body(world: &mut World<f32, i32>, position: Vector2<f32>) -> BodyId {
        let id = world.create_body(BodyDef {
            data: 10,
            type_: BodyType::Dynamic,
            ..BodyDef::default()
        });
        let body = world.body_mut(id).unwrap();
        body.create_fixture(FixtureDef {
            density: 1.0,
            ..FixtureDef::new(ShapeCircle::new(position, 50.0))
        });
        id
    }

    #[test]
    fn test() {
        let mut world = World::<f32, _>::new(Vector2::new(0.0, -10.0));
        let body = create_body(&mut world, Vector2::new(100.0, 100.0));
        //        let body2 = create_body(&mut world, Vector2::new(130.0, 100.0));
        for _ in 0..1000 {
            world.step(0.1, 6, 2);
            println!("{:?}", world.body(body).unwrap().position());
        }
    }
}
