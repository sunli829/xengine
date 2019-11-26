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
        let world = World::new(Vector2::zero());
        world.create_body(BodyDef{

        })
    }
}
