use crate::test::TestImpl;
use xmath::{Real, Vector2};
use xphysics::{
    BodyDef, BodyId, BodyType, FixtureDef, ShapeChain, ShapeCircle, ShapeEdge, ShapePolygon, World,
};

pub struct CharacterCollision {
    character: BodyId,
}

impl<T: Real> TestImpl<T> for CharacterCollision {
    fn new(world: &mut World<T, ()>) -> CharacterCollision {
        // Ground body
        let id = world.create_body_with_fixture(
            BodyDef::default(),
            FixtureDef::new(
                ShapeEdge::new(
                    Vector2::new(T::f32(-20.0), T::f32(0.0)),
                    Vector2::new(T::f32(20.0), T::f32(0.0)),
                ),
                T::zero(),
            ),
        );

        // Collinear edges with no adjacency information.
        // This shows the problematic case where a box shape can hit
        // an internal vertex.
        world.create_body_with_fixtures(
            BodyDef::default(),
            vec![
                FixtureDef::new(
                    ShapeEdge::new((T::f32(-8.0), T::f32(1.0)), (T::f32(-6.0), T::f32(1.0))),
                    T::f32(0.0),
                ),
                FixtureDef::new(
                    ShapeEdge::new((T::f32(-6.0), T::f32(1.0)), (T::f32(-4.0), T::f32(1.0))),
                    T::f32(0.0),
                ),
                FixtureDef::new(
                    ShapeEdge::new((T::f32(-4.0), T::f32(1.0)), (T::f32(-2.0), T::f32(1.0))),
                    T::f32(0.0),
                ),
            ],
        );

        // Chain shape
        world.create_body_with_fixture(
            BodyDef {
                angle: T::f32(0.25) * T::pi(),
                ..BodyDef::default()
            },
            FixtureDef::new(
                ShapeChain::create_chain(vec![
                    (T::f32(5.0), T::f32(7.0)),
                    (T::f32(6.0), T::f32(8.0)),
                    (T::f32(7.0), T::f32(8.0)),
                    (T::f32(8.0), T::f32(7.0)),
                ]),
                T::zero(),
            ),
        );

        // Square tiles. This shows that adjacency shapes may
        // have non-smooth collision. There is no solution
        // to this problem.
        world.create_body_with_fixtures(
            BodyDef::default(),
            vec![
                FixtureDef::new(
                    ShapePolygon::new_box(
                        T::f32(1.0),
                        T::f32(1.0),
                        (T::f32(4.0), T::f32(3.0)),
                        T::f32(0.0),
                    ),
                    T::zero(),
                ),
                FixtureDef::new(
                    ShapePolygon::new_box(
                        T::f32(1.0),
                        T::f32(1.0),
                        (T::f32(6.0), T::f32(3.0)),
                        T::f32(0.0),
                    ),
                    T::zero(),
                ),
                FixtureDef::new(
                    ShapePolygon::new_box(
                        T::f32(1.0),
                        T::f32(1.0),
                        (T::f32(8.0), T::f32(3.0)),
                        T::f32(0.0),
                    ),
                    T::zero(),
                ),
            ],
        );

        // Square made from an edge loop. Collision should be smooth.
        world.create_body_with_fixture(
            BodyDef::default(),
            FixtureDef::new(
                ShapeChain::create_loop(vec![
                    (T::f32(-1.0), T::f32(3.0)),
                    (T::f32(1.0), T::f32(3.0)),
                    (T::f32(1.0), T::f32(5.0)),
                    (T::f32(-1.0), T::f32(5.0)),
                ]),
                T::f32(0.0),
            ),
        );

        // Edge loop. Collision should be smooth.
        world.create_body_with_fixture(
            BodyDef {
                position: (T::f32(-10.0), T::f32(4.0)).into(),
                ..BodyDef::default()
            },
            FixtureDef::new(
                ShapeChain::create_loop(vec![
                    (T::f32(0.0), T::f32(0.0)),
                    (T::f32(6.0), T::f32(0.0)),
                    (T::f32(6.0), T::f32(2.0)),
                    (T::f32(4.0), T::f32(1.0)),
                    (T::f32(2.0), T::f32(2.0)),
                    (T::f32(0.0), T::f32(2.0)),
                    (T::f32(-2.0), T::f32(2.0)),
                    (T::f32(-4.0), T::f32(3.0)),
                    (T::f32(-6.0), T::f32(3.0)),
                    (T::f32(-6.0), T::f32(2.0)),
                    (T::f32(-6.0), T::f32(0.0)),
                ]),
                T::f32(0.0),
            ),
        );

        // Square character 1
        world.create_body_with_fixture(
            BodyDef {
                position: (T::f32(-3.0), T::f32(8.0)).into(),
                type_: BodyType::Dynamic,
                fixed_rotation: true,
                allow_sleep: false,
                ..BodyDef::default()
            },
            FixtureDef::new(
                ShapePolygon::new_box_center(T::f32(0.5), T::f32(0.5)),
                T::f32(20.0),
            ),
        );

        // Square character 2
        world.create_body_with_fixture(
            BodyDef {
                position: (T::f32(-5.0), T::f32(5.0)).into(),
                type_: BodyType::Dynamic,
                fixed_rotation: true,
                allow_sleep: false,
                ..BodyDef::default()
            },
            FixtureDef::new(
                ShapePolygon::new_box_center(T::f32(0.25), T::f32(0.25)),
                T::f32(20.0),
            ),
        );

        // Hexagon character
        world.create_body_with_fixture(
            BodyDef {
                position: (T::f32(-5.0), T::f32(8.0)).into(),
                type_: BodyType::Dynamic,
                fixed_rotation: true,
                allow_sleep: false,
                ..BodyDef::default()
            },
            FixtureDef::new(
                ShapePolygon::new({
                    let mut vertices = Vec::with_capacity(6);
                    let mut angle = 0.0;
                    let delta = std::f32::consts::PI / 3.0;
                    for _ in 0..6 {
                        vertices.push((T::f32(0.5 * angle.cos()), T::f32(0.5 * angle.sin())));
                        angle += delta;
                    }
                    vertices
                }),
                T::f32(20.0),
            ),
        );

        // Circle character
        world.create_body_with_fixture(
            BodyDef {
                position: (T::f32(3.0), T::f32(5.0)).into(),
                type_: BodyType::Dynamic,
                fixed_rotation: true,
                allow_sleep: false,
                ..BodyDef::default()
            },
            FixtureDef::new(ShapeCircle::new_with_radius(T::f32(0.5)), T::f32(20.0)),
        );

        // Circle character
        let id = world.create_body_with_fixture(
            BodyDef {
                position: (T::f32(-7.0), T::f32(6.0)).into(),
                type_: BodyType::Dynamic,
                allow_sleep: false,
                ..BodyDef::default()
            },
            FixtureDef {
                friction: T::f32(1.0),
                ..FixtureDef::new(ShapeCircle::new_with_radius(T::f32(0.25)), T::f32(20.0))
            },
        );

        CharacterCollision { character: id }
    }

    fn step(
        &self,
        world: &mut World<T, ()>,
        time_step: T,
        velocity_iterations: usize,
        position_iterations: usize,
    ) {
        let body = world.body_mut(self.character).unwrap();
        let mut v = *body.linear_velocity();
        v.x = T::f32(-5.0);
        body.set_linear_velocity(v);
        world.step(time_step, velocity_iterations, position_iterations);
    }
}
