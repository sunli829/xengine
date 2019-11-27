use crate::test::TestImpl;
use xmath::{Real, Vector2};
use xphysics::{BodyDef, BodyId, FixtureDef, ShapeChain, ShapeEdge, ShapePolygon, World};

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
        //
        //        // Collinear edges with no adjacency information.
        //        // This shows the problematic case where a box shape can hit
        //        // an internal vertex.
        //        world.create_body_with_fixtures(
        //            BodyDef::default(),
        //            vec![
        //                FixtureDef::new(
        //                    ShapeEdge::new((T::f32(-8.0), T::f32(1.0)), (T::f32(-6.0), T::f32(1.0))),
        //                    T::f32(0.0),
        //                ),
        //                FixtureDef::new(
        //                    ShapeEdge::new((T::f32(-6.0), T::f32(1.0)), (T::f32(-4.0), T::f32(1.0))),
        //                    T::f32(0.0),
        //                ),
        //                FixtureDef::new(
        //                    ShapeEdge::new((T::f32(-4.0), T::f32(1.0)), (T::f32(-2.0), T::f32(1.0))),
        //                    T::f32(0.0),
        //                ),
        //            ],
        //        );
        //
        //        // Chain shape
        //        world.create_body_with_fixture(
        //            BodyDef {
        //                angle: T::f32(0.25) * T::pi(),
        //                ..BodyDef::default()
        //            },
        //            FixtureDef::new(
        //                ShapeChain::create_chain(vec![
        //                    (T::f32(5.0), T::f32(7.0)),
        //                    (T::f32(6.0), T::f32(8.0)),
        //                    (T::f32(7.0), T::f32(8.0)),
        //                    (T::f32(8.0), T::f32(7.0)),
        //                ]),
        //                T::zero(),
        //            ),
        //        );
        //
        //        // Square tiles. This shows that adjacency shapes may
        //        // have non-smooth collision. There is no solution
        //        // to this problem.
        //        world.create_body_with_fixtures(
        //            BodyDef::default(),
        //            vec![
        //                FixtureDef::new(
        //                    ShapePolygon::new_box(
        //                        T::f32(1.0),
        //                        T::f32(1.0),
        //                        (T::f32(4.0), T::f32(3.0)),
        //                        T::f32(0.0),
        //                    ),
        //                    T::zero(),
        //                ),
        //                FixtureDef::new(
        //                    ShapePolygon::new_box(
        //                        T::f32(1.0),
        //                        T::f32(1.0),
        //                        (T::f32(6.0), T::f32(3.0)),
        //                        T::f32(0.0),
        //                    ),
        //                    T::zero(),
        //                ),
        //                FixtureDef::new(
        //                    ShapePolygon::new_box(
        //                        T::f32(1.0),
        //                        T::f32(1.0),
        //                        (T::f32(8.0), T::f32(3.0)),
        //                        T::f32(0.0),
        //                    ),
        //                    T::zero(),
        //                ),
        //            ],
        //        );
        //
        //        // Square made from an edge loop. Collision should be smooth.
        //        let id = world.create_body_with_fixture(
        //            BodyDef::default(),
        //            FixtureDef::new(
        //                ShapeChain::create_loop(vec![
        //                    (T::f32(-1.0), T::f32(3.0)),
        //                    (T::f32(1.0), T::f32(3.0)),
        //                    (T::f32(1.0), T::f32(5.0)),
        //                    (T::f32(-1.0), T::f32(5.0)),
        //                ]),
        //                T::f32(0.0),
        //            ),
        //        );

        CharacterCollision { character: id }
    }
}
