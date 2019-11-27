use crate::collision::distance::DistanceProxy;
use crate::settings;
use crate::{MassData, RayCastInput, RayCastOutput, Shape, ShapeEdge, ShapeType};
use std::borrow::Cow;
use xmath::{Multiply, Real, Transform, Vector2, AABB};

pub struct ShapeChain<T> {
    pub(crate) vertices: Vec<Vector2<T>>,
    pub(crate) prev_vertex: Option<Vector2<T>>,
    pub(crate) next_vertex: Option<Vector2<T>>,
}

impl<T: Real> ShapeChain<T> {
    fn check_vertices(vertices: &Vec<Vector2<T>>) {
        for i in 1..vertices.len() {
            let v1 = vertices[i - 1];
            let v2 = vertices[i];
            assert!(
                v1.distance_squared(&v2)
                    > settings::linear_slop::<T>() * settings::linear_slop::<T>()
            );
        }
    }

    pub fn create_loop<I, V>(vertices: I) -> ShapeChain<T>
    where
        I: IntoIterator<Item = V>,
        V: Into<Vector2<T>>,
    {
        let mut vertices = vertices.into_iter().map(|v| v.into()).collect::<Vec<_>>();
        assert!(vertices.len() >= 3);
        Self::check_vertices(&vertices);

        vertices.push(vertices[0]);
        let prev_vertex = Some(vertices[vertices.len() - 2]);
        let next_vertex = Some(vertices[1]);
        ShapeChain {
            vertices,
            prev_vertex,
            next_vertex,
        }
    }

    pub fn create_chain<I, V>(vertices: I) -> ShapeChain<T>
    where
        I: IntoIterator<Item = V>,
        V: Into<Vector2<T>>,
    {
        let vertices = vertices.into_iter().map(|v| v.into()).collect::<Vec<_>>();
        assert!(vertices.len() >= 2);
        Self::check_vertices(&vertices);

        ShapeChain {
            vertices,
            prev_vertex: None,
            next_vertex: None,
        }
    }

    pub fn get_child_edge(&self, index: usize) -> ShapeEdge<T> {
        assert!(index < self.vertices.len() - 1);

        ShapeEdge {
            vertex1: self.vertices[index + 0],
            vertex2: self.vertices[index + 1],
            vertex0: if index > 0 {
                Some(self.vertices[index - 1])
            } else {
                self.prev_vertex
            },
            vertex3: if index < self.vertices.len() - 2 {
                Some(self.vertices[index + 2])
            } else {
                self.next_vertex
            },
        }
    }
}

impl<T: Real> Shape<T> for ShapeChain<T> {
    fn shape_type(&self) -> ShapeType {
        ShapeType::Chain
    }

    fn radius(&self) -> T {
        settings::polygon_radius()
    }

    fn child_count(&self) -> usize {
        self.vertices.len() - 1
    }

    fn test_point(&self, _xf: &Transform<T>, _p: &Vector2<T>) -> bool {
        false
    }

    fn ray_cast(
        &self,
        input: &RayCastInput<T>,
        xf: &Transform<T>,
        child_index: usize,
    ) -> Option<RayCastOutput<T>> {
        assert!(child_index < self.vertices.len());

        let i1 = child_index;
        let mut i2 = child_index + 1;
        if i2 == self.vertices.len() {
            i2 = 0;
        }

        let edge = ShapeEdge::new(self.vertices[i1], self.vertices[i2]);
        edge.ray_cast(input, xf, 0)
    }

    fn compute_aabb(&self, xf: &Transform<T>, child_index: usize) -> AABB<T> {
        assert!(child_index < self.vertices.len());

        let i1 = child_index;
        let mut i2 = child_index + 1;
        if i2 == self.vertices.len() {
            i2 = 0;
        }

        let v1 = xf.multiply(self.vertices[i1]);
        let v2 = xf.multiply(self.vertices[i2]);

        AABB {
            lower_bound: v1.min(v2),
            upper_bound: v1.max(v2),
        }
    }

    fn compute_mass(&self, _density: T) -> MassData<T> {
        MassData {
            mass: T::zero(),
            center: Vector2::zero(),
            i: T::zero(),
        }
    }

    fn distance_proxy(&self, index: usize) -> DistanceProxy<'_, T> {
        assert!(index < self.vertices.len());

        let v1 = self.vertices[index];
        let v2 = if index + 1 < self.vertices.len() {
            self.vertices[index + 1]
        } else {
            self.vertices[0]
        };
        let vertices = vec![v1, v2];
        DistanceProxy {
            vertices: Cow::Owned(vertices),
            radius: self.radius(),
        }
    }
}
