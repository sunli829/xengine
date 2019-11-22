use crate::collision::distance::{distance, DistanceInput, DistanceProxy, SimpleCache};
use crate::settings;
use xmath::{CrossTrait, DotTrait, Multiply, Real, Sweep, Transform, TransposeMultiply, Vector2};

pub struct TOIInput<'a, T: Real> {
    proxy_a: &'a DistanceProxy<'a, T>,
    proxy_b: &'a DistanceProxy<'a, T>,
    sweep_a: Sweep<T>,
    sweep_b: Sweep<T>,
    max: T,
}

pub enum TOIOutputState {
    Unknown,
    Field,
    Overlapped,
    Touching,
    Separated,
}

pub struct TOIOutput<T> {
    state: TOIOutputState,
    t: T,
}

enum SeparationFunctionType {
    Points,
    FaceA,
    FaceB,
}

struct SeparationFunction<'a, T: Real> {
    proxy_a: &'a DistanceProxy<'a, T>,
    proxy_b: &'a DistanceProxy<'a, T>,
    sweep_a: Sweep<T>,
    sweep_b: Sweep<T>,
    type_: SeparationFunctionType,
    local_point: Vector2<T>,
    axis: Vector2<T>,
}

impl<'a, T: Real> SeparationFunction<'a, T> {
    fn init(
        cache: &SimpleCache<T>,
        proxy_a: &'a DistanceProxy<'_, T>,
        sweep_a: &Sweep<T>,
        proxy_b: &'a DistanceProxy<'_, T>,
        sweep_b: &Sweep<T>,
        t1: T,
    ) -> (SeparationFunction<'a, T>, T) {
        let count = cache.count;
        assert!(0 < count && count < 3);

        let xf_a = sweep_a.get_transform(t1);
        let xf_b = sweep_b.get_transform(t1);

        if count == 1 {
            let local_point_a = proxy_a.vertices.as_ref()[cache.index_a[0]];
            let local_point_b = proxy_b.vertices.as_ref()[cache.index_b[0]];
            let point_a = xf_a.multiply(local_point_a);
            let point_b = xf_b.multiply(local_point_b);
            let s = (point_b - point_a).length();
            let axis = (point_b - point_a).normalize();
            (
                SeparationFunction {
                    proxy_a,
                    proxy_b,
                    sweep_a: sweep_a.clone(),
                    sweep_b: sweep_b.clone(),
                    type_: SeparationFunctionType::Points,
                    local_point: Vector2::zero(),
                    axis,
                },
                s,
            )
        } else if cache.index_a[0] == cache.index_a[1] {
            let local_point_b1 = proxy_b.vertices.as_ref()[cache.index_b[0]];
            let local_point_b2 = proxy_b.vertices.as_ref()[cache.index_b[1]];

            let mut axis = (local_point_b2 - local_point_b1)
                .cross(T::one())
                .normalize();
            let normal = xf_b.q.multiply(axis);

            let local_point = (local_point_b1 + local_point_b2) * T::half();
            let point_b = xf_b.multiply(local_point);

            let local_point_a = proxy_a.vertices.as_ref()[cache.index_a[0]];
            let point_a = xf_a.multiply(local_point_a);

            let mut s = (point_a - point_b).dot(normal);
            if s < T::zero() {
                axis = -axis;
                s = -s;
            }

            (
                SeparationFunction {
                    proxy_a,
                    proxy_b,
                    sweep_a: sweep_a.clone(),
                    sweep_b: sweep_b.clone(),
                    type_: SeparationFunctionType::FaceB,
                    local_point,
                    axis,
                },
                s,
            )
        } else {
            let local_point_a1 = proxy_a.vertices.as_ref()[cache.index_a[0]];
            let local_point_a2 = proxy_a.vertices.as_ref()[cache.index_a[1]];

            let mut axis = (local_point_a2 - local_point_a1)
                .cross(T::one())
                .normalize();
            let normal = xf_a.q.multiply(axis);

            let local_point = (local_point_a1 + local_point_a2) * T::half();
            let point_a = xf_a.multiply(local_point);

            let local_point_b = proxy_b.vertices.as_ref()[cache.index_b[0]];
            let point_b = xf_b.multiply(local_point_b);

            let mut s = (point_b - point_a).dot(normal);
            if s < T::zero() {
                axis = -axis;
                s = -s;
            }
            (
                SeparationFunction {
                    proxy_a,
                    proxy_b,
                    sweep_a: sweep_a.clone(),
                    sweep_b: sweep_b.clone(),
                    type_: SeparationFunctionType::FaceA,
                    local_point,
                    axis,
                },
                s,
            )
        }
    }

    fn find_min_separation(&self, t: T, index_a: &mut usize, index_b: &mut usize) -> T {
        let xf_a = self.sweep_a.get_transform(t);
        let xf_b = self.sweep_b.get_transform(t);

        match self.type_ {
            SeparationFunctionType::Points => {
                let axis_a = xf_a.q.transpose_multiply(self.axis);
                let axis_b = xf_b.q.transpose_multiply(-self.axis);

                *index_a = self.proxy_a.get_support(&axis_a);
                *index_b = self.proxy_b.get_support(&axis_b);

                let local_point_a = self.proxy_a.vertices.as_ref()[*index_a];
                let local_point_b = self.proxy_b.vertices.as_ref()[*index_b];

                let point_a = xf_a.multiply(local_point_a);
                let point_b = xf_b.multiply(local_point_b);

                (point_b - point_a).dot(self.axis)
            }
            SeparationFunctionType::FaceA => {
                let normal = xf_a.q.multiply(self.axis);
                let point_a = xf_a.multiply(self.local_point);

                let axis_b = xf_b.q.transpose_multiply(-normal);

                *index_b = self.proxy_b.get_support(&axis_b);

                let local_point_b = self.proxy_b.vertices.as_ref()[*index_b];
                let point_b = xf_b.multiply(local_point_b);

                (point_b - point_a).dot(normal)
            }
            SeparationFunctionType::FaceB => {
                let normal = xf_b.q.multiply(self.axis);
                let point_b = xf_b.multiply(self.local_point);

                let axis_a = xf_a.q.transpose_multiply(-normal);

                *index_a = self.proxy_a.get_support(&axis_a);

                let local_point_a = self.proxy_a.vertices.as_ref()[*index_a];
                let point_a = xf_a.multiply(local_point_a);

                (point_a - point_b).dot(normal)
            }
        }
    }

    fn evaluate(&self, index_a: usize, index_b: usize, t: T) -> T {
        let xf_a = self.sweep_a.get_transform(t);
        let xf_b = self.sweep_b.get_transform(t);

        match self.type_ {
            SeparationFunctionType::Points => {
                let local_point_a = self.proxy_a.vertices.as_ref()[index_a];
                let local_point_b = self.proxy_b.vertices.as_ref()[index_b];

                let point_a = xf_a.multiply(local_point_a);
                let point_b = xf_b.multiply(local_point_b);
                (point_b - point_a).dot(self.axis)
            }
            SeparationFunctionType::FaceA => {
                let normal = xf_a.q.multiply(self.axis);
                let point_a = xf_a.multiply(self.local_point);

                let local_point_b = self.proxy_b.vertices.as_ref()[index_b];
                let point_b = xf_b.multiply(local_point_b);

                (point_b - point_a).dot(normal)
            }
            SeparationFunctionType::FaceB => {
                let normal = xf_b.q.multiply(self.axis);
                let point_b = xf_b.multiply(self.local_point);

                let local_point_a = self.proxy_a.vertices.as_ref()[index_a];
                let point_a = xf_a.multiply(local_point_a);

                (point_a - point_b).dot(normal)
            }
        }
    }
}

pub fn time_of_impact<T: Real>(input: TOIInput<'_, T>) -> TOIOutput<T> {
    let mut output = TOIOutput {
        state: TOIOutputState::Unknown,
        t: input.max,
    };

    let proxy_a = input.proxy_a;
    let proxy_b = input.proxy_b;

    let sweep_a = input.sweep_a.normalize();
    let sweep_b = input.sweep_b.normalize();

    let max = input.max;

    let total_radius = proxy_a.radius + proxy_b.radius;
    let target = settings::linear_slop::<T>()
        .max(total_radius - T::from_i32(3) * settings::linear_slop::<T>());
    let tolerance = T::from_f32(0.25) * settings::linear_slop::<T>();
    assert!(target > tolerance);

    let t1 = T::zero();
    const MAX_ITERATIONS: usize = 20;
    let mut iter = 0;

    let mut cache = SimpleCache::<T>::default();
    let mut distance_input = DistanceInput {
        proxy_a: input.proxy_a,
        proxy_b: input.proxy_b,
        transform_a: Transform::identity(),
        transform_b: Transform::identity(),
        use_radii: false,
    };

    loop {
        let xf_a = sweep_a.get_transform(t1);
        let xf_b = sweep_b.get_transform(t1);

        distance_input.transform_a = xf_a;
        distance_input.transform_b = xf_b;
        let distance_output = distance(&distance_input, &mut cache);

        if distance_output.distance <= T::zero() {
            output.state = TOIOutputState::Overlapped;
            output.t = T::zero();
            break;
        }

        if distance_output.distance < target + tolerance {
            output.state = TOIOutputState::Touching;
            output.t = t1;
            break;
        }

        let fcn = SeparationFunction::init(&cache, proxy_a, &sweep_a, proxy_b, &sweep_b, t1).0;
    }

    unimplemented!()
}
