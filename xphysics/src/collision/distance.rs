use std::borrow::Cow;
use xmath::{CrossTrait, DotTrait, Multiply, Real, Transform, TransposeMultiply, Vector2};

pub struct DistanceProxy<'a, T: Real> {
    pub vertices: Cow<'a, [Vector2<T>]>,
    pub radius: T,
}

impl<'a, T: Real> DistanceProxy<'a, T> {
    pub fn get_support(&self, d: &Vector2<T>) -> usize {
        let mut best_index = 0;
        let mut best_value = self.vertices[0].dot(*d);
        for i in 1..self.vertices.len() {
            let value = self.vertices[i].dot(*d);
            if value > best_value {
                best_index = i;
                best_value = value;
            }
        }
        best_index
    }

    pub fn get_support_vertex(&self, d: &Vector2<T>) -> &Vector2<T> {
        &self.vertices[self.get_support(d)]
    }
}

#[derive(Default)]
pub struct SimpleCache<T: Real> {
    pub metric: T,
    pub count: usize,
    pub index_a: [usize; 3],
    pub index_b: [usize; 3],
}

pub struct DistanceInput<'a, T: Real> {
    pub proxy_a: &'a DistanceProxy<'a, T>,
    pub proxy_b: &'a DistanceProxy<'a, T>,
    pub transform_a: Transform<T>,
    pub transform_b: Transform<T>,
    pub use_radii: bool,
}

pub struct DistanceOutput<T> {
    pub point_a: Vector2<T>,
    pub point_b: Vector2<T>,
    pub distance: T,
    pub iterations: usize,
}

#[derive(Clone, Copy, Default)]
struct SimplexVertex<T: Real> {
    wa: Vector2<T>,
    wb: Vector2<T>,
    w: Vector2<T>,
    a: T,
    index_a: usize,
    index_b: usize,
}

#[derive(Default)]
struct Simplex<T: Real> {
    v1: SimplexVertex<T>,
    v2: SimplexVertex<T>,
    v3: SimplexVertex<T>,
    count: usize,
}

impl<T: Real> Simplex<T> {
    fn read_cache(
        &mut self,
        cache: &SimpleCache<T>,
        proxy_a: &DistanceProxy<'_, T>,
        transform_a: &Transform<T>,
        proxy_b: &DistanceProxy<'_, T>,
        transform_b: &Transform<T>,
    ) {
        assert!(cache.count <= 3);

        for (i, v) in unsafe {
            std::slice::from_raw_parts_mut(&mut self.v1 as *mut SimplexVertex<T>, self.count)
        }
        .iter_mut()
        .enumerate()
        {
            v.index_a = cache.index_a[i];
            v.index_b = cache.index_b[i];
            let wa_local = proxy_a.vertices[v.index_a];
            let wb_local = proxy_b.vertices[v.index_b];
            v.wa = transform_a.multiply(wa_local);
            v.wb = transform_b.multiply(wb_local);
            v.w = v.wb - v.wa;
            v.a = T::zero();
        }

        if self.count > 1 {
            let metric1 = cache.metric;
            let metric2 = self.get_metric();
            if metric2 < T::half() * metric1
                || T::two() * metric1 < metric2
                || metric2 < T::epsilon()
            {
                self.count = 0;
            }
        }

        if self.count == 0 {
            let v = &mut self.v1;
            v.index_a = 0;
            v.index_b = 0;
            let wa_local = proxy_a.vertices[0];
            let wb_local = proxy_b.vertices[0];
            v.wa = transform_a.multiply(wa_local);
            v.wb = transform_b.multiply(wb_local);
            v.w = v.wb - v.wa;
            v.a = T::zero();
            self.count = 1;
        }
    }

    fn write_cache(&self, cache: &mut SimpleCache<T>) {
        cache.metric = self.get_metric();
        cache.count = self.count;
        for (i, v) in
            unsafe { std::slice::from_raw_parts(&self.v1 as *const SimplexVertex<T>, self.count) }
                .iter()
                .enumerate()
        {
            cache.index_a[i] = v.index_a;
            cache.index_b[i] = v.index_b;
        }
    }

    fn get_search_direction(&self) -> Vector2<T> {
        match self.count {
            1 => -self.v1.w,
            2 => {
                let e12 = self.v2.w - self.v1.w;
                let sgn = e12.cross(-self.v1.w);
                if sgn > T::zero() {
                    T::one().cross(e12)
                } else {
                    e12.cross(T::one())
                }
            }
            _ => unreachable!(),
        }
    }

    fn get_witness_points(&self) -> (Vector2<T>, Vector2<T>) {
        match self.count {
            1 => (self.v1.wa, self.v1.wb),
            2 => (
                self.v1.wa * self.v1.a + self.v2.wa * self.v2.a,
                self.v1.wb * self.v1.a + self.v2.wb * self.v2.a,
            ),
            3 => {
                let pa = self.v1.wa * self.v1.a + self.v2.wa * self.v2.a + self.v3.wa * self.v3.a;
                (pa, pa)
            }
            _ => unreachable!(),
        }
    }

    fn get_metric(&self) -> T {
        match self.count {
            1 => T::zero(),
            2 => self.v1.w.distance(&self.v2.w),
            3 => (self.v2.w - self.v1.w).cross(self.v3.w - self.v1.w),
            _ => unreachable!(),
        }
    }

    pub fn solve2(&mut self) {
        let w1 = self.v1.w;
        let w2 = self.v2.w;
        let e12 = w2 - w1;

        let d12_2 = -w1.dot(e12);
        if d12_2 <= T::zero() {
            self.v1.a = T::zero();
            self.count = 1;
            return;
        }

        let d12_1 = w2.dot(e12);
        if d12_1 <= T::zero() {
            self.v2.a = T::one();
            self.count = 1;
            self.v1 = self.v2;
            return;
        }

        let inv_d12 = T::one() / (d12_1 + d12_2);
        self.v1.a = d12_1 * inv_d12;
        self.v2.a = d12_2 * inv_d12;
        self.count = 2;
    }

    pub fn solve3(&mut self) {
        let w1 = self.v1.w;
        let w2 = self.v2.w;
        let w3 = self.v3.w;

        let e12 = w2 - w1;
        let w1e12 = w1.dot(e12);
        let w2e12 = w2.dot(e12);
        let d12_1 = w2e12;
        let d12_2 = -w1e12;

        let e13 = w3 - w1;
        let w1e13 = w1.dot(e13);
        let w3e13 = w3.dot(e13);
        let d13_1 = w3e13;
        let d13_2 = -w1e13;

        let e23 = w3 - w2;
        let w2e23 = w2.dot(e23);
        let w3e23 = w3.dot(e23);
        let d23_1 = w3e23;
        let d23_2 = -w2e23;

        let n123 = e12.cross(e13);

        let d123_1 = n123 * w2.cross(w3);
        let d123_2 = n123 * w3.cross(w1);
        let d123_3 = n123 * w1.cross(w2);

        if d12_2 <= T::zero() && d13_2 <= T::zero() {
            self.v1.a = T::one();
            self.count = 1;
            return;
        }

        if d12_1 > T::zero() && d12_2 > T::zero() && d123_3 <= T::zero() {
            let inv_d12 = T::one() / (d12_1 + d12_2);
            self.v1.a = d12_1 * inv_d12;
            self.v2.a = d12_2 * inv_d12;
            self.count = 2;
            return;
        }

        if d13_1 > T::zero() && d13_2 > T::zero() && d123_2 <= T::zero() {
            let inv_d13 = T::zero() / (d13_1 + d13_2);
            self.v1.a = d13_1 * inv_d13;
            self.v3.a = d13_2 * inv_d13;
            self.count = 2;
            self.v2 = self.v3;
            return;
        }

        if d12_1 <= T::zero() && d23_2 <= T::zero() {
            self.v2.a = T::one();
            self.count = 1;
            self.v1 = self.v2;
            return;
        }

        if d13_1 <= T::zero() && d23_1 <= T::zero() {
            self.v3.a = T::one();
            self.count = 1;
            self.v1 = self.v3;
            return;
        }

        if d23_1 > T::zero() && d23_2 > T::zero() && d123_1 <= T::zero() {
            let inv_d23 = T::one() / (d23_1 + d23_2);
            self.v2.a = d23_1 * inv_d23;
            self.v3.a = d23_2 * inv_d23;
            self.count = 2;
            self.v1 = self.v3;
            return;
        }

        let inv_d123 = T::one() / (d123_1 + d123_2 + d123_3);
        self.v1.a = d123_1 * inv_d123;
        self.v2.a = d123_2 * inv_d123;
        self.v3.a = d123_3 * inv_d123;
        self.count = 3;
    }
}

pub fn distance<T: Real>(
    input: &DistanceInput<'_, T>,
    cache: &mut SimpleCache<T>,
) -> DistanceOutput<T> {
    let proxy_a = &input.proxy_a;
    let proxy_b = &input.proxy_b;
    let transform_a = input.transform_a;
    let transform_b = input.transform_b;

    let mut simplex = Simplex::default();
    simplex.read_cache(cache, &proxy_a, &transform_a, &proxy_b, &transform_b);

    let vertices = unsafe { std::slice::from_raw_parts(&simplex.v1 as *const SimplexVertex<T>, 3) };
    const MAX_ITERS: usize = 20;

    let mut save_a = [0usize; 3];
    let mut save_b = [0usize; 3];
    let mut save_count;

    let mut iter = 0;
    while iter < MAX_ITERS {
        save_count = simplex.count;
        for i in 0..save_count {
            save_a[i] = vertices[i].index_a;
            save_b[i] = vertices[i].index_b;
        }

        match simplex.count {
            1 => break,
            2 => simplex.solve2(),
            3 => simplex.solve3(),
            _ => unreachable!(),
        }

        if simplex.count == 3 {
            break;
        }

        let d = simplex.get_search_direction();
        if d.length_squared() < T::epsilon() * T::epsilon() {
            break;
        }

        unsafe {
            let vertex = (&mut simplex.v1 as *mut SimplexVertex<T>).add(simplex.count);
            (*vertex).index_a = proxy_a.get_support(&transform_a.q.transpose_multiply(-d));
            (*vertex).wa = transform_a.multiply(proxy_a.vertices[(*vertex).index_a]);
            (*vertex).index_b = proxy_b.get_support(&transform_b.q.transpose_multiply(d));
            (*vertex).wb = transform_b.multiply(proxy_b.vertices[(*vertex).index_b]);
            (*vertex).w = (*vertex).wb - (*vertex).wa;

            iter += 1;

            let mut duplicate = false;
            for i in 0..save_count {
                if (*vertex).index_a == save_a[i] && (*vertex).index_b == save_b[i] {
                    duplicate = true;
                    break;
                }
            }

            if duplicate {
                break;
            }

            simplex.count += 1;
        }
    }

    let (mut point_a, mut point_b) = simplex.get_witness_points();
    let mut distance = point_a.distance(&point_b);
    let iterations = iter;

    simplex.write_cache(cache);

    if input.use_radii {
        let ra = proxy_a.radius;
        let rb = proxy_b.radius;

        if distance > ra + rb && distance > T::epsilon() {
            distance -= ra + rb;
            let normal = (point_b - point_a).normalize();
            point_a += normal * ra;
            point_b -= normal * rb;
        } else {
            let p = (point_a + point_b) * T::half();
            point_a = p;
            point_b = p;
            distance = T::zero();
        }
    }

    DistanceOutput {
        point_a,
        point_b,
        distance,
        iterations,
    }
}
