use xmath::Vector2;

pub struct Profile<T> {
    step: T,
    collide: T,
    solve: T,
    solve_init: T,
    solve_velocity: T,
    solve_position: T,
    broadphase: T,
    solve_toi: T,
}

pub struct TimeStep<T> {
    dt: T,
    inv_dt: T,
    dt_ratio: T,
    velocity_iterations: usize,
    position_iterations: usize,
    warm_starting: bool,
}

pub struct Position<T> {
    pub c: Vector2<T>,
    pub a: T,
}

pub struct Velocity<T> {
    pub v: Vector2<T>,
    pub w: T,
}

pub struct SolverData<'a, T> {
    step: TimeStep<T>,
    positions: &'a [Position<T>],
    velocities: &'a [Velocity<T>],
}
