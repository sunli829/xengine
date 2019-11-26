use std::time::Duration;
use xmath::Vector2;

#[derive(Debug, Default)]
pub struct Profile {
    pub step: Duration,
    pub collide: Duration,
    pub solve: Duration,
    pub solve_init: Duration,
    pub solve_velocity: Duration,
    pub solve_position: Duration,
    pub broad_phase: Duration,
    pub solve_toi: Duration,
}

#[derive(Copy, Clone)]
pub struct TimeStep<T> {
    pub dt: T,
    pub inv_dt: T,
    pub dt_ratio: T,
    pub velocity_iterations: usize,
    pub position_iterations: usize,
    pub warm_starting: bool,
}

#[derive(Copy, Clone)]
pub struct Position<T> {
    pub c: Vector2<T>,
    pub a: T,
}

#[derive(Copy, Clone)]
pub struct Velocity<T> {
    pub v: Vector2<T>,
    pub w: T,
}

//pub struct SolverData<'a, T> {
//    pub step: TimeStep<T>,
//    pub positions: &'a [Position<T>],
//    pub velocities: &'a [Velocity<T>],
//}
