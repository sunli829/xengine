use crate::dynamic::contacts::Contact;
use crate::dynamic::time_step::{Position, TimeStep, Velocity};
use crate::{settings, ManifoldType, WorldManifold};
use xmath::{
    CrossTrait, DotTrait, Matrix22, Multiply, Real, Rotation, Transform, TransposeMultiply, Vector2,
};

const BLOCK_SOLVE: bool = true;

struct VelocityConstraintPoint<T> {
    ra: Vector2<T>,
    rb: Vector2<T>,
    normal_impulse: T,
    tangent_impulse: T,
    normal_mass: T,
    tangent_mass: T,
    velocity_bias: T,
}

struct ContactVelocityConstraint<T> {
    points: [VelocityConstraintPoint<T>; settings::MAX_MANIFOLD_POINTS],
    normal: Vector2<T>,
    normal_mass: Matrix22<T>,
    k: Matrix22<T>,
    index_a: usize,
    index_b: usize,
    inv_mass_a: T,
    inv_mass_b: T,
    inv_i_a: T,
    inv_i_b: T,
    friction: T,
    restitution: T,
    tangent_speed: T,
    point_count: usize,
    contact_index: usize,
}

struct ContactPositionConstraint<T> {
    local_points: [Vector2<T>; settings::MAX_MANIFOLD_POINTS],
    local_normal: Vector2<T>,
    local_point: Vector2<T>,
    index_a: usize,
    index_b: usize,
    inv_mass_a: T,
    inv_mass_b: T,
    local_center_a: Vector2<T>,
    local_center_b: Vector2<T>,
    inv_i_a: T,
    inv_i_b: T,
    type_: ManifoldType,
    radius_a: T,
    radius_b: T,
    point_count: usize,
}

pub struct ContactSolverDef<'a, T, D> {
    step: TimeStep<T>,
    contacts: &'a mut [&'a mut Contact<T, D>],
    positions: &'a mut [Position<T>],
    velocities: &'a mut [Velocity<T>],
}

pub struct ContactSolver<'a, T, D> {
    step: TimeStep<T>,
    positions: &'a mut [Position<T>],
    velocities: &'a mut [Velocity<T>],
    position_constraints: Vec<ContactPositionConstraint<T>>,
    velocity_constraints: Vec<ContactVelocityConstraint<T>>,
    contacts: &'a mut [&'a mut Contact<T, D>],
}

impl<'a, T: Real, D> ContactSolver<'a, T, D> {
    pub fn new(def: ContactSolverDef<'a, T, D>) -> ContactSolver<'a, T, D> {
        let mut position_constraints =
            Vec::<ContactPositionConstraint<T>>::with_capacity(def.contacts.len());
        position_constraints.resize_with(def.contacts.len(), || unsafe { std::mem::zeroed() });
        let mut velocity_constraints =
            Vec::<ContactVelocityConstraint<T>>::with_capacity(def.contacts.len());
        velocity_constraints.resize_with(def.contacts.len(), || unsafe { std::mem::zeroed() });

        for i in 0..def.contacts.len() {
            let contact = &mut def.contacts[i];
            let fixture_a = contact.fixture_a();
            let fixture_b = contact.fixture_b();
            let shape_a = fixture_a.shape();
            let shape_b = fixture_a.shape();
            let radius_a = shape_a.radius();
            let radius_b = shape_b.radius();
            let body_a = fixture_a.body();
            let body_b = fixture_b.body();
            let manifold = contact.manifold();
            let point_count = manifold.point_count;
            assert!(point_count > 0);

            let vc = &mut velocity_constraints[i];
            vc.friction = contact.friction();
            vc.restitution = contact.restitution();
            vc.tangent_speed = contact.tangent_speed();
            vc.index_a = body_a.island_index;
            vc.index_b = body_b.island_index;
            vc.inv_mass_a = body_a.inv_mass;
            vc.inv_mass_b = body_b.inv_mass;
            vc.inv_i_a = body_a.inv_i;
            vc.inv_i_b = body_b.inv_i;
            vc.contact_index = i;
            vc.point_count = point_count;
            vc.k = Matrix22::zero();
            vc.normal_mass = Matrix22::zero();

            let pc = &mut position_constraints[i];
            pc.index_a = body_a.island_index;
            pc.index_b = body_b.island_index;
            pc.inv_mass_a = body_a.inv_mass;
            pc.inv_mass_b = body_b.inv_mass;
            pc.local_center_a = body_a.sweep.local_center;
            pc.local_center_b = body_b.sweep.local_center;
            pc.inv_i_a = body_a.inv_i;
            pc.inv_i_b = body_b.inv_i;
            pc.local_normal = manifold.local_normal;
            pc.local_point = manifold.local_point;
            pc.point_count = point_count;
            pc.radius_a = radius_a;
            pc.radius_b = radius_b;
            pc.type_ = manifold.type_;
        }

        ContactSolver {
            step: def.step,
            positions: def.positions,
            velocities: def.velocities,
            position_constraints,
            velocity_constraints,
            contacts: def.contacts,
        }
    }

    fn initialize_velocity_constraints(&mut self) {
        for i in 0..self.contacts.len() {
            let vc = &mut self.velocity_constraints[i];
            let pc = &mut self.position_constraints[i];

            let radius_a = pc.radius_a;
            let radius_b = pc.radius_b;
            let manifold = self.contacts[vc.contact_index].manifold();

            let index_a = vc.index_a;
            let index_b = vc.index_b;

            let ma = vc.inv_mass_a;
            let mb = vc.inv_mass_b;
            let ia = vc.inv_i_a;
            let ib = vc.inv_i_b;
            let local_center_a = pc.local_center_a;
            let local_center_b = pc.local_center_b;

            let ca = self.positions[index_a].c;
            let aa = self.positions[index_a].a;
            let va = self.velocities[index_a].v;
            let wa = self.velocities[index_a].w;

            let cb = self.positions[index_b].c;
            let ab = self.positions[index_b].a;
            let vb = self.velocities[index_b].v;
            let wb = self.velocities[index_b].w;

            assert!(manifold.point_count > 0);

            let xf_a = {
                let r = Rotation::new(aa);
                Transform {
                    q: r,
                    p: ca - r.multiply(local_center_a),
                }
            };
            let xf_b = {
                let r = Rotation::new(ab);
                Transform {
                    q: r,
                    p: cb - r.multiply(local_center_b),
                }
            };

            let world_manifold = WorldManifold::new(manifold, &xf_a, radius_a, &xf_b, radius_b);
            vc.normal = world_manifold.normal;

            let point_count = vc.point_count;
            for j in 0..point_count {
                let vcp = &mut vc.points[j];

                vcp.ra = world_manifold.points[j] - ca;
                vcp.rb = world_manifold.points[j] - cb;

                let rna = vcp.ra.cross(vc.normal);
                let rnb = vcp.rb.cross(vc.normal);

                let k_normal = ma + mb + ia * rna * rna + ib * rnb * rnb;

                vcp.normal_mass = if k_normal > T::zero() {
                    T::one() / k_normal
                } else {
                    T::zero()
                };

                let tangent = vc.normal.cross(T::one());

                let rta = vcp.ra.cross(tangent);
                let rtb = vcp.rb.cross(tangent);

                let k_tangent = ma + mb + ia * rta * rta + ib * rtb * rtb;

                vcp.tangent_mass = if k_tangent > T::zero() {
                    T::one() / k_tangent
                } else {
                    T::zero()
                };

                vcp.velocity_bias = T::zero();
                let v_rel = vc.normal.dot(vb + wb.cross(vcp.rb) - va - wa.cross(vcp.ra));
                if v_rel < -settings::velocity_threshold::<T>() {
                    vcp.velocity_bias = -vc.restitution * v_rel;
                }
            }

            if vc.point_count == 2 && BLOCK_SOLVE {
                let vcp1 = &vc.points[0];
                let vcp2 = &vc.points[1];

                let rn1a = vcp1.ra.cross(vc.normal);
                let rn1b = vcp1.rb.cross(vc.normal);
                let rn2a = vcp2.ra.cross(vc.normal);
                let rn2b = vcp2.rb.cross(vc.normal);

                let k11 = ma + mb + ia * rn1a * rn1a + ib * rn1b * rn1b;
                let k22 = ma + mb + ia * rn2a * rn2a + ib * rn2b * rn2b;
                let k12 = ma + mb + ia * rn1a * rn2a + ib * rn1b * rn2b;

                if k11 * k11 < T::from_i32(1000) * (k11 * k22 - k12 * k12) {
                    vc.k.ex = Vector2::new(k11, k12);
                    vc.k.ey = Vector2::new(k12, k22);
                    vc.normal_mass = vc.k.inverse();
                } else {
                    vc.point_count = 1;
                }
            }
        }
    }

    pub fn warm_start(&mut self) {
        for i in 0..self.contacts.len() {
            let vc = &self.velocity_constraints[i];

            let index_a = vc.index_a;
            let index_b = vc.index_b;
            let ma = vc.inv_mass_a;
            let ia = vc.inv_i_a;
            let mb = vc.inv_mass_b;
            let ib = vc.inv_i_b;
            let point_count = vc.point_count;

            let mut va = self.velocities[index_a].v;
            let mut wa = self.velocities[index_a].w;
            let mut vb = self.velocities[index_b].v;
            let mut wb = self.velocities[index_b].w;

            let normal = vc.normal;
            let tangent = normal.cross(T::one());

            for j in 0..point_count {
                let vcp = &vc.points[j];
                let p = normal * vcp.normal_impulse + tangent * vcp.tangent_impulse;
                wa -= ia * vcp.ra.cross(p);
                va -= p * ma;
                wb += ib * vcp.rb.cross(p);
                vb += p * mb;
            }

            self.velocities[index_a].v = va;
            self.velocities[index_a].w = wa;
            self.velocities[index_b].v = vb;
            self.velocities[index_b].w = wb;
        }
    }

    pub fn solve_velocity_constraints(&mut self) {
        for i in 0..self.contacts.len() {
            let vc = &mut self.velocity_constraints[i];

            let index_a = vc.index_a;
            let index_b = vc.index_b;
            let ma = vc.inv_mass_a;
            let ia = vc.inv_i_a;
            let mb = vc.inv_mass_b;
            let ib = vc.inv_i_b;
            let point_count = vc.point_count;

            let mut va = self.velocities[index_a].v;
            let mut wa = self.velocities[index_a].w;
            let mut vb = self.velocities[index_b].v;
            let mut wb = self.velocities[index_b].w;

            let normal = vc.normal;
            let tangent = normal.cross(T::one());
            let friction = vc.friction;

            assert!(point_count == 1 || point_count == 2);

            for j in 0..point_count {
                let vcp = &mut vc.points[j];

                let dv = vb + wb.cross(vcp.rb) - va - wa.cross(vcp.ra);

                let vt = dv.dot(tangent) - vc.tangent_speed;
                let mut lambda = vcp.tangent_mass * -vt;

                let max_friction = friction * vcp.normal_impulse;
                let new_impulse = (vcp.tangent_impulse + lambda).clamp(-max_friction, max_friction);
                lambda = new_impulse - vcp.tangent_impulse;
                vcp.tangent_impulse = new_impulse;

                let p = tangent * lambda;

                va -= p * ma;
                wa -= ia * vcp.ra.cross(p);

                vb += p * mb;
                wb += ib * vcp.rb.cross(p);
            }

            if point_count == 1 || !BLOCK_SOLVE {
                for j in 0..point_count {
                    let vcp = &mut vc.points[j];

                    let dv = vb + wb.cross(vcp.rb) - va - wa.cross(vcp.ra);

                    let vn = dv.dot(normal);
                    let mut lambda = -vcp.normal_mass * (vn - vcp.velocity_bias);

                    let new_impulse = (vcp.normal_impulse + lambda).max(T::zero());
                    lambda = new_impulse - vcp.normal_impulse;
                    vcp.normal_impulse = new_impulse;

                    let p = normal * lambda;
                    va -= p * ma;
                    wa -= ia * vcp.ra.cross(p);

                    vb += p * mb;
                    wb += ib * vcp.rb.cross(p);
                }
            } else {
                let cp1 = unsafe { vc.points.as_mut_ptr().as_mut().unwrap() };
                let cp2 = unsafe { vc.points.as_mut_ptr().add(1).as_mut().unwrap() };

                let a = Vector2::new(cp1.normal_impulse, cp2.normal_impulse);
                assert!(a.x >= T::zero() && a.y >= T::zero());

                let dv1 = vb + wb.cross(cp1.rb) - va - wa.cross(cp1.ra);
                let dv2 = vb + wb.cross(cp2.rb) - va - wa.cross(cp2.ra);

                let mut vn1 = dv1.dot(normal);
                let mut vn2 = dv2.dot(normal);

                let mut b = Vector2::new(vn1 - cp1.velocity_bias, vn2 - cp2.velocity_bias);

                b -= vc.k.multiply(a);

                loop {
                    let mut x = -vc.normal_mass.multiply(b);

                    if x.x >= T::zero() && x.y >= T::zero() {
                        let d = x - a;

                        let p1 = normal * d.x;
                        let p2 = normal * d.y;
                        va -= (p1 + p2) * ma;
                        wa -= ia * cp1.ra.cross(p1) + cp2.ra.cross(p2);

                        vb += (p1 + p2) * mb;
                        wb += ib * cp1.rb.cross(p1) + cp2.rb.cross(p2);

                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;
                        break;
                    }

                    x.x = -cp1.normal_mass * b.x;
                    x.y = T::zero();
                    //                    vn1 = T::zero();
                    vn2 = vc.k.ex.y * x.x + b.y;
                    if x.x >= T::zero() && vn2 >= T::zero() {
                        let d = x - a;

                        let p1 = normal * d.x;
                        let p2 = normal * d.y;
                        va -= (p1 + p2) * ma;
                        wa -= ia * (cp1.ra.cross(p1) + cp2.ra.cross(p2));

                        vb += (p1 + p2) * mb;
                        wb += ib * (cp1.rb.cross(p1) + cp2.rb.cross(p2));

                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;
                        break;
                    }

                    x.x = T::zero();
                    x.y = -cp2.normal_mass * b.y;
                    vn1 = vc.k.ey.x * x.y + b.x;
                    //                    vn2 = T::zero();

                    if x.y >= T::zero() && vn1 >= T::zero() {
                        let d = x - a;

                        let p1 = normal * d.x;
                        let p2 = normal * d.y;
                        va -= (p1 + p2) * ma;
                        wa -= ia * (cp1.ra.cross(p1) + cp2.ra.cross(p2));

                        vb += (p1 + p2) * mb;
                        wb += ib * (cp1.rb.cross(p1) + cp2.rb.cross(p2));

                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;
                        break;
                    }

                    x.x = T::zero();
                    x.y = T::zero();
                    vn1 = b.x;
                    vn2 = b.y;

                    if vn1 >= T::zero() && vn2 >= T::zero() {
                        let d = x - a;

                        let p1 = normal * d.x;
                        let p2 = normal * d.y;
                        va -= (p1 + p2) * ma;
                        wa -= ia * (cp1.ra.cross(p1) + cp2.ra.cross(p2));

                        vb += (p1 + p2) * mb;
                        wb += ib * (cp1.rb.cross(p1) + cp2.rb.cross(p2));

                        cp1.normal_impulse = x.x;
                        cp2.normal_impulse = x.y;
                        break;
                    }

                    break;
                }
            }

            self.velocities[index_a].v = va;
            self.velocities[index_a].w = wa;
            self.velocities[index_b].v = vb;
            self.velocities[index_b].w = wb;
        }
    }

    pub fn store_impulses(&mut self) {
        for i in 0..self.contacts.len() {
            let vc = &self.velocity_constraints[i];
            let manifold = self.contacts[vc.contact_index].manifold_mut();

            for j in 0..vc.point_count {
                manifold.points[j].normal_impulse = vc.points[j].normal_impulse;
                manifold.points[j].tangent_impulse = vc.points[j].tangent_impulse;
            }
        }
    }

    pub fn solve_position_constraints(&mut self) -> bool {
        let mut min_separation = T::zero();

        for i in 0..self.contacts.len() {
            let pc = &self.position_constraints[i];

            let index_a = pc.index_a;
            let index_b = pc.index_b;
            let local_center_a = pc.local_center_a;
            let ma = pc.inv_mass_a;
            let ia = pc.inv_i_a;
            let local_center_b = pc.local_center_b;
            let mb = pc.inv_mass_b;
            let ib = pc.inv_i_b;
            let point_count = pc.point_count;

            let mut ca = self.positions[index_a].c;
            let mut aa = self.positions[index_a].a;

            let mut cb = self.positions[index_b].c;
            let mut ab = self.positions[index_b].a;

            for j in 0..point_count {
                let xf_a = {
                    let r = Rotation::new(aa);
                    Transform {
                        q: r,
                        p: ca - r.multiply(local_center_a),
                    }
                };
                let xf_b = {
                    let r = Rotation::new(ab);
                    Transform {
                        q: r,
                        p: cb - r.multiply(local_center_b),
                    }
                };

                let psm = PositionSolverManifold::new(pc, &xf_a, &xf_b, j);
                let normal = psm.normal;

                let point = psm.point;
                let separation = psm.separation;

                let ra = point - ca;
                let rb = point - cb;

                min_separation = min_separation.min(separation);

                let c = (settings::baumgarte::<T>() * (separation + settings::linear_slop::<T>()))
                    .clamp(-settings::max_linear_correction::<T>(), T::zero());

                let rna = ra.cross(normal);
                let rnb = rb.cross(normal);
                let k = ma + mb + ia * rna * rna + ib * rnb * rnb;

                let impulse = if k > T::zero() { -c / k } else { T::zero() };

                let p = normal * impulse;

                ca -= p * ma;
                aa -= ia * ra.cross(p);

                cb += p * mb;
                ab += ib * rb.cross(p);
            }

            self.positions[index_a].c = ca;
            self.positions[index_a].a = aa;

            self.positions[index_b].c = cb;
            self.positions[index_b].a = ab;
        }

        min_separation >= -T::from_i32(3) * settings::linear_slop()
    }
}

struct PositionSolverManifold<T> {
    normal: Vector2<T>,
    point: Vector2<T>,
    separation: T,
}

impl<T: Real> PositionSolverManifold<T> {
    fn new(
        pc: &ContactPositionConstraint<T>,
        xf_a: &Transform<T>,
        xf_b: &Transform<T>,
        index: usize,
    ) -> PositionSolverManifold<T> {
        assert!(pc.point_count > 0);

        match pc.type_ {
            ManifoldType::Circles => {
                let point_a = xf_a.multiply(pc.local_point);
                let point_b = xf_b.multiply(pc.local_points[0]);
                let normal = (point_b - point_a).normalize();
                let point = (point_a + point_b) * T::half();
                let separation = (point_b - point_a).dot(normal) - pc.radius_a - pc.radius_b;
                PositionSolverManifold {
                    normal,
                    point,
                    separation,
                }
            }
            ManifoldType::FaceA => {
                let normal = xf_a.q.transpose_multiply(pc.local_normal);
                let plane_point = xf_a.multiply(pc.local_point);
                let clip_point = xf_b.multiply(pc.local_points[index]);
                let separation = (clip_point - plane_point).dot(normal) - pc.radius_a - pc.radius_b;
                let point = clip_point;
                PositionSolverManifold {
                    normal,
                    point,
                    separation,
                }
            }
            ManifoldType::FaceB => {
                let normal = xf_b.q.transpose_multiply(pc.local_normal);
                let plane_point = xf_b.multiply(pc.local_point);
                let clip_point = xf_a.multiply(pc.local_points[index]);
                let separation = (clip_point - plane_point).dot(normal) - pc.radius_a - pc.radius_b;
                let point = clip_point;
                PositionSolverManifold {
                    normal,
                    point,
                    separation,
                }
            }
        }
    }
}
