use crate::dynamic::contacts::{Contact, ContactFilter, ContactListener};
use crate::dynamic::fixture::FixtureProxy;
use crate::{Body, BroadPhase, Fixture};
use xmath::Real;

pub(crate) struct ContactManager<T, D> {
    broad_phase: BroadPhase<T, *mut FixtureProxy<T, D>>,
    contact_list: *mut Contact<T, D>,
    contact_filter: Box<dyn ContactFilter<T, D>>,
    contact_listener: Box<dyn ContactListener<T, D> + 'static>,
}

impl<T: Real, D> ContactManager<T, D> {
    pub fn find_new_contacts(&mut self) {
        self.broad_phase.update_pairs(|proxy_a, proxy_b| unsafe {
            let fixture_a = (*(*proxy_a)).fixture;
            let fixture_b = (*(*proxy_a)).fixture;
            let index_a = (*(*proxy_a)).child_index;
            let index_b = (*(*proxy_a)).child_index;
            let body_a = (*fixture_a).body();
            let body_b = (*fixture_b).body();

            if body_a as *const Body<T, D> == body_b as *const Body<T, D> {
                return;
            }

            let mut edge = body_b.get_contact_list();
            while !edge.is_null() {
                if (*edge).other == body_a as *const Body<T, D> as *mut Body<T, D> {
                    let fa = (*(*edge).contact).fixture_a();
                    let fb = (*(*edge).contact).fixture_b();
                    let ia = (*(*edge).contact).child_index_a();
                    let ib = (*(*edge).contact).child_index_b();

                    if fa as *const Fixture<T, D> == fixture_a
                        && fb as *const Fixture<T, D> == fixture_b
                        && ia == index_a
                        && ib == index_b
                    {
                        return;
                    }

                    if fa as *const Fixture<T, D> == fixture_b
                        && fb as *const Fixture<T, D> == fixture_a
                        && ia == index_b
                        && ib == index_a
                    {
                        return;
                    }
                }

                edge = (*edge).next;
            }
        });
    }
}
