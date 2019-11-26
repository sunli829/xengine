use crate::dynamic::contacts::{
    Contact, ContactEdge, ContactFilter, ContactFlags, ContactListener, DefaultContactFilter,
};
use crate::dynamic::fixture::FixtureProxy;
use crate::{BodyType, BroadPhase};
use xmath::Real;

pub(crate) struct ContactManager<T, D> {
    pub(crate) broad_phase: BroadPhase<T, *mut FixtureProxy<T, D>>,
    pub(crate) contact_list: *mut Contact<T, D>,
    pub(crate) contact_count: usize,
    pub(crate) contact_filter: Box<dyn ContactFilter<T, D>>,
    pub(crate) contact_listener: Option<Box<dyn ContactListener<T, D>>>,
}

impl<T: Real, D> ContactManager<T, D> {
    pub fn new() -> ContactManager<T, D> {
        ContactManager {
            broad_phase: BroadPhase::new(),
            contact_list: std::ptr::null_mut(),
            contact_count: 0,
            contact_filter: Box::new(DefaultContactFilter),
            contact_listener: None,
        }
    }

    pub fn destroy(&mut self, c: *mut Contact<T, D>) {
        unsafe {
            let fixture_a = (*c).fixture_a_ptr;
            let fixture_b = (*c).fixture_b_ptr;
            let body_a = (*fixture_a).body_ptr;
            let body_b = (*fixture_b).body_ptr;

            if let Some(listener) = &mut self.contact_listener {
                if (*c).is_touching() {
                    listener.end_contact(c.as_mut().unwrap());
                }
            }

            if !(*c).prev_ptr.is_null() {
                (*(*c).prev_ptr).next_ptr = (*c).next_ptr;
            }

            if !(*c).next_ptr.is_null() {
                (*(*c).next_ptr).prev_ptr = (*c).prev_ptr;
            }

            if c == self.contact_list {
                self.contact_list = (*c).next_ptr;
            }

            if !(*c).node_a.prev_ptr.is_null() {
                (*(*c).node_a.prev_ptr).next_ptr = (*c).node_a.next_ptr;
            }

            if !(*c).node_a.next_ptr.is_null() {
                (*(*c).node_a.next_ptr).prev_ptr = (*c).node_a.prev_ptr;
            }

            if &mut (*c).node_a as *mut ContactEdge<T, D> == (*body_a).contact_list_ptr {
                (*body_a).contact_list_ptr = (*c).node_a.next_ptr;
            }

            if !(*c).node_b.prev_ptr.is_null() {
                (*(*c).node_b.prev_ptr).next_ptr = (*c).node_b.next_ptr;
            }

            if !(*c).node_b.next_ptr.is_null() {
                (*(*c).node_b.next_ptr).prev_ptr = (*c).node_b.prev_ptr;
            }

            if &mut (*c).node_b as *mut ContactEdge<T, D> == (*body_b).contact_list_ptr {
                (*body_b).contact_list_ptr = (*c).node_b.next_ptr;
            }

            Contact::destroy(c);
            self.contact_count -= 1;
        }
    }

    pub fn find_new_contacts(&mut self) {
        unsafe {
            let broad_phase = &mut self.broad_phase as *mut BroadPhase<T, *mut FixtureProxy<T, D>>;
            (*broad_phase).update_pairs(|proxy_a, proxy_b| {
                let fixture_a = (*(*proxy_a)).fixture_ptr;
                let fixture_b = (*(*proxy_b)).fixture_ptr;
                let index_a = (*(*proxy_a)).child_index;
                let index_b = (*(*proxy_b)).child_index;
                let body_a = (*fixture_a).body_ptr;
                let body_b = (*fixture_b).body_ptr;

                if body_a == body_b {
                    return;
                }

                let mut edge = (*body_b).contact_list_ptr;
                while !edge.is_null() {
                    if (*edge).other_ptr == body_a {
                        let fa = (*(*edge).contact_ptr).fixture_a_ptr;
                        let fb = (*(*edge).contact_ptr).fixture_b_ptr;
                        let ia = (*(*edge).contact_ptr).child_index_a();
                        let ib = (*(*edge).contact_ptr).child_index_b();

                        if fa == fixture_a && fb == fixture_b && ia == index_a && ib == index_b {
                            return;
                        }

                        if fa == fixture_b && fb == fixture_a && ia == index_b && ib == index_a {
                            return;
                        }
                    }

                    edge = (*edge).next_ptr;
                }

                if !(*body_b).should_collide(body_a.as_ref().unwrap()) {
                    return;
                }

                if self
                    .contact_filter
                    .should_collide(fixture_a.as_ref().unwrap(), fixture_b.as_ref().unwrap())
                {
                    return;
                }

                let c = Contact::new(fixture_a, index_a, fixture_b, index_b);
                let fixture_a = (*c).fixture_a_ptr;
                let fixture_b = (*c).fixture_b_ptr;
                let body_a = (*fixture_a).body_ptr;
                let body_b = (*fixture_b).body_ptr;

                (*c).prev_ptr = std::ptr::null_mut();
                (*c).next_ptr = self.contact_list;
                if !self.contact_list.is_null() {
                    (*self.contact_list).prev_ptr = c;
                }
                self.contact_list = c;

                (*c).node_a.contact_ptr = c;
                (*c).node_a.other_ptr = body_b;

                (*c).node_a.prev_ptr = std::ptr::null_mut();
                (*c).node_a.next_ptr = (*body_a).contact_list_ptr;
                if !(*body_a).contact_list_ptr.is_null() {
                    (*(*body_a).contact_list_ptr).prev_ptr = &mut (*c).node_a;
                }
                (*body_a).contact_list_ptr = &mut (*c).node_a;

                (*c).node_b.contact_ptr = c;
                (*c).node_b.other_ptr = body_a;

                (*c).node_b.prev_ptr = std::ptr::null_mut();
                (*c).node_b.next_ptr = (*body_b).contact_list_ptr;
                if !(*body_b).contact_list_ptr.is_null() {
                    (*(*body_b).contact_list_ptr).prev_ptr = &mut (*c).node_b;
                }
                (*body_b).contact_list_ptr = &mut (*c).node_b;

                if !(*fixture_a).is_sensor() && !(*fixture_b).is_sensor() {
                    (*body_a).set_awake(true);
                    (*body_b).set_awake(true);
                }

                self.contact_count += 1;
            });
        }
    }

    pub fn collide(&mut self) {
        unsafe {
            let mut c = self.contact_list;
            while !c.is_null() {
                let fixture_a = (*c).fixture_a_ptr;
                let fixture_b = (*c).fixture_a_ptr;
                let index_a = (*c).child_index_a();
                let index_b = (*c).child_index_b();
                let body_a = (*fixture_a).body_ptr;
                let body_b = (*fixture_b).body_ptr;

                if (*c).flags.contains(ContactFlags::FILTER) {
                    if !(*body_b).should_collide(body_a.as_ref().unwrap()) {
                        let cnuke = c;
                        c = (*cnuke).next_ptr;
                        self.destroy(cnuke);
                        continue;
                    }

                    if !self
                        .contact_filter
                        .should_collide(fixture_a.as_ref().unwrap(), fixture_b.as_ref().unwrap())
                    {
                        let cnuke = c;
                        c = (*cnuke).next_ptr;
                        self.destroy(cnuke);
                        continue;
                    }

                    (*c).flags.remove(ContactFlags::FILTER);
                }

                let active_a = (*body_a).is_awake() && (*body_a).body_type() != BodyType::Static;
                let active_b = (*body_b).is_awake() && (*body_b).body_type() != BodyType::Static;

                if !active_a && !active_b {
                    c = (*c).next_ptr;
                    continue;
                }

                let proxy_id_a = (*fixture_a).proxies[index_a].proxy_id;
                let proxy_id_b = (*fixture_b).proxies[index_b].proxy_id;
                let overlap = self.broad_phase.test_overlap(proxy_id_a, proxy_id_b);

                if !overlap {
                    let cnuke = c;
                    c = (*cnuke).next_ptr;
                    self.destroy(cnuke);
                    continue;
                }

                (*c).update(&self.contact_listener);
                c = (*c).next_ptr;
            }
        }
    }
}
