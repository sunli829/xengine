use crate::dynamic::contact_manager::ContactManager;
use crate::Body;
bitflags! {
    struct WorldFlags: u32 {
        const NEW_FIXTURE = 0x0001;
        const LOCKED = 0x0002;
        const CLEAR_FORCES = 0x0004;
    }
}

pub(crate) struct WorldInner<T, D> {
    flags: WorldFlags,
    contact_manager: ContactManager<T, D>,
    body_list: *mut Body<T, D>,
}

pub struct World<T, D>(Box<WorldInner<T, D>>);
