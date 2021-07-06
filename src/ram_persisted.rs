use core::mem::MaybeUninit;
use core::sync::atomic::AtomicU32;
use core::sync::atomic::Ordering;

pub struct RamPersisted<T> {
    bytes: MaybeUninit<T>,
    magic: AtomicU32,
    fully_inited: bool,
    checksum: [u8; 20],
}

fn to_checksum<T>(reference: &T) -> [u8; 20] {
    let opaque = reference as *const T as *const u8;

    let bytes = unsafe { core::slice::from_raw_parts(opaque, core::mem::size_of::<T>()) };

    use sha1::Digest;
    let mut hasher = sha1::Sha1::new();
    hasher.update(bytes);
    hasher.finalize().into()
}

pub enum Cached<T> {
    Cached(T),
    StillInitializing(T),
    Fresh(T),
}

impl<T> RamPersisted<T> {
    pub const fn new() -> Self {
        RamPersisted {
            bytes: MaybeUninit::uninit(),
            magic: AtomicU32::new(0),
            fully_inited: false,
            checksum: [0; 20],
        }
    }

    pub async fn async_take_with_default<F, G>(
        &mut self,
        init_from_scratch: F,
    ) -> Cached<&'static mut T>
    where
        G: core::future::Future<Output = T>,
        F: FnOnce() -> G,
        T: 'static,
    {
        if (self.magic)
            .compare_exchange(MAGIC, 0, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
        {
            // value is initialized, magic word is cleared
            let reference = unsafe { &mut *self.bytes.as_mut_ptr() };

            let ck = to_checksum(reference);

            if ck == self.checksum {
                // core::mem::forget any peripherals that would normally be needed
                core::mem::forget(init_from_scratch);

                self.checksum = Default::default();

                if self.fully_inited {
                    return Cached::Cached(reference);
                } else {
                    return Cached::StillInitializing(reference);
                }
            }
        }

        // defmt::info!("no sensor stored, making a new one",);

        self.magic.store(0, Ordering::SeqCst);

        let ptr = self.bytes.as_mut_ptr();

        let value = init_from_scratch().await;

        unsafe { ptr.write(value) };

        let reference = unsafe { &mut *ptr };

        self.checksum = to_checksum(reference);

        Cached::Fresh(reference)
    }

    pub fn take_with_default<F>(&mut self, init_from_scratch: F) -> Cached<&'static mut T>
    where
        F: FnOnce() -> T,
        T: 'static,
    {
        if (self.magic)
            .compare_exchange(MAGIC, 0, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
        {
            // value is initialized, magic word is cleared
            let reference = unsafe { &mut *self.bytes.as_mut_ptr() };

            let ck = to_checksum(reference);

            if ck == self.checksum {
                // core::mem::forget any peripherals that would normally be needed
                core::mem::forget(init_from_scratch);

                self.checksum = Default::default();

                if self.fully_inited {
                    return Cached::Cached(reference);
                } else {
                    return Cached::StillInitializing(reference);
                }
            }
        }

        // defmt::info!("no sensor stored, making a new one",);

        self.magic.store(0, Ordering::SeqCst);

        let ptr = self.bytes.as_mut_ptr();

        let value = init_from_scratch();

        unsafe { ptr.write(value) };

        let reference = unsafe { &mut *ptr };

        self.checksum = to_checksum(reference);

        Cached::Fresh(reference)
    }

    /// # SAFETY
    ///
    /// the given out mutable pointer should not be used at this point
    pub unsafe fn unsafe_put_back(&mut self) {
        match self
            .magic
            .compare_exchange(0, MAGIC, Ordering::SeqCst, Ordering::SeqCst)
        {
            Ok(_) => {
                self.checksum = to_checksum(&*self.bytes.as_ptr());
            }
            Err(value) => {
                panic!("put back a pointer, but it was not given out {:x}", value);
            }
        }
    }

    /// SAFETY: the given out mutable pointer should not be used at this point
    unsafe fn unsafe_put_back_init(&mut self) {
        match self
            .magic
            .compare_exchange(0, MAGIC, Ordering::SeqCst, Ordering::SeqCst)
        {
            Ok(_) => {
                self.checksum = to_checksum(&*self.bytes.as_ptr());
                self.fully_inited = true;
            }
            Err(value) => {
                panic!("put back a pointer, but it was not given out {:x}", value);
            }
        }
    }

    pub fn put_back_init<'a>(&mut self, _: &'a mut T) {
        // SAFETY: since we own the given-out mutable pointer, it cannot be used any more
        unsafe { self.unsafe_put_back_init() }
    }
}

/// Magic word
const MAGIC: u32 = 0xDEADBEEF;
