use core::convert::Infallible;
use core::future::Future;
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::AtomicPtr;
use core::sync::atomic::Ordering;
use core::task::Poll;

use embassy::interrupt::InterruptExt;
use embassy::traits;
use embassy::util::Unborrow;
use embassy::waitqueue::AtomicWaker;
use embassy_hal_common::drop::OnDrop;
use embassy_hal_common::unborrow;
use futures::future::poll_fn;
use rand_core::RngCore;

use crate::interrupt;
use crate::pac;
use crate::peripherals::RNG;

impl RNG {
    fn regs() -> &'static pac::rng::RegisterBlock {
        unsafe { &*pac::RNG::ptr() }
    }
}

static STATE: State = State {
    ptr: AtomicPtr::new(ptr::null_mut()),
    end: AtomicPtr::new(ptr::null_mut()),
    waker: AtomicWaker::new(),
};

struct State {
    ptr: AtomicPtr<u8>,
    end: AtomicPtr<u8>,
    waker: AtomicWaker,
}

/// A wrapper around an nRF RNG peripheral.
///
/// It has a non-blocking API, through `embassy::traits::Rng`, and a blocking api through `rand`.
pub struct Rng<'d> {
    irq: interrupt::RNG,
    phantom: PhantomData<(&'d mut RNG, &'d mut interrupt::RNG)>,
}

impl<'d> Rng<'d> {
    /// Creates a new RNG driver from the `RNG` peripheral and interrupt.
    ///
    /// SAFETY: The future returned from `fill_bytes` must not have its lifetime end without running its destructor,
    /// e.g. using `mem::forget`.
    ///
    /// The synchronous API is safe.
    pub unsafe fn new(
        _rng: impl Unborrow<Target = RNG> + 'd,
        irq: impl Unborrow<Target = interrupt::RNG> + 'd,
    ) -> Self {
        unborrow!(irq);

        let this = Self {
            irq,
            phantom: PhantomData,
        };

        this.stop();
        this.disable_irq();

        this.irq.set_handler(Self::on_interrupt);
        this.irq.unpend();
        this.irq.enable();

        this
    }

    fn on_interrupt(_: *mut ()) {
        // Clear the event.
        RNG::regs().events_valrdy.reset();

        // Mutate the slice within a critical section,
        // so that the future isn't dropped in between us loading the pointer and actually dereferencing it.
        let (ptr, end) = critical_section::with(|_| {
            let ptr = STATE.ptr.load(Ordering::Relaxed);
            // We need to make sure we haven't already filled the whole slice,
            // in case the interrupt fired again before the executor got back to the future.
            let end = STATE.end.load(Ordering::Relaxed);
            if !ptr.is_null() && ptr != end {
                // If the future was dropped, the pointer would have been set to null,
                // so we're still good to mutate the slice.
                // The safety contract of `Rng::new` means that the future can't have been dropped
                // without calling its destructor.
                unsafe {
                    *ptr = RNG::regs().value.read().value().bits();
                }
            }
            (ptr, end)
        });

        if ptr.is_null() || ptr == end {
            // If the future was dropped, there's nothing to do.
            // If `ptr == end`, we were called by mistake, so return.
            return;
        }

        let new_ptr = unsafe { ptr.add(1) };
        match STATE
            .ptr
            .compare_exchange(ptr, new_ptr, Ordering::Relaxed, Ordering::Relaxed)
        {
            Ok(_) => {
                let end = STATE.end.load(Ordering::Relaxed);
                // It doesn't matter if `end` was changed under our feet, because then this will just be false.
                if new_ptr == end {
                    STATE.waker.wake();
                }
            }
            Err(_) => {
                // If the future was dropped or finished, there's no point trying to wake it.
                // It will have already stopped the RNG, so there's no need to do that either.
            }
        }
    }

    fn stop(&self) {
        RNG::regs().tasks_stop.write(|w| unsafe { w.bits(1) })
    }

    fn start(&self) {
        RNG::regs().tasks_start.write(|w| unsafe { w.bits(1) })
    }

    fn enable_irq(&self) {
        RNG::regs().intenset.write(|w| w.valrdy().set());
    }

    fn disable_irq(&self) {
        RNG::regs().intenclr.write(|w| w.valrdy().clear());
    }

    /// Enable or disable the RNG's bias correction.
    ///
    /// Bias correction removes any bias towards a '1' or a '0' in the bits generated.
    /// However, this makes the generation of numbers slower.
    ///
    /// Defaults to disabled.
    pub fn bias_correction(&self, enable: bool) {
        RNG::regs().config.write(|w| w.dercen().bit(enable))
    }
}

impl<'d> Drop for Rng<'d> {
    fn drop(&mut self) {
        self.irq.disable()
    }
}

impl<'d> traits::rng::Rng for Rng<'d> {
    type Error = Infallible;

    #[rustfmt::skip] // For some reason rustfmt removes the where clause
    type RngFuture<'a> where 'd: 'a = impl Future<Output = Result<(), Self::Error>> + 'a;

    fn fill_bytes<'a>(&'a mut self, dest: &'a mut [u8]) -> Self::RngFuture<'a> {
        async move {
            if dest.len() == 0 {
                return Ok(()); // Nothing to fill
            }

            let range = dest.as_mut_ptr_range();
            // Even if we've preempted the interrupt, it can't preempt us again,
            // so we don't need to worry about the order we write these in.
            STATE.ptr.store(range.start, Ordering::Relaxed);
            STATE.end.store(range.end, Ordering::Relaxed);

            self.enable_irq();
            self.start();

            let on_drop = OnDrop::new(|| {
                self.stop();
                self.disable_irq();

                // The interrupt is now disabled and can't preempt us anymore, so the order doesn't matter here.
                STATE.ptr.store(ptr::null_mut(), Ordering::Relaxed);
                STATE.end.store(ptr::null_mut(), Ordering::Relaxed);
            });

            poll_fn(|cx| {
                STATE.waker.register(cx.waker());

                // The interrupt will never modify `end`, so load it first and then get the most up-to-date `ptr`.
                let end = STATE.end.load(Ordering::Relaxed);
                let ptr = STATE.ptr.load(Ordering::Relaxed);

                if ptr == end {
                    // We're done.
                    Poll::Ready(())
                } else {
                    Poll::Pending
                }
            })
            .await;

            // Trigger the teardown
            drop(on_drop);

            Ok(())
        }
    }
}

impl<'d> RngCore for Rng<'d> {
    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.start();

        for byte in dest.iter_mut() {
            let regs = RNG::regs();
            while regs.events_valrdy.read().bits() == 0 {}
            regs.events_valrdy.reset();
            *byte = regs.value.read().value().bits();
        }

        self.stop();
    }

    fn next_u32(&mut self) -> u32 {
        let mut bytes = [0; 4];
        self.fill_bytes(&mut bytes);
        // We don't care about the endianness, so just use the native one.
        u32::from_ne_bytes(bytes)
    }

    fn next_u64(&mut self) -> u64 {
        let mut bytes = [0; 8];
        self.fill_bytes(&mut bytes);
        u64::from_ne_bytes(bytes)
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

// TODO: Should `Rng` implement `CryptoRng`? It's 'suitable for cryptographic purposes' according to the specification.
