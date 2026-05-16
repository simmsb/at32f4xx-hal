use core::task::Poll;

use cortex_m::peripheral::NVIC;
use defmt::unreachable;
use embassy_sync::waitqueue::AtomicWaker;

use crate::gpio::{ExtiPin, ReadPin};
use crate::pac::EXINT;
use crate::pac::interrupt;

pub struct ExtiChannel<const CHANNEL: u8>;

impl<const CHANNEL: u8> ExtiChannel<CHANNEL> {
    const fn new() -> Self {
        Self
    }
}

pub trait ExtiExt {
    type Parts;

    fn split(self) -> Parts;
}

macro_rules! exti {
    ($EXTIX:ident, [$($channel:literal),*]) => {
        paste::paste! {
            pub struct Parts {
                $(
                  pub [<ch $channel>]: ExtiChannel<$channel>,
                )*
            }

            impl ExtiExt for $EXTIX {
                type Parts = Parts;

                fn split(self) -> Parts {
                    // any clocks we need here?

                    Parts {
                        $(
                            [<ch $channel>]: ExtiChannel::<$channel>::new(),
                        )*
                    }
                }
            }
        }
    };
}

exti!(
    EXINT,
    [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
);

pub struct ExtiInput<P, const CHANNEL: u8> {
    pin: P,
}

impl<P: ReadPin, const CHANNEL: u8> ExtiInput<P, CHANNEL> {
    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }

    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }
}

impl<const CHANNEL: u8, P: ExtiPin<CHANNEL> + ReadPin> ExtiInput<P, CHANNEL> {
    pub fn new(pin: P, _channel: ExtiChannel<CHANNEL>) -> Self {
        unmask_exti_int(CHANNEL);
        Self { pin }
    }

    pub async fn wait_for_high(&mut self) {
        let fut = ExtiInputFuture::new(CHANNEL, P::BITS, Edge::Rising);
        if self.is_high() {
            return;
        }
        fut.await
    }

    pub async fn wait_for_low(&mut self) {
        let fut = ExtiInputFuture::new(CHANNEL, P::BITS, Edge::Falling);
        if self.is_low() {
            return;
        }
        fut.await
    }

    pub async fn wait_for_rising(&mut self) {
        ExtiInputFuture::new(CHANNEL, P::BITS, Edge::Rising).await
    }

    pub async fn wait_for_falling(&mut self) {
        ExtiInputFuture::new(CHANNEL, P::BITS, Edge::Falling).await
    }

    pub async fn wait_for_any_edge(&mut self) {
        ExtiInputFuture::new(CHANNEL, P::BITS, Edge::Any).await
    }
}

#[derive(Clone, Copy, Debug)]
enum Edge {
    Rising,
    Falling,
    Any,
}

pub struct ExtiInputFuture {
    pin_num: u8,
}

impl ExtiInputFuture {
    fn new(pin_num: u8, port: u8, edge: Edge) -> Self {
        critical_section::with(|_| {
            configure_exti(pin_num, port, edge);
            enable_exti_interrupt(pin_num, true);
        });

        Self { pin_num }
    }
}

impl Drop for ExtiInputFuture {
    fn drop(&mut self) {
        critical_section::with(|_| {
            let exti = unsafe { crate::pac::EXINT::steal() };

            exti.inten().modify(|_, w| w.inten(self.pin_num).disable());
        });
    }
}

impl Future for ExtiInputFuture {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let exti = unsafe { crate::pac::EXINT::steal() };

        EXTI_WAKERS[self.pin_num as usize].register(cx.waker());

        if exti.inten().read().inten(self.pin_num).is_enabled() {
            Poll::Pending
        } else {
            Poll::Ready(())
        }
    }
}

const EXTI_COUNT: usize = 16;
static EXTI_WAKERS: [AtomicWaker; EXTI_COUNT] = [const { AtomicWaker::new() }; EXTI_COUNT];

struct BitIter(u32);

impl Iterator for BitIter {
    type Item = u32;

    fn next(&mut self) -> Option<Self::Item> {
        match self.0.trailing_zeros() {
            32 => None,
            b => {
                self.0 &= !(1 << b);
                Some(b)
            }
        }
    }
}

fn on_interrupt() {
    let exti = unsafe { crate::pac::EXINT::steal() };
    let bits = exti.intsts().read().bits();

    let bits = bits & 0x0000FFFF;

    exti.inten()
        .modify(|r, w| unsafe { w.bits(r.bits() & !bits) });

    for i in BitIter(bits) {
        EXTI_WAKERS[i as usize].wake();
    }

    exti.intsts().write(|w| unsafe { w.bits(bits) });
}

fn configure_exti(pin: u8, port: u8, trigger: Edge) {
    critical_section::with(|_| {
        let iomux = unsafe { crate::pac::IOMUX::steal() };
        let exti = unsafe { crate::pac::EXINT::steal() };

        match pin / 4 {
            0 => iomux
                .exintc1()
                .modify(|_, w| unsafe { w.exint(pin % 4).bits(port) }),
            1 => iomux
                .exintc2()
                .modify(|_, w| unsafe { w.exint(pin % 4).bits(port) }),
            2 => iomux
                .exintc3()
                .modify(|_, w| unsafe { w.exint(pin % 4).bits(port) }),
            3 => iomux
                .exintc4()
                .modify(|_, w| unsafe { w.exint(pin % 4).bits(port) }),
            _ => unreachable!(),
        };

        let (rising, falling) = match trigger {
            Edge::Falling => (false, true),
            Edge::Rising => (true, false),
            Edge::Any => (true, true),
        };

        exti.polcfg1().modify(|_, w| w.rp(pin).bit(rising));
        exti.polcfg2().modify(|_, w| w.fp(pin).bit(falling));

        exti.intsts().write(|w| unsafe { w.bits(1 << pin) });
    });
}

fn enable_exti_interrupt(pin: u8, enabled: bool) {
    critical_section::with(|_| {
        let exti = unsafe { crate::pac::EXINT::steal() };

        exti.inten().modify(|_, w| w.inten(pin).bit(enabled));
    })
}

macro_rules! exti_int {
    ($($NAME:ident,)*) => {
        $(
            #[interrupt]
            fn $NAME() {
                on_interrupt();
            }
        )*
    };
}

fn unmask_exti_int(n: u8) {
    let intr = match n {
        0 => crate::interrupt::EXTINT0,
        1 => crate::interrupt::EXTINT1,
        2 => crate::interrupt::EXTINT2,
        3 => crate::interrupt::EXTINT3,
        4 => crate::interrupt::EXTINT4,
        5..=9 => crate::interrupt::EXTINT9_5,
        10..=15 => crate::interrupt::EXTINT15_10,
        _ => return,
    };

    NVIC::unpend(intr);
    unsafe {
        NVIC::unmask(intr);
    }
}

exti_int!(
    EXTINT0,
    EXTINT1,
    EXTINT2,
    EXTINT3,
    EXTINT4,
    EXTINT9_5,
    EXTINT15_10,
);
