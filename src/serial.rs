//!
//! Asynchronous serial communication using USART peripherals
//!
//! # Word length
//!
//! By default, the UART/USART uses 8 data bits. The `Serial`, `Rx`, and `Tx` structs implement
//! the embedded-hal read and write traits with `u8` as the word type.
//!
//! You can also configure the hardware to use 9 data bits with the `Config` `wordlength_9()`
//! function. After creating a `Serial` with this option, use the `with_u16_data()` function to
//! convert the `Serial<_, u8>` object into a `Serial<_, u16>` that can send and receive `u16`s.
//!
//! In this mode, the `Serial<_, u16>`, `Rx<_, u16>`, and `Tx<_, u16>` structs instead implement
//! the embedded-hal read and write traits with `u16` as the word type. You can use these
//! implementations for 9-bit words.

use core::{
    marker::PhantomData, ops::Deref, sync::atomic::{Ordering, compiler_fence}, task::Poll
};

mod hal;

use super::interrupt;

pub(crate) mod uart_impls;
use embassy_sync::waitqueue::AtomicWaker;
pub use uart_impls::Instance;
use uart_impls::RegisterBlockImpl;

use crate::gpio::{self, Input, PushPull};

use crate::pac;

use crate::crm::Clocks;
use crate::gpio::NoPin;

use pac::NVIC;

/// Serial error
pub use embedded_hal_nb::serial::ErrorKind as Error;

static USART1_STATE: State = State {
    rx_waker: AtomicWaker::new(),
    tx_waker: AtomicWaker::new(),
};
static USART2_STATE: State = State {
    rx_waker: AtomicWaker::new(),
    tx_waker: AtomicWaker::new(),
};
static USART3_STATE: State = State {
    rx_waker: AtomicWaker::new(),
    tx_waker: AtomicWaker::new(),
};
pub static UART4_STATE: State = State {
    rx_waker: AtomicWaker::new(),
    tx_waker: AtomicWaker::new(),
};
pub static UART5_STATE: State = State {
    rx_waker: AtomicWaker::new(),
    tx_waker: AtomicWaker::new(),
};

pub struct State {
    rx_waker: AtomicWaker,
    tx_waker: AtomicWaker,
}

#[interrupt]
fn USART1() {
    on_interrupt(unsafe { &*pac::USART1::ptr() }, &USART1_STATE);
}

#[interrupt]
fn USART2() {
    on_interrupt(unsafe { &*pac::USART2::ptr() }, &USART2_STATE);
}

#[interrupt]
fn USART3() {
    on_interrupt(unsafe { &*pac::USART3::ptr() }, &USART3_STATE);
}

#[interrupt]
fn UART4() {
    on_interrupt(unsafe { &*pac::UART4::ptr() }, &UART4_STATE);
}

#[interrupt]
fn UART5() {
    on_interrupt(unsafe { &*pac::UART5::ptr() }, &UART5_STATE);
}

fn on_interrupt(r: &pac::usart1::RegisterBlock, state: &State) {
    let (sr, cr1, cr3) = (r.sts().read(), r.ctrl1().read(), r.ctrl3().read());

    let has_errors = (sr.perr().is_error() && cr1.perrien().is_enabled())
        || ((sr.ferr().is_error() || sr.nerr().is_noise() || sr.roerr().is_overflow())
            && cr3.errien().is_enabled());

    if has_errors {
        // clear all interrupts and DMA Rx Request
        r.ctrl1().modify(|_, w| {
            // disable RXNE interrupt
            w.rdbfien().disable();
            // disable parity interrupt
            w.perrien().disable();
            // disable idle line interrupt
            w.idleien().disable();
            w
        });
        r.ctrl3().modify(|_, w| {
            // disable Error Interrupt: (Frame error, Noise error, Overrun error)
            w.errien().disable();
            // disable DMA Rx Request
            w.dmaren().disable();
            w
        });
    }

    if cr1.idleien().is_enabled() && sr.idlef().is_idle() {
        // IDLE detected: no more data will come
        r.ctrl1().modify(|_, w| {
            // disable idle line detection
            w.idleien().disable();
            w
        });
    }

    if cr1.tdcien().is_enabled() && sr.tdc().is_completed() {
        // Transmission complete detected
        r.ctrl1().modify(|_, w| {
            // disable Transmission complete interrupt
            w.tdcien().disable();
            w
        });
    }

    if cr1.tdbeien().is_enabled() && sr.tdbe().is_empty() {
        // Transmission complete detected
        r.ctrl1().modify(|_, w| {
            // disable Transmission complete interrupt
            w.tdbeien().disable();
            w
        });
    }

    if cr1.rdbfien().is_enabled() && sr.rdbf().is_full() {
        r.ctrl1().modify(|_, w| {
            w.rdbfien().disable();
            w
        });
    }

    // defmt::debug!("uart intr: {:b}", sr.bits());

    compiler_fence(Ordering::SeqCst);
    state.rx_waker.wake();
    state.tx_waker.wake();
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// Idle line state detected
    Idle,
}

pub mod config;

pub use config::Config;

/// A filler type for when the Tx pin is unnecessary
pub use gpio::NoPin as NoTx;
/// A filler type for when the Rx pin is unnecessary
pub use gpio::NoPin as NoRx;

pub use gpio::alt::SerialAsync as CommonPins;

/// Trait for [`Rx`] interrupt handling.
pub trait RxISR {
    /// Return true if the line idle status is set
    fn is_idle(&self) -> bool;

    /// Return true if the rx register is not empty (and can be read)
    fn is_rx_not_empty(&self) -> bool;

    /// Clear idle line interrupt flag
    fn clear_idle_interrupt(&self);
}

/// Trait for [`Tx`] interrupt handling.
pub trait TxISR {
    /// Return true if the tx register is empty (and can accept data)
    fn is_tx_empty(&self) -> bool;
}

/// Trait for listening [`Rx`] interrupt events.
pub trait RxListen {
    /// Start listening for an rx not empty interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    fn listen(&mut self);

    /// Stop listening for the rx not empty interrupt event
    fn unlisten(&mut self);

    /// Start listening for a line idle interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    fn listen_idle(&mut self);

    /// Stop listening for the line idle interrupt event
    fn unlisten_idle(&mut self);
}

/// Trait for listening [`Tx`] interrupt event.
pub trait TxListen {
    /// Start listening for a tx empty interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    fn listen(&mut self);

    /// Stop listening for the tx empty interrupt event
    fn unlisten(&mut self);
}

/// Trait for listening [`Serial`] interrupt events.
pub trait Listen {
    /// Starts listening for an interrupt event
    ///
    /// Note, you will also have to enable the corresponding interrupt
    /// in the NVIC to start receiving events.
    fn listen(&mut self, event: Event);

    /// Stop listening for an interrupt event
    fn unlisten(&mut self, event: Event);
}

/// Serial abstraction
pub struct Serial<USART: CommonPins, WORD = u8> {
    tx: Tx<USART, WORD>,
    rx: Rx<USART, WORD>,
}

/// Serial receiver containing RX pin
pub struct Rx<USART: CommonPins, WORD = u8> {
    _word: PhantomData<(USART, WORD)>,
    pin: USART::Rx<Input>,
}

/// Serial transmitter containing TX pin
pub struct Tx<USART: CommonPins, WORD = u8> {
    _word: PhantomData<WORD>,
    usart: USART,
    pin: USART::Tx<PushPull>,
}

pub trait SerialExt: Sized + Instance {
    fn serial<WORD>(
        self,
        pins: (impl Into<Self::Tx<PushPull>>, impl Into<Self::Rx<Input>>),
        config: impl Into<config::Config>,
        clocks: &Clocks,
    ) -> Result<Serial<Self, WORD>, config::InvalidConfig>;

    fn tx<WORD>(
        self,
        tx_pin: impl Into<Self::Tx<PushPull>>,
        config: impl Into<config::Config>,
        clocks: &Clocks,
    ) -> Result<Tx<Self, WORD>, config::InvalidConfig>
    where
        NoPin: Into<Self::Rx<Input>>;

    fn rx<WORD>(
        self,
        rx_pin: impl Into<Self::Rx<Input>>,
        config: impl Into<config::Config>,
        clocks: &Clocks,
    ) -> Result<Rx<Self, WORD>, config::InvalidConfig>
    where
        NoPin: Into<Self::Tx<PushPull>>;
}

impl<USART: Instance, WORD> Serial<USART, WORD> {
    pub fn new(
        usart: USART,
        pins: (
            impl Into<USART::Tx<PushPull>>,
            impl Into<USART::Rx<Input>>,
        ),
        config: impl Into<config::Config>,
        clocks: &Clocks,
    ) -> Result<Self, config::InvalidConfig>
    where
        <USART as Instance>::RegisterBlock: uart_impls::RegisterBlockImpl,
    {
        <USART as Instance>::RegisterBlock::new(usart, pins, config, clocks)
    }
}
// impl<USART: Instance> embedded_io_async::ErrorType for Rx<USART, u8> {
//     type Error = ();
// }

impl<USART: Instance> embedded_io_async::Read for Rx<USART, u8>
where
    <USART as Instance>::RegisterBlock: RegisterBlockImpl,
{
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        if buf.len() == 0 {
            return Ok(0);
        }

        core::future::poll_fn(|cx| {
            USART::STATE.rx_waker.register(cx.waker());
            <Self as RxListen>::listen(self);

            if let Ok(b) = <Self as embedded_hal_nb::serial::Read>::read(self) {
                buf[0] = b;
                return Poll::Ready(Ok(1));
            }

            defmt::trace!("uart listen pend");

            Poll::Pending
        })
        .await
    }
}


impl<USART: Instance> embedded_io_async::Write for Tx<USART, u8>
where
    <USART as Instance>::RegisterBlock: RegisterBlockImpl,
    USART: Deref<Target = <USART as Instance>::RegisterBlock>,
{
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        if buf.len() == 0 {
            return Ok(0);
        }

        core::future::poll_fn(|cx| {
            USART::STATE.tx_waker.register(cx.waker());
            <Self as TxListen>::listen(self);

            if let Ok(b) = <Self as embedded_hal_nb::serial::Write>::write(self, buf[0]) {
                return Poll::Ready(Ok(1));
            }

            defmt::trace!("uart write pend");

            Poll::Pending
        })
        .await
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl<UART: CommonPins, WORD> Serial<UART, WORD> {
    pub fn split(self) -> (Tx<UART, WORD>, Rx<UART, WORD>) {
        (self.tx, self.rx)
    }

    #[allow(clippy::type_complexity)]
    pub fn release(self) -> (UART, (UART::Tx<PushPull>, UART::Rx<Input>)) {
        (self.tx.usart, (self.tx.pin, self.rx.pin))
    }
}

macro_rules! halUsart {
    ($USART:ty, $Serial:ident, $Rx:ident, $Tx:ident, $state:ident, $intr:ident) => {
        pub type $Serial<WORD = u8> = Serial<$USART, WORD>;
        pub type $Rx<WORD = u8> = Rx<$USART, WORD>;
        pub type $Tx<WORD = u8> = Tx<$USART, WORD>;

        impl Instance for $USART {
            const STATE: &'static State = &$state;

            type RegisterBlock = crate::serial::uart_impls::RegisterBlockUsart;

            fn setup_interrupts() {
                defmt::trace!("Setting up uart interrupts");
                NVIC::unpend(interrupt::$intr);
                unsafe { NVIC::unmask(interrupt::$intr) };
            }

            fn ptr() -> *const crate::serial::uart_impls::RegisterBlockUsart {
                <$USART>::ptr() as *const _
            }

            fn set_stopbits(&self, bits: config::StopBits) {
                use crate::pac::usart1::ctrl2::STOPBN_A;
                use config::StopBits;

                self.ctrl2().write(|w| {
                    w.stopbn().variant(match bits {
                        StopBits::STOP0P5 => STOPBN_A::Bit05,
                        StopBits::STOP1 => STOPBN_A::Bit1,
                        StopBits::STOP1P5 => STOPBN_A::Bit15,
                        StopBits::STOP2 => STOPBN_A::Bit2,
                    })
                });
            }
        }
    };
}

halUsart! { pac::USART1, Serial1, Rx1, Tx1, USART1_STATE, USART1 }
halUsart! { pac::USART2, Serial2, Rx2, Tx2, USART2_STATE, USART2 }

#[cfg(feature = "usart3")]
halUsart! { pac::USART3, Serial3, Rx3, Tx3, USART3_STATE, USART3 }

impl<UART: CommonPins> Rx<UART, u8> {
    pub(crate) fn with_u16_data(self) -> Rx<UART, u16> {
        Rx::new(self.pin)
    }
}

impl<UART: CommonPins> Rx<UART, u16> {
    pub(crate) fn with_u8_data(self) -> Rx<UART, u8> {
        Rx::new(self.pin)
    }
}

impl<UART: CommonPins> Tx<UART, u8> {
    pub(crate) fn with_u16_data(self) -> Tx<UART, u16> {
        Tx::new(self.usart, self.pin)
    }
}

impl<UART: CommonPins> Tx<UART, u16> {
    pub(crate) fn with_u8_data(self) -> Tx<UART, u8> {
        Tx::new(self.usart, self.pin)
    }
}

impl<UART: CommonPins, WORD> Rx<UART, WORD> {
    pub(crate) fn new(pin: UART::Rx<Input>) -> Self {
        Self {
            _word: PhantomData,
            pin,
        }
    }

    pub fn join(self, tx: Tx<UART, WORD>) -> Serial<UART, WORD> {
        Serial { tx, rx: self }
    }
}

impl<UART: CommonPins, WORD> Tx<UART, WORD> {
    pub(crate) fn new(usart: UART, pin: UART::Tx<PushPull>) -> Self {
        Self {
            _word: PhantomData,
            usart,
            pin,
        }
    }

    pub fn join(self, rx: Rx<UART, WORD>) -> Serial<UART, WORD> {
        Serial { tx: self, rx }
    }
}

impl<UART: Instance, WORD> AsRef<Tx<UART, WORD>> for Serial<UART, WORD> {
    #[inline(always)]
    fn as_ref(&self) -> &Tx<UART, WORD> {
        &self.tx
    }
}

impl<UART: Instance, WORD> AsRef<Rx<UART, WORD>> for Serial<UART, WORD> {
    #[inline(always)]
    fn as_ref(&self) -> &Rx<UART, WORD> {
        &self.rx
    }
}

impl<UART: Instance, WORD> AsMut<Tx<UART, WORD>> for Serial<UART, WORD> {
    #[inline(always)]
    fn as_mut(&mut self) -> &mut Tx<UART, WORD> {
        &mut self.tx
    }
}

impl<UART: Instance, WORD> AsMut<Rx<UART, WORD>> for Serial<UART, WORD> {
    #[inline(always)]
    fn as_mut(&mut self) -> &mut Rx<UART, WORD> {
        &mut self.rx
    }
}

impl<UART: Instance> Serial<UART, u8> {
    /// Converts this Serial into a version that can read and write `u16` values instead of `u8`s
    ///
    /// This can be used with a word length of 9 bits.
    pub fn with_u16_data(self) -> Serial<UART, u16> {
        Serial {
            tx: self.tx.with_u16_data(),
            rx: self.rx.with_u16_data(),
        }
    }
}

impl<UART: Instance> Serial<UART, u16> {
    /// Converts this Serial into a version that can read and write `u8` values instead of `u16`s
    ///
    /// This can be used with a word length of 8 bits.
    pub fn with_u8_data(self) -> Serial<UART, u8> {
        Serial {
            tx: self.tx.with_u8_data(),
            rx: self.rx.with_u8_data(),
        }
    }
}
