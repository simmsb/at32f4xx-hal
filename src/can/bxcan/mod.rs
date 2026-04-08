// pub mod filter;
mod registers;

use core::{future::poll_fn, marker::PhantomData};
use core::task::Poll;

use crate::{crm::{BusTimerClock, Clocks, Enable as _, Reset as _}, interrupt};

use at32f4xx_pac::at32f415::CAN1;
use cortex_m::peripheral::NVIC;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_can::{ExtendedId, Id, StandardId};

// use self::filter::MasterFilters;
use self::registers::{Registers, RxFifo};
pub use super::common::{BufferedCanReceiver, BufferedCanSender};
use super::frame::{Envelope, Frame};
use super::util;
use crate::can::enums::{BusError, TryReadError};

const REGS: Registers = Registers(unsafe { crate::pac::CAN1::steal() });

#[interrupt]
fn CAN1_TX() {
    defmt::debug!("interrupt: Can1 TX");
    let regs = unsafe { CAN1::steal() };
    regs.tsts().write(|w| {
        w.tmtcf(0).clear_bit_by_one();
        w.tmtcf(1).clear_bit_by_one();
        w.tmtcf(2).clear_bit_by_one();
        w
    });

    STATE.lock(|state| {
        state.borrow().tx_mode.on_interrupt();
    })
}

#[interrupt]
fn CAN1_RX0() {
    defmt::debug!("interrupt: Can1 RX0");
    STATE.lock(|state| {
        state.borrow().rx_mode.on_interrupt(RxFifo::Fifo0);
    });
}

#[interrupt]
fn CAN1_RX1() {
    defmt::debug!("interrupt: Can1 RX1");
    STATE.lock(|state| {
        state.borrow().rx_mode.on_interrupt(RxFifo::Fifo1);
    });
}

#[interrupt]
fn CAN1_SE() {
    defmt::debug!("interrupt: Can1 SE");
    let regs = unsafe { CAN1::steal() };
    let msr = regs.msts();
    let msr_val = msr.read();

    defmt::debug!("Can1 SE: {}", regs.ests().read().etr().bits());

    if msr_val.edzif().bit_is_set() {
        msr.modify(|_, m| m.edzif().clear_bit_by_one());
        STATE.lock(|state| {
            state.borrow().err_waker.wake();
        });
    } else if msr_val.eoif().is_error() {
        // Disable the interrupt, but don't acknowledge the error, so that it can be
        // forwarded off the bus message consumer. If we don't provide some way for
        // downstream code to determine that it has already provided this bus error instance
        // to the bus message consumer, we are doomed to re-provide a single error instance for
        // an indefinite amount of time.
        let ier = regs.inten();
        ier.modify(|_, i| i.eoien().disable());
        STATE.lock(|state| {
            state.borrow().err_waker.wake();
        });
    }
}

/// Configuration proxy returned by [`Can::modify_config`].
pub struct CanConfig {
    periph_clock: crate::time::Hertz,
}

impl CanConfig {
    /// Configures the bit timings.
    ///
    /// You can use <http://www.bittiming.can-wiki.info/> to calculate the `btr` parameter. Enter
    /// parameters as follows:
    ///
    /// - *Clock Rate*: The input clock speed to the CAN peripheral (*not* the CPU clock speed).
    ///   This is the clock rate of the peripheral bus the CAN peripheral is attached to (eg. APB1).
    /// - *Sample Point*: Should normally be left at the default value of 87.5%.
    /// - *SJW*: Should normally be left at the default value of 1.
    ///
    /// Then copy the `CAN_BUS_TIME` register value from the table and pass it as the `btr`
    /// parameter to this method.
    pub fn set_bit_timing(self, bt: crate::can::util::NominalBitTiming) -> Self {
        REGS.set_bit_timing(bt);
        self
    }

    /// Configure the CAN bit rate.
    ///
    /// This is a helper that internally calls `set_bit_timing()`[Self::set_bit_timing].
    pub fn set_bitrate(self, bitrate: u32) -> Self {
        let bit_timing = util::calc_can_timings(self.periph_clock, bitrate).unwrap();
        self.set_bit_timing(bit_timing)
    }

    /// Enables or disables loopback mode: Internally connects the TX and RX
    /// signals together.
    pub fn set_loopback(self, enabled: bool) -> Self {
        REGS.set_loopback(enabled);
        self
    }

    /// Enables or disables silent mode: Disconnects the TX signal from the pin.
    pub fn set_silent(self, enabled: bool) -> Self {
        REGS.set_silent(enabled);
        self
    }

    /// Enables or disables automatic retransmission of frames.
    ///
    /// If this is enabled, the CAN peripheral will automatically try to retransmit each frame
    /// until it can be sent. Otherwise, it will try only once to send each frame.
    ///
    /// Automatic retransmission is enabled by default.
    pub fn set_automatic_retransmit(self, enabled: bool) -> Self {
        REGS.set_automatic_retransmit(enabled);
        self
    }
}

impl Drop for CanConfig {
    #[inline]
    fn drop(&mut self) {
        REGS.leave_init_mode();
    }
}

/// CAN driver
pub struct Can {
    periph_clock: crate::time::Hertz,
}

/// Error returned by `try_write`
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TryWriteError {
    /// All transmit mailboxes are full
    Full,
}

impl Can {
    /// Creates a new Bxcan instance, keeping the peripheral in sleep mode.
    /// You must call [Can::enable_non_blocking] to use the peripheral.
    pub fn new(
        _can: CAN1,
        rx: impl Into<crate::gpio::alt::can1::Rx>,
        tx: impl Into<crate::gpio::alt::can1::Tx>,
        clocks: &Clocks,
    ) -> Self {
        let _rx = rx.into();
        let _tx = tx.into();

        let crm = unsafe { crate::pac::CRM::steal() };
        crate::pac::CAN1::enable(&crm);
        crate::pac::CAN1::reset(&crm);

        let regs = unsafe { crate::pac::CAN1::steal() };

        {
            regs.inten().write(|w| {
                w.eoien().set_bit();
                w.rfoien(0).set_bit();
                w.rfoien(1).set_bit();
                w.tcien().set_bit();
                w.boien().set_bit();
                w.epien().set_bit();
                w.eaien().set_bit();
                w.etrien().set_bit();
                w
            });

            regs.mctrl().write(|w| {
                // Enable timestamps on rx messages

                w.ttcen().set_bit();
                w
            });
        }

        unsafe {
            NVIC::unpend(interrupt::CAN1_TX);
            NVIC::unmask(interrupt::CAN1_TX);

            NVIC::unpend(interrupt::CAN1_RX0);
            NVIC::unmask(interrupt::CAN1_RX0);

            NVIC::unpend(interrupt::CAN1_RX1);
            NVIC::unmask(interrupt::CAN1_RX1);

            NVIC::unpend(interrupt::CAN1_SE);
            NVIC::unmask(interrupt::CAN1_SE);
        }

        Registers(regs).leave_init_mode();

        let periph_clock = CAN1::timer_clock(clocks);
        defmt::debug!("Starting can with clock: {}", periph_clock.to_Hz());

        Self {
            periph_clock,
        }
    }

    /// Set CAN bit rate.
    pub fn set_bitrate(&mut self, bitrate: u32) {
        let bit_timing = util::calc_can_timings(self.periph_clock, bitrate).unwrap();
        self.modify_config().set_bit_timing(bit_timing);
    }

    /// Configure bit timings and silent/loop-back mode.
    ///
    /// Calling this method will enter initialization mode. You must enable the peripheral
    /// again afterwards with [`enable`](Self::enable).
    pub fn modify_config(&mut self) -> CanConfig {
        REGS.enter_init_mode();

        CanConfig {
            periph_clock: self.periph_clock,
        }
    }

    /// Enables the peripheral and synchronizes with the bus.
    ///
    /// This will wait for 11 consecutive recessive bits (bus idle state).
    /// Contrary to enable method from bxcan library, this will not freeze the executor while waiting.
    pub async fn enable(&mut self) {
        while REGS.enable_non_blocking().is_err() {
            // SCE interrupt is only generated for entering sleep mode, but not leaving.
            // Yield to allow other tasks to execute while can bus is initializing.
            embassy_futures::yield_now().await;
        }
    }

    /// Enables or disables the peripheral from automatically wakeup when a SOF is detected on the bus
    /// while the peripheral is in sleep mode
    pub fn set_automatic_wakeup(&mut self, enabled: bool) {
        REGS.set_automatic_wakeup(enabled);
    }

    /// Manually wake the peripheral from sleep mode.
    ///
    /// Waking the peripheral manually does not trigger a wake-up interrupt.
    /// This will wait until the peripheral has acknowledged it has awoken from sleep mode
    pub fn wakeup(&mut self) {
        REGS.wakeup()
    }

    /// Check if the peripheral is currently in sleep mode
    pub fn is_sleeping(&self) -> bool {
        REGS.0.msts().read().dzc().is_sleep()
    }

    /// Put the peripheral in sleep mode
    ///
    /// When the peripherial is in sleep mode, messages can still be queued for transmission
    /// and any previously received messages can be read from the receive FIFOs, however
    /// no messages will be transmitted and no additional messages will be received.
    ///
    /// If the peripheral has automatic wakeup enabled, when a Start-of-Frame is detected
    /// the peripheral will automatically wake and receive the incoming message.
    pub async fn sleep(&mut self) {
        REGS.0.inten().modify(|_, i| i.edzien().enable());
        REGS.0.mctrl().modify(|_, m| m.dzen().enable());

        poll_fn(|cx| {
            STATE.lock(|s| {
                s.borrow().err_waker.register(cx.waker());
            });
            if self.is_sleeping() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;

        REGS.0.inten().modify(|_, i| i.edzien().disable());
    }

    /// Enable FIFO scheduling of outgoing frames.
    ///
    /// If this is enabled, frames will be transmitted in the order that they are passed to
    /// [`write()`][Self::write] or [`try_write()`][Self::try_write()].
    ///
    /// If this is disabled, frames are transmitted in order of priority.
    ///
    /// FIFO scheduling is disabled by default.
    pub fn set_tx_fifo_scheduling(&mut self, enabled: bool) {
        REGS.set_tx_fifo_scheduling(enabled)
    }

    /// Checks if FIFO scheduling of outgoing frames is enabled.
    pub fn tx_fifo_scheduling_enabled(&self) -> bool {
        REGS.tx_fifo_scheduling_enabled()
    }

    /// Queues the message to be sent.
    ///
    /// If the TX queue is full, this will wait until there is space, therefore exerting backpressure.
    pub async fn write(&mut self, frame: &Frame) -> TransmitStatus {
        self.split().0.write(frame).await
    }

    /// Attempts to transmit a frame without blocking.
    ///
    /// Returns [Err(TryWriteError::Full)] if the frame can not be queued for transmission now.
    ///
    /// If FIFO scheduling is enabled, any empty mailbox will be used.
    ///
    /// Otherwise, the frame will only be accepted if there is no frame with the same priority already queued.
    /// This is done to work around a hardware limitation that could lead to out-of-order delivery
    /// of frames with the same priority.
    pub fn try_write(&mut self, frame: &Frame) -> Result<TransmitStatus, TryWriteError> {
        self.split().0.try_write(frame)
    }

    /// Waits for a specific transmit mailbox to become empty
    pub async fn flush(&self, mb: Mailbox) {
        CanTx { _phantom: PhantomData }
        .flush_inner(mb)
        .await;
    }

    /// Waits until any of the transmit mailboxes become empty
    ///
    /// Note that [`Self::try_write()`] may fail with [`TryWriteError::Full`],
    /// even after the future returned by this function completes.
    /// This will happen if FIFO scheduling of outgoing frames is not enabled,
    /// and a frame with equal priority is already queued for transmission.
    pub async fn flush_any(&self) {
        CanTx { _phantom: PhantomData }
        .flush_any_inner()
        .await
    }

    /// Waits until all of the transmit mailboxes become empty
    pub async fn flush_all(&self) {
        CanTx { _phantom: PhantomData }
        .flush_all_inner()
        .await
    }

    /// Attempts to abort the sending of a frame that is pending in a mailbox.
    ///
    /// If there is no frame in the provided mailbox, or its transmission succeeds before it can be
    /// aborted, this function has no effect and returns `false`.
    ///
    /// If there is a frame in the provided mailbox, and it is canceled successfully, this function
    /// returns `true`.
    pub fn abort(&mut self, mailbox: Mailbox) -> bool {
        REGS.abort(mailbox)
    }

    /// Returns `true` if no frame is pending for transmission.
    pub fn is_transmitter_idle(&self) -> bool {
        REGS.is_idle()
    }

    /// Read a CAN frame.
    ///
    /// If no CAN frame is in the RX buffer, this will wait until there is one.
    ///
    /// Returns a tuple of the time the message was received and the message frame
    pub async fn read(&mut self) -> Result<Envelope, BusError> {
        RxMode::read().await
    }

    /// Attempts to read a CAN frame without blocking.
    ///
    /// Returns [Err(TryReadError::Empty)] if there are no frames in the rx queue.
    pub fn try_read(&mut self) -> Result<Envelope, TryReadError> {
        RxMode::try_read()
    }

    /// Waits while receive queue is empty.
    pub async fn wait_not_empty(&mut self) {
        RxMode::wait_not_empty().await
    }

    /// Split the CAN driver into transmit and receive halves.
    ///
    /// Useful for doing separate transmit/receive tasks.
    pub fn split(&mut self) -> (CanTx<'_>, CanRx<'_>) {
        (
            CanTx { _phantom: PhantomData },
            CanRx { _phantom: PhantomData },
        )
    }

    /// Return a buffered instance of driver. User must supply Buffers
    pub fn buffered<'a, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize>(
        &'a mut self,
        txb: &'static mut TxBuf<TX_BUF_SIZE>,
        rxb: &'static mut RxBuf<RX_BUF_SIZE>,
    ) -> BufferedCan<'a, TX_BUF_SIZE, RX_BUF_SIZE> {
        let (tx, rx) = self.split();
        BufferedCan {
            tx: tx.buffered(txb),
            rx: rx.buffered(rxb),
        }
    }
}

// impl Can {
//     /// Accesses the filter banks owned by this CAN peripheral.
//     ///
//     /// To modify filters of a slave peripheral, `modify_filters` has to be called on the master
//     /// peripheral instead.
//     pub fn modify_filters(&mut self) -> MasterFilters<'_> {
//         unsafe { MasterFilters::new(&self.info) }
//     }
// }

/// Buffered CAN driver.
pub struct BufferedCan<'a, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize> {
    tx: BufferedCanTx<'a, TX_BUF_SIZE>,
    rx: BufferedCanRx<'a, RX_BUF_SIZE>,
}

impl<'a, const TX_BUF_SIZE: usize, const RX_BUF_SIZE: usize>
    BufferedCan<'a, TX_BUF_SIZE, RX_BUF_SIZE>
{
    /// Async write frame to TX buffer.
    pub async fn write(&mut self, frame: &Frame) {
        self.tx.write(frame).await
    }

    /// Returns a sender that can be used for sending CAN frames.
    pub fn writer(&self) -> BufferedCanSender {
        self.tx.writer()
    }

    /// Async read frame from RX buffer.
    pub async fn read(&mut self) -> Result<Envelope, BusError> {
        self.rx.read().await
    }

    /// Attempts to read a CAN frame without blocking.
    ///
    /// Returns [Err(TryReadError::Empty)] if there are no frames in the rx queue.
    pub fn try_read(&mut self) -> Result<Envelope, TryReadError> {
        self.rx.try_read()
    }

    /// Waits while receive queue is empty.
    pub async fn wait_not_empty(&mut self) {
        self.rx.wait_not_empty().await
    }

    /// Returns a receiver that can be used for receiving CAN frames. Note, each CAN frame will only be received by one receiver.
    pub fn reader(&self) -> BufferedCanReceiver {
        self.rx.reader()
    }

    // /// Accesses the filter banks owned by this CAN peripheral.
    // ///
    // /// To modify filters of a slave peripheral, `modify_filters` has to be called on the master
    // /// peripheral instead.
    // pub fn modify_filters(&mut self) -> MasterFilters<'_> {
    //     self.rx.modify_filters()
    // }
}

/// CAN driver, transmit half.
pub struct CanTx<'a> {
    _phantom: PhantomData<&'a ()>,
}

impl<'a> CanTx<'a> {
    /// Queues the message to be sent.
    ///
    /// If the TX queue is full, this will wait until there is space, therefore exerting backpressure.
    pub async fn write(&mut self, frame: &Frame) -> TransmitStatus {
        poll_fn(|cx| {
            STATE.lock(|s| {
                s.borrow().tx_mode.register(cx.waker());
            });
            if let Ok(status) = REGS.transmit(frame) {
                return Poll::Ready(status);
            }

            Poll::Pending
        })
        .await
    }

    /// Attempts to transmit a frame without blocking.
    ///
    /// Returns [Err(TryWriteError::Full)] if the frame can not be queued for transmission now.
    ///
    /// If FIFO scheduling is enabled, any empty mailbox will be used.
    ///
    /// Otherwise, the frame will only be accepted if there is no frame with the same priority already queued.
    /// This is done to work around a hardware limitation that could lead to out-of-order delivery
    /// of frames with the same priority.
    pub fn try_write(&mut self, frame: &Frame) -> Result<TransmitStatus, TryWriteError> {
        REGS
            .transmit(frame)
            .map_err(|_| TryWriteError::Full)
    }

    async fn flush_inner(&self, mb: Mailbox) {
        poll_fn(|cx| {
            STATE.lock(|s| {
                s.borrow().tx_mode.register(cx.waker());
            });
            if REGS.0.tsts().read().tmef(mb.index()).is_empty() {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }

    /// Waits for a specific transmit mailbox to become empty
    pub async fn flush(&self, mb: Mailbox) {
        self.flush_inner(mb).await
    }

    async fn flush_any_inner(&self) {
        poll_fn(|cx| {
            STATE.lock(|s| {
                s.borrow().tx_mode.register(cx.waker());
            });

            let tsr = REGS.0.tsts().read();
            if tsr.tmef(Mailbox::Mailbox0.index()).is_empty()
                || tsr.tmef(Mailbox::Mailbox1.index()).is_empty()
                || tsr.tmef(Mailbox::Mailbox2.index()).is_empty()
            {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }

    /// Waits until any of the transmit mailboxes become empty
    ///
    /// Note that [`Self::try_write()`] may fail with [`TryWriteError::Full`],
    /// even after the future returned by this function completes.
    /// This will happen if FIFO scheduling of outgoing frames is not enabled,
    /// and a frame with equal priority is already queued for transmission.
    pub async fn flush_any(&self) {
        self.flush_any_inner().await
    }

    async fn flush_all_inner(&self) {
        poll_fn(|cx| {
            STATE.lock(|s| {
                s.borrow().tx_mode.register(cx.waker());
            });

            let tsr = REGS.0.tsts().read();
            if tsr.tmef(Mailbox::Mailbox0.index()).is_empty()
                && tsr.tmef(Mailbox::Mailbox1.index()).is_empty()
                && tsr.tmef(Mailbox::Mailbox2.index()).is_empty()
            {
                return Poll::Ready(());
            }

            Poll::Pending
        })
        .await;
    }

    /// Waits until all of the transmit mailboxes become empty
    pub async fn flush_all(&self) {
        self.flush_all_inner().await
    }

    /// Attempts to abort the sending of a frame that is pending in a mailbox.
    ///
    /// If there is no frame in the provided mailbox, or its transmission succeeds before it can be
    /// aborted, this function has no effect and returns `false`.
    ///
    /// If there is a frame in the provided mailbox, and it is canceled successfully, this function
    /// returns `true`.
    pub fn abort(&mut self, mailbox: Mailbox) -> bool {
        REGS.abort(mailbox)
    }

    /// Returns `true` if no frame is pending for transmission.
    pub fn is_idle(&self) -> bool {
        REGS.is_idle()
    }

    /// Return a buffered instance of driver. User must supply Buffers
    pub fn buffered<const TX_BUF_SIZE: usize>(
        self,
        txb: &'static mut TxBuf<TX_BUF_SIZE>,
    ) -> BufferedCanTx<'a, TX_BUF_SIZE> {
        BufferedCanTx::new(self, txb)
    }
}

/// User supplied buffer for TX buffering
pub type TxBuf<const BUF_SIZE: usize> = Channel<CriticalSectionRawMutex, Frame, BUF_SIZE>;

/// Buffered CAN driver, transmit half.
pub struct BufferedCanTx<'a, const TX_BUF_SIZE: usize> {
    _tx: CanTx<'a>,
    tx_buf: &'static TxBuf<TX_BUF_SIZE>,
}

impl<'a, const TX_BUF_SIZE: usize> BufferedCanTx<'a, TX_BUF_SIZE> {
    fn new(_tx: CanTx<'a>, tx_buf: &'static TxBuf<TX_BUF_SIZE>) -> Self {
        Self {
            _tx,
            tx_buf,
        }
        .setup()
    }

    fn setup(self) -> Self {
        // We don't want interrupts being processed while we change modes.
        critical_section::with(|_| {
            let tx_inner_b = super::common::ClassicBufferedTxInner {
                tx_receiver: self.tx_buf.receiver().into(),
            };
            STATE.lock(|s| {
                s.borrow_mut().tx_mode = TxMode::Buffered(tx_inner_b);
            });
        });
        self
    }

    /// Async write frame to TX buffer.
    pub async fn write(&mut self, frame: &Frame) {
        self.tx_buf.send(*frame).await;
        tx_waker(); // Wake for Tx
    }

    /// Returns a sender that can be used for sending CAN frames.
    pub fn writer(&self) -> BufferedCanSender {
        BufferedCanSender {
            tx_buf: self.tx_buf.sender().into(),
        }
    }
}

/// CAN driver, receive half.
#[allow(dead_code)]
pub struct CanRx<'a> {
    _phantom: PhantomData<&'a ()>
}

impl<'a> CanRx<'a> {
    /// Read a CAN frame.
    ///
    /// If no CAN frame is in the RX buffer, this will wait until there is one.
    ///
    /// Returns a tuple of the time the message was received and the message frame
    pub async fn read(&mut self) -> Result<Envelope, BusError> {
        RxMode::read().await
    }

    /// Attempts to read a CAN frame without blocking.
    ///
    /// Returns [Err(TryReadError::Empty)] if there are no frames in the rx queue.
    pub fn try_read(&mut self) -> Result<Envelope, TryReadError> {
        RxMode::try_read()
    }

    /// Waits while receive queue is empty.
    pub async fn wait_not_empty(&mut self) {
        RxMode::wait_not_empty().await
    }

    /// Return a buffered instance of driver. User must supply Buffers
    pub fn buffered<const RX_BUF_SIZE: usize>(
        self,
        rxb: &'static mut RxBuf<RX_BUF_SIZE>,
    ) -> BufferedCanRx<'a, RX_BUF_SIZE> {
        BufferedCanRx::new(self, rxb)
    }

    // /// Accesses the filter banks owned by this CAN peripheral.
    // ///
    // /// To modify filters of a slave peripheral, `modify_filters` has to be called on the master
    // /// peripheral instead.
    // pub fn modify_filters(&mut self) -> MasterFilters<'_> {
    //     unsafe { MasterFilters::new(&self.info) }
    // }
}

/// User supplied buffer for RX Buffering
pub type RxBuf<const BUF_SIZE: usize> =
    Channel<CriticalSectionRawMutex, Result<Envelope, BusError>, BUF_SIZE>;

/// CAN driver, receive half in Buffered mode.
pub struct BufferedCanRx<'a, const RX_BUF_SIZE: usize> {
    _rx: CanRx<'a>,
    rx_buf: &'static RxBuf<RX_BUF_SIZE>,
}

impl<'a, const RX_BUF_SIZE: usize> BufferedCanRx<'a, RX_BUF_SIZE> {
    fn new(_rx: CanRx<'a>, rx_buf: &'static RxBuf<RX_BUF_SIZE>) -> Self {
        BufferedCanRx {
            _rx,
            rx_buf,
        }
        .setup()
    }

    fn setup(self) -> Self {
        // We don't want interrupts being processed while we change modes.
        critical_section::with(|_| {
            let rx_inner = super::common::ClassicBufferedRxInner {
                rx_sender: self.rx_buf.sender().into(),
            };
            STATE.lock(|s| {
                s.borrow_mut().rx_mode = RxMode::Buffered(rx_inner);
            });
        });
        self
    }

    /// Async read frame from RX buffer.
    pub async fn read(&mut self) -> Result<Envelope, BusError> {
        self.rx_buf.receive().await
    }

    /// Attempts to read a CAN frame without blocking.
    ///
    /// Returns [Err(TryReadError::Empty)] if there are no frames in the rx queue.
    pub fn try_read(&mut self) -> Result<Envelope, TryReadError> {
        STATE.lock(|s| match &s.borrow().rx_mode {
            RxMode::Buffered(_) => {
                if let Ok(result) = self.rx_buf.try_receive() {
                    match result {
                        Ok(envelope) => Ok(envelope),
                        Err(e) => Err(TryReadError::BusError(e)),
                    }
                } else {
                    if let Some(err) = REGS.curr_error() {
                        return Err(TryReadError::BusError(err));
                    } else {
                        Err(TryReadError::Empty)
                    }
                }
            }
            _ => {
                panic!("Bad Mode")
            }
        })
    }

    /// Waits while receive queue is empty.
    pub async fn wait_not_empty(&mut self) {
        poll_fn(|cx| self.rx_buf.poll_ready_to_receive(cx)).await
    }

    /// Returns a receiver that can be used for receiving CAN frames. Note, each CAN frame will only be received by one receiver.
    pub fn reader(&self) -> BufferedCanReceiver {
        BufferedCanReceiver {
            rx_buf: self.rx_buf.receiver().into(),
        }
    }

    // /// Accesses the filter banks owned by this CAN peripheral.
    // ///
    // /// To modify filters of a slave peripheral, `modify_filters` has to be called on the master
    // /// peripheral instead.
    // pub fn modify_filters(&mut self) -> MasterFilters<'_> {
    //     self.rx.modify_filters()
    // }
}

impl Drop for Can {
    fn drop(&mut self) {
        // Cannot call `free()` because it moves the instance.
        // Manually reset the peripheral.
        REGS.0.mctrl().write(|w| w.sprst().set_bit());
        REGS.enter_init_mode();
        REGS.leave_init_mode();
        //rcc::disable::<T>();
    }
}

/// Identifies one of the two receive FIFOs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Fifo {
    /// First receive FIFO
    Fifo0 = 0,
    /// Second receive FIFO
    Fifo1 = 1,
}

/// Identifies one of the three transmit mailboxes.
#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Mailbox {
    /// Transmit mailbox 0
    Mailbox0 = 0,
    /// Transmit mailbox 1
    Mailbox1 = 1,
    /// Transmit mailbox 2
    Mailbox2 = 2,
}

/// Contains information about a frame enqueued for transmission via [`Can::transmit`] or
/// [`Tx::transmit`].
pub struct TransmitStatus {
    dequeued_frame: Option<Frame>,
    mailbox: Mailbox,
}

impl TransmitStatus {
    /// Returns the lower-priority frame that was dequeued to make space for the new frame.
    #[inline]
    pub fn dequeued_frame(&self) -> Option<&Frame> {
        self.dequeued_frame.as_ref()
    }

    /// Returns the [`Mailbox`] the frame was enqueued in.
    #[inline]
    pub fn mailbox(&self) -> Mailbox {
        self.mailbox
    }
}

pub(crate) enum RxMode {
    NonBuffered(AtomicWaker),
    Buffered(super::common::ClassicBufferedRxInner),
}

impl RxMode {
    pub fn on_interrupt(&self, fifo: RxFifo) {
        match self {
            Self::NonBuffered(waker) => {
                // Disable interrupts until read
                let fifo_idx = match fifo {
                    RxFifo::Fifo0 => 0u8,
                    RxFifo::Fifo1 => 1u8,
                };
                REGS.0.inten().modify(|_, w| {
                    w.rfmien(fifo_idx).disable()
                });
                waker.wake();
            }
            Self::Buffered(buf) => {
                loop {
                    match REGS.receive_fifo(fifo) {
                        Some(envelope) => {
                            // NOTE: consensus was reached that if rx_queue is full, packets should be dropped
                            let _ = buf.rx_sender.try_send(Ok(envelope));
                        }
                        None => return,
                    };
                }
            }
        }
    }

    pub(crate) async fn read() -> Result<Envelope, BusError> {
        poll_fn(|cx| {
            STATE.lock(|state| {
                let state = state.borrow();
                state.err_waker.register(cx.waker());
                match &state.rx_mode {
                    Self::NonBuffered(waker) => {
                        waker.register(cx.waker());
                    }
                    _ => {
                        panic!("Bad Mode")
                    }
                }
            });
            match RxMode::try_read() {
                Ok(result) => Poll::Ready(Ok(result)),
                Err(TryReadError::Empty) => Poll::Pending,
                Err(TryReadError::BusError(be)) => Poll::Ready(Err(be)),
            }
        })
        .await
    }
    pub(crate) fn try_read() -> Result<Envelope, TryReadError> {
        STATE.lock(|state| match state.borrow().rx_mode {
            Self::NonBuffered(_) => {
                let registers = REGS;
                if let Some(msg) = registers.receive_fifo(RxFifo::Fifo0) {
                    registers.0.inten().modify(|_, w| {
                        w.rfmien(0).enable()
                    });
                    Ok(msg)
                } else if let Some(msg) = registers.receive_fifo(RxFifo::Fifo1) {
                    registers.0.inten().modify(|_, w| {
                        w.rfmien(1).enable()
                    });
                    Ok(msg)
                } else if let Some(err) = registers.curr_error() {
                    Err(TryReadError::BusError(err))
                } else {
                    registers.0.inten().modify(|_, w| {
                        w.rfmien(0).enable();
                        w.rfmien(1).enable();
                        w
                    });
                    Err(TryReadError::Empty)
                }
            }
            _ => {
                panic!("Bad Mode")
            }
        })
    }
    pub(crate) async fn wait_not_empty() {
        poll_fn(|cx| {
            STATE.lock(|s| {
                let state = s.borrow();
                match &state.rx_mode {
                    Self::NonBuffered(waker) => {
                        waker.register(cx.waker());
                    }
                    _ => {
                        panic!("Bad Mode")
                    }
                }
            });
            if REGS.receive_frame_available() {
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await
    }
}

pub(crate) enum TxMode {
    NonBuffered(AtomicWaker),
    Buffered(super::common::ClassicBufferedTxInner),
}

impl TxMode {
    pub fn buffer_free(&self) -> bool {
        let tsr = REGS.0.tsts().read();
        tsr.tmef(Mailbox::Mailbox0.index()).is_empty()
            || tsr.tmef(Mailbox::Mailbox1.index()).is_empty()
            || tsr.tmef(Mailbox::Mailbox2.index()).is_empty()
    }
    pub fn on_interrupt(&self) {
        STATE.lock(|state| {
            let tx_mode = &state.borrow().tx_mode;

            match tx_mode {
                TxMode::NonBuffered(waker) => waker.wake(),
                TxMode::Buffered(buf) => {
                    while self.buffer_free() {
                        match buf.tx_receiver.try_receive() {
                            Ok(frame) => {
                                _ = REGS.transmit(&frame);
                            }
                            Err(_) => {
                                break;
                            }
                        }
                    }
                }
            }
        });
    }

    fn register(&self, arg: &core::task::Waker) {
        match self {
            TxMode::NonBuffered(waker) => {
                waker.register(arg);
            }
            _ => {
                panic!("Bad mode");
            }
        }
    }
}

pub(crate) struct State {
    pub(crate) rx_mode: RxMode,
    pub(crate) tx_mode: TxMode,
    pub err_waker: AtomicWaker,
    // receiver_instance_count: usize,
    // sender_instance_count: usize,
}

impl State {
    pub const fn new() -> Self {
        Self {
            rx_mode: RxMode::NonBuffered(AtomicWaker::new()),
            tx_mode: TxMode::NonBuffered(AtomicWaker::new()),
            err_waker: AtomicWaker::new(),
            // receiver_instance_count: 1,
            // sender_instance_count: 1,
        }
    }
}

pub(crate) fn tx_waker() {
    NVIC::pend(interrupt::CAN1_TX);
}

type SharedState =
    embassy_sync::blocking_mutex::Mutex<CriticalSectionRawMutex, core::cell::RefCell<State>>;
// pub(crate) struct Info {
//     regs: Registers,
//     tx_interrupt: crate::interrupt::Interrupt,
//     rx0_interrupt: crate::interrupt::Interrupt,
//     rx1_interrupt: crate::interrupt::Interrupt,
//     sce_interrupt: crate::interrupt::Interrupt,
//     pub(crate) tx_waker: fn(),
//     state: SharedState,

//     /// The total number of filter banks available to the instance.
//     ///
//     /// This is usually either 14 or 28, and should be specified in the chip's reference manual or datasheet.
//     num_filter_banks: u8,
// }

static STATE: SharedState =
    embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(State::new()));

// impl Info {
//     pub(crate) fn adjust_reference_counter(&self, val: RefCountOp) {
//         STATE.lock(|s| {
//             let mut mut_state = s.borrow_mut();
//             match val {
//                 RefCountOp::NotifySenderCreated => {
//                     mut_state.sender_instance_count += 1;
//                 }
//                 RefCountOp::NotifySenderDestroyed => {
//                     mut_state.sender_instance_count -= 1;
//                     if 0 == mut_state.sender_instance_count {
//                         (*mut_state).tx_mode =
//                             TxMode::NonBuffered(embassy_sync::waitqueue::AtomicWaker::new());
//                     }
//                 }
//                 RefCountOp::NotifyReceiverCreated => {
//                     mut_state.receiver_instance_count += 1;
//                 }
//                 RefCountOp::NotifyReceiverDestroyed => {
//                     mut_state.receiver_instance_count -= 1;
//                     if 0 == mut_state.receiver_instance_count {
//                         (*mut_state).rx_mode =
//                             RxMode::NonBuffered(embassy_sync::waitqueue::AtomicWaker::new());
//                     }
//                 }
//             }
//         });
//     }
// }

// trait SealedInstance {
//     fn info() -> &'static Info;
//     fn regs() -> crate::pac::can::Can;
// }

// /// CAN instance trait.
// #[allow(private_bounds)]
// pub trait Instance: SealedInstance + PeripheralType + RccPeripheral + 'static {
//     /// TX interrupt for this instance.
//     type TXInterrupt: crate::interrupt::typelevel::Interrupt;
//     /// RX0 interrupt for this instance.
//     type RX0Interrupt: crate::interrupt::typelevel::Interrupt;
//     /// RX1 interrupt for this instance.
//     type RX1Interrupt: crate::interrupt::typelevel::Interrupt;
//     /// SCE interrupt for this instance.
//     type SCEInterrupt: crate::interrupt::typelevel::Interrupt;
// }

// /// A bxCAN instance that owns filter banks.
// ///
// /// In master-slave-instance setups, only the master instance owns the filter banks, and needs to
// /// split some of them off for use by the slave instance. In that case, the master instance should
// /// implement [`FilterOwner`] and [`MasterInstance`], while the slave instance should only implement
// /// [`Instance`].
// ///
// /// In single-instance configurations, the instance owns all filter banks and they can not be split
// /// off. In that case, the instance should implement [`Instance`] and [`FilterOwner`].
// ///
// /// # Safety
// ///
// /// This trait must only be implemented if the instance does, in fact, own its associated filter
// /// banks, and `NUM_FILTER_BANKS` must be correct.
// pub unsafe trait FilterOwner: Instance {
//     /// The total number of filter banks available to the instance.
//     ///
//     /// This is usually either 14 or 28, and should be specified in the chip's reference manual or datasheet.
//     const NUM_FILTER_BANKS: u8;
// }

// /// A bxCAN master instance that shares filter banks with a slave instance.
// ///
// /// In master-slave-instance setups, this trait should be implemented for the master instance.
// ///
// /// # Safety
// ///
// /// This trait must only be implemented when there is actually an associated slave instance.
// pub unsafe trait MasterInstance: FilterOwner {}

// foreach_peripheral!(
//     (can, $inst:ident) => {
//         impl SealedInstance for peripherals::$inst {

//             fn info() -> &'static Info {
//                 static INFO: Info = Info {
//                     regs: Registers(crate::pac::$inst),
//                     tx_interrupt: crate::_generated::peripheral_interrupts::$inst::TX::IRQ,
//                     rx0_interrupt: crate::_generated::peripheral_interrupts::$inst::RX0::IRQ,
//                     rx1_interrupt: crate::_generated::peripheral_interrupts::$inst::RX1::IRQ,
//                     sce_interrupt: crate::_generated::peripheral_interrupts::$inst::SCE::IRQ,
//                     tx_waker: crate::_generated::peripheral_interrupts::$inst::TX::pend,
//                     num_filter_banks: peripherals::$inst::NUM_FILTER_BANKS,
//                     state: embassy_sync::blocking_mutex::Mutex::new(core::cell::RefCell::new(State::new())),
//                 };
//                 &INFO
//             }
//             fn regs() -> crate::pac::can::Can {
//                 crate::pac::$inst
//             }
//         }

//         impl Instance for peripherals::$inst {
//             type TXInterrupt = crate::_generated::peripheral_interrupts::$inst::TX;
//             type RX0Interrupt = crate::_generated::peripheral_interrupts::$inst::RX0;
//             type RX1Interrupt = crate::_generated::peripheral_interrupts::$inst::RX1;
//             type SCEInterrupt = crate::_generated::peripheral_interrupts::$inst::SCE;
//         }
//     };
// );

// foreach_peripheral!(
//     (can, CAN) => {
//         unsafe impl FilterOwner for peripherals::CAN {
//             const NUM_FILTER_BANKS: u8 = 14;
//         }
//     };
//     // CAN1 and CAN2 is a combination of master and slave instance.
//     // CAN1 owns the filter bank and needs to be enabled in order
//     // for CAN2 to receive messages.
//     (can, CAN1) => {
//         cfg_if::cfg_if! {
//             if #[cfg(all(
//                 any(stm32l4, stm32f72x, stm32f73x),
//                 not(any(stm32l49x, stm32l4ax))
//             ))] {
//                 // Most L4 devices and some F7 devices use the name "CAN1"
//                 // even if there is no "CAN2" peripheral.
//                 unsafe impl FilterOwner for peripherals::CAN1 {
//                     const NUM_FILTER_BANKS: u8 = 14;
//                 }
//             } else {
//                 unsafe impl FilterOwner for peripherals::CAN1 {
//                     const NUM_FILTER_BANKS: u8 = 28;
//                 }
//                 unsafe impl MasterInstance for peripherals::CAN1 {}
//             }
//         }
//     };
//     (can, CAN2) => {
//         unsafe impl FilterOwner for peripherals::CAN2 {
//             const NUM_FILTER_BANKS: u8 = 0;
//         }
//     };
//     (can, CAN3) => {
//         unsafe impl FilterOwner for peripherals::CAN3 {
//             const NUM_FILTER_BANKS: u8 = 14;
//         }
//     };
// );

// pin_trait!(RxPin, Instance, @A);
// pin_trait!(TxPin, Instance, @A);

trait Index {
    fn index(&self) -> u8;
}

impl Index for Mailbox {
    fn index(&self) -> u8 {
        match self {
            Mailbox::Mailbox0 => 0,
            Mailbox::Mailbox1 => 1,
            Mailbox::Mailbox2 => 2,
        }
    }
}
