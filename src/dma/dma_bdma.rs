use super::ringbuffer::{DmaCtrl, Error, ReadableDmaRingBuffer, WritableDmaRingBuffer};
use super::word::{Word, WordSize};
use super::{Channel, Dir, Increment, STATE};

use crate::{interrupt, pac};
use core::future::{Future, poll_fn};
use core::pin::Pin;
use core::sync::atomic::{AtomicUsize, Ordering, compiler_fence, fence};
use core::task::{Context, Poll, Waker};
use embassy_sync::waitqueue::AtomicWaker;

fn htif(r: crate::pac::dma1::sts::R, n: u8) -> bool {
    match n {
        0 => r.hdtf1().bit_is_set(),
        1 => r.hdtf2().bit_is_set(),
        2 => r.hdtf3().bit_is_set(),
        3 => r.hdtf4().bit_is_set(),
        4 => r.hdtf5().bit_is_set(),
        5 => r.hdtf6().bit_is_set(),
        6 => r.hdtf7().bit_is_set(),
        _ => unreachable!(),
    }
}

fn tcif(r: crate::pac::dma1::sts::R, n: u8) -> bool {
    match n {
        0 => r.fdtf1().bit_is_set(),
        1 => r.fdtf2().bit_is_set(),
        2 => r.fdtf3().bit_is_set(),
        3 => r.fdtf4().bit_is_set(),
        4 => r.fdtf5().bit_is_set(),
        5 => r.fdtf6().bit_is_set(),
        6 => r.fdtf7().bit_is_set(),
        _ => unreachable!(),
    }
}


fn dterr_w(r: &mut crate::pac::dma1::clr::W, n: u8, val: bool) ->  &mut crate::pac::dma1::clr::W {
    match n {
        0 => r.dterrfc1().bit(val),
        1 => r.dterrfc2().bit(val),
        2 => r.dterrfc3().bit(val),
        3 => r.dterrfc4().bit(val),
        4 => r.dterrfc5().bit(val),
        5 => r.dterrfc6().bit(val),
        6 => r.dterrfc7().bit(val),
        _ => unreachable!(),
    }
}

fn fdtfc_w(r: &mut crate::pac::dma1::clr::W, n: u8, val: bool) ->  &mut crate::pac::dma1::clr::W {
    match n {
        0 => r.fdtfc1().bit(val),
        1 => r.fdtfc2().bit(val),
        2 => r.fdtfc3().bit(val),
        3 => r.fdtfc4().bit(val),
        4 => r.fdtfc5().bit(val),
        5 => r.fdtfc6().bit(val),
        6 => r.fdtfc7().bit(val),
        _ => unreachable!(),
    }
}

pub(crate) unsafe fn on_irq(channel: u8) {
    let r: &pac::dma1::RegisterBlock = unsafe { &*crate::pac::DMA1::ptr() };
    let state = &STATE[channel as usize];
    let sts = r.sts().read();
    let cr = r.channel(channel as usize).ctrl();
    // if sts.dterrf (info.num) {
    //     {
    //         {
    //             ::core::panicking::panic_fmt(format_args!(
    //                 "DMA: error on BDMA@{0:08x} channel {1}",
    //                 r.as_ptr() as u32,
    //                 info.num,
    //             ));
    //         };
    //     };
    // }
    let mut activity = false;
    if htif(sts, channel) && cr.read().hdtien().bit_is_set() {
        r.clr().write(|w| dterr_w(w, channel, true));
        activity = true;
    }
    if tcif(sts, channel) && cr.read().fdtien().bit_is_set() {
        r.clr().write(|w| fdtfc_w(w, channel, true));
        state.complete_count.fetch_add(1, Ordering::Release);
        activity = true;
    }
    if !activity {
        return;
    }
    state.waker.wake();
}

pub(crate) struct ChannelInfo {
    pub(crate) dma: DmaInfo,
    pub(crate) num: usize,
}


/// DMA transfer options.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct TransferOptions {
    /// Request priority level
    pub priority: Priority,
    /// Enable circular DMA
    ///
    /// Note:
    /// If you enable circular mode manually, you may want to build and `.await` the `Transfer` in a separate task.
    /// Since DMA in circular mode need manually stop, `.await` in current task would block the task forever.
    pub circular: bool,
    /// Enable half transfer interrupt
    pub half_transfer_ir: bool,
    /// Enable transfer complete interrupt
    pub complete_transfer_ir: bool,
}
i
    mpl Default for TransferOptions {
    fn default() -> Self {
        Self {
            priority: Priority::VeryHigh,
            circular: false,
            half_transfer_ir: false,
            complete_transfer_ir: true,
        }
    }
}

/// DMA request priority
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Priority {
    /// Low Priority
    Low,
    /// Medium Priority
    Medium,
    /// High Priority
    High,
    /// Very High Priority
    VeryHigh,
}

// impl From<Priority> for crate::pac::dma1:: {
//     fn from(value: Priority) -> Self {
//         match value {
//             Priority::Low => pac::bdma::vals::Pl::Low,
//             Priority::Medium => pac::bdma::vals::Pl::Medium,
//             Priority::High => pac::bdma::vals::Pl::High,
//             Priority::VeryHigh => pac::bdma::vals::Pl::VeryHigh,
//         }
//     }
// }

mod bdma_only {
    use super::*;
    impl From<WordSize> for vals::Size {
        fn from(raw: WordSize) -> Self {
            match raw {
                WordSize::OneByte => Self::Bits8,
                WordSize::TwoBytes => Self::Bits16,
                WordSize::FourBytes => Self::Bits32,
                WordSize::EightBytes => ::core::panicking::panic("not implemented"),
            }
        }
    }
    impl From<Dir> for vals::Dir {
        fn from(raw: Dir) -> Self {
            match raw {
                Dir::MemoryToPeripheral => Self::FromMemory,
                Dir::PeripheralToMemory => Self::FromPeripheral,
                Dir::MemoryToMemory => Self::FromMemory,
            }
        }
    }
}

pub(crate) struct ChannelState {
    waker: AtomicWaker,
    complete_count: AtomicUsize,
}
impl ChannelState {
    pub(crate) const NEW: Self = Self {
        waker: AtomicWaker::new(),
        complete_count: AtomicUsize::new(0),
    };
}
/// safety: must be called only once
pub(crate) unsafe fn init(
    cs: critical_section::CriticalSection,
    bdma_priority: interrupt::Priority,
) {
    crate::interrupt::typelevel::DMA1_CHANNEL1::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL1::enable();
    crate::interrupt::typelevel::DMA1_CHANNEL2::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL2::enable();
    crate::interrupt::typelevel::DMA1_CHANNEL3::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL3::enable();
    crate::interrupt::typelevel::DMA1_CHANNEL4::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL4::enable();
    crate::interrupt::typelevel::DMA1_CHANNEL5::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL5::enable();
    crate::interrupt::typelevel::DMA1_CHANNEL6::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL6::enable();
    crate::interrupt::typelevel::DMA1_CHANNEL7::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA1_CHANNEL7::enable();
    crate::interrupt::typelevel::DMA2_CHANNEL1::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA2_CHANNEL1::enable();
    crate::interrupt::typelevel::DMA2_CHANNEL2::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA2_CHANNEL2::enable();
    crate::interrupt::typelevel::DMA2_CHANNEL3::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA2_CHANNEL3::enable();
    crate::interrupt::typelevel::DMA2_CHANNEL4_5::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA2_CHANNEL4_5::enable();
    crate::interrupt::typelevel::DMA2_CHANNEL4_5::set_priority_with_cs(cs, bdma_priority);
    crate::interrupt::typelevel::DMA2_CHANNEL4_5::enable();
    crate::_generated::init_dma();
    crate::_generated::init_bdma();
}

impl<'d> Channel<'d> {
    fn info(&self) -> &'static super::ChannelInfo {
        super::info(self.channel)
    }
    unsafe fn configure(
        &self,
        dir: Dir,
        peri_addr: *const u32,
        mem_addr: *mut u32,
        mem_len: usize,
        incr_mem: Increment,
        mem_size: WordSize,
        peri_size: WordSize,
        options: TransferOptions,
    ) {
        fence(Ordering::SeqCst);
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => {
                {
                    if !(mem_len > 0 && mem_len <= 0xFFFF) {
                        ::core::panicking::panic(
                            "assertion failed: mem_len > 0 && mem_len <= 0xFFFF",
                        )
                    }
                };
                let state: &ChannelState = &STATE[self.channel as usize];
                let ch = r.ch(info.num);
                state.complete_count.store(0, Ordering::Release);
                self.clear_irqs();
                ch.par().write_value(peri_addr as u32);
                ch.mar().write_value(mem_addr as u32);
                ch.ndtr().write(|w| w.set_ndt(mem_len as u16));
                ch.ctrl().write(|w| {
                    w.set_psize(peri_size.into());
                    w.set_msize(mem_size.into());
                    match incr_mem {
                        Increment::None => {
                            w.set_minc(false);
                            w.set_pinc(false);
                        }
                        Increment::Peripheral => {
                            w.set_minc(false);
                            w.set_pinc(true);
                        }
                        Increment::Memory => {
                            w.set_minc(true);
                            w.set_pinc(false);
                        }
                        Increment::Both => {
                            w.set_minc(true);
                            w.set_pinc(true);
                        }
                    }
                    w.set_dir(dir.into());
                    w.set_teie(true);
                    w.set_tcie(options.complete_transfer_ir);
                    w.set_htie(options.half_transfer_ir);
                    w.set_circ(options.circular);
                    w.set_pl(options.priority.into());
                    w.set_en(false);
                });
            }
        }
    }
    fn start(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => {
                let ch = r.ch(info.num);
                ch.ctrl().modify(|_, w| w.set_en(true));
            }
        }
    }
    fn clear_irqs(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => {
                r.clr().write(|w| {
                    w.set_htif(info.num, true);
                    w.set_tcif(info.num, true);
                    w.set_teif(info.num, true);
                });
            }
        }
    }
    fn request_pause(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => {
                r.ch(info.num).ctrl().modify(|_, w| {
                    w.set_en(false);
                });
            }
        }
    }
    fn request_resume(&self) {
        self.start()
    }
    fn request_reset(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => {
                r.ch(info.num).ctrl().write(|w| {
                    w.set_teie(true);
                    w.set_tcie(true);
                });
            }
        }
        while self.is_running() {}
    }
    fn is_running(&self) -> bool {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => {
                let state: &ChannelState = &STATE[self.channel as usize];
                let ch = r.ch(info.num);
                let en = ch.ctrl().read().en();
                let circular = ch.ctrl().read().circ();
                let tcif = state.complete_count.load(Ordering::Acquire) != 0;
                en && (circular || !tcif)
            }
        }
    }
    fn get_remaining_transfers(&self) -> u32 {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(r) => r.ch(info.num).ndtr().read().ndt() as u32,
        }
    }
    fn disable_circular_mode(&self) {
        let info = self.info();
        match self.info().dma {
            DmaInfo::Bdma(regs) => regs.ch(info.num).ctrl().modify(|_, w| {
                w.set_circ(false);
            }),
        }
    }
    fn poll_stop(&self) -> Poll<()> {
        compiler_fence(Ordering::SeqCst);
        if !self.is_running() {
            fence(Ordering::Acquire);
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    }
    /// Create a memory DMA transfer (memory to memory), using raw pointers.
    pub unsafe fn transfer<'a, MW: Word, PW: Word>(
        &'a mut self,
        buf: *const [PW],
        dest_addr: *mut MW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        unsafe { self.transfer_raw(buf as *const PW, buf.len(), dest_addr, options) }
    }
    /// Create a memory DMA transfer (memory to memory), using raw pointers.
    pub unsafe fn transfer_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        src_addr: *const MW,
        src_size: usize,
        dest_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        self.configure(
            Dir::MemoryToMemory,
            src_addr as *mut u32,
            dest_addr as *mut u32,
            src_size,
            Increment::Both,
            MW::size(),
            PW::size(),
            options,
        );
        self.start();
        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }
    /// Create a read DMA transfer (peripheral to memory).
    pub unsafe fn read<'a, W: Word>(
        &'a mut self,
        peri_addr: *mut W,
        buf: &'a mut [W],
        options: TransferOptions,
    ) -> Transfer<'a> {
        self.read_raw(peri_addr, buf, options)
    }
    /// Create a read DMA transfer (peripheral to memory), using raw pointers.
    pub unsafe fn read_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        peri_addr: *mut PW,
        buf: *mut [MW],
        options: TransferOptions,
    ) -> Transfer<'a> {
        let mem_len = buf.len();
        self.configure(
            Dir::PeripheralToMemory,
            peri_addr as *const u32,
            buf as *mut MW as *mut u32,
            mem_len,
            Increment::Memory,
            MW::size(),
            PW::size(),
            options,
        );
        self.start();
        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }
    /// Create a write DMA transfer (memory to peripheral).
    pub unsafe fn write<'a, MW: Word, PW: Word>(
        &'a mut self,
        buf: &'a [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        self.write_raw(buf, peri_addr, options)
    }
    /// Create a write DMA transfer (memory to peripheral), using raw pointers.
    pub unsafe fn write_raw<'a, MW: Word, PW: Word>(
        &'a mut self,
        buf: *const [MW],
        peri_addr: *mut PW,
        options: TransferOptions,
    ) -> Transfer<'a> {
        let mem_len = buf.len();
        self.configure(
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            buf as *const MW as *mut u32,
            mem_len,
            Increment::Memory,
            MW::size(),
            PW::size(),
            options,
        );
        self.start();
        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }
    /// Create a write DMA transfer (memory to peripheral), writing the same value repeatedly.
    pub unsafe fn write_repeated<'a, W: Word>(
        &'a mut self,
        repeated: &'a W,
        count: usize,
        peri_addr: *mut W,
        options: TransferOptions,
    ) -> Transfer<'a> {
        {
            if !(count > 0 && count <= 0xFFFF) {
                ::core::panicking::panic("assertion failed: count > 0 && count <= 0xFFFF")
            }
        };
        self.configure(
            Dir::MemoryToPeripheral,
            peri_addr as *const u32,
            repeated as *const W as *mut u32,
            count,
            Increment::None,
            W::size(),
            W::size(),
            options,
        );
        self.start();
        Transfer {
            _wake_guard: self.info().wake_guard(),
            channel: self.reborrow(),
        }
    }
}
/// DMA transfer.
#[must_use = "futures do nothing unless you `.await` or poll them"]
pub struct Transfer<'a> {
    channel: Channel<'a>,
    _wake_guard: WakeGuard,
}
impl<'a> Transfer<'a> {
    /// Request the transfer to pause, keeping the existing configuration for this channel.
    ///
    /// To resume the transfer, call [`request_resume`](Self::request_resume) again.
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }
    /// Request the transfer to resume after having been paused.
    pub fn request_resume(&mut self) {
        self.channel.request_resume()
    }
    /// Request the DMA to reset.
    ///
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    pub fn request_reset(&mut self) {
        self.channel.request_reset()
    }
    /// Return whether this transfer is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_pause`](Self::request_pause).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }
    /// Gets the total remaining transfers for the channel
    /// Note: this will be zero for transfers that completed without cancellation.
    pub fn get_remaining_transfers(&self) -> u32 {
        self.channel.get_remaining_transfers()
    }
    /// Blocking wait until the transfer finishes.
    pub fn blocking_wait(mut self) {
        while self.is_running() {}
        fence(Ordering::SeqCst);
        core::mem::forget(self);
    }
    pub(crate) unsafe fn unchecked_extend_lifetime(self) -> Transfer<'static> {
        unsafe { core::mem::transmute(self) }
    }
}
impl<'a> Drop for Transfer<'a> {
    fn drop(&mut self) {
        self.request_reset();
        while self.is_running() {}
        fence(Ordering::SeqCst);
    }
}
impl<'a> Unpin for Transfer<'a> {}
impl<'a> Future for Transfer<'a> {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let state: &ChannelState = &STATE[self.channel.channel as usize];
        state.waker.register(cx.waker());
        compiler_fence(Ordering::SeqCst);
        if self.is_running() {
            Poll::Pending
        } else {
            fence(Ordering::Acquire);
            Poll::Ready(())
        }
    }
}
struct DmaCtrlImpl<'a>(Channel<'a>);
impl<'a> DmaCtrl for DmaCtrlImpl<'a> {
    fn get_remaining_transfers(&self) -> usize {
        self.0.get_remaining_transfers() as _
    }
    fn reset_complete_count(&mut self) -> usize {
        let state = &STATE[self.0.channel as usize];
        return state.complete_count.swap(0, Ordering::AcqRel);
    }
    fn set_waker(&mut self, waker: &Waker) {
        STATE[self.0.channel as usize].waker.register(waker);
    }
}
/// Ringbuffer for receiving data using DMA circular mode.
pub struct ReadableRingBuffer<'a, W: Word> {
    channel: Channel<'a>,
    _wake_guard: WakeGuard,
    ringbuf: ReadableDmaRingBuffer<'a, W>,
}
impl<'a, W: Word> ReadableRingBuffer<'a, W> {
    /// Create a new ring buffer.
    pub unsafe fn new(
        channel: Channel<'a>,
        _request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        mut options: TransferOptions,
    ) -> Self {
        let channel: Channel<'a> = channel.into();
        let buffer_ptr = buffer.as_mut_ptr();
        let len = buffer.len();
        let dir = Dir::PeripheralToMemory;
        let data_size = W::size();
        options.half_transfer_ir = true;
        options.complete_transfer_ir = true;
        options.circular = true;
        channel.configure(
            _request,
            dir,
            peri_addr as *mut u32,
            buffer_ptr as *mut u32,
            len,
            Increment::Memory,
            data_size,
            data_size,
            options,
        );
        Self {
            _wake_guard: channel.info().wake_guard(),
            channel,
            ringbuf: ReadableDmaRingBuffer::new(buffer),
        }
    }
    /// Start the ring buffer operation.
    ///
    /// You must call this after creating it for it to work.
    pub fn start(&mut self) {
        self.channel.start();
    }
    /// Set the frame alignment for the ring buffer.
    ///
    /// See [`ReadableDmaRingBuffer::set_alignment`] for details.
    pub fn set_alignment(&mut self, alignment: usize) {
        self.ringbuf.set_alignment(alignment);
    }
    /// Clear all data in the ring buffer.
    pub fn clear(&mut self) {
        self.ringbuf
            .reset(&mut DmaCtrlImpl(self.channel.reborrow()));
    }
    /// Read elements from the ring buffer
    /// Return a tuple of the length read and the length remaining in the buffer
    /// If not all of the elements were read, then there will be some elements in the buffer remaining
    /// The length remaining is the capacity, ring_buf.len(), less the elements remaining after the read
    /// Error is returned if the portion to be read was overwritten by the DMA controller.
    pub fn read(&mut self, buf: &mut [W]) -> Result<(usize, usize), Error> {
        self.ringbuf
            .read(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
    }
    /// Read an exact number of elements from the ringbuffer.
    ///
    /// Returns the remaining number of elements available for immediate reading.
    /// Error is returned if the portion to be read was overwritten by the DMA controller.
    ///
    /// Async/Wake Behavior:
    /// The underlying DMA peripheral only can wake us when its buffer pointer has reached the halfway point,
    /// and when it wraps around. This means that when called with a buffer of length 'M', when this
    /// ring buffer was created with a buffer of size 'N':
    /// - If M equals N/2 or N/2 divides evenly into M, this function will return every N/2 elements read on the DMA source.
    /// - Otherwise, this function may need up to N/2 extra elements to arrive before returning.
    pub async fn read_exact(&mut self, buffer: &mut [W]) -> Result<usize, Error> {
        self.ringbuf
            .read_exact(&mut DmaCtrlImpl(self.channel.reborrow()), buffer)
            .await
    }
    /// The current length of the ringbuffer
    pub fn len(&mut self) -> Result<usize, Error> {
        Ok(self
            .ringbuf
            .sync_len(&mut DmaCtrlImpl(self.channel.reborrow()))?)
    }
    /// Read the most recent elements from the ring buffer, discarding any older data.
    ///
    /// Returns the number of elements actually read into `buf`. Unlike [`read`](Self::read),
    /// this method **never returns an overrun error**. If the DMA has lapped the read pointer,
    /// old data is silently discarded and only the most recent samples are returned.
    ///
    /// This is ideal for use cases like ADC sampling where the consumer only cares about
    /// the latest values.
    pub fn read_latest(&mut self, buf: &mut [W]) -> usize {
        self.ringbuf
            .read_latest(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
    }
    /// The capacity of the ringbuffer
    pub const fn capacity(&self) -> usize {
        self.ringbuf.cap()
    }
    /// Set a waker to be woken when at least one byte is received.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl(self.channel.reborrow()).set_waker(waker);
    }
    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`start`](Self::start) again.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }
    /// Request the transfer to resume after having been paused.
    pub fn request_resume(&mut self) {
        self.channel.request_resume()
    }
    /// Request the DMA to reset.
    ///
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    pub fn request_reset(&mut self) {
        self.channel.request_reset()
    }
    /// Return whether DMA is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_reset`](Self::request_reset).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }
    /// Stop the DMA transfer and await until the buffer is full.
    ///
    /// This disables the DMA transfer's circular mode so that the transfer
    /// stops when the buffer is full.
    ///
    /// This is designed to be used with streaming input data such as the
    /// I2S/SAI or ADC.
    ///
    /// When using the UART, you probably want `request_reset()`.
    pub async fn stop(&mut self) {
        self.channel.disable_circular_mode();
        poll_fn(|cx| {
            self.set_waker(cx.waker());
            self.channel.poll_stop()
        })
        .await
    }
}
impl<'a, W: Word> Drop for ReadableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_reset();
        while self.is_running() {}
        fence(Ordering::SeqCst);
    }
}
/// Ringbuffer for writing data using DMA circular mode.
pub struct WritableRingBuffer<'a, W: Word> {
    channel: Channel<'a>,
    _wake_guard: WakeGuard,
    ringbuf: WritableDmaRingBuffer<'a, W>,
}
impl<'a, W: Word> WritableRingBuffer<'a, W> {
    /// Create a new ring buffer.
    pub unsafe fn new(
        channel: Channel<'a>,
        _request: Request,
        peri_addr: *mut W,
        buffer: &'a mut [W],
        mut options: TransferOptions,
    ) -> Self {
        let channel: Channel<'a> = channel.into();
        let len = buffer.len();
        let dir = Dir::MemoryToPeripheral;
        let data_size = W::size();
        let buffer_ptr = buffer.as_mut_ptr();
        options.half_transfer_ir = true;
        options.complete_transfer_ir = true;
        options.circular = true;
        channel.configure(
            _request,
            dir,
            peri_addr as *mut u32,
            buffer_ptr as *mut u32,
            len,
            Increment::Memory,
            data_size,
            data_size,
            options,
        );
        Self {
            _wake_guard: channel.info().wake_guard(),
            channel,
            ringbuf: WritableDmaRingBuffer::new(buffer),
        }
    }
    /// Start the ring buffer operation.
    ///
    /// You must call this after creating it for it to work.
    pub fn start(&mut self) {
        self.channel.start();
    }
    /// Clear all data in the ring buffer.
    pub fn clear(&mut self) {
        self.ringbuf
            .reset(&mut DmaCtrlImpl(self.channel.reborrow()));
    }
    /// Write elements directly to the raw buffer.
    /// This can be used to fill the buffer before starting the DMA transfer.
    pub fn write_immediate(&mut self, buf: &[W]) -> Result<(usize, usize), Error> {
        self.ringbuf.write_immediate(buf)
    }
    /// Write elements from the ring buffer
    /// Return a tuple of the length written and the length remaining in the buffer
    pub fn write(&mut self, buf: &[W]) -> Result<(usize, usize), Error> {
        self.ringbuf
            .write(&mut DmaCtrlImpl(self.channel.reborrow()), buf)
    }
    /// Write an exact number of elements to the ringbuffer.
    pub async fn write_exact(&mut self, buffer: &[W]) -> Result<usize, Error> {
        self.ringbuf
            .write_exact(&mut DmaCtrlImpl(self.channel.reborrow()), buffer)
            .await
    }
    /// Wait for any ring buffer write error.
    pub async fn wait_write_error(&mut self) -> Result<usize, Error> {
        self.ringbuf
            .wait_write_error(&mut DmaCtrlImpl(self.channel.reborrow()))
            .await
    }
    /// The current length of the ringbuffer
    pub fn len(&mut self) -> Result<usize, Error> {
        Ok(self
            .ringbuf
            .sync_len(&mut DmaCtrlImpl(self.channel.reborrow()))?)
    }
    /// The capacity of the ringbuffer
    pub const fn capacity(&self) -> usize {
        self.ringbuf.cap()
    }
    /// Return the current write position in the DMA buffer.
    ///
    /// See [`WritableDmaRingBuffer::write_pos`] for details.
    pub fn write_pos(&self) -> usize {
        self.ringbuf.write_pos()
    }
    /// Set a waker to be woken when at least one byte is received.
    pub fn set_waker(&mut self, waker: &Waker) {
        DmaCtrlImpl(self.channel.reborrow()).set_waker(waker);
    }
    /// Request the DMA to stop.
    /// The configuration for this channel will **not be preserved**. If you need to restart the transfer
    /// at a later point with the same configuration, see [`request_pause`](Self::request_pause) instead.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_reset(&mut self) {
        self.channel.request_reset()
    }
    /// Request the transfer to pause, keeping the existing configuration for this channel.
    /// To restart the transfer, call [`start`](Self::start) again.
    ///
    /// This doesn't immediately stop the transfer, you have to wait until [`is_running`](Self::is_running) returns false.
    pub fn request_pause(&mut self) {
        self.channel.request_pause()
    }
    /// Return whether DMA is still running.
    ///
    /// If this returns `false`, it can be because either the transfer finished, or
    /// it was requested to stop early with [`request_reset`](Self::request_reset).
    pub fn is_running(&mut self) -> bool {
        self.channel.is_running()
    }
    /// Stop the DMA transfer and await until the buffer is empty.
    ///
    /// This disables the DMA transfer's circular mode so that the transfer
    /// stops when all available data has been written.
    ///
    /// This is designed to be used with streaming output data such as the
    /// I2S/SAI or DAC.
    pub async fn stop(&mut self) {
        self.channel.disable_circular_mode();
        poll_fn(|cx| {
            self.set_waker(cx.waker());
            self.channel.poll_stop()
        })
        .await
    }
}
impl<'a, W: Word> Drop for WritableRingBuffer<'a, W> {
    fn drop(&mut self) {
        self.request_reset();
        while self.is_running() {}
        fence(Ordering::SeqCst);
    }
}
