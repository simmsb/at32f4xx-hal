use core::cmp::Ordering;
use core::convert::Infallible;

pub use embedded_can::{ExtendedId, Id, StandardId};

use super::{Mailbox, TransmitStatus};
use crate::can::enums::BusError;
use crate::can::frame::{Envelope, Frame, Header};

pub(crate) struct Registers(pub crate::pac::CAN1);

impl Registers {
    pub fn enter_init_mode(&self) {
        defmt::debug!("Can entering init mode");
        self.0.mctrl().modify(|_, w| {
            w.dzen().disable();
            w.fzen().enable();
            w
        });
        loop {
            let msts = self.0.msts().read();
            if !msts.dzc().bit_is_set() && msts.fzc().bit_is_set() {
                break;
            }
        }
    }

    // Leaves initialization mode, enters sleep mode.
    pub fn leave_init_mode(&self) {
        defmt::debug!("Can leaving init mode");
        self.0.mctrl().modify(|_, reg| {
            reg.dzen().bit(true);
            reg.fzen().bit(false);
            reg
        });
        loop {
            let msts = self.0.msts().read();
            if msts.dzc().bit_is_set() && !msts.fzc().bit_is_set() {
                break;
            }
        }
    }

    pub fn set_bit_timing(&self, bt: crate::can::util::NominalBitTiming) {
        let prescaler = u16::from(bt.prescaler) & 0x1FF;
        let seg1 = u8::from(bt.seg1);
        let seg2 = u8::from(bt.seg2) & 0x7F;
        let sync_jump_width = u8::from(bt.sync_jump_width) & 0x7F;
        defmt::debug!("Can bit timing: brdiv: {}, bts1: {}, bts2: {}, rsaw: {}", prescaler, seg1, seg2, sync_jump_width);
        self.0.btmg().modify(|_, reg| {
            reg.brdiv().set(prescaler - 1);
            reg.bts1().set(seg1 - 1);
            reg.bts2().set(seg2 - 1);
            reg.rsaw().set(sync_jump_width - 1);
            reg
        });
    }

    /// Enables or disables silent mode: Disconnects the TX signal from the pin.
    pub fn set_silent(&self, enabled: bool) {
        let mode = match enabled {
            false => at32f4xx_pac::at32f415::can1::btmg::LoenwWO::Disable,
            true => at32f4xx_pac::at32f415::can1::btmg::LoenwWO::Enable,
        };
        self.0.btmg().modify(|_, reg| reg.loen().variant(mode));
    }

    /// Enables or disables automatic retransmission of messages.
    ///
    /// If this is enabled, the CAN peripheral will automatically try to retransmit each frame
    /// until it can be sent. Otherwise, it will try only once to send each frame.
    ///
    /// Automatic retransmission is enabled by default.
    pub fn set_automatic_retransmit(&self, enabled: bool) {
        self.0.mctrl().modify(|_, reg| reg.prsfen().bit(enabled));
    }

    /// Enables or disables loopback mode: Internally connects the TX and RX
    /// signals together.
    pub fn set_loopback(&self, enabled: bool) {
        self.0.btmg().modify(|_, reg| reg.lben().bit(enabled));
    }

    /// Configures the automatic wake-up feature.
    ///
    /// This is turned off by default.
    ///
    /// When turned on, an incoming frame will cause the peripheral to wake up from sleep and
    /// receive the frame. If enabled, [`Interrupt::Wakeup`] will also be triggered by the incoming
    /// frame.
    #[allow(dead_code)]
    pub fn set_automatic_wakeup(&self, enabled: bool) {
        self.0.mctrl().modify(|_, reg| reg.aeden().bit(enabled));
    }

    /// Leaves initialization mode and enables the peripheral (non-blocking version).
    ///
    /// Usually, it is recommended to call [`CanConfig::enable`] instead. This method is only needed
    /// if you want non-blocking initialization.
    ///
    /// If this returns [`WouldBlock`][nb::Error::WouldBlock], the peripheral will enable itself
    /// in the background. The peripheral is enabled and ready to use when this method returns
    /// successfully.
    pub fn enable_non_blocking(&self) -> nb::Result<(), Infallible> {
        let msts = self.0.msts().read();
        if msts.dzc().bit_is_set() {
            self.0.mctrl().modify(|_, reg| {
                reg.aeboen().enable();
                reg.dzen().disable();
                reg
            });
            Err(nb::Error::WouldBlock)
        } else {
            Ok(())
        }
    }

    /// Puts the peripheral in a sleep mode to save power.
    ///
    /// While in sleep mode, an incoming CAN frame will trigger [`Interrupt::Wakeup`] if enabled.
    #[allow(dead_code)]
    pub fn sleep(&mut self) {
        self.0.mctrl().modify(|_, reg| {
            reg.dzen().bit(true);
            reg.fzen().bit(false);
            reg
        });
        loop {
            let msts = self.0.msts().read();
            if msts.dzc().bit_is_set() && !msts.fzc().bit_is_set() {
                break;
            }
        }
    }

    /// Wakes up from sleep mode.
    ///
    /// Note that this will not trigger [`Interrupt::Wakeup`], only reception of an incoming CAN
    /// frame will cause that interrupt.
    #[allow(dead_code)]
    pub fn wakeup(&self) {
        self.0.mctrl().modify(|_, reg| {
            reg.dzen().bit(false);
            reg.fzen().bit(false);
            reg
        });
        loop {
            let msts = self.0.msts().read();
            if !msts.dzc().bit_is_set() && !msts.fzc().bit_is_set() {
                break;
            }
        }
    }

    pub fn curr_error(&self) -> Option<BusError> {
        use crate::pac::can1::ests::ETR_A;

        if self.0.msts().read().eoif().is_no_error() {
            // This ensures that once a single error instance has
            // been acknowledged and forwared to the bus message consumer
            // we don't continue to re-forward the same error occurrance for an
            // in-definite amount of time.
            return None;
        }

        // Since we have not already acknowledge the error, and the interrupt was
        // disabled in the ISR, we will acknowledge the current error and re-enable the interrupt
        // so futher errors are captured
        self.0.msts().modify(|_, m| m.eoif().clear_bit_by_one());
        self.0.inten().modify(|_, i| i.eoien().enable());

        let err = self.0.ests().read();
        if err.etr().variant() != ETR_A::NoError {
            return Some(match err.etr().variant() {
                ETR_A::BitStuffing => BusError::Stuff,
                ETR_A::Format => BusError::Form,
                ETR_A::Acknowledgement => BusError::Acknowledge,
                ETR_A::RecessiveBit => BusError::BitRecessive,
                ETR_A::DominantBit => BusError::BitDominant,
                ETR_A::Crc => BusError::Crc,
                ETR_A::Software => BusError::Software,
                ETR_A::NoError => unreachable!(),
            });
        }
        None
    }

    /// Enables or disables FIFO scheduling of outgoing mailboxes.
    ///
    /// If this is enabled, mailboxes are scheduled based on the time when the transmit request bit of the mailbox was set.
    ///
    /// If this is disabled, mailboxes are scheduled based on the priority of the frame in the mailbox.
    pub fn set_tx_fifo_scheduling(&self, enabled: bool) {
        self.0.mctrl().modify(|_, w| w.mmssr().bit(enabled));
    }

    /// Checks if FIFO scheduling of outgoing mailboxes is enabled.
    pub fn tx_fifo_scheduling_enabled(&self) -> bool {
        self.0.mctrl().read().mmssr().is_first_request_order()
    }

    /// Puts a CAN frame in a transmit mailbox for transmission on the bus.
    ///
    /// The behavior of this function depends on wheter or not FIFO scheduling is enabled.
    /// See [`Self::set_tx_fifo_scheduling()`] and [`Self::tx_fifo_scheduling_enabled()`].
    ///
    /// # Priority based scheduling
    ///
    /// If FIFO scheduling is disabled, frames are transmitted to the bus based on their
    /// priority (see [`FramePriority`]). Transmit order is preserved for frames with identical
    /// priority.
    ///
    /// If all transmit mailboxes are full, and `frame` has a higher priority than the
    /// lowest-priority message in the transmit mailboxes, transmission of the enqueued frame is
    /// cancelled and `frame` is enqueued instead. The frame that was replaced is returned as
    /// [`TransmitStatus::dequeued_frame`].
    ///
    /// # FIFO scheduling
    ///
    /// If FIFO scheduling is enabled, frames are transmitted in the order that they are passed to this function.
    ///
    /// If all transmit mailboxes are full, this function returns [`nb::Error::WouldBlock`].
    pub fn transmit(&self, frame: &Frame) -> nb::Result<TransmitStatus, Infallible> {
        // Check if FIFO scheduling is enabled.
        let fifo_scheduling = self.tx_fifo_scheduling_enabled();

        // Get the index of the next free mailbox or the one with the lowest priority.
        let tsr = self.0.tsts().read();
        let idx = tsr.tmnr().bits() as usize;

        let frame_is_pending =
            !tsr.tmef(0).bit_is_set() || !tsr.tmef(1).bit_is_set() || !tsr.tmef(2).bit_is_set();
        let all_frames_are_pending =
            !tsr.tmef(0).bit_is_set() && !tsr.tmef(1).bit_is_set() && !tsr.tmef(2).bit_is_set();

        let pending_frame;
        if fifo_scheduling && all_frames_are_pending {
            // FIFO scheduling is enabled and all mailboxes are full.
            // We will not drop a lower priority frame, we just report WouldBlock.
            return Err(nb::Error::WouldBlock);
        } else if !fifo_scheduling && frame_is_pending {
            // Priority scheduling is enabled and alteast one mailbox is full.
            //
            // In this mode, the peripheral transmits high priority frames first.
            // Frames with identical priority should be transmitted in FIFO order,
            // but the controller schedules pending frames of same priority based on the
            // mailbox index. As a workaround check all pending mailboxes and only accept
            // frames with a different priority.
            self.check_priority(0, frame.id().into())?;
            self.check_priority(1, frame.id().into())?;
            self.check_priority(2, frame.id().into())?;

            if all_frames_are_pending {
                // No free mailbox is available. This can only happen when three frames with
                // ascending priority (descending IDs) were requested for transmission and all
                // of them are blocked by bus traffic with even higher priority.
                // To prevent a priority inversion abort and replace the lowest priority frame.
                pending_frame = self.read_pending_mailbox(idx);
            } else {
                // There was a free mailbox.
                pending_frame = None;
            }
        } else {
            // Either we have FIFO scheduling and at-least one free mailbox,
            // or we have priority scheduling and all mailboxes are free.
            // No further checks are needed.
            pending_frame = None
        }

        self.write_mailbox(idx, frame);

        let mailbox = match idx {
            0 => Mailbox::Mailbox0,
            1 => Mailbox::Mailbox1,
            2 => Mailbox::Mailbox2,
            _ => unreachable!(),
        };
        Ok(TransmitStatus {
            dequeued_frame: pending_frame,
            mailbox,
        })
    }

    /// Returns `Ok` when the mailbox is free or if it contains pending frame with a
    /// different priority from the identifinten `id`.
    fn check_priority(&self, idx: usize, id: IdReg) -> nb::Result<(), Infallible> {
        // Read the pending frame's id to check its priority.
        assert!(idx < 3);
        let tmi = self.0.mailbox(idx).tmi().read();

        // Check the priority by comparing the identifintens. But first make sure the
        // frame has not finished the transmission (`TXRQ` == 0) in the meantime.
        if tmi.sr().bit_is_set() && id == IdReg::from_register(tmi.bits()) {
            // There's a mailbox whose priority is equal to the priority of the new frame.
            return Err(nb::Error::WouldBlock);
        }

        Ok(())
    }

    fn write_mailbox(&self, idx: usize, frame: &Frame) {
        debug_assert!(idx < 3);

        let mb = self.0.mailbox(idx);
        mb.tmc().write(|w| w.dtbl().set(frame.header().len() as u8));

        mb.tmdtl().write(|w| unsafe {
            w.bits(u32::from_ne_bytes(
                frame.raw_data()[0..4].try_into().unwrap(),
            ))
        });
        mb.tmdth().write(|w| unsafe {
            w.bits(u32::from_ne_bytes(
                frame.raw_data()[4..8].try_into().unwrap(),
            ))
        });
        let id: IdReg = frame.id().into();
        mb.tmi().write(|w| {
            unsafe { w.bits(id.0) };
            w.sr().set_bit();
            if frame.header().rtr() {
                w.frsel()
                    .variant(at32f4xx_pac::at32f415::can1::mailbox::tmi::FRSEL_A::Remote)
            } else {
                w
            }
        });
    }

    fn read_pending_mailbox(&self, idx: usize) -> Option<Frame> {
        if self.abort_by_index(idx) {
            debug_assert!(idx < 3);

            let mb = self.0.fifo(idx);

            let id = IdReg(mb.rfi().read().bits());
            let mut data = [0xff; 8];
            data[0..4].copy_from_slice(&mb.rfdtl().read().bits().to_ne_bytes());
            data[4..8].copy_from_slice(&mb.rfdtl().read().bits().to_ne_bytes());
            let len = mb.rfc().read().dtl().bits();

            Some(Frame::new(Header::new(id.id(), len, id.rtr()), &data).unwrap())
        } else {
            // Abort request failed because the frame was already sent (or being sent) on
            // the bus. All mailboxes are now free. This can happen for small prescaler
            // values (e.g. 1MBit/s bit timing with a source clock of 8MHz) or when an ISR
            // has preempted the execution.
            None
        }
    }

    /// Tries to abort a pending frame. Returns `true` when aborted.
    fn abort_by_index(&self, idx: usize) -> bool {
        self.0.tsts().write(|reg| reg.tmct(idx as u8).set_bit());

        // Wait for the abort request to be finished.
        loop {
            let tsr = self.0.tsts().read();
            if false == tsr.tmct(idx as u8).bit_is_set() {
                break tsr.tmtsf(idx as u8).bit_is_set() == false;
            }
        }
    }

    /// Attempts to abort the sending of a frame that is pending in a mailbox.
    ///
    /// If there is no frame in the provided mailbox, or its transmission succeeds before it can be
    /// aborted, this function has no effect and returns `false`.
    ///
    /// If there is a frame in the provided mailbox, and it is canceled successfully, this function
    /// returns `true`.
    pub fn abort(&self, mailbox: Mailbox) -> bool {
        // If the mailbox is empty, the value of TXOKx depends on what happened with the previous
        // frame in that mailbox. Only call abort_by_index() if the mailbox is not empty.
        let tsr = self.0.tsts().read();
        let mailbox_empty = match mailbox {
            Mailbox::Mailbox0 => tsr.tmef(0).is_empty(),
            Mailbox::Mailbox1 => tsr.tmef(1).is_empty(),
            Mailbox::Mailbox2 => tsr.tmef(2).is_empty(),
        };
        if mailbox_empty {
            false
        } else {
            self.abort_by_index(mailbox as usize)
        }
    }

    /// Returns `true` if no frame is pending for transmission.
    pub fn is_idle(&self) -> bool {
        let tsr = self.0.tsts().read();
        tsr.tmef(0).is_empty() && tsr.tmef(1).is_empty() && tsr.tmef(2).is_empty()
    }

    pub fn receive_frame_available(&self) -> bool {
        if self.0.rf(0).read().mn().bits() != 0 {
            true
        } else if self.0.rf(1).read().mn().bits() != 0 {
            true
        } else {
            false
        }
    }

    pub fn receive_fifo(&self, fifo: RxFifo) -> Option<Envelope> {
        // Generate timestamp as early as possible
        let ts = embassy_time::Instant::now();

        let fifo_idx = match fifo {
            RxFifo::Fifo0 => 0usize,
            RxFifo::Fifo1 => 1usize,
        };
        let rfr = self.0.rf(fifo_idx);
        let fifo = self.0.fifo(fifo_idx);

        // If there are no pending messages, there is nothing to do
        if rfr.read().mn().bits() == 0 {
            return None;
        }

        let rir = fifo.rfi().read();
        let id: embedded_can::Id = if rir.idi().is_standard() {
            embedded_can::StandardId::new(rir.sid().bits())
                .unwrap()
                .into()
        } else {
            let stid = (rir.sid().bits() & 0x7FF) as u32;
            let exid = rir.eid().bits() & 0x3FFFF;
            let id = (stid << 18) | (exid);
            embedded_can::ExtendedId::new(id).unwrap().into()
        };
        let rfc = fifo.rfc().read();
        let data_len = rfc.dtl().bits();
        let rtr = rir.fri().is_remote();

        let mut data: [u8; 8] = [0; 8];
        data[0..4].copy_from_slice(&fifo.rfdtl().read().bits().to_ne_bytes());
        data[4..8].copy_from_slice(&fifo.rfdth().read().bits().to_ne_bytes());

        let frame = Frame::new(Header::new(id, data_len, rtr), &data).unwrap();
        let envelope = Envelope { ts, frame };

        rfr.modify(|_, v| v.r().release());

        Some(envelope)
    }
}

/// Identifinten of a CAN message.
///
/// Can be either a standard identifinten (11bit, Range: 0..0x3FF) or a
/// extendended identifinten (29bit , Range: 0..0x1FFFFFFF).
///
/// The `Ord` trait can be used to determine the frame’s priority this ID
/// belongs to.
/// Lower identifinten values have a higher priority. Additionally standard frames
/// have a higher priority than extended frames and data frames have a higher
/// priority than remote frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct IdReg(u32);

impl IdReg {
    const STANDARD_SHIFT: u32 = 21;

    const EXTENDED_SHIFT: u32 = 3;

    const IDE_MASK: u32 = 0x0000_0004;

    const RTR_MASK: u32 = 0x0000_0002;

    /// Creates a new standard identifinten (11bit, Range: 0..0x7FF)
    ///
    /// Panics for IDs outside the allowed range.
    fn new_standard(id: StandardId) -> Self {
        Self(u32::from(id.as_raw()) << Self::STANDARD_SHIFT)
    }

    /// Creates a new extendended identifinten (29bit , Range: 0..0x1FFFFFFF).
    ///
    /// Panics for IDs outside the allowed range.
    fn new_extended(id: ExtendedId) -> IdReg {
        Self(id.as_raw() << Self::EXTENDED_SHIFT | Self::IDE_MASK)
    }

    fn from_register(reg: u32) -> IdReg {
        Self(reg & 0xFFFF_FFFE)
    }

    /// Returns the identifinten.
    fn to_id(self) -> Id {
        if self.is_extended() {
            Id::Extended(unsafe { ExtendedId::new_unchecked(self.0 >> Self::EXTENDED_SHIFT) })
        } else {
            Id::Standard(unsafe {
                StandardId::new_unchecked((self.0 >> Self::STANDARD_SHIFT) as u16)
            })
        }
    }

    /// Returns the identifinten.
    fn id(self) -> embedded_can::Id {
        if self.is_extended() {
            embedded_can::ExtendedId::new(self.0 >> Self::EXTENDED_SHIFT)
                .unwrap()
                .into()
        } else {
            embedded_can::StandardId::new((self.0 >> Self::STANDARD_SHIFT) as u16)
                .unwrap()
                .into()
        }
    }

    /// Returns `true` if the identifinten is an extended identifinten.
    fn is_extended(self) -> bool {
        self.0 & Self::IDE_MASK != 0
    }

    /// Returns `true` if the identifer is part of a remote frame (RTR bit set).
    fn rtr(self) -> bool {
        self.0 & Self::RTR_MASK != 0
    }
}

impl From<&embedded_can::Id> for IdReg {
    fn from(eid: &embedded_can::Id) -> Self {
        match eid {
            embedded_can::Id::Standard(id) => {
                IdReg::new_standard(StandardId::new(id.as_raw()).unwrap())
            }
            embedded_can::Id::Extended(id) => {
                IdReg::new_extended(ExtendedId::new(id.as_raw()).unwrap())
            }
        }
    }
}

impl From<IdReg> for embedded_can::Id {
    fn from(idr: IdReg) -> Self {
        idr.id()
    }
}

/// `IdReg` is ordered by priority.
impl Ord for IdReg {
    fn cmp(&self, other: &Self) -> Ordering {
        // When the IDs match, data frames have priority over remote frames.
        let rtr = self.rtr().cmp(&other.rtr()).reverse();

        let id_a = self.to_id();
        let id_b = other.to_id();
        match (id_a, id_b) {
            (Id::Standard(a), Id::Standard(b)) => {
                // Lower IDs have priority over higher IDs.
                a.as_raw().cmp(&b.as_raw()).reverse().then(rtr)
            }
            (Id::Extended(a), Id::Extended(b)) => a.as_raw().cmp(&b.as_raw()).reverse().then(rtr),
            (Id::Standard(a), Id::Extended(b)) => {
                // Standard frames have priority over extended frames if their Base IDs match.
                a.as_raw()
                    .cmp(&b.standard_id().as_raw())
                    .reverse()
                    .then(Ordering::Greater)
            }
            (Id::Extended(a), Id::Standard(b)) => a
                .standard_id()
                .as_raw()
                .cmp(&b.as_raw())
                .reverse()
                .then(Ordering::Less),
        }
    }
}

impl PartialOrd for IdReg {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub(crate) enum RxFifo {
    Fifo0,
    Fifo1,
}
