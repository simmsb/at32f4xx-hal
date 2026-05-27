//! Serial Peripheral Interface (SPI)
#![macro_use]

use core::ptr;
use core::{future::poll_fn, marker::PhantomData, task::Poll};

use crate::interrupt;
use cortex_m::peripheral::NVIC;
use embassy_futures::join::join;
use embassy_sync::waitqueue::AtomicWaker;
pub use embedded_hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3, Mode, Phase, Polarity};

use crate::{
    crm,
    gpio::{EAPin, EraseAlt as _, alt::SpiCommon},
};
use crate::{crm::Clocks, gpio::Input};

/// SPI error.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Invalid framing.
    Framing,
    /// CRC error (only if hardware CRC checking is enabled).
    Crc,
    /// Mode fault
    ModeFault,
    /// Overrun.
    Overrun,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let message = match self {
            Self::Framing => "Invalid Framing",
            Self::Crc => "Hardware CRC Check Failed",
            Self::ModeFault => "Mode Fault",
            Self::Overrun => "Buffer Overrun",
        };

        write!(f, "{}", message)
    }
}

impl core::error::Error for Error {}

/// SPI bit order
#[derive(Copy, Clone)]
pub enum BitOrder {
    /// Least significant bit first.
    Ltf,
    /// Most significant bit first.
    MsbFirst,
}

/// SPI Direction.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Direction {
    /// Transmit
    Transmit,
    /// Receive
    Receive,
}

/// Slave Select (SS) pin polarity.
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SlaveSelectPolarity {
    /// SS active high
    ActiveHigh,
    /// SS active low
    ActiveLow,
}

/// SPI configuration.
#[non_exhaustive]
#[derive(Copy, Clone)]
pub struct Config {
    /// SPI mode.
    pub mode: Mode,
    /// Bit order.
    pub bit_order: BitOrder,
    /// Clock frequency.
    pub frequency: Hertz,
    /// Enable internal pullup on MISO.
    ///
    /// There are some ICs that require a pull-up on the MISO pin for some applications.
    /// If you  are unsure, you probably don't need this.
    pub miso_pull: Pull,
    /// signal rise/fall speed (slew rate) - defaults to `VeryHigh`.
    /// Increase for high SPI speeds. Change to `Low` to reduce ringing.
    pub gpio_speed: Speed,
    /// If True sets HWCSOE to zero even if SPI is in Master Mode.
    /// NSS output enabled (SWCSEN = 0, HWCSOE = 1): The NSS signal is driven low when the master starts the communication and is kept low until the SPI is disabled.
    /// NSS output disabled (SWCSEN = 0, HWCSOE = 0): For devices set as slave, the NSS pin acts as a claswcsilcal NSS input: the slave is selected when NSS is low and deselected when NSS high.
    pub nss_output_disable: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            mode: MODE_0,
            bit_order: BitOrder::MsbFirst,
            frequency: Hertz::Hz(1_000_000),
            miso_pull: Pull::None,
            gpio_speed: Speed::High,
            nss_output_disable: false,
        }
    }
}

mod vals {
    pub type Clkpha = crate::pac::spi1::ctrl1::CLKPHA_A;
    pub type Cpol = crate::pac::spi1::ctrl1::CLKPOL_A;
    pub type Ltf = crate::pac::spi1::ctrl1::LTF_A;
    pub type Msten = crate::pac::spi1::ctrl1::MSTEN_A;
    pub type Slben = crate::pac::spi1::ctrl1::SlbenwWO;
    pub type Ora = crate::pac::spi1::ctrl1::ORA_A;
    pub type Bidioe = crate::pac::spi1::ctrl1::SLBTD_A;
    pub type Fbn = crate::pac::spi1::ctrl1::FBN_A;
}

impl Config {
    fn raw_phase(&self) -> vals::Clkpha {
        match self.mode.phase {
            Phase::CaptureOnSecondTransition => vals::Clkpha::Second,
            Phase::CaptureOnFirstTransition => vals::Clkpha::First,
        }
    }

    fn raw_polarity(&self) -> vals::Cpol {
        match self.mode.polarity {
            Polarity::IdleHigh => vals::Cpol::High,
            Polarity::IdleLow => vals::Cpol::Low,
        }
    }

    fn raw_byte_order(&self) -> vals::Ltf {
        match self.bit_order {
            BitOrder::Ltf => vals::Ltf::Lsb,
            BitOrder::MsbFirst => vals::Ltf::Msb,
        }
    }

    // fn sck_af(&self) -> AfType {
    //     AfType::output(OutputType::PushPull, self.gpio_speed)
    // }
}

pub trait Instance: crate::Sealed + crm::Enable + crm::Reset + crm::BusClock + SpiCommon {
    fn info() -> &'static Info;
}

static SPI1_INFO: Info = Info {
    regs: crate::pac::SPI1::PTR,
    rcc: RccStuff {
        enable_and_reset: || {
            use crate::crm::Enable;
            use crate::crm::Reset;
            unsafe {
                crate::pac::SPI1::enable_unchecked();
                crate::pac::SPI1::reset_unchecked();
            }
        },
        disable: || unsafe { <crate::pac::SPI1 as crate::crm::Enable>::disable_unchecked() },
    },
    waker: AtomicWaker::new(),
};

impl Instance for crate::pac::SPI1 {
    fn info() -> &'static Info {
        &SPI1_INFO
    }
}

/// SPI communication mode
pub mod mode {
    use super::vals;

    trait SealedMode {}

    /// Trait for SPI communication mode operations.
    #[allow(private_bounds)]
    pub trait CommunicationMode: SealedMode {
        /// Spi communication mode
        const MASTER: vals::Msten;
    }

    /// Mode allowing for SPI master operations.
    pub struct Master;
    /// Mode allowing for SPI slave operations.
    pub struct Slave;

    impl SealedMode for Master {}
    impl CommunicationMode for Master {
        const MASTER: vals::Msten = vals::Msten::Master;
    }

    impl SealedMode for Slave {}
    impl CommunicationMode for Slave {
        const MASTER: vals::Msten = vals::Msten::Slave;
    }
}
use crate::{
    gpio::{Pull, Speed},
    time::Hertz,
};
use mode::{CommunicationMode, Master, Slave};

/// SPI driver.
pub struct Spi<CM: CommunicationMode> {
    pub(crate) info: &'static Info,
    kernel_clock: Hertz,
    _sck: Option<EAPin<PushPull>>,
    _mosi: Option<EAPin<PushPull>>,
    miso: Option<EAPin<Input>>,
    nss: Option<EAPin<PushPull>>,
    // tx_dma: Option<ChannelAndRequest>,
    // rx_dma: Option<ChannelAndRequest>,
    _phantom: PhantomData<CM>,
    current_word_size: word_impl::Config,
    gpio_speed: Speed,
}

impl<CM: CommunicationMode> Spi<CM> {
    fn new_inner<T: Instance>(
        _peri: T,
        sck: Option<EAPin<PushPull>>,
        mosi: Option<EAPin<PushPull>>,
        miso: Option<EAPin<Input>>,
        nss: Option<EAPin<PushPull>>,
        // tx_dma: Option<ChannelAndRequest>,
        // rx_dma: Option<ChannelAndRequest>,
        config: Config,
        clocks: &Clocks,
    ) -> Self {
        let mut this = Self {
            info: T::info(),
            kernel_clock: T::clock(clocks),
            _sck: sck,
            _mosi: mosi,
            miso,
            nss,
            // tx_dma,
            // rx_dma,
            current_word_size: <u8 as SealedWord>::CONFIG,
            _phantom: PhantomData,
            gpio_speed: config.gpio_speed,
        };
        this.enable_and_init(config);
        this
    }

    fn enable_and_init(&mut self, config: Config) {
        let br = compute_baud_rate(self.kernel_clock, config.frequency);
        let clkpha = config.raw_phase();
        let cpol = config.raw_polarity();
        let ltf = config.raw_byte_order();

        defmt::trace!("SPI init with br: {}, cpol: {}, clkpha: {}", br, defmt::Debug2Format(&cpol), defmt::Debug2Format(&clkpha));

        (self.info.rcc.enable_and_reset)();

        unsafe {
            NVIC::unpend(crate::interrupt::SPI1);
            NVIC::unmask(crate::interrupt::SPI1);
        }

        /*
        - Software NSS management (SWCSEN = 1)
        The slave select information is driven internally by the value of the SWCSIL bit in the
        SPI_CTRL1 register. The external NSS pin remains free for other application uses.

        - Hardware NSS management (SWCSEN = 0)
        Two configurations are poswcsilble depending on the NSS output configuration (HWCSOE bit
        in register SPI_CTRL1).

        -- NSS output enabled (SWCSEN = 0, HWCSOE = 1)
          This configuration is used only when the device operates in master mode. The
          NSS signal is driven low when the master starts the communication and is kept
          low until the SPI is disabled.

        -- NSS output disabled (SWCSEN = 0, HWCSOE = 0)
            This configuration allows multimaster capability for devices operating in master
            mode. For devices set as slave, the NSS pin acts as a claswcsilcal NSS input: the
            slave is selected when NSS is low and deselected when NSS high
         */
        let swcsen = self.nss.is_none();

        let regs = self.info.regs();
        {
            let hwcsoe = CM::MASTER == vals::Msten::Master && !config.nss_output_disable;
            regs.ctrl2().modify(|_, w| w.hwcsoe().bit(hwcsoe));
            regs.ctrl1().modify(|_, w| {
                w.clkpha().variant(clkpha);
                w.clkpol().variant(cpol);

                w.msten().variant(CM::MASTER);
                w.mdiv2_0().set(br);
                w.spien().bit(true);
                w.ltf().variant(ltf);
                w.swcsil().bit(CM::MASTER == vals::Msten::Master);
                w.swcsen().bit(swcsen);
                w.ccen().bit(false);
                w.slben().disable();
                // we're doing "fake ora", by actually writing one
                // byte to TXDR for each byte we want to receive. if we
                // set OUTPUTDISABLED here, this hangs.
                w.ora().variant(vals::Ora::RxTx);
                w.fbn().variant(<u8 as SealedWord>::CONFIG)
            });
        }
    }

    /// Reconfigures it with the supplied config.
    pub fn set_config(&mut self, config: &Config) -> Result<(), ()> {
        let clkpha = config.raw_phase();
        let cpol = config.raw_polarity();

        let ltf = config.raw_byte_order();

        let br = compute_baud_rate(self.kernel_clock, config.frequency);

        // TODO
        // {
        //     self.gpio_speed = config.gpio_speed;
        //     if let Some(sck) = self._sck.as_ref() {
        //         sck.
        //         sck.pin.set_spiened(config.gpio_speed);
        //     }
        //     if let Some(mosi) = self._mosi.as_ref() {
        //         mosi.pin.set_spiened(config.gpio_speed);
        //     }
        // }

        {
            self.info.regs().ctrl1().modify(|_, w| w.spien().disable());
            self.info.regs().ctrl1().modify(|_, w| {
                w.clkpha().variant(clkpha);
                w.clkpol().variant(cpol);
                w.mdiv2_0().set(br);
                w.ltf().variant(ltf)
            });
            self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
        }

        Ok(())
    }

    /// Set SPI direction for bidirectional mode.
    ///
    /// This properly handles the STM32 requirement that BIDIOE cannot be changed
    /// while the SPI peripheral is enabled (SPE=1). Per the STM32 reference manual,
    /// we must wait for TXE=1 and BSY=0 before disabling SPE to ensure any ongoing
    /// transfer completes cleanly.
    ///
    /// The SPE state is preserved: if SPI was enabled before this call, it will
    /// be re-enabled after; if it was disabled, it remains disabled.
    pub fn set_direction(&mut self, dir: Option<Direction>) {
        let (slben, bidioe) = match dir {
            Some(Direction::Transmit) => (vals::Slben::Enable, vals::Bidioe::Transmit),
            Some(Direction::Receive) => (vals::Slben::Enable, vals::Bidioe::Receive),
            None => (vals::Slben::Disable, vals::Bidioe::Transmit),
        };

        let was_enabled = self.info.regs().ctrl1().read().spien().is_enabled();

        // If SPE is currently enabled, wait for any ongoing transfer to complete.
        // Per STM32 reference manual: wait for TXE=1 then BSY=0 before disabling SPE.
        if was_enabled {
            while !self.info.regs().sts().read().tdbe().bit() {}
            while self.info.regs().sts().read().bf().bit() {}
        }

        // BIDIOE cannot be changed while SPE=1, so disable first
        self.info.regs().ctrl1().modify(|_, w| w.spien().disable());
        self.info.regs().ctrl1().modify(|_, w| {
            w.slben().variant(slben);
            w.slbtd().variant(bidioe)
        });

        // Restore previous SPE state
        if was_enabled {
            self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
        }
    }

    /// Get current SPI configuration.
    pub fn get_current_config(&self) -> Config {
        let cfg = self.info.regs().ctrl1().read();

        let hwcsoe = self.info.regs().ctrl2().read().hwcsoe().bit();

        let polarity = if cfg.clkpol() == vals::Cpol::Low {
            Polarity::IdleLow
        } else {
            Polarity::IdleHigh
        };
        let phase = if cfg.clkpha() == vals::Clkpha::First {
            Phase::CaptureOnFirstTransition
        } else {
            Phase::CaptureOnSecondTransition
        };

        let bit_order = if cfg.ltf() == vals::Ltf::Lsb {
            BitOrder::Ltf
        } else {
            BitOrder::MsbFirst
        };

        let miso_pull = match &self.miso {
            None => Pull::None,
            // TODO: fix
            Some(pin) => Pull::None,
            // Some(pin) => pin.pin.pull(),
        };

        let br = cfg.mdiv2_0().bits();

        let frequency = compute_frequency(self.kernel_clock, br);

        // NSS output disabled if HWCSOE=0 or if SWCSEN=1 software slave management enabled
        let nss_output_disable = !hwcsoe || cfg.swcsen().bit();

        Config {
            mode: Mode { polarity, phase },
            bit_order,
            frequency,
            miso_pull,
            gpio_speed: self.gpio_speed,
            nss_output_disable,
        }
    }

    pub(crate) fn set_word_size(&mut self, word_size: word_impl::Config) {
        if self.current_word_size == word_size {
            return;
        }

        self.info.regs().ctrl1().modify(|_, w| w.spien().disable());

        self.info
            .regs()
            .ctrl1()
            .modify(|_, reg| reg.fbn().variant(word_size));
        self.current_word_size = word_size;
    }

    // /// Blocking write.
    // pub fn blocking_write<W: Word>(&mut self, words: &[W]) -> Result<(), Error> {
    //     // needed in v3+ to avoid overrun causing the SPI RX state machine to get stuck...?
    //     self.set_word_size(W::CONFIG);
    //     self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
    //     flush_rx_fifo(self.info.regs());
    //     for word in words.iter() {
    //         // if we're doing tx only, after writing the last byte to FIFO we have to wait
    //         // until it's actually sent. On SPIv1 you're supposed to use the BSY flag for this
    //         // but apparently it's broken, it clears too soon. Workaround is to wait for RXNE:
    //         // when it gets set you know the transfer is done, even if you don't care about rx.
    //         // Luckily this doesn't affect SPIv2+.
    //         // See http://efton.sk/STM32/gotcha/g68.html
    //         // ST doesn't seem to document this in errata sheets (?)
    //         transfer_word(self.info, *word).await?;
    //     }

    //     Ok(())
    // }

    // /// Blocking read.
    // pub fn blocking_read<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
    //     // needed in v3+ to avoid overrun causing the SPI RX state machine to get stuck...?
    //     self.set_word_size(W::CONFIG);
    //     self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
    //     flush_rx_fifo(self.info.regs());
    //     for word in words.iter_mut() {
    //         *word = transfer_word(self.info, W::default())?;
    //     }
    //     Ok(())
    // }

    // /// Blocking in-place bidirectional transfer.
    // ///
    // /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    // pub fn blocking_transfer_in_place<W: Word>(&mut self, words: &mut [W]) -> Result<(), Error> {
    //     // needed in v3+ to avoid overrun causing the SPI RX state machine to get stuck...?
    //     self.set_word_size(W::CONFIG);
    //     self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
    //     flush_rx_fifo(self.info.regs());
    //     for word in words.iter_mut() {
    //         *word = transfer_word(self.info, *word)?;
    //     }
    //     Ok(())
    // }

    // Blocking bidirectional transfer.
    //
    // This transfers both buffers at the same time, so it is NOT equivalent to `write` followed by `read`.
    //
    // The transfer runs for `max(read.len(), write.len())` bytes. If `read` is shorter extra bytes are ignored.
    // If `write` is shorter it is padded with zero bytes.
    // pub async fn blocking_transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
    //     // needed in v3+ to avoid overrun causing the SPI RX state machine to get stuck...?
    //     self.set_word_size(W::CONFIG);
    //     self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
    //     flush_rx_fifo(self.info.regs());
    //     let len = read.len().max(write.len());
    //     for i in 0..len {
    //         let wb = write.get(i).copied().unwrap_or_default();
    //         let rb = transfer_word(self.info, wb).await?;
    //         if let Some(r) = read.get_mut(i) {
    //             *r = rb;
    //         }
    //     }
    //     Ok(())
    // }
}

impl Spi<Master> {
    /// Create a new SPI driver.
    pub fn new<T: Instance>(
        peri: T,
        sck: Option<impl Into<T::Sck>>,
        mosi: Option<impl Into<T::Mosi>>,
        miso: Option<impl Into<T::Miso>>,
        nss: Option<impl Into<T::Nss>>,
        config: Config,
        clocks: &Clocks,
    ) -> Self {
        Self::new_inner(
            peri,
            sck.map(Into::into).map(|p| p.erase_alt()),
            mosi.map(Into::into).map(|p| p.erase_alt()),
            miso.map(Into::into).map(|p| p.erase_alt()),
            nss.map(Into::into).map(|p| p.erase_alt()),
            config,
            clocks,
        )
    }

    #[allow(dead_code)]
    pub(crate) fn new_internal<T: Instance>(
        peri: T,
        // tx_dma: Option<ChannelAndRequest>,
        // rx_dma: Option<ChannelAndRequest>,
        config: Config,
        clocks: &Clocks,
    ) -> Self {
        Self::new_inner(
            peri, None, None, None, None, // tx_dma, rx_dma,
            config, clocks,
        )
    }
}

impl<CM: CommunicationMode> Spi<CM> {
    /// SPI write
    pub async fn write<W: Word>(&mut self, data: &[W]) -> Result<(), Error> {
        self.info.regs().ctrl1().modify(|_, w| w.spien().disable());
        self.set_word_size(W::CONFIG);
        self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
        flush_rx_fifo(self.info.regs());

        defmt::trace!("SPI Write: {}", data);

        for word in data.iter() {
            transfer_word(self.info, *word).await?;
        }

        Ok(())
    }

    /// SPI read
    pub async fn read<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        self.info.regs().ctrl1().modify(|_, w| w.spien().disable());
        self.set_word_size(W::CONFIG);
        self.info.regs().ctrl1().modify(|_, w| w.spien().enable());
        flush_rx_fifo(self.info.regs());

        defmt::trace!("SPI Read (len: {})", data.len());

        for word in data.iter_mut() {
            *word = transfer_word(self.info, W::default()).await?;
        }

        Ok(())
    }

    /// Bidirectional transfer
    ///
    /// This transfers both buffers at the same time, so it is NOT equivalent to `write` followed by `read`.
    ///
    /// The transfer runs for `max(read.len(), write.len())` bytes. If `read` is shorter extra bytes are ignored.
    /// If `write` is shorter it is padded with zero bytes.
    pub async fn transfer<W: Word>(&mut self, read: &mut [W], write: &[W]) -> Result<(), Error> {
        self.info.regs().ctrl1().modify(|_, w| w.spien().disable());
        self.set_word_size(W::CONFIG);
        self.info.regs().ctrl1().modify(|_, w| w.spien().enable());

        flush_rx_fifo(self.info.regs());

        let len = read.len().max(write.len());

        defmt::trace!("SPI Transfer (len: {}): {}", len, write);

        for i in 0..len {
            let wb = write.get(i).copied().unwrap_or_default();
            let rb = transfer_word(self.info, wb).await?;
            if let Some(r) = read.get_mut(i) {
                *r = rb;
            }
        }

        Ok(())
    }

    /// In-place bidirectional transfer, using DMA.
    ///
    /// This writes the contents of `data` on MOSI, and puts the received data on MISO in `data`, at the same time.
    pub async fn transfer_in_place<W: Word>(&mut self, data: &mut [W]) -> Result<(), Error> {
        self.info.regs().ctrl1().modify(|_, w| w.spien().disable());
        self.set_word_size(W::CONFIG);
        self.info.regs().ctrl1().modify(|_, w| w.spien().enable());

        flush_rx_fifo(self.info.regs());

        defmt::trace!("SPI Transfer in place (len: {}): {}", data.len(), data);

        for word in data.iter_mut() {
            *word = transfer_word(self.info, *word).await?;
        }

        Ok(())
    }
}

impl<CM: CommunicationMode> Drop for Spi<CM> {
    fn drop(&mut self) {
        (self.info.rcc.disable)();
    }
}

type Br = u8;

use crate::gpio::{Alternate, EPin, PushPull};

fn compute_baud_rate(kernel_clock: Hertz, freq: Hertz) -> Br {
    match kernel_clock / freq {
        0 => panic!("You are trying to reach a frequency higher than the clock"),
        1..=2 => 0b000,
        3..=5 => 0b001,
        6..=11 => 0b010,
        12..=23 => 0b011,
        24..=39 => 0b100,
        40..=95 => 0b101,
        96..=191 => 0b110,
        _ => 0b111,
    }
}

fn compute_frequency(kernel_clock: Hertz, br: Br) -> Hertz {
    let div: u16 = match br {
        0 => 2,
        1 => 4,
        2 => 8,
        3 => 16,
        4 => 32,
        5 => 64,
        6 => 128,
        7 => 256,
        _ => panic!("Nope"),
    };

    kernel_clock / div as u32
}

pub(crate) trait RegsExt {
    fn tx_ptr<W>(&self) -> *mut W;
    fn rx_ptr<W>(&self) -> *mut W;
}

impl RegsExt for Regs {
    fn tx_ptr<W>(&self) -> *mut W {
        let dr = self.dt();
        dr.as_ptr() as *mut W
    }

    fn rx_ptr<W>(&self) -> *mut W {
        let dr = self.dt();
        dr.as_ptr() as *mut W
    }
}

fn check_error_flags(sr: &crate::pac::spi1::sts::R, ovr: bool) -> Result<(), Error> {
    if sr.roerr().bit() && ovr {
        return Err(Error::Overrun);
    }
    if sr.mmerr().bit() {
        return Err(Error::ModeFault);
    }
    if sr.ccerr().bit() {
        return Err(Error::Crc);
    }

    Ok(())
}

async fn wait_until_tx_ready(info: &Info, ovr: bool) -> Result<(), Error> {
    defmt::trace!("Waiting tx ready");
    poll_fn(|cx| -> Poll<Result<(), Error>> {
        info.waker.register(cx.waker());

        let sr = info.regs().sts().read();

        defmt::trace!("Poll: {}", defmt::Debug2Format(&sr));

        if let Err(e) = check_error_flags(&sr, true) {
            return Poll::Ready(Err(e));
        }

        if sr.tdbe().is_empty() {
            Poll::Ready(Ok(()))
        } else {
            info.regs().ctrl2().modify(|_, w| w.tdbeie().enable());

            Poll::Pending
        }
    })
    .await?;
    defmt::trace!("tx ready");
    Ok(())
}

async fn wait_until_rx_ready(info: &Info) -> Result<(), Error> {
    defmt::trace!("Waiting rx ready");
    poll_fn(|cx| {
        info.waker.register(cx.waker());

        let sr = info.regs().sts().read();

        if let Err(e) = check_error_flags(&sr, true) {
            return Poll::Ready(Err(e));
        }

        if sr.rdbf().is_full() {
            Poll::Ready(Ok(()))
        } else {
            info.regs().ctrl2().modify(|_, w| w.rdbfie().enable());

            Poll::Pending
        }
    })
    .await?;
    defmt::trace!("rx ready");
    Ok(())
}

#[inline(never)]
pub(crate) fn flush_rx_fifo(regs: Regs) {
    // let sr = regs.sts().read();

    // defmt::trace!("Flush: {}", defmt::Debug2Format(&sr));
    while regs.sts().read().rdbf().is_full() {
        let _ = regs.dt().read().bits();
    }
}

async fn transfer_word<W: Word>(info: &Info, tx_word: W) -> Result<W, Error> {
    wait_until_tx_ready(info, true).await?;

    unsafe {
        ptr::write_volatile(info.regs().tx_ptr(), tx_word);
    }

    wait_until_rx_ready(info).await?;

    let rx_word = unsafe { ptr::read_volatile(info.regs().rx_ptr()) };
    Ok(rx_word)
}

// #[allow(unused)] // unused in SPIv1
// async fn write_word<W: Word>(regs: Regs, tx_word: W) -> Result<(), Error> {
//     // for write, we intentionally ignore the rx fifo, which will cause
//     // overrun errors that we have to ignore.
//     spin_until_tx_ready(regs, false)?;

//     unsafe {
//         ptr::write_volatile(regs.tx_ptr(), tx_word);
//     }
//     Ok(())
// }

// // Note: It is not poswcsilble to impl these traits generically in embedded-hal 0.2 due to a conflict with
// // some marker traits. For details, see https://github.com/rust-embedded/embedded-hal/pull/289
// macro_rules! impl_blocking {
//     ($w:ident) => {
//         impl<'d, M: PeriMode, CM: CommunicationMode> embedded_hal_02::blocking::spi::Write<$w> for Spi<'d, M, CM> {
//             type Error = Error;

//             fn write(&mut self, words: &[$w]) -> Result<(), Self::Error> {
//                 self.blocking_write(words)
//             }
//         }

//         impl<'d, M: PeriMode, CM: CommunicationMode> embedded_hal_02::blocking::spi::Transfer<$w> for Spi<'d, M, CM> {
//             type Error = Error;

//             fn transfer<'w>(&mut self, words: &'w mut [$w]) -> Result<&'w [$w], Self::Error> {
//                 self.blocking_transfer_in_place(words)?;
//                 Ok(words)
//             }
//         }
//     };
// }

// impl_blocking!(u8);
// impl_blocking!(u16);

impl<CM: CommunicationMode> embedded_hal::spi::ErrorType for Spi<CM> {
    type Error = Error;
}

// impl<W: Word, CM: CommunicationMode> embedded_hal::spi::SpiBus<W> for Spi<CM> {
//     fn flush(&mut self) -> Result<(), Self::Error> {
//         Ok(())
//     }

//     fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
//         self.blocking_read(words)
//     }

//     fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
//         self.blocking_write(words)
//     }

//     fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
//         self.blocking_transfer(read, write)
//     }

//     fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
//         self.blocking_transfer_in_place(words)
//     }
// }

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> embedded_hal::spi::ErrorKind {
        match *self {
            Self::Framing => embedded_hal::spi::ErrorKind::FrameFormat,
            Self::Crc => embedded_hal::spi::ErrorKind::Other,
            Self::ModeFault => embedded_hal::spi::ErrorKind::ModeFault,
            Self::Overrun => embedded_hal::spi::ErrorKind::Overrun,
        }
    }
}

impl<W: Word, CM: CommunicationMode> embedded_hal_async::spi::SpiBus<W> for Spi<CM> {
    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.write(words).await
    }

    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.read(words).await
    }

    async fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        self.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.transfer_in_place(words).await
    }
}

impl<W: Word, CM: CommunicationMode> embedded_hal_async::spi::SpiDevice<W> for Spi<CM> {
    async fn write(&mut self, words: &[W]) -> Result<(), Self::Error> {
        self.write(words).await
    }

    async fn read(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.read(words).await
    }

    async fn transfer(&mut self, read: &mut [W], write: &[W]) -> Result<(), Self::Error> {
        self.transfer(read, write).await
    }

    async fn transfer_in_place(&mut self, words: &mut [W]) -> Result<(), Self::Error> {
        self.transfer_in_place(words).await
    }

    async fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, W>],
    ) -> Result<(), Self::Error> {
        for op in operations {
            match op {
                embedded_hal::spi::Operation::Read(items) => {
                    self.read(items).await?;
                }
                embedded_hal::spi::Operation::Write(items) => {
                    self.write(items).await?;
                }
                embedded_hal::spi::Operation::Transfer(to_read, to_write) => {
                    self.transfer(to_read, to_write).await?;
                }
                embedded_hal::spi::Operation::TransferInPlace(items) => {
                    self.transfer_in_place(items).await?;
                }
                embedded_hal::spi::Operation::DelayNs(ns) => {
                    embassy_time::Timer::after_nanos(*ns as u64).await;
                }
            }
        }

        Ok(())
    }
}

pub(crate) trait SealedWord {
    const CONFIG: word_impl::Config;
}

/// Word sizes usable for SPI.
#[allow(private_bounds)]
pub trait Word: defmt::Format + Copy + SealedWord + Default + 'static {}

macro_rules! impl_word {
    ($T:ty, $config:expr) => {
        impl SealedWord for $T {
            const CONFIG: Config = $config;
        }
        impl Word for $T {}
    };
}

mod word_impl {
    use super::*;

    pub type Config = vals::Fbn;

    impl_word!(u8, vals::Fbn::Bit8);
    impl_word!(u16, vals::Fbn::Bit16);
}

type Regs = &'static crate::pac::spi1::RegisterBlock;

struct RccStuff {
    enable_and_reset: fn(),
    disable: fn(),
}

pub(crate) struct Info {
    pub(crate) regs: *const crate::pac::spi1::RegisterBlock,
    pub(crate) rcc: RccStuff,
    pub(crate) waker: AtomicWaker,
}

impl Info {
    fn regs(&self) -> &'static crate::pac::spi1::RegisterBlock {
        unsafe { self.regs.as_ref_unchecked() }
    }
}

unsafe impl Sync for Info {}
unsafe impl Send for Info {}

struct State {}

impl State {
    #[allow(unused)]
    const fn new() -> Self {
        Self {}
    }
}

fn on_interrupt(info: &Info) {
    let cr2 = info.regs().ctrl2().read();
    let sts = info.regs().sts().read();

    if cr2.tdbeie().is_enabled() && sts.tdbe().is_empty() {
        info.regs().ctrl2().modify(|_, w| w.tdbeie().disable());

        info.waker.wake();
    }

    if cr2.rdbfie().is_enabled() && sts.rdbf().is_full() {
        info.regs().ctrl2().modify(|_, w| w.rdbfie().disable());

        info.waker.wake();
    }
}

// #[interrupt]
// fn SPI1() {
//     on_interrupt(&SPI1_INFO);
// }

// peri_trait!();

// pin_trait!(SdExtPin, Instance);
// pin_trait!(SckPin, Instance, @A);
// pin_trait!(MosiPin, Instance, @A);
// pin_trait!(MisoPin, Instance, @A);
// pin_trait!(CsPin, Instance, @A);
// pin_trait!(MckPin, Instance, @A);
// pin_trait!(CkPin, Instance, @A);
// pin_trait!(WsPin, Instance, @A);
// pin_trait!(I2sSdPin, Instance, @A);
// dma_trait!(RxDma, Instance);
// dma_trait!(TxDma, Instance);
// dma_trait!(RxDmaExt, Instance);
