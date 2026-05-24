//! Interface to the real time clock. See STM32F303 reference manual, section 27.
//! For more details, see
//! [ST AN4759](https:/www.st.com%2Fresource%2Fen%2Fapplication_note%2Fdm00226326-using-the-hardware-realtime-clock-rtc-and-the-tamper-management-unit-tamp-with-stm32-microcontrollers-stmicroelectronics.pdf&usg=AOvVaw3PzvL2TfYtwS32fw-Uv37h)

use crate::pac;
use crate::pac::{CRM, ERTC, PWC};
use crate::{bb, crm::Enable as _};
use core::fmt;
use chrono::{Datelike, NaiveDate, NaiveDateTime, NaiveTime, Timelike as _, Weekday};
use fugit::RateExtU32;

/// Invalid input error
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Error {
    InvalidInputData,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Event {
    AlarmA,
    AlarmB,
    Wakeup,
    Timestamp,
}

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum Alarm {
    AlarmA = 0,
    AlarmB = 1,
}

impl From<Alarm> for Event {
    fn from(a: Alarm) -> Self {
        match a {
            Alarm::AlarmA => Event::AlarmA,
            Alarm::AlarmB => Event::AlarmB,
        }
    }
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum AlarmDay {
    Date(NaiveDate),
    Weekday(Weekday),
    EveryDay,
}

impl From<NaiveDate> for AlarmDay {
    fn from(date: NaiveDate) -> Self {
        Self::Date(date)
    }
}

impl From<Weekday> for AlarmDay {
    fn from(day: Weekday) -> Self {
        Self::Weekday(day)
    }
}

/// ERTC clock source LSE oscillator clock (type state)
pub struct Lse;
/// ERTC clock source LSI oscillator clock (type state)
pub struct Lsi;

#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClockSource {
    Lse(LSEClockMode),
    Lsi,
}

impl From<LSEClockMode> for ClockSource {
    fn from(value: LSEClockMode) -> Self {
        Self::Lse(value)
    }
}

impl ClockSource {
    pub fn frequency(self) -> fugit::Hertz<u32> {
        match self {
            Self::Lse(_) => 32_768_u32.Hz(),
            Self::Lsi => 32.kHz(),
        }
    }
}

/// Real Time Clock peripheral
pub struct Rtc {
    /// ERTC Peripheral register
    pub regs: ERTC,
    clock_source: ClockSource,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Rtc {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(f, "Rtc");
    }
}

impl fmt::Debug for Rtc {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("Rtc")
    }
}

/// LSE clock mode.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LSEClockMode {
    /// Enable LSE oscillator to use external crystal or ceramic resonator.
    Oscillator,
    /// Bypass LSE oscillator to use external clock source.
    /// Use this if an external oscillator is used which is not connected to `OSC32_IN` such as a MEMS resonator.
    Bypass,
}

impl Rtc {
    /// Create and enable a new ERTC with external crystal or ceramic resonator and default prescalers.
    pub fn new(regs: ERTC, rcc: &mut CRM, pwr: &mut PWC) -> Self {
        Self::with_config(regs, rcc, pwr, LSEClockMode::Oscillator, 255, 127)
    }
    /// Create and enable a new ERTC, and configure its clock source and prescalers.
    ///
    /// From AN3371, Table 3,
    /// set `divb` to 255 (249 for LSI), and `prediv_a` to 127 to get a calendar clock of 1Hz.
    pub fn with_config(
        regs: ERTC,
        rcc: &mut CRM,
        pwr: &mut PWC,
        clock_source: impl Into<ClockSource>,
        divb: u16,
        prediv_a: u8,
    ) -> Self {
        let mut result = Self {
            regs,
            clock_source: clock_source.into(),
        };

        // Steps:
        // Enable PWC and BPWEN
        // Enable LSE/LSI (if needed)
        // Enable ERTC Clock
        // Disable Write Protect
        // Enter Init
        // Configure 24 hour format
        // Set prescalers
        // Exit Init
        // Enable write protect

        // As per the sample code, unlock comes first. (Enable PWC and BPWEN)
        result.unlock(rcc, pwr);
        match result.clock_source {
            ClockSource::Lse(mode) => {
                // If necessary, enable the LSE.
                if rcc.bpdc().read().lextstbl().bit_is_clear() {
                    result.enable_lse(rcc, mode);
                }
                // Set clock source to LSE.
                rcc.bpdc().modify(|_, w| unsafe { w.ertcsel().bits(0b01) });
            }
            ClockSource::Lsi => {
                // If necessary, enable the LSE.
                if rcc.ctrlsts().read().lickstbl().bit_is_clear() {
                    result.enable_lsi(rcc);
                }
                // Set clock source to LSI.
                rcc.bpdc().modify(|_, w| unsafe { w.ertcsel().bits(0b10) });
            }
        }
        result.enable(rcc);

        result.modify(true, |regs| {
            // Set 24 Hour
            regs.ctrl().modify(|_, w| w.hm().clear_bit());
            // Set prescalers
            regs.div().modify(|_, w| {
                unsafe { w.divb().bits(divb) };
                w.diva().set(prediv_a)
            });
        });

        result
    }

    /// Enable the low frequency external oscillator. This is the only mode currently
    /// supported, to avoid exposing the `CTRL` and `CRS` registers.
    fn enable_lse(&mut self, rcc: &mut CRM, mode: LSEClockMode) {
        unsafe {
            // Force a reset of the backup domain.
            self.backup_reset(rcc);
            // Enable the LSE.
            // Set BPDC - Bit 0 (LSEON)
            bb::set(rcc.bpdc(), 0);
            match mode {
                // Set BPDC - Bit 2 (LSEBYP)
                LSEClockMode::Bypass => bb::set(rcc.bpdc(), 2),
                // Clear BPDC - Bit 2 (LSEBYP)
                LSEClockMode::Oscillator => bb::clear(rcc.bpdc(), 2),
            }
            while rcc.bpdc().read().lextstbl().bit_is_clear() {}
        }
    }

    /// Create and enable a new ERTC with internal crystal and default prescalers.
    pub fn new_lsi(regs: ERTC, rcc: &mut CRM, pwr: &mut PWC) -> Self {
        Self::with_config(regs, rcc, pwr, ClockSource::Lsi, 249, 127)
    }

    /// Create and enable a new ERTC, and configure its clock source and prescalers.
    ///
    /// From AN3371, Table 3, when using the LSI,
    /// set `divb` to 249, and `prediv_a` to 127 to get a calendar clock of 1Hz.
    pub fn lsi_with_config(
        regs: ERTC,
        rcc: &mut CRM,
        pwr: &mut PWC,
        divb: u16,
        prediv_a: u8,
    ) -> Self {
        Self::with_config(regs, rcc, pwr, ClockSource::Lsi, divb, prediv_a)
    }

    fn enable_lsi(&mut self, rcc: &mut CRM) {
        // Force a reset of the backup domain.
        self.backup_reset(rcc);
        // Enable the LSI.
        rcc.ctrlsts().modify(|_, w| w.licken().set_bit());
        while rcc.ctrlsts().read().lickstbl().bit_is_clear() {}
    }

    fn unlock(&mut self, rcc: &mut CRM, pwr: &mut PWC) {
        // Enable the backup interface
        // Set APB1 - Bit 28 (PWREN)
        PWC::enable(rcc);

        // Enable access to the backup registers
        pwr.ctrl().modify(|_, w| w.bpwen().set_bit());
    }

    fn backup_reset(&mut self, rcc: &mut CRM) {
        unsafe {
            // Set BPDC - Bit 16 (BDRST)
            bb::set(rcc.bpdc(), 16);
            // Clear BPDC - Bit 16 (BDRST)
            bb::clear(rcc.bpdc(), 16);
        }
    }

    fn enable(&mut self, rcc: &mut CRM) {
        // Start the actual ERTC.
        // Set BPDC - Bit 15 (RTCEN)
        unsafe {
            bb::set(rcc.bpdc(), 15);
        }
    }

    pub fn set_prescalers(&mut self, divb: u16, prediv_a: u8) {
        self.modify(true, |regs| {
            // Set prescalers
            regs.div().modify(|_, w| {
                unsafe { w.divb().bits(divb) };
                w.diva().set(prediv_a)
            });
        });
    }

    /// As described in Section 27.3.7 in RM0316,
    /// this function is used to disable write protection
    /// when modifying an ERTC register
    fn modify<F>(&mut self, init_mode: bool, mut closure: F)
    where
        F: FnMut(&mut ERTC),
    {
        // Disable write protection
        // This is safe, as we're only writin the correct and expected values.
        self.regs.wp().write(|w| unsafe { w.bits(0xCA) });
        self.regs.wp().write(|w| unsafe { w.bits(0x53) });
        // Enter init mode
        if init_mode && self.regs.sts().read().imf().bit_is_clear() {
            self.regs.sts().modify(|_, w| w.imen().set_bit());
            // wait till init state entered
            // ~2 RTCCLK cycles
            while self.regs.sts().read().imf().bit_is_clear() {}
        }
        // Invoke closure
        closure(&mut self.regs);
        // Exit init mode
        if init_mode {
            self.regs.sts().modify(|_, w| w.imen().clear_bit());
        }
        // wait for last write to be done
        while !self.regs.sts().read().imf().bit_is_clear() {}

        // Re-enable write protection.
        // This is safe, as the field accepts the full range of 8-bit values.
        self.regs.wp().write(|w| unsafe { w.bits(0xFF) });
    }

    /// Set the time using time::Time.
    pub fn set_time(&mut self, time: &NaiveTime) -> Result<(), Error> {
        let (ht, hu) = bcd2_encode(time.hour().into())?;
        let (mt, mu) = bcd2_encode(time.minute().into())?;
        let (st, su) = bcd2_encode(time.second().into())?;
        self.modify(true, |regs| {
            regs.time().write(|w| {
                w.ht().set(ht);
                w.hu().set(hu);
                w.mt().set(mt);
                w.mu().set(mu);
                w.st().set(st);
                w.su().set(su);
                w.ampm().clear_bit()
            });
        });

        Ok(())
    }

    /// Set the seconds [0-59].
    pub fn set_seconds(&mut self, seconds: u8) -> Result<(), Error> {
        if seconds > 59 {
            return Err(Error::InvalidInputData);
        }
        let (st, su) = bcd2_encode(seconds.into())?;
        self.modify(true, |regs| {
            regs.time().modify(|_, w| w.st().set(st).su().set(su));
        });

        Ok(())
    }

    /// Set the minutes [0-59].
    pub fn set_minutes(&mut self, minutes: u8) -> Result<(), Error> {
        if minutes > 59 {
            return Err(Error::InvalidInputData);
        }
        let (mt, mu) = bcd2_encode(minutes.into())?;
        self.modify(true, |regs| {
            regs.time().modify(|_, w| w.mt().set(mt).mu().set(mu));
        });

        Ok(())
    }

    /// Set the hours [0-23].
    pub fn set_hours(&mut self, hours: u8) -> Result<(), Error> {
        if hours > 23 {
            return Err(Error::InvalidInputData);
        }
        let (ht, hu) = bcd2_encode(hours.into())?;

        self.modify(true, |regs| {
            regs.time().modify(|_, w| w.ht().set(ht).hu().set(hu));
        });

        Ok(())
    }

    /// Set the day of week [1-7].
    pub fn set_weekday(&mut self, weekday: u8) -> Result<(), Error> {
        if !(1..=7).contains(&weekday) {
            return Err(Error::InvalidInputData);
        }
        self.modify(true, |regs| {
            regs.date().modify(|_, w| unsafe { w.wk().bits(weekday) });
        });

        Ok(())
    }

    /// Set the day of month [1-31].
    pub fn set_day(&mut self, day: u8) -> Result<(), Error> {
        if !(1..=31).contains(&day) {
            return Err(Error::InvalidInputData);
        }
        let (dt, du) = bcd2_encode(day as u32)?;
        self.modify(true, |regs| {
            regs.date().modify(|_, w| w.dt().set(dt).du().set(du));
        });

        Ok(())
    }

    /// Set the month [1-12].
    pub fn set_month(&mut self, month: u8) -> Result<(), Error> {
        if !(1..=12).contains(&month) {
            return Err(Error::InvalidInputData);
        }
        let (mt, mu) = bcd2_encode(month as u32)?;
        self.modify(true, |regs| {
            regs.date().modify(|_, w| w.mt().bit(mt > 0).mu().set(mu));
        });

        Ok(())
    }

    /// Set the year [1970-2069].
    ///
    /// The year cannot be less than 1970, since the Unix epoch is assumed (1970-01-01 00:00:00).
    /// Also, the year cannot be greater than 2069 since the ERTC range is 0 - 99.
    pub fn set_year(&mut self, year: u16) -> Result<(), Error> {
        if !(1970..=2069).contains(&year) {
            return Err(Error::InvalidInputData);
        }
        let (yt, yu) = bcd2_encode(year as u32 - 1970)?;
        self.modify(true, |regs| {
            regs.date().modify(|_, w| w.yt().set(yt).yu().set(yu));
        });

        Ok(())
    }

    /// Set the date.
    ///
    /// The year cannot be less than 1970, since the Unix epoch is assumed (1970-01-01 00:00:00).
    /// Also, the year cannot be greater than 2069 since the ERTC range is 0 - 99.
    pub fn set_date(&mut self, date: &NaiveDate) -> Result<(), Error> {
        if !(1970..=2069).contains(&date.year()) {
            return Err(Error::InvalidInputData);
        }

        let (yt, yu) = bcd2_encode((date.year() - 1970) as u32)?;
        let (mt, mu) = bcd2_encode(date.month().into())?;
        let (dt, du) = bcd2_encode(date.day().into())?;
        let wk = date.weekday().number_from_monday() as u8;

        self.modify(true, |regs| {
            regs.date().write(|w| {
                w.dt().set(dt);
                w.du().set(du);
                w.mt().bit(mt > 0);
                w.mu().set(mu);
                w.yt().set(yt);
                w.yu().set(yu);
                unsafe { w.wk().bits(wk) }
            });
        });

        Ok(())
    }

    /// Set the date and time.
    ///
    /// The year cannot be less than 1970, since the Unix epoch is assumed (1970-01-01 00:00:00).
    /// Also, the year cannot be greater than 2069 since the ERTC range is 0 - 99.
    pub fn set_datetime(&mut self, date: &NaiveDateTime) -> Result<(), Error> {
        if !(1970..=2069).contains(&date.year()) {
            return Err(Error::InvalidInputData);
        }

        let (yt, yu) = bcd2_encode((date.year() - 1970) as u32)?;
        let (mnt, mnu) = bcd2_encode(date.month().into())?;
        let (dt, du) = bcd2_encode(date.day().into())?;
        let wk = date.weekday().number_from_monday() as u8;

        let (ht, hu) = bcd2_encode(date.hour().into())?;
        let (mt, mu) = bcd2_encode(date.minute().into())?;
        let (st, su) = bcd2_encode(date.second().into())?;

        self.modify(true, |regs| {
            regs.date().write(|w| {
                w.dt().set(dt);
                w.du().set(du);
                w.mt().bit(mnt > 0);
                w.mu().set(mnu);
                w.yt().set(yt);
                w.yu().set(yu);
                unsafe { w.wk().bits(wk) }
            });
            regs.time().write(|w| {
                w.ht().set(ht);
                w.hu().set(hu);
                w.mt().set(mt);
                w.mu().set(mu);
                w.st().set(st);
                w.su().set(su);
                w.ampm().clear_bit()
            });
        });

        Ok(())
    }

    pub fn get_datetime(&mut self) -> NaiveDateTime {
        // Wait for Registers synchronization flag,  to ensure consistency between the RTC_SSR, RTC_TR and RTC_DR shadow registers.
        while self.regs.sts().read().updf().bit_is_clear() {}

        // Reading either RTC_SSR or RTC_TR locks the values in the higher-order calendar shadow registers until RTC_DR is read.
        // So it is important to always read SBS, TIME and then DATE or TIME and then DATE.
        let sbs = self.regs.sbs().read().bits();
        let time = self.regs.time().read();
        let date = self.regs.date().read();
        // In case the software makes read accesses to the calendar in a time interval smaller
        // than 2 RTCCLK periods: UPDF must be cleared by software after the first calendar read.
        self.regs.sts().modify(|_, w| w.updf().clear_bit());

        let seconds = decode_seconds(&time);
        let minutes = decode_minutes(&time);
        let hours = decode_hours(&time);
        let day = decode_day(&date);
        let month = decode_month(&date);
        let year = decode_year(&date);
        let divb = self.regs.div().read().divb().bits();
        let nano = ss_to_nano(sbs, divb);

        NaiveDateTime::new(
            NaiveDate::from_ymd_opt(year.into(), month.try_into().unwrap(), day.into()).unwrap(),
            NaiveTime::from_hms_nano_opt(hours.into(), minutes.into(), seconds.into(), nano).unwrap(),
        )
    }

    /// Configures the wakeup timer to trigger periodically every `interval` duration
    ///
    /// # Panics
    ///
    /// Panics if interval is greater than 2¹⁷-1 seconds.
    pub fn enable_wakeup(&mut self, interval: fugit::MicrosDurationU64) {
        let clock_source = self.clock_source;
        self.modify(false, |regs| {
            regs.ctrl().modify(|_, w| w.waten().clear_bit());
            regs.sts().modify(|_, w| w.watf().clear_bit());
            while regs.sts().read().watwf().bit_is_clear() {}

            if interval < fugit::MicrosDurationU64::secs(32) {
                // Use RTCCLK as the wakeup timer clock source
                let frequency: fugit::Hertz<u64> = (clock_source.frequency() / 2).into();
                let freq_duration: fugit::MicrosDurationU64 = frequency.into_duration();
                let ticks_per_interval = interval / freq_duration;

                let mut prescaler = 0;
                while ticks_per_interval >> prescaler > 1 << 16 {
                    prescaler += 1;
                }

                let wucksel = match prescaler {
                    0 => 0b11, //WUCKSEL::Div2,
                    1 => 0b10, //WUCKSEL::Div4,
                    2 => 0b01, //WUCKSEL::Div8,
                    3 => 0b00, //WUCKSEL::Div16,
                    _ => unreachable!("Longer durations should use ck_spre"),
                };

                let interval = u16::try_from((ticks_per_interval >> prescaler) - 1).unwrap();

                regs.ctrl()
                    .modify(|_, w| unsafe { w.watclk().bits(wucksel) });
                regs.wat().write(|w| unsafe { w.bits(interval) });
            } else {
                // Use ck_spre (1Hz) as the wakeup timer clock source
                let interval = interval.to_secs();
                if interval > 1 << 16 {
                    regs.ctrl().modify(|_, w| unsafe { w.watclk().bits(0b110) });
                    let interval = u16::try_from(interval - (1 << 16) - 1)
                        .expect("Interval was too large for wakeup timer");
                    regs.wat().write(|w| unsafe { w.bits(interval) });
                } else {
                    regs.ctrl().modify(|_, w| unsafe { w.watclk().bits(0b100) });
                    let interval = u16::try_from(interval - 1)
                        .expect("Interval was too large for wakeup timer");
                    regs.wat().write(|w| unsafe { w.bits(interval) });
                }
            }

            regs.ctrl().modify(|_, w| w.waten().set_bit());
        });
    }

    /// Disables the wakeup timer
    pub fn disable_wakeup(&mut self) {
        self.modify(false, |regs| {
            regs.ctrl().modify(|_, w| w.waten().clear_bit());
            regs.sts().modify(|_, w| w.watf().clear_bit());
        });
    }

    /// Configures the timestamp to be captured when the ERTC switches to Vbat power
    pub fn enable_vbat_timestamp(&mut self) {
        self.modify(false, |regs| {
            regs.ctrl().modify(|_, w| w.tsen().clear_bit());
            regs.sts().modify(|_, w| w.tsf().clear_bit());
            regs.ctrl().modify(|_, w| w.tsen().set_bit());
        });
    }

    /// Disables the timestamp
    pub fn disable_timestamp(&mut self) {
        self.modify(false, |regs| {
            regs.ctrl().modify(|_, w| w.tsen().clear_bit());
            regs.sts().modify(|_, w| w.tsf().clear_bit());
        });
    }

    /// Reads the stored value of the timestamp if present
    ///
    /// Clears the timestamp interrupt flags.
    pub fn read_timestamp(&self) -> NaiveDateTime {
        while self.regs.sts().read().updf().bit_is_clear() {}

        // Timestamp doesn't include year, get it from the main calendar
        let sbs = self.regs.tssbs().read().bits() as u16;

        // TODO: remove unsafe after PAC update
        let time = self.regs.tstm().read();
        let date = self.regs.tsdt().read();
        let dry = self.regs.date().read();
        let seconds = decode_seconds_ts(&time);
        let minutes = decode_minutes_ts(&time);
        let hours = decode_hours_ts(&time);
        let day = decode_day_ts(&date);
        let month = decode_month_ts(&date);
        let year = decode_year(&dry);
        let divb = self.regs.div().read().divb().bits();
        let nano = ss_to_nano(sbs, divb);

        NaiveDateTime::new(
            NaiveDate::from_ymd_opt(year.into(), month.try_into().unwrap(), day.into()).unwrap(),
            NaiveTime::from_hms_nano_opt(hours.into(), minutes.into(), seconds.into(), nano).unwrap(),
        )
    }

    /// Sets the time at which an alarm will be triggered
    /// This also clears the alarm flag if it is set
    pub fn set_alarm(
        &mut self,
        alarm: Alarm,
        date: impl Into<AlarmDay>,
        time: NaiveTime,
    ) -> Result<(), Error> {
        let date = date.into();
        let (daymask, wdsel, (dt, du)) = match date {
            AlarmDay::Date(date) => (false, false, bcd2_encode(date.day().into())?),
            AlarmDay::Weekday(weekday) => (false, true, (0, weekday.num_days_from_monday() as u8)),
            AlarmDay::EveryDay => (true, false, (0, 0)),
        };
        let (ht, hu) = bcd2_encode(time.hour().into())?;
        let (mt, mu) = bcd2_encode(time.minute().into())?;
        let (st, su) = bcd2_encode(time.second().into())?;

        self.modify(false, |rtc| {
            unsafe {
                bb::clear(rtc.ctrl(), 8 + (alarm as u8));
                bb::clear(rtc.sts(), 8 + (alarm as u8));
            }
            while rtc.sts().read().bits() & (1 << (alarm as u32)) == 0 {}
            let reg = match alarm {
                Alarm::AlarmA => rtc.ala(),
                Alarm::AlarmB => rtc.alb(),
            };
            reg.modify(|_, w| {
                w.dt().set(dt);
                w.du().set(du);
                w.ht().set(ht);
                w.hu().set(hu);
                w.mt().set(mt);
                w.mu().set(mu);
                w.st().set(st);
                w.su().set(su);
                w.ampm().clear_bit();
                w.wksel().bit(wdsel);
                w.mask4().bit(daymask)
            });
            // subsecond alarm not implemented
            // would need a conversion method between `time.micros` and ERTC ticks
            // write the SBS value and mask to `rtc.alrmssr[alarm]`

            // enable alarm and reenable interrupt if it was enabled
            unsafe {
                bb::set(rtc.ctrl(), 8 + (alarm as u8));
            }
        });
        Ok(())
    }

    /// Start listening for `event`
    pub fn listen(&mut self, exti: &mut pac::EXINT, event: Event) {
        // Input Mapping:
        // EXINT 17 = ERTC Alarms
        // EXINT 21 = ERTC Tamper, ERTC Timestamp
        // EXINT 22 = ERTC Wakeup Timer
        self.modify(false, |regs| match event {
            Event::AlarmA => {
                exti.polcfg1().modify(|_, w| w.rp17().set_bit());
                exti.inten().modify(|_, w| w.inten17().set_bit());
                regs.ctrl().modify(|_, w| w.alaien().set_bit());
            }
            Event::AlarmB => {
                exti.polcfg1().modify(|_, w| w.rp17().set_bit());
                exti.inten().modify(|_, w| w.inten17().set_bit());
                regs.ctrl().modify(|_, w| w.albien().set_bit());
            }
            Event::Wakeup => {
                exti.polcfg1().modify(|_, w| w.rp22().set_bit());
                exti.inten().modify(|_, w| w.inten22().set_bit());
                regs.ctrl().modify(|_, w| w.watien().set_bit());
            }
            Event::Timestamp => {
                exti.polcfg1().modify(|_, w| w.rp21().set_bit());
                exti.inten().modify(|_, w| w.inten21().set_bit());
                regs.ctrl().modify(|_, w| w.tsien().set_bit());
            }
        });
    }

    /// Stop listening for `event`
    pub fn unlisten(&mut self, exti: &mut pac::EXINT, event: Event) {
        // See the note in listen() about EXINT
        self.modify(false, |regs| match event {
            Event::AlarmA => {
                regs.ctrl().modify(|_, w| w.alaien().clear_bit());
                exti.inten().modify(|_, w| w.inten17().clear_bit());
                exti.polcfg1().modify(|_, w| w.rp17().clear_bit());
            }
            Event::AlarmB => {
                regs.ctrl().modify(|_, w| w.albien().clear_bit());
                exti.inten().modify(|_, w| w.inten17().clear_bit());
                exti.polcfg1().modify(|_, w| w.rp17().clear_bit());
            }
            Event::Wakeup => {
                regs.ctrl().modify(|_, w| w.watien().clear_bit());
                exti.inten().modify(|_, w| w.inten22().clear_bit());
                exti.polcfg1().modify(|_, w| w.rp22().clear_bit());
            }
            Event::Timestamp => {
                regs.ctrl().modify(|_, w| w.tsien().clear_bit());
                exti.inten().modify(|_, w| w.inten21().clear_bit());
                exti.polcfg1().modify(|_, w| w.rp21().clear_bit());
            }
        });
    }

    /// Returns `true` if `event` is pending
    pub fn is_pending(&self, event: Event) -> bool {
        match event {
            Event::AlarmA => self.regs.sts().read().alaf().bit_is_set(),
            Event::AlarmB => self.regs.sts().read().albf().bit_is_set(),
            Event::Wakeup => self.regs.sts().read().watf().bit_is_set(),
            Event::Timestamp => self.regs.sts().read().tsf().bit_is_set(),
        }
    }

    /// Clears the interrupt flag for `event`
    pub fn clear_interrupt(&mut self, event: Event) {
        match event {
            Event::AlarmA => {
                self.regs.sts().modify(|_, w| w.alaf().clear_bit());
                unsafe {
                    (*pac::EXINT::ptr())
                        .intsts()
                        .write(|w| w.line17().clear_bit_by_one())
                };
            }
            Event::AlarmB => {
                self.regs.sts().modify(|_, w| w.albf().clear_bit());
                unsafe {
                    (*pac::EXINT::ptr())
                        .intsts()
                        .write(|w| w.line17().clear_bit_by_one())
                };
            }
            Event::Wakeup => {
                self.regs.sts().modify(|_, w| w.watf().clear_bit());
                unsafe {
                    (*pac::EXINT::ptr())
                        .intsts()
                        .write(|w| w.line22().clear_bit_by_one())
                };
            }
            Event::Timestamp => {
                self.regs.sts().modify(|_, w| w.tsf().clear_bit());
                unsafe {
                    (*pac::EXINT::ptr())
                        .intsts()
                        .write(|w| w.line21().clear_bit_by_one())
                };
            }
        }
    }
}

// Two 32-bit registers (RTC_TR and RTC_DR) contain the seconds, minutes, hours (12- or 24-hour format), day (day
// of week), date (day of month), month, and year, expressed in binary coded decimal format
// (BCD). The sub-seconds value is also available in binary format.
//
// The following helper functions encode into BCD format from integer and
// decode to an integer from a BCD value respectively.
fn bcd2_encode(word: u32) -> Result<(u8, u8), Error> {
    let l = match (word / 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidInputData);
        }
    };
    let r = match (word % 10).try_into() {
        Ok(v) => v,
        Err(_) => {
            return Err(Error::InvalidInputData);
        }
    };

    Ok((l, r))
}

const fn bcd2_decode(fst: u8, snd: u8) -> u8 {
    fst * 10 + snd
}

#[inline(always)]
fn decode_seconds_ts(time: &pac::ertc::tstm::R) -> u8 {
    bcd2_decode(time.st().bits(), time.su().bits())
}

#[inline(always)]
fn decode_minutes_ts(time: &pac::ertc::tstm::R) -> u8 {
    bcd2_decode(time.mt().bits(), time.mu().bits())
}

#[inline(always)]
fn decode_hours_ts(time: &pac::ertc::tstm::R) -> u8 {
    bcd2_decode(time.ht().bits(), time.hu().bits())
}

#[inline(always)]
fn decode_day_ts(date: &pac::ertc::tsdt::R) -> u8 {
    bcd2_decode(date.dt().bits(), date.du().bits())
}

#[inline(always)]
fn decode_month_ts(date: &pac::ertc::tsdt::R) -> u8 {
    let mt = u8::from(date.mt().bit());
    bcd2_decode(mt, date.mu().bits())
}

#[inline(always)]
fn decode_seconds(time: &pac::ertc::time::R) -> u8 {
    bcd2_decode(time.st().bits(), time.su().bits())
}

#[inline(always)]
fn decode_minutes(time: &pac::ertc::time::R) -> u8 {
    bcd2_decode(time.mt().bits(), time.mu().bits())
}

#[inline(always)]
fn decode_hours(time: &pac::ertc::time::R) -> u8 {
    bcd2_decode(time.ht().bits(), time.hu().bits())
}

#[inline(always)]
fn decode_day(date: &pac::ertc::date::R) -> u8 {
    bcd2_decode(date.dt().bits(), date.du().bits())
}

#[inline(always)]
fn decode_month(date: &pac::ertc::date::R) -> u8 {
    let mt = u8::from(date.mt().bit());
    bcd2_decode(mt, date.mu().bits())
}

#[inline(always)]
fn decode_year(date: &pac::ertc::date::R) -> u16 {
    let year = (bcd2_decode(date.yt().bits(), date.yu().bits()) as u32) + 1970; // 1970-01-01 is the epoch begin.
    year as u16
}

const fn ss_to_nano(sbs: u16, divb: u16) -> u32 {
    let sbs = sbs as u32;
    let divb = divb as u32;
    assert!(sbs <= divb);

    (((divb - sbs) * 100_000) / (divb + 1)) * 10_000
}
