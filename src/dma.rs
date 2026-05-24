//! Direct Memory Access (DMA)
#![macro_use]
mod dma_bdma;
pub use dma_bdma::*;

mod util;

pub(crate) use util::*;

pub(crate) mod ringbuffer;

pub mod word;

use core::marker::PhantomData;

/// The direction of a DMA transfer.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Dir {
    /// Transfer from memory to a peripheral.
    MemoryToPeripheral,
    /// Transfer from a peripheral to memory.
    PeripheralToMemory,
    /// Transfer from memory to another memory address.
    MemoryToMemory,
}

/// Which pointer in the transfer to increment.
pub enum Increment {
    /// DMA will not increment either of the addresses.
    None,
    /// DMA will increment the peripheral address.
    Peripheral,
    /// DMA will increment the memory address.
    Memory,
    /// DMA will increment both peripheral and memory addresses simultaneously.
    Both,
}

/// Which pointer in the transfer to increment.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Increment {
    /// DMA will not increment either of the addresses.
    None,
    /// DMA will increment the peripheral address.
    Peripheral,
    /// DMA will increment the memory address.
    Memory,
    /// DMA will increment both peripheral and memory addresses simultaneously.
    Both,
}

/// DMA channel driver
pub struct Channel {
    pub(crate) channel: DmaChannel,
}

impl Channel {
    /// Create a new DMA channel driver.
    pub fn new<T: ChannelInstance>(
        _ch: T,
    ) -> Self {
        Self {
            channel: T::CHANNEL,
        }
    }
    pub(crate) unsafe fn clone_unchecked(&self) -> Channel {
        Channel {
            channel: self.channel,
        }
    }
}
pub(crate) trait SealedChannelInstance {
    const CHANNEL: DmaChannel;
}

/// DMA channel.
#[allow(private_bounds)]
pub trait ChannelInstance: SealedChannelInstance + PeripheralType + 'static {
}

const CHANNEL_COUNT: usize = 7;

static STATE: [ChannelState; CHANNEL_COUNT] = [ChannelState::NEW; CHANNEL_COUNT];

pub(crate) fn info(channel: DmaChannel) -> &'static ChannelInfo {
    &crate::_generated::DMA_CHANNELS[channel as usize]
}

pub(crate) unsafe fn init(
    cs: critical_section::CriticalSection,
    bdma_priority: interrupt::Priority,
) {
    dma_bdma::init(cs, bdma_priority);
}
