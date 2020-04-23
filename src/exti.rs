use crate::stm32::{self, EXTI};
use stm32::Interrupt;

pub enum TriggerEdge {
    Rising,
    Falling,
    Both,
}


pub struct Exti {
    regs: EXTI,
}

impl Exti {
    /// Creates a new `Exti` wrapper from the raw `EXTI` peripheral.
    pub fn new(regs: EXTI) -> Self {
        Self { regs }
    }

    /// Destroys this `Exti` instance, returning the raw `EXTI` peripheral.
    pub fn release(self) -> EXTI {
        self.regs
    }

    /// Starts listening on a configurable interrupt line
    /// 
    /// The edges that should trigger the interrupt can be configured with 'edge'
    pub fn listen_configurable(&mut self, line: ConfigurableLine, edge: TriggerEdge) {
        let bm: u32 = 1 << line.raw_line();

        unsafe {
            match edge {
                TriggerEdge::Rising => self.regs.rtsr1.modify(|r, w| w.bits(r.bits() | bm)),
                TriggerEdge::Falling => self.regs.ftsr1.modify(|r, w| w.bits(r.bits() | bm)),
                TriggerEdge::Both => {
                    self.regs.rtsr1.modify(|r, w| w.bits(r.bits() | bm));
                    self.regs.ftsr1.modify(|r, w| w.bits(r.bits() | bm));
                }
            }

            self.regs.imr1.modify(|r, w| w.bits(r.bits() | bm));
        }
    }

    /// Marks `line` as "pending".
    ///
    /// This will cause an interrupt if the EXTI was previously configured to
    /// listen on `line`.
    ///
    /// If `line` is already pending, this does nothing.
    pub fn pend<L: ExtiLine>(line: L) {
        let line = line.raw_line();

        // Safety:
        // - We've ensured that the only 1-bit written is a valid line.
        // - This mirrors the `NVIC::pend` API and implementation, which is
        //   presumed safe.
        // - This is a "set by writing 1" register (ie. writing 0 does nothing),
        //   and this is a single write operation that cannot be interrupted.
        unsafe {
            (*EXTI::ptr()).swier1.write(|w| w.bits(1 << line));
        }
    }

    /// Marks `line` as "not pending".
    ///
    /// This should be called from an interrupt handler to ensure that the
    /// interrupt doesn't continuously fire.
    pub fn unpend<L: ExtiLine>(line: L) {
        let line = line.raw_line();

        // Safety:
        // - We've ensured that the only 1-bit written is a valid line.
        // - This mirrors the `NVIC::pend` API and implementation, which is
        //   presumed safe.
        // - This is a "clear by writing 1" register, and this is a single write
        //   operation that cannot be interrupted.
        unsafe {
            (*EXTI::ptr()).pr1.write(|w| w.bits(1 << line));
        }
    }

    /// Returns whether `line` is currently marked as pending.
    pub fn is_pending<L: ExtiLine>(line: L) -> bool {
        let bm: u32 = 1 << line.raw_line();

        // Safety: This is a read without side effects that cannot be
        // interrupted.
        let pr = unsafe { (*EXTI::ptr()).pr1.read().bits() };

        pr & bm != 0
    }

    // / Enters a low-power mode until an interrupt occurs.
    // /
    // / Please note that this method will return after _any_ interrupt that can
    // / wake up the microcontroller from the given power mode.
    // / COMMENTED OUT TILL PWR module is implemented
    // pub fn wait_for_irq<L, M>(&mut self, line: L, mut power_mode: M)
    //     where L: ExtiLine, M: PowerMode,
    // {
    //     let interrupt = line.interrupt();

    //     // This construct allows us to wait for the interrupt without having to
    //     // define an interrupt handler.
    //     interrupt::free(|_| {
    //         // Safety: Interrupts are globally disabled, and we re-mask and unpend the interrupt
    //         // before reenabling interrupts and returning.
    //         unsafe { NVIC::unmask(interrupt); }

    //         power_mode.enter();

    //         Self::unpend(line);
    //         NVIC::unpend(interrupt);
    //         NVIC::mask(interrupt);
    //     });
    // }
}

mod sealed {
    pub trait Sealed {}

    impl Sealed for super::ConfigurableLine {}
}

pub trait ExtiLine: Sized + sealed::Sealed {
    /// Returns the line object corresponding to a raw EXTI line number.
    ///
    /// If `raw` is not a valid line for type `Self`, `None` is returned.
    fn from_raw_line(raw: u8) -> Option<Self>;

    /// Returns that raw EXTI line number corresponding to `self`.
    fn raw_line(&self) -> u8;

    /// Returns the NVIC interrupt corresponding to `self`.
    fn interrupt(&self) -> stm32::Interrupt;
}

/// NOTE: implementation is still not complete
/// More configurable lines available
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum ConfigurableLine {
    Pvd = 16,
    RtcAlarm = 17,
    RtcTamper_CssLse = 19,
    RtcWakeup = 20,
    Comp1 = 21,
    Comp2 = 22,
    Comp3 = 29,
}

impl ExtiLine for ConfigurableLine  {
    fn from_raw_line(line: u8) -> Option<Self> {
        use ConfigurableLine::*;

        Some(match line {
            16 => Pvd,
            17 => RtcAlarm,
            // 18 = USB (or reserved)
            19 => RtcTamper_CssLse,
            20 => RtcWakeup,
            21 => Comp1,
            22 => Comp2,
            29 => Comp3,
            _ => return None,
        })
    }

    fn raw_line(&self) -> u8 {
        *self as u8
    }

    fn interrupt(&self) -> stm32::Interrupt {
        use stm32::Interrupt;
        use ConfigurableLine::*;

        match self {
            Pvd => Interrupt::PVD,
            RtcAlarm | RtcTamper_CssLse => Interrupt::RTCALARM,
            RtcWakeup => Interrupt::RTC_WKUP,
            Comp1 | Comp2 | Comp3 => Interrupt::COMP1_2_3,
        }
    }
}