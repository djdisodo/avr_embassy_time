#![no_std]
#![feature(const_trait_impl)]
extern crate static_assertions as sa;

/// embassy_time implementation for avr
///
/// configure using feature flags
///
/// adjust QUEUE_SIZE so you don't run out of stack
///
/// adjust resolution if timer can't keep up,
/// this is because timer interrupt doesn't finish fast enough
///
///
/// TODO: allow configuration from env
///
/// use macro to define interrupt
/// ```rust
/// define_interrupt!(atmega328p)
/// ```
///
/// you need to initialize timer as well
/// ```rust
/// init_system_time(&mut dp.TC0);
/// ```
/// note you **must** initialize timer before using embassy_time

use core::task::Waker;
use atmega_hal::pac::TC0;
use env_int::env_int;
use embassy_time::driver::{AlarmHandle, Driver};
use embassy_time::Instant;
use embassy_time::queue::TimerQueue;
use core::mem::{size_of, transmute};

sa::const_assert!(true);

pub static mut TICKS_ELAPSED: u64 = 0;

pub struct AvrTc0EmbassyTimeDriver{}

#[cfg(feature = "freq16MHz")]
#[allow(dead_code)]
const FREQ: u64 = 16_000_000;

#[cfg(feature = "prescalar8")]
mod prescalar {
    use atmega_hal::pac::tc0::tccr0b::TCCR0B_SPEC;
    use avr_device::generic::W;

    pub const PRE: u64 = 8;
    pub fn set_prescalar(reg: &mut atmega_hal::pac::tc0::tccr0b::W) -> &mut W<TCCR0B_SPEC> {
        reg.cs0().prescale_8()
    }
}

#[cfg(feature = "divider2")]
const DIVIDER: u64 = 2;

#[allow(dead_code)]
const CLOCKS_PER_COUNT: u64 = prescalar::PRE * 256;
#[allow(dead_code)]
const CLOCKS_PER_TICK: u64 = prescalar::PRE * DIVIDER;
#[allow(dead_code)]
const TICKS_PER_COUNT: u64 = 256 / DIVIDER;

const QUEUE_SIZE: usize = env_int!(AVR_EMBASSY_TIME_QUEUE_SIZE, 4);

sa::const_assert_eq!(FREQ % CLOCKS_PER_TICK, 0);
sa::const_assert_eq!(FREQ / CLOCKS_PER_TICK, embassy_time::TICK_HZ);
sa::const_assert_eq!(256 % DIVIDER, 0);

#[derive(Debug, Clone)]
pub struct LinkedList {
    next: Option<u8>, // there's way to store none value without using option but implementation will became too complex
    at: u64,
    v: AlarmOrWaker
}

#[derive(Debug, Clone)]
pub enum AlarmOrWaker {
    Alarm {
        callback: fn(*mut ()),
        ctx: *mut ()
    },
    Waker(Waker),
    Empty
}

pub static mut QUEUE: [LinkedList; QUEUE_SIZE] = unsafe {
    transmute(
        [
            transmute::<_, [u8; size_of::<LinkedList>()]>(LinkedList {
                next: Option::None,
                at: 0,
                v: AlarmOrWaker::Empty,
            });
            QUEUE_SIZE
        ]
    )
};

//lifo linked list queue utilizing "next" field
pub static mut QUEUE_ID: Option<u8> = None;

pub static mut QUEUE_NEXT: Option<u8> = None;

fn pop_queue() -> Option<u8> {
    unsafe {
        QUEUE_ID.map(|x| {
            QUEUE_ID = QUEUE[x as usize].next;
            x
        })
    }
}

fn push_queue(id: u8) {
    unsafe {
        QUEUE[id as usize].next = QUEUE_ID;
        QUEUE_ID = Some(id);
    }
}


impl Driver for AvrTc0EmbassyTimeDriver {
    #[inline(always)]
    fn now(&self) -> u64 {
        avr_hal_generic::avr_device::interrupt::free(|_| {
            unsafe {
                TICKS_ELAPSED
            }
        })
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        avr_hal_generic::avr_device::interrupt::free(|_| pop_queue()).map(|n| {
            // QUEUE[n as usize] = LinkedList {
            //     next: None,
            //     at: 0, //uninitialized values doesn't matter since it's not linked anywhere
            //     v: AlarmOrWaker::Empty
            // };
            AlarmHandle::new(n)
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        unsafe {
            QUEUE[alarm.id() as usize].v = AlarmOrWaker::Alarm {
                callback,
                ctx,
            }
        }
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        unsafe {
            let this_alarm = &mut QUEUE[alarm.id() as usize];
            this_alarm.at = timestamp;
            avr_device::interrupt::free(|_| {
                let mut next = &mut QUEUE_NEXT;
                while let &mut Some(i) = next {
                    let next_i = &mut QUEUE[i as usize];
                    let next_at = next_i.at;
                    if next_at > timestamp {
                        this_alarm.next = Some(i);
                        break;
                    } else {
                        next = &mut next_i.next;
                    }
                }
                next.replace(alarm.id());
            });
        }
        true
    }
}


impl TimerQueue for AvrTc0EmbassyTimeDriver {
    #[inline(never)]
    fn schedule_wake(&'static self, at: Instant, waker: &Waker) {
        unsafe {
            avr_device::interrupt::free(|_| pop_queue().map(|id| {
                let this_alarm = &mut QUEUE[id as usize];
                *this_alarm = LinkedList {
                    next: None,
                    at: at.as_ticks(),
                    v: AlarmOrWaker::Waker(waker.clone()),
                };
                let mut next = &mut QUEUE_NEXT;
                while let &mut Some(i) = next {
                    let next_i = &mut QUEUE[i as usize];
                    let next_at = next_i.at;

                    if next_at > at.as_ticks() {
                        this_alarm.next = Some(i);
                        break;
                    } else {
                        next = &mut next_i.next;
                    }
                }
                next.replace(id);
            })).expect("queue full, increase queue size");
        }
    }
}

#[macro_export]
macro_rules! define_interrupt {
    ($mcu:ident) => {
        #[avr_device::interrupt($mcu)]
        unsafe fn TIMER0_OVF() {
            $crate::__tc0_ovf()
        }
    };
}

#[inline(always)]
pub unsafe fn __tc0_ovf() {
    let (mut queue_next, ticks_elapsed) = avr_device::interrupt::free(|_| {
        TICKS_ELAPSED += TICKS_PER_COUNT;
        (QUEUE_NEXT.take(), TICKS_ELAPSED)
    }); //minimize critical section
    let (mut next_option, ticks_elapsed) = (|| {
        if let Some(n) = queue_next {
            return if QUEUE[n as usize].at <= ticks_elapsed {
                let next = QUEUE[n as usize].clone();
                queue_next = next.next;
                avr_device::interrupt::free(|_| push_queue(n));
                (Some(next.v), ticks_elapsed)
            } else {
                (None, ticks_elapsed)
            }
        }
        (None, ticks_elapsed)
    })();
    while let Some(next) = next_option {
        match next {
            AlarmOrWaker::Alarm { callback, ctx } => callback(ctx),
            AlarmOrWaker::Waker(waker) => waker.wake(),
            AlarmOrWaker::Empty => panic!("alarm fired before setting callback")
        }
        next_option = (|| {
            if let Some(n) = queue_next {
                return if QUEUE[n as usize].at <= ticks_elapsed {
                    let next = QUEUE[n as usize].clone();
                    queue_next = next.next;
                    avr_device::interrupt::free(|_| push_queue(n));
                    Some(next.v)
                } else {
                    None
                }
            }
            None
        })()
    }
    avr_device::interrupt::free(|_| QUEUE_NEXT = queue_next)
}

pub fn init_system_time(tc: &mut TC0) {
    unsafe {
        let mut iter = 1..(QUEUE_SIZE as u8);
        QUEUE = [(); QUEUE_SIZE].map(|_| LinkedList {
            next: iter.next(),
            at: 0,
            v: AlarmOrWaker::Empty,
        });
        QUEUE_ID = Some(0);
        QUEUE_NEXT = None;
        avr_device::interrupt::enable();
        avr_device::interrupt::free(|_| {
            TICKS_ELAPSED = 0;
            tc.tccr0b.write(prescalar::set_prescalar);
            tc.timsk0.write(|w| w.toie0().bit(true));
            tc.tcnt0.write(|w| w.bits(0));
        });
    }
}

embassy_time::time_driver_impl!(static DRIVER: AvrTc0EmbassyTimeDriver = AvrTc0EmbassyTimeDriver{});
embassy_time::timer_queue_impl!(static QUEUE_DRIVER: AvrTc0EmbassyTimeDriver = AvrTc0EmbassyTimeDriver{});