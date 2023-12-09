#![no_std]
#![feature(const_trait_impl, const_maybe_uninit_zeroed)]
extern crate static_assertions as sa;

/// embassy_time implementation for avr
///
/// configure using feature flags
///
/// adjust QUEUE_SIZE so you don't run out of stack
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

use core::mem::{MaybeUninit, size_of, transmute};
use core::task::Waker;
use arraydeque::ArrayDeque;
use atmega_hal::pac::TC0;
use embassy_time::driver::{AlarmHandle, Driver};
use embassy_time::Instant;
use embassy_time::queue::TimerQueue;

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

#[cfg(feature = "queue4")]
const QUEUE_SIZE: usize = 4;
#[cfg(feature = "queue8")]
const QUEUE_SIZE: usize = 8;

#[cfg(feature = "queue16")]
const QUEUE_SIZE: usize = 8;

#[cfg(feature = "queue32")]
const QUEUE_SIZE: usize = 8;

sa::const_assert_eq!(FREQ % CLOCKS_PER_TICK, 0);
sa::const_assert_eq!(FREQ / CLOCKS_PER_TICK, embassy_time::TICK_HZ);
sa::const_assert_eq!(256 % DIVIDER, 0);

#[derive(Debug)]
pub struct LinkedList {
    next: Option<u8>,
    at: Option<u64>,
    v: AlarmOrWaker
}

#[derive(Debug)]
pub enum AlarmOrWaker {
    Alarm {
        callback: fn(*mut ()),
        ctx: *mut ()
    },
    Waker(Waker),
    Empty
}

//insane or something ?
pub static mut QUEUE: [Option<LinkedList>; QUEUE_SIZE] = unsafe {
    transmute(
        [
            transmute::<_, [u8; size_of::<Option<LinkedList>>()]>(Option::<LinkedList>::None);
            QUEUE_SIZE
        ]
    )
};

pub static mut QUEUE_ID: ArrayDeque<u8, QUEUE_SIZE> = unsafe { MaybeUninit::zeroed().assume_init() };

pub static mut QUEUE_NEXT: Option<u8> = None;


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
        avr_hal_generic::avr_device::interrupt::free(|_| QUEUE_ID.pop_front()).map(|n| {
            QUEUE[n as usize] = Some(LinkedList {
                next: None,
                at: None,
                v: AlarmOrWaker::Empty
            });
            AlarmHandle::new(n)
        })
    }

    fn set_alarm_callback(&self, alarm: AlarmHandle, callback: fn(*mut ()), ctx: *mut ()) {
        unsafe {
            if let Some(li) = &mut QUEUE[alarm.id() as usize] {
                li.v = AlarmOrWaker::Alarm {
                    callback,
                    ctx,
                }
            }
        }
    }

    fn set_alarm(&self, alarm: AlarmHandle, timestamp: u64) -> bool {
        unsafe {
            avr_device::interrupt::free(|_| {
                let mut next = &mut QUEUE_NEXT;
                while let &mut Some(i) = next{
                    let next_i = QUEUE[i as usize].as_mut().unwrap();
                    let next_at = next_i.at.unwrap();
                    let this_alarm = QUEUE[alarm.id() as usize].as_mut().unwrap();
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
            avr_device::interrupt::free(|_| QUEUE_ID.pop_front().map(|id| {
                QUEUE[id as usize] = Some(LinkedList {
                    next: None,
                    at: Some(at.as_ticks()),
                    v: AlarmOrWaker::Waker(waker.clone()),
                });
                let this_alarm = QUEUE[id as usize].as_mut().unwrap();
                let mut next = &mut QUEUE_NEXT;
                while let &mut Some(i) = next {
                    let next_i = QUEUE[i as usize].as_mut();
                    let next_i = next_i.unwrap();
                    let next_at = next_i.at.unwrap();

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
    let (mut next_option, ticks_elapsed) = avr_device::interrupt::free(|_| {
        TICKS_ELAPSED += TICKS_PER_COUNT;
        if let Some(n) = QUEUE_NEXT {
            return if QUEUE[n as usize].as_ref().unwrap().at.unwrap() <= TICKS_ELAPSED {
                let next = QUEUE[n as usize].take().unwrap();
                QUEUE_NEXT = next.next;
                QUEUE_ID.push_back(n).unwrap();
                (Some(next.v), TICKS_ELAPSED)
            } else {
                (None, TICKS_ELAPSED)
            }
        }
        (None, TICKS_ELAPSED)
    });
    while let Some(next) = next_option {
        match next {
            AlarmOrWaker::Alarm { callback, ctx } => callback(ctx),
            AlarmOrWaker::Waker(waker) => waker.wake(),
            AlarmOrWaker::Empty => panic!("alarm fired before setting callback")
        }
        next_option = avr_device::interrupt::free(|_| {
            if let Some(n) = QUEUE_NEXT {
                return if QUEUE[n as usize].as_ref().unwrap().at.unwrap() <= ticks_elapsed {
                    let next = QUEUE[n as usize].take().unwrap();
                    QUEUE_NEXT = next.next;
                    QUEUE_ID.push_back(n).unwrap();
                    Some(next.v)
                } else {
                    None
                }
            }
            None
        })
    }
}

pub fn init_system_time(tc: &mut TC0) {
    unsafe {
        QUEUE_ID = ArrayDeque::from_iter(0..(QUEUE_SIZE as u8));
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