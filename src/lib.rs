#![no_std]
#![feature(const_trait_impl)]
#![feature(int_roundings)]
#![feature(let_chains)]
extern crate static_assertions as sa;

use core::cmp::max;
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
/// init_system_time(&mut dp.TC1);
/// ```
/// note you **must** initialize timer before using embassy_time

use core::task::Waker;
use atmega_hal::pac::TC1;
use env_int::env_int;
use embassy_time_driver::{AlarmHandle, Driver};
use embassy_time_queue_driver::TimerQueue;
use core::mem::{size_of, transmute};
use crate::prescalar::{MARGIN_TICKS, PRESCALAR};

sa::const_assert!(true);

pub static mut TICKS_ELAPSED: u64 = 0;

pub struct AvrTc1EmbassyTimeDriver{}

#[cfg(feature = "prescalar1")]
mod prescalar {
    use atmega_hal::pac::tc1::tccr1b::TCCR1B_SPEC;
    use avr_device::generic::W;

    pub const PRESCALAR: u64 = 1;
    pub const MARGIN_TICKS: i64 = 160;
    pub fn set_prescalar(reg: &mut atmega_hal::pac::tc1::tccr1b::W) -> &mut W<TCCR1B_SPEC> {
        unsafe {
            reg.bits(0).cs1().direct()
        }
    }
}

#[cfg(feature = "prescalar8")]
mod prescalar {
    use atmega_hal::pac::tc1::tccr1b::TCCR1B_SPEC;
    use avr_device::generic::W;

    pub const PRESCALAR: u64 = 8;
    pub const MARGIN_TICKS: i64 = 80;
    pub fn set_prescalar(reg: &mut atmega_hal::pac::tc1::tccr1b::W) -> &mut W<TCCR1B_SPEC> {
        unsafe {
            reg.bits(0).cs1().prescale_8()
        }
    }
}

#[cfg(feature = "prescalar64")]
mod prescalar {
    use atmega_hal::pac::tc1::tccr1b::TCCR1B_SPEC;
    use avr_device::generic::W;

    pub const PRESCALAR: u64 = 64;
    pub const MARGIN_TICKS: i64 = 20;
    pub fn set_prescalar(reg: &mut atmega_hal::pac::tc1::tccr1b::W) -> &mut W<TCCR1B_SPEC> {
        unsafe {
            reg.bits(0).cs1().prescale_64()
        }
    }
}

#[cfg(feature = "prescalar256")]
mod prescalar {
    use atmega_hal::pac::tc1::tccr1b::TCCR1B_SPEC;
    use avr_device::generic::W;

    pub const PRESCALAR: u64 = 256;
    pub const MARGIN_TICKS: i64 = 20;
    pub fn set_prescalar(reg: &mut atmega_hal::pac::tc1::tccr1b::W) -> &mut W<TCCR1B_SPEC> {
        unsafe {
            reg.bits(0).cs1().prescale_256()
        }
    }
}

#[cfg(feature = "prescalar1024")]
mod prescalar {
    use atmega_hal::pac::tc1::tccr1b::TCCR1B_SPEC;
    use avr_device::generic::W;

    pub const PRESCALAR: u64 = 1024;
    pub const MARGIN_TICKS: i64 = 20;
    pub fn set_prescalar(reg: &mut atmega_hal::pac::tc1::tccr1b::W) -> &mut W<TCCR1B_SPEC> {
        unsafe {
            reg.bits(0).cs1().prescale_1024()
        }
    }
}

const QUEUE_SIZE: usize = env_int!(AVR_EMBASSY_TIME_QUEUE_SIZE, 4);

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
                next: None,
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


macro_rules! tc1 {
    () => {
        // tc1 is always zst currently
        core::mem::uninitialized::<TC1>()
    };
}


impl Driver for AvrTc1EmbassyTimeDriver {
    #[inline(always)]
    fn now(&self) -> u64 {
        avr_device::interrupt::free(|_| unsafe { time_now() }) * PRESCALAR
    }

    unsafe fn allocate_alarm(&self) -> Option<AlarmHandle> {
        avr_hal_generic::avr_device::interrupt::free(|_| pop_queue()).map(|n| {
            QUEUE[n as usize] = LinkedList {
                next: None,
                at: 0, //uninitialized values doesn't matter since it's not linked anywhere
                v: AlarmOrWaker::Empty
            };
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

    fn set_alarm(&self, alarm: AlarmHandle, mut timestamp: u64) -> bool {
        unsafe {
            //apply prescalar
            timestamp = timestamp.div_ceil(PRESCALAR);

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
                if QUEUE_NEXT == Some(alarm.id()) {
                    let ticks_elapsed = time_now();
                    let ticks_remaining = timestamp as i64 - ticks_elapsed as i64;
                    if ticks_remaining < u16::MAX as i64 {
                        let ticks_remaining = max(MARGIN_TICKS, ticks_remaining);
                        let ocr1a = u16::wrapping_add(tc1!().tcnt1.read().bits(), ticks_remaining as u16);
                        tc1!().ocr1a.write(|w| w.bits(ocr1a));
                        tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().set_bit());
                    }
                }
            });
        }
        true
    }
}

impl TimerQueue for AvrTc1EmbassyTimeDriver {
    #[inline(never)]
    fn schedule_wake(&'static self, at: u64, waker: &Waker) {
        unsafe {
            //apply prescalar
            let at = at.div_ceil(PRESCALAR);
            avr_device::interrupt::free(|_| pop_queue().map(|id| {
                let this_alarm = &mut QUEUE[id as usize];
                *this_alarm = LinkedList {
                    next: None,
                    at,
                    v: AlarmOrWaker::Waker(waker.clone()),
                };
                let mut next_prev = &mut None;
                let mut next = &mut QUEUE_NEXT;
                let mut scheduled = false;
                while let &mut Some(i) = next {
                    let next_i = &mut QUEUE[i as usize];
                    let next_at = next_i.at;
                    if let AlarmOrWaker::Waker(i_waker) = &next_i.v && i_waker.will_wake(waker) {
                        if scheduled {
                            //remove current iteration
                            *next_prev = next_i.next;
                            push_queue(i);
                        } else {
                            scheduled = true;
                            push_queue(id);
                        }
                        break;
                    }

                    next_prev = if next_at > at && !scheduled {
                        this_alarm.next = Some(i);
                        next.replace(id);
                        scheduled = true;
                        &mut this_alarm.next
                    } else {
                        next
                    };
                    next = &mut next_i.next;
                }
                if !scheduled {
                    next.replace(id);
                }

                if QUEUE_NEXT == Some(id) {
                    let ticks_elapsed = time_now();
                    let ticks_remaining = at as i64 - ticks_elapsed as i64;
                    if ticks_remaining < u16::MAX as i64 {
                        let ticks_remaining = max(MARGIN_TICKS, ticks_remaining);
                        let ocr1a = u16::wrapping_add(tc1!().tcnt1.read().bits(), ticks_remaining as u16);
                        tc1!().ocr1a.write(|w| w.bits(ocr1a));
                        tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().set_bit());
                    }
                }
            })).expect("queue full, increase queue size");
        }
    }
}

#[macro_export]
macro_rules! define_interrupt {
    ($mcu:ident) => {
        #[avr_device::interrupt($mcu)]
        unsafe fn TIMER1_OVF() {
            $crate::__tc1_ovf()
        }

        #[avr_device::interrupt($mcu)]
        unsafe fn TIMER1_COMPA() {
            $crate::__tc1_compa()
        }
    };
}

//always run in critical section

static mut LAST_TICKS_ELAPSED: u64 = 0;

#[inline(never)]
unsafe fn time_now() -> u64 {

    let tcnt = tc1!().tcnt1.read().bits();
    let mut ticks_elapsed = TICKS_ELAPSED + tcnt as u64;
    if tcnt <= 100 && ticks_elapsed < LAST_TICKS_ELAPSED {
        ticks_elapsed += u16::MAX as u64; //likely overflowed
    }
    LAST_TICKS_ELAPSED = ticks_elapsed;
    ticks_elapsed
}

#[inline(always)]
pub unsafe fn __tc1_compa() {
    avr_device::interrupt::free(|_| {
        let (mut queue_next, ticks_elapsed) = (QUEUE_NEXT.take(), time_now());
        loop {
            if let Some(n) = queue_next {
                if QUEUE[n as usize].at <= ticks_elapsed {
                    let next = QUEUE[n as usize].clone();
                    queue_next = next.next;
                    avr_device::interrupt::free(|_| push_queue(n));

                    match next.v {
                        AlarmOrWaker::Alarm { callback, ctx } => callback(ctx),
                        AlarmOrWaker::Waker(waker) => waker.wake(),
                        AlarmOrWaker::Empty => panic!("alarm fired before setting callback")
                    }
                } else {
                    let ticks_remaining = QUEUE[n as usize].at - ticks_elapsed;
                    if ticks_remaining < u16::MAX as u64 {
                        let ticks_remaining = max(MARGIN_TICKS as u16, ticks_remaining as u16);
                        let ocr1a = u16::wrapping_add(tc1!().tcnt1.read().bits(), ticks_remaining);
                        tc1!().ocr1a.write(|w| w.bits(ocr1a));
                        tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().set_bit());
                    } else {
                        tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().clear_bit());
                    }
                    break;
                }
            } else {
                tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().clear_bit());
                break;
            };
        }
        QUEUE_NEXT = queue_next;
    });
}

#[inline(always)]
pub unsafe fn __tc1_ovf() {
    avr_device::interrupt::free(|_| {
        TICKS_ELAPSED += u16::MAX as u64;
        LAST_TICKS_ELAPSED = TICKS_ELAPSED;
        let (mut queue_next, ticks_elapsed) = (QUEUE_NEXT.take(), TICKS_ELAPSED);

        loop {
            if let Some(n) = queue_next {
                if QUEUE[n as usize].at <= ticks_elapsed {
                    let next = QUEUE[n as usize].clone();
                    queue_next = next.next;
                    avr_device::interrupt::free(|_| push_queue(n));

                    match next.v {
                        AlarmOrWaker::Alarm { callback, ctx } => callback(ctx),
                        AlarmOrWaker::Waker(waker) => waker.wake(),
                        AlarmOrWaker::Empty => panic!("alarm fired before setting callback")
                    }
                } else {
                    let ticks_remaining = QUEUE[n as usize].at - ticks_elapsed;
                    if ticks_remaining < u16::MAX as u64 {
                        let ticks_remaining = max(MARGIN_TICKS as u16, ticks_remaining as u16);
                        let ocr1a = u16::wrapping_add(tc1!().tcnt1.read().bits(), ticks_remaining);
                        tc1!().ocr1a.write(|w| w.bits(ocr1a));
                        tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().set_bit());
                    } else {
                        tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().clear_bit());
                    }
                    break;
                }
            } else {
                tc1!().timsk1.modify(|r, w| w.bits(r.bits()).ocie1a().clear_bit());
                break;
            };
        }
        QUEUE_NEXT = queue_next;
    });
}

pub fn init_system_time(tc: &mut TC1) {
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
            LAST_TICKS_ELAPSED = 0;
            tc.tccr1b.write(prescalar::set_prescalar);
            tc.tccr1a.write(|w| w.com1a().disconnected());
            tc.timsk1.write(|w| w.toie1().bit(true).ocie1a().bit(true));
            tc.tcnt1.write(|w| w.bits(0));
        });
    }
}

embassy_time_driver::time_driver_impl!(static DRIVER: AvrTc1EmbassyTimeDriver = AvrTc1EmbassyTimeDriver{});
embassy_time_queue_driver::timer_queue_impl!(static QUEUE_DRIVER: AvrTc1EmbassyTimeDriver = AvrTc1EmbassyTimeDriver{});