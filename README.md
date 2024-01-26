embassy_time implementation for avr

configure using feature flags

note: it's not really configurable feel free to add more configuration to support more boards

`TICK` is embassy_time single tick
`CLOCK` is timer clock
`FREQUENCY` is timer clock frequency(per sec)
`DIVDER` exist to allow use of lower embassy_time single ticks(time resolution isn't going to be as high as embassy_time frequency anyway)
```rust
#[allow(dead_code)]
const CLOCKS_PER_COUNT: u64 = prescalar::PRE * 256;
#[allow(dead_code)]
const CLOCKS_PER_TICK: u64 = prescalar::PRE * DIVIDER;
#[allow(dead_code)]
const TICKS_PER_COUNT: u64 = 256 / DIVIDER;
```

however it works on arduino uno

change `AVR_EMBASSY_TIME_QUEUE_SIZE` environmental variable to change queue_size

you can use cargo as well
```toml
[env]
AVR_EMBASSY_TIME_QUEUE_SIZE=4
```

note: bug or not use of join! macro on time bound tasks it will use lot of queue space

use macro to define interrupt

recommend to define this on main.rs but can be any module;
```rust
define_interrupt!(atmega328p)
```

initialize driver by running this code before using any time related function
```rust
init_system_time(&mut dp.TC0);
```
