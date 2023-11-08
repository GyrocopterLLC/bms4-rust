// Blinky example from STM32F0xx-HAL

// This is simply a stub for now. The real work is yet to be done. 
// The toolchain (VScode + rust + GDB + openocd) is tested and works.

#![no_main]
#![no_std]

use panic_halt as _;

use stm32f0xx_hal as hal;

use crate::hal::{gpio::*, pac, prelude::*};

use cortex_m::{interrupt::Mutex, peripheral::syst::SystClkSource::Core, Peripherals};
use cortex_m_rt::{entry, exception};

use core::cell::RefCell;
use core::ops::DerefMut;

struct LEDS{
    green: gpioa::PA15<Output<PushPull>>,
    red: gpiob::PB3<Output<PushPull>>
}

// Mutex protected structure for our shared GPIO pin
//static GPIO: Mutex<RefCell<Option<gpioa::PA15<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));
static GPIO_LEDS: Mutex<RefCell<Option<LEDS>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

            let gpioa = p.GPIOA.split(&mut rcc);
            let gpiob = p.GPIOB.split(&mut rcc);

            // (Re-)configure PA15 and PB3 as output
            let ledg = gpioa.pa15.into_push_pull_output(cs);
            let ledr = gpiob.pb3.into_push_pull_output(cs);

            // Transfer GPIO into a shared structure
            *GPIO_LEDS.borrow(cs).borrow_mut() = Some(LEDS{green:ledg, red:ledr});

            let mut syst = cp.SYST;

            // Initialise SysTick counter with a defined value
            unsafe { syst.cvr.write(1) };

            // Set source for SysTick counter, here full operating frequency (== 48MHz)
            syst.set_clock_source(Core);

            // Set reload value, i.e. timer delay 48 MHz/4 Mcounts == 12Hz or 83ms
            syst.set_reload(4_000_000 - 1);

            // Start counting
            syst.enable_counter();

            // Enable interrupt generation
            syst.enable_interrupt();
        });
    }

    loop {
        continue;
    }
}

// Define an exception handler, i.e. function to call when exception occurs. Here, if our SysTick
// timer generates an exception the following handler will be called
#[exception]
fn SysTick() {
    // Exception handler state variable
    static mut STATE: u8 = 1;

    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // Borrow access to our GPIO pin from the shared structure
        if let Some(ref mut led) = *GPIO_LEDS.borrow(cs).borrow_mut().deref_mut() {
            // Check state variable, keep LED off most of the time and turn it on every 10th tick
            if *STATE < 10 {
                // Turn off the LED
                led.green.set_low().ok();
                led.red.set_high().ok();

                // And now increment state variable
                *STATE += 1;
            } else {
                // Turn on the LED
                led.green.set_high().ok();
                led.red.set_low().ok();

                // And set new state variable back to 0
                *STATE = 0;
            }
        }
    });
}