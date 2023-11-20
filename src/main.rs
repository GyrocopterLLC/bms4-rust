// Blinky example from STM32F0xx-HAL

// This is simply a stub for now. The real work is yet to be done. 
// The toolchain (VScode + rust + GDB + openocd) is tested and works.

#![no_main]
#![no_std]

use adc_helper::HelpfulAdc;
use hal::pac::Interrupt;
use hal::pac::interrupt;
use hal::pac::adc::smpr::SMP_A;
use panic_halt as _;

use stm32f0xx_hal as hal;

use crate::hal::{gpio::*, pac, adc::*, prelude::*};

use cortex_m::{interrupt::Mutex, peripheral::syst::SystClkSource::Core, Peripherals};
use cortex_m_rt::{entry, exception};

use core::cell::RefCell;
use core::cell::UnsafeCell;
use core::ops::{DerefMut, Deref};

mod adc_helper;

struct LEDS{
    green: gpioa::PA15<Output<PushPull>>,
    red: gpiob::PB3<Output<PushPull>>
}

struct AdcPins {
    bat1: gpioa::PA5<Analog>,
    bat2: gpioa::PA4<Analog>,
    bat3: gpioa::PA1<Analog>,
    bat4: gpioa::PA0<Analog>
}

struct ForceSync<T> {
    inner: UnsafeCell<T>,
}

impl<T> ForceSync<T> {
    /// Creates a new sync forcer
    const fn new(value: T) -> Self {
        ForceSync {
            inner: UnsafeCell::new(value),
        }
    }

    fn borrow(& self) -> & T {
        unsafe { &*self.inner.get() }
    }
}

unsafe impl<T> Sync for ForceSync<T> where T: Send {}


const ADC_FLAG:u32 = 1;
const ADC_COMPLETE_FLAG:u32 = 2;

static GPIO_LEDS: Mutex<RefCell<Option<LEDS>>> = Mutex::new(RefCell::new(None));
static GPIO_ADCS: Mutex<RefCell<Option<AdcPins>>> = Mutex::new(RefCell::new(None));
static MAIN_LOOP_FLAGS: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static ADC_RESULTS: Mutex<RefCell<[u16;4]>> = Mutex::new(RefCell::new([0;4]));
//#[link_section=".data"]
//static ADC_RESULTS: [u16;4] = [0;4];
//static ADC_RESULTS: ForceSync<RefCell<[u16;4]>> = ForceSync::new(RefCell::new([0;4]));
static DMA_PERIPH: Mutex<RefCell<Option<pac::DMA1>>> = Mutex::new(RefCell::new(None));
// static ADC_PERIPH: Mutex<RefCell<Option<Adc>>> = Mutex::new(RefCell::new(None));
static ADC_PERIPH: Mutex<RefCell<Option<HelpfulAdc>>> = Mutex::new(RefCell::new(None));
#[entry]
fn main() -> ! {
    //let mut adc_results = [0_u16;4];


    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        cortex_m::interrupt::free(move |cs| {
            
            let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

            let gpioa = p.GPIOA.split(&mut rcc);
            let gpiob = p.GPIOB.split(&mut rcc);

            // (Re-)configure PA15 and PB3 as output
            let mut ledg = gpioa.pa15.into_push_pull_output(cs);
            let mut ledr = gpiob.pb3.into_push_pull_output(cs);

            // Ensure the LEDs are off
            ledg.set_low().ok();
            ledr.set_low().ok();

            // Configure ADC inputs: PA5, 4, 1, and 0 (In1, 2, 3, and 4 respectively)
            let mut in1 = gpioa.pa5.into_analog(cs);
            let mut in2 = gpioa.pa4.into_analog(cs);
            let mut in3 = gpioa.pa1.into_analog(cs);
            let mut in4 = gpioa.pa0.into_analog(cs);
            
            // let mut madc = Adc::new(p.ADC, &mut rcc);
            //let mut madc = HelpfulAdc::new(p.ADC, &mut rcc.regs);
            // Annoyingly, using the HAL causes us to lose access to the underlying RCC registers
            // And the features we need (like enabling HSI14, enabling DMA clock) are not available
            // through the HAL. 
            // So we resort to thievery.
            let mut stolen_p: pac::Peripherals;
            unsafe{ stolen_p = pac::Peripherals::steal(); }

            let mut madc = HelpfulAdc::new(p.ADC, &mut stolen_p.RCC); 
            let dest:u32 = ADC_RESULTS.borrow(cs).as_ptr() as u32;
            madc.dma_en(&mut p.DMA1, &mut p.SYSCFG, &mut stolen_p.RCC, dest);
            //madc.dma_en(&mut p.DMA1, &mut p.SYSCFG, &mut stolen_p.RCC, &ADC_RESULTS);

            madc.set_sample_time(SMP_A::CYCLES13_5);
            madc.set_pin(&mut in1);
            madc.set_pin(&mut in2);
            madc.set_pin(&mut in3);
            madc.set_pin(&mut in4);
            *DMA_PERIPH.borrow(cs).borrow_mut() = Some(p.DMA1);
            *ADC_PERIPH.borrow(cs).borrow_mut() = Some(madc);

            // Transfer GPIO into a shared structure
            *GPIO_LEDS.borrow(cs).borrow_mut() = Some(LEDS{green:ledg, red:ledr});
            *GPIO_ADCS.borrow(cs).borrow_mut() = Some(AdcPins{bat1:in1, bat2:in2, bat3: in3, bat4: in4});

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
        // Superloop controls the different functions
        // Timer used to initiate functions

        let run_adc: bool = cortex_m::interrupt::free(|cs| -> bool {
            let flags = *MAIN_LOOP_FLAGS.borrow(cs).borrow();
            (flags & ADC_FLAG) != 0
            //flags & ADC_FLAG != 0 // return value is true if the ADC_FLAG bit is set
        });

        let adc_complete: bool = cortex_m::interrupt::free(|cs| -> bool {
            let flags = *MAIN_LOOP_FLAGS.borrow(cs).borrow();
            (flags & ADC_COMPLETE_FLAG) != 0
            //flags & ADC_FLAG != 0 // return value is true if the ADC_FLAG bit is set
        });

        if run_adc {
                      
            cortex_m::interrupt::free(|cs| {
                let mut flags = *MAIN_LOOP_FLAGS.borrow(cs).borrow();
                flags = flags & (!ADC_FLAG);
                *MAIN_LOOP_FLAGS.borrow(cs).borrow_mut() = flags;

                if let (Some(adc_periph), Some(dma_periph)) = 
                    (ADC_PERIPH.borrow(cs).borrow_mut().deref_mut(), DMA_PERIPH.borrow(cs).borrow_mut().deref_mut()) {
                        adc_periph.dma_start(dma_periph);
                }

                /*
                if let (Some(adc_channels), Some(adc_periph)) = 
                    (GPIO_ADCS.borrow(cs).borrow_mut().deref_mut(), ADC_PERIPH.borrow(cs).borrow_mut().deref_mut())
                {
                    let mut adc_results:[u32;4] = [0;4];
                    for i in 0..4 {
                        adc_results[i] = match i {
                            0 => adc_periph.read(&mut adc_channels.bat1).ok().unwrap_or_default(),
                            1 => adc_periph.read(&mut adc_channels.bat2).ok().unwrap_or_default(),
                            2 => adc_periph.read(&mut adc_channels.bat3).ok().unwrap_or_default(),
                            _ => adc_periph.read(&mut adc_channels.bat4).ok().unwrap_or_default()
                        };
                    }

                    *ADC_RESULTS.borrow(cs).borrow_mut() = adc_results;

                }; */

            });
        }

        if adc_complete {
            // check our values
            //if(ADC_RESULTS[0] < ADC_RESULTS[3]) {
                // put on a light show
            cortex_m::interrupt::free(|cs| {
                let adc_conversions = *ADC_RESULTS.borrow(cs).borrow();
                if adc_conversions[0] < adc_conversions[3] {

                    if let Some(ref mut led) = GPIO_LEDS.borrow(cs).borrow_mut().deref_mut() {
                        led.red.set_high().ok();
                    }
                }
            });
            //}
        }
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
        if let Some(ref mut led) = GPIO_LEDS.borrow(cs).borrow_mut().deref_mut() {
            // Check state variable, keep LED off most of the time and turn it on every 10th tick
            if *STATE < 10 {
                // Turn off the LED
                led.green.set_low().ok();
                //led.red.set_high().ok();

                // And now increment state variable
                *STATE += 1;
            } else {
                // Turn on the LED
                led.green.set_high().ok();
                //led.red.set_low().ok();

                // And set new state variable back to 0
                *STATE = 0;

                let mut flags = *MAIN_LOOP_FLAGS.borrow(cs).borrow().deref();
                flags = flags | ADC_FLAG;
                *MAIN_LOOP_FLAGS.borrow(cs).borrow_mut() = flags;
            }
        }
    });
}


#[interrupt]
fn DMA1_CH1() {
    cortex_m::interrupt::free(|cs| {
        if let Some(dma) = DMA_PERIPH.borrow(cs).borrow_mut().deref_mut() {
            dma.ifcr.write(|w| {w.ctcif1().clear()});
            dma.ch1.cr.modify(|_,w|{w.en().disabled()}); // does not automatically disable the channel when transfer is complete !?
            if let Some(adc) = ADC_PERIPH.borrow(cs).borrow_mut().deref_mut() {
                adc.shutdown();
            }

            let mut flags = *MAIN_LOOP_FLAGS.borrow(cs).borrow().deref();
            flags = flags | ADC_COMPLETE_FLAG;
            *MAIN_LOOP_FLAGS.borrow(cs).borrow_mut() = flags;
        };
    });
}

#[interrupt]
fn DMA1_CH2_3_DMA2_CH1_2() {
    cortex_m::interrupt::free(|cs| {
        if let Some(dma) = DMA_PERIPH.borrow(cs).borrow_mut().deref_mut() {
            dma.ifcr.write(|w| {w.ctcif2().clear()});
            dma.ch2.cr.modify(|_,w|{w.en().disabled()});
            if let Some(adc) = ADC_PERIPH.borrow(cs).borrow_mut().deref_mut() {
                adc.shutdown();
            }

            let mut flags = *MAIN_LOOP_FLAGS.borrow(cs).borrow().deref();
            flags = flags | ADC_COMPLETE_FLAG;
            *MAIN_LOOP_FLAGS.borrow(cs).borrow_mut() = flags;
        };
    });
}