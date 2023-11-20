const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
const VDD_CALIB: u16 = 3300;

use crate::{gpioa, gpiob, gpioc, Analog};

use crate::pac::{
    adc::{
        cfgr1::{ALIGN_A, RES_A},
        smpr::SMP_A
    },
    RCC,
    rcc::{
        apb2enr,
        cr2
    },
    ADC,
    DMA1,
    dma1::ch::cr::{PSIZE_A, MSIZE_A},
    SYSCFG, 
    NVIC,
    Interrupt
};

pub trait Channel {
    type ID; // ex: u8
    fn channel() -> Self::ID;
}

// Convert pins to ADC channels by implementing the Channel trait
macro_rules! adc_pins {
    ($($pin:ty => $chan:expr),+ $(,)*) => {
        $(
            impl Channel for $pin {
                type ID = u8;

                fn channel() -> u8 { $chan }
            }
        )+
    };
}

adc_pins!(
    gpioa::PA0<Analog> => 0_u8,
    gpioa::PA1<Analog> => 1_u8,
    gpioa::PA2<Analog> => 2_u8,
    gpioa::PA3<Analog> => 3_u8,
    gpioa::PA4<Analog> => 4_u8,
    gpioa::PA5<Analog> => 5_u8,
    gpioa::PA6<Analog> => 6_u8,
    gpioa::PA7<Analog> => 7_u8,
    gpiob::PB0<Analog> => 8_u8,
    gpiob::PB1<Analog> => 9_u8,
    gpioc::PC0<Analog> => 10_u8,
    gpioc::PC1<Analog> => 11_u8,
    gpioc::PC2<Analog> => 12_u8,
    gpioc::PC3<Analog> => 13_u8,
    gpioc::PC4<Analog> => 14_u8,
    gpioc::PC5<Analog> => 15_u8
);

#[derive(Debug, Default)]
/// Internal temperature sensor (ADC Channel 16)
pub struct VTemp;

#[derive(Debug, Default)]
/// Internal voltage reference (ADC Channel 17)
pub struct VRef;

#[derive(Debug, Default)]
/// Battery reference voltage (ADC Channel 18)
pub struct VBat;

adc_pins!(
    VTemp => 16_u8,
    VRef  => 17_u8,
    VBat  => 18_u8
);

fn sum_of_bits(inp: u32) -> u32 {
    let mut sum = 0_u32;
    for i in 0..31 {
        if (inp & (1 << i)) != 0 {
            sum+=1;
        }
    }
    sum
}

pub struct HelpfulAdc {
    rb: ADC,
    sample_time: SMP_A,
    align: ALIGN_A,
    precision: RES_A,
    dma_mode: bool, 
    use_channel2: bool,
    channels_converted: u32
}

impl HelpfulAdc{

    pub fn new(madc: ADC, mrcc: &mut RCC) -> HelpfulAdc {
        let s: HelpfulAdc = Self { rb: madc, 
            sample_time: SMP_A::CYCLES239_5, 
            align: ALIGN_A::RIGHT, 
            precision: RES_A::TWELVEBIT,
            dma_mode: false,
            use_channel2: false,
            channels_converted: 0 };
        // Startup clocks - enable clock to ADC
        // and the HSI14 oscillator
        mrcc.apb2enr.modify(|_, w| {
            w.adcen().set_bit()
        });
        mrcc.cr2.modify(|_,w| {
            w.hsi14on().set_bit()
        });
        while mrcc.cr2.read().hsi14rdy().is_not_ready() {}

        if s.rb.cr.read().aden().is_enabled() {
            /* Clear ADEN by setting ADDIS */
            s.rb.cr.modify(|_, w| w.addis().disable());
        }
        while s.rb.cr.read().aden().is_enabled() {}

        /* Clear DMAEN */
        s.rb.cfgr1.modify(|_, w| w.dmaen().disabled());

        /* Start calibration by setting ADCAL */
        s.rb.cr.modify(|_, w| w.adcal().start_calibration());

        /* Wait until calibration is finished and ADCAL = 0 */
        while s.rb.cr.read().adcal().is_calibrating() {}

        //madc.ccr.
        s
    }

    pub fn set_sample_time(&mut self, sample_time: SMP_A) {
        self.sample_time = sample_time;
        self.rb.smpr.write(|w| {w.smp().variant(sample_time)});
    }

    pub fn set_channels(&mut self, channels_converted: u32) {
        self.channels_converted = channels_converted;
        self.rb.chselr.write(|w| {unsafe{w.bits(channels_converted)}});
    }

    pub fn set_pin<PIN:Channel<ID=u8>>(&mut self, _pin_converted: &mut PIN) {
        self.channels_converted += 1 << PIN::channel();
        self.rb.chselr.write(|w| {unsafe{w.bits(self.channels_converted)}});
    }

    //pub fn dma_en(&mut self, mdma: &mut DMA1, msyscfg: &mut SYSCFG, mrcc: &mut RCC, dest: & [u16]) -> () {
    pub fn dma_en(&mut self, mdma: &mut DMA1, msyscfg: &mut SYSCFG, mrcc: &mut RCC, dest_addr: u32) -> () {

        self.dma_mode = true;

        self.rb.cfgr1.modify(|_,w|{
            w.dmaen().enabled()
            .dmacfg().one_shot()
        });

        mrcc.ahbenr.modify(|_,w|{w.dmaen().enabled()});

        // confirm which channel in syscfg is assigned to the ADC
        self.use_channel2 = msyscfg.cfgr1.read().adc_dma_rmp().is_remapped();

        mdma.ifcr.write(|w| {
            w.ctcif1().clear()
            .ctcif2().clear()
        });

        let dmachannel = if self.use_channel2 {&mdma.ch2} else {&mdma.ch1};
        
        dmachannel.cr.write(|w| {
            w.psize().variant(PSIZE_A::BITS16)
            .msize().variant(MSIZE_A::BITS16)
            .dir().from_peripheral()
            .minc().enabled()
            .tcie().enabled()
        });

        dmachannel.mar.write(|w| {
            unsafe{w.bits(dest_addr)}
        });

        dmachannel.par.write(|w| {
            unsafe{w.bits(self.rb.dr.as_ptr() as u32)}
        });

        //dmachannel.ndtr.write(|w| {w.ndt().bits(dest.len() as u16)});

        
        
    }

    pub fn dma_start(&self, mdma: &mut DMA1) {
        // Enable DMA interrupt in NVIC
        let irq: Interrupt = if self.use_channel2 {Interrupt::DMA1_CH2_3_DMA2_CH1_2} else {Interrupt::DMA1_CH1};

        unsafe {NVIC::unmask(irq);}

        let num_conversions = sum_of_bits(self.channels_converted);

        let dmachannel = if self.use_channel2 {&mdma.ch2} else {&mdma.ch1};

        dmachannel.ndtr.write(|w| {w.ndt().bits(num_conversions as u16)});
        dmachannel.cr.modify(|_,w|{w.en().enabled()});

        if self.rb.isr.read().adrdy().is_ready() {
            self.rb.isr.modify(|_, w| w.adrdy().clear());
        }
        self.rb.cr.modify(|_, w| w.aden().enabled());
        while self.rb.isr.read().adrdy().is_not_ready() {}

        self.rb.cr.modify(|_,w|{w.adstart().start_conversion()});
    }

    pub fn shutdown(&self) {
        self.rb.cr.modify(|_, w| w.adstp().stop_conversion());
        while self.rb.cr.read().adstp().is_stopping() {}
        self.rb.cr.modify(|_, w| w.addis().disable());
        while self.rb.cr.read().aden().is_enabled() {}
    }

}

