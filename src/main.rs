#![no_std]
#![no_main]

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

#[cfg(debug_assertions)]
use cortex_m_log::printer::semihosting;

#[cfg(debug_assertions)]
use cortex_m_log::{println};

use cortex_m_log::{d_println};

// use cortex_m::asm;
use cortex_m_rt::{entry, ExceptionFrame};

use stm32f4xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32 as stm32;
use stm32::I2C1;
use p_hal::gpio::GpioExt;
// use p_hal::rcc::RccExt;

use em7180::USFS;

#[macro_use]
extern crate cortex_m_rt;


#[cfg(debug_assertions)]
//type DebugLog = cortex_m_log::printer::dummy::Dummy;
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;


// type GpioTypeUserLed1 =  p_hal::gpio::gpioc::PC13<p_hal::gpio::Output<p_hal::gpio::PushPull>>; //stm32f401CxUx

pub type I2cPortType = p_hal::i2c::I2c<I2C1,
    (p_hal::gpio::gpiob::PB8<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>,
     p_hal::gpio::gpiob::PB9<p_hal::gpio::AlternateOD<p_hal::gpio::AF4>>)
>;


// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(_irqn: i16) {
    d_println!(get_debug_log(), "IRQn = {}", _irqn);
}

// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("HardFault: {:?}", ef);
}

/// Used in debug builds to provide a logging outlet
#[cfg(debug_assertions)]
fn get_debug_log() -> DebugLog {
    //cortex_m_log::printer::Dummy::new()
    semihosting::InterruptFree::<_>::stdout().unwrap()
}

#[entry]
fn main() -> ! {
    let mut log = get_debug_log();

    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    // HSI: use default internal oscillator
    let clocks = rcc.cfgr.freeze();
    // HSE: external crystal oscillator must be connected
    //let clocks = rcc.cfgr.use_hse(SystemCoreClock.hz()).freeze();

    let mut delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx
    //set initial states of user LEDs
    user_led1.set_high().unwrap();

    // setup i2c1 and imu driver
    // NOTE: stm32f401CxUx lacks external pull-ups on i2c pins
    // NOTE: eg f407 discovery board already has external pull-ups
    let scl = gpiob.pb8
        .into_alternate_af4()
        .internal_pull_up(true)
        .set_open_drain();

    let sda = gpiob.pb9
        .into_alternate_af4()
        .internal_pull_up(true)
        .set_open_drain();
    let i2c_port = p_hal::i2c::I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks);

    let mut driver = USFS::new_st_usfs1(i2c_port,
                                      em7180::EM7180_DEFAULT_ADDRESS,
                                      0, //unused for now
                                      false).unwrap();


    let fflags = driver.check_feature_flags().unwrap_or(0);
    if fflags != 0x05 { //barometer + temperature sensor installed
        d_println!(log, "fflags {}", fflags);
   }

    delay_source.delay_ms(1u8);

    loop {
        if let Ok(err_check) = driver.check_errors() {
            if 0 != err_check {
                d_println!(log, "err {:?}", err_check);
                break;
            }
        }
        if driver.quat_available() {
            //bkpt();
            let qata =  driver.read_sentral_quat_qata();
            if let Ok(quat) = qata {
                d_println!(log, "{:?}", quat);
            }
            else {
                d_println!(log, "{:?}", qata);
            }
        }
        delay_source.delay_ms(1u8);
        user_led1.toggle().unwrap();
    }

    loop {
        // we reach here if something went wrong
        delay_source.delay_ms(250u8);
    }
}
