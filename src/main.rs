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

#[cfg(feature = "stm32h7x")]
use stm32h7xx_hal as p_hal;

#[cfg(feature = "stm32f4x")]
use stm32f4xx_hal as p_hal;

#[cfg(feature = "stm32f3x")]
use stm32f3xx_hal as p_hal;

use p_hal::prelude::*;
use p_hal::stm32;

use em7180::USFS;
use cortex_m::asm::bkpt;


#[macro_use]
extern crate cortex_m_rt;


#[cfg(debug_assertions)]
// type DebugLog = cortex_m_log::printer::dummy::Dummy;
type DebugLog = cortex_m_log::printer::semihosting::Semihosting<cortex_m_log::modes::InterruptFree, cortex_m_semihosting::hio::HStdout>;


// cortex-m-rt is setup to call DefaultHandler for a number of fault conditions
// we can override this in debug mode for handy debugging
#[exception]
fn DefaultHandler(_irqn: i16) {
    bkpt();
    d_println!(get_debug_log(), "IRQn = {}", _irqn);
}

// cortex-m-rt calls this for serious faults.  can set a breakpoint to debug
#[exception]
fn HardFault(_ef: &ExceptionFrame) -> ! {
    bkpt();
    loop {}
    //panic!("HardFault: {:?}", ef);
}

/// Used in debug builds to provide a logging outlet
#[cfg(debug_assertions)]
fn get_debug_log() -> DebugLog {
    // cortex_m_log::printer::Dummy::new()
    semihosting::InterruptFree::<_>::stdout().unwrap()
}



#[cfg(feature = "stm32f3x")]
fn run_it() -> ! {
    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();

    // HSI: use default internal oscillator
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    // HSE: external crystal oscillator must be connected
    //let clocks = rcc.cfgr.use_hse(SystemCoreClock.hz()).freeze();

    let mut delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb);
    // let gpioc = dp.GPIOC.split();

    let mut user_led1 = gpiob.pb6.into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    user_led1.set_high().unwrap();


    // setup i2c1 and imu driver
    let scl = gpiob.pb8
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
        .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let sda = gpiob.pb9
        .into_open_drain_output(&mut gpiob.moder, &mut gpiob.otyper)
        .into_af4(&mut gpiob.moder, &mut gpiob.afrh);

    let i2c_port = p_hal::i2c::I2c::i2c1(
        dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1);

    let mut ahrs = USFS::new_inv_usfs_03(i2c_port, //i2c_bus.acquire(),
                                         em7180::EM7180_DEFAULT_ADDRESS,
                                         0, //unused for now
                                           false).unwrap();

    // let fflags = ahrs.check_feature_flags().unwrap_or(0);
    // if fflags != 0x05 { //barometer + temperature sensor installed
    //     d_println!(get_debug_log(), "fflags {}", fflags);
    // }


    //set initial states of user LEDs
    user_led1.set_low().unwrap();

    delay_source.delay_ms(1u8);
    //cortex_m::asm::delay(1_000);

    loop {
        // if let Ok(err_check) = ahrs.check_errors() {
        //     if 0 != err_check {
        //         d_println!(get_debug_log(), "err {:?}", err_check);
        //         break;
        //     }
        // }

        if ahrs.quat_available() {
            let qata =  ahrs.read_sentral_quat_qata();
            // if let Ok(_quat) = qata {
            //     d_println!(get_debug_log(), ".");
            //     //d_println!(log, "{:?}", quat);
            // }
            if qata.is_err() {
                bkpt();
                //panic!("read_err {:?}", qata);
            }
            else {
                user_led1.set_high().unwrap();
               // user_led1.toggle().unwrap();
            }
        }
        //cortex_m::asm::delay(1_000);
        delay_source.delay_ms(1u8);
    }

    //panic!("early termination")

}

#[cfg(feature = "stm32f4x")]
fn run_it() -> ! {

    let dp = stm32::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Set up the system clock
    let rcc = dp.RCC.constrain();
    // HSI: use default internal oscillator
    //let clocks = rcc.cfgr.freeze();
    // HSE: external crystal oscillator must be connected
    let clocks = rcc.cfgr.use_hse(25_000_000.hz()).freeze();

    let mut delay_source =  p_hal::delay::Delay::new(cp.SYST, clocks);

    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut user_led1 = gpioc.pc13.into_push_pull_output(); //f401CxUx


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

    let mut ahrs = USFS::new_inv_usfs_03(i2c_port,
                                         em7180::EM7180_DEFAULT_ADDRESS,
                                         0, //unused for now
                                      false).unwrap();


    let fflags = ahrs.check_feature_flags().unwrap_or(0);
    if fflags != 0x05 { //barometer + temperature sensor installed
        d_println!(get_debug_log(), "fflags {}", fflags);
   }

    //set initial states of user LEDs
    user_led1.set_high().unwrap();

    delay_source.delay_ms(1u8);


    loop {
        if let Ok(err_check) = ahrs.check_errors() {
            if 0 != err_check {
                d_println!(get_debug_log(), "err {:?}", err_check);
                break;
            }
        }
        if ahrs.quat_available() {
            user_led1.toggle().unwrap();
            //d_println!(get_debug_log(), ".");
            //let qata =  ahrs.read_sentral_quat_qata();
            //if let Ok(_quat) = qata {
            //    user_led1.toggle().unwrap();
                //d_println!(get_debug_log(), ".");
                //d_println!(get_debug_log(), "{:?}", quat);
            //}
            //else {
            //    bkpt();
            //    panic!("read_err {:?}", qata);
            //}
        }
        //user_led1.toggle().unwrap();
        delay_source.delay_ms(1u8);
    }

    loop {
        d_println!(get_debug_log(), "early termination");
        bkpt();
    }
    //panic!("early termination")

}




#[entry]
fn main() -> ! {
    run_it();
}
