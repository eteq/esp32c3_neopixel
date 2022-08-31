use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use core::ptr;

use riscv::asm::nop;

use std::sync::Arc;

use embedded_hal::digital::v2::OutputPin;

use embedded_svc::wifi::*;
use embedded_svc::sys_time::SystemTime;

use esp_idf_svc::sysloop::EspSysLoopStack;
use esp_idf_svc::nvs::EspDefaultNvs;
use esp_idf_svc::netif::EspNetifStack;
use esp_idf_svc::wifi::EspWifi;
use esp_idf_svc::systime::EspSystemTime;

use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::gpio::*;


#[allow(unreachable_code)]
fn main()  -> Result<(), Box<dyn std::error::Error>>  {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    println!("Hello, world!");

    let sys_loop_stack = Arc::new(EspSysLoopStack::new()?);
    let wifi = EspWifi::new(
        Arc::new(EspNetifStack::new()?),
        sys_loop_stack.clone(),
        Arc::new(EspDefaultNvs::new()?)
    );

    let ap_infos = wifi.expect("WIFI not present!").scan()?;

    println!("Found these:");
    for row in ap_infos {
        println!("SSID: {}, c:{}, ss:{}", row.ssid, row.channel, row.signal_strength);
    }

    clock_timing_tests();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let mut g2d = pins.gpio2.into_output()?.degrade();

    println!("Starting neopixel loop");

    loop { 
        set_neopixel(&mut g2d, 255_u8, 0_u8, 0_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
        set_neopixel(&mut g2d, 0_u8, 255_u8, 0_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
        set_neopixel(&mut g2d, 0_u8, 0_u8, 255_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
        set_neopixel(&mut g2d, 255_u8, 255_u8, 0_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
        set_neopixel(&mut g2d, 255_u8, 0_u8, 255_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
        set_neopixel(&mut g2d, 0_u8, 255_u8, 255_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
        set_neopixel(&mut g2d, 255_u8, 255_u8, 255_u8)?; 
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }
    //do_unsafe_timing_tests();

    Ok(())
}

fn set_neopixel(pin: &mut GpioPin<Output>,  r: u8, g: u8, b: u8) -> Result<(), Box<dyn std::error::Error>> {
    
    // hold low for Treset
    pin.set_low()?;
    let start = EspSystemTime{}.now();
    while (EspSystemTime{}.now() - start).subsec_micros() < 50 {} //wait 50 us
    let grb = ((g as u32) << 16) | ((r as u32) << 8) | ((b as u32));
    write_neopixel_24(pin, grb)?;

    Ok(())
}

fn write_neopixel_24(pin: &mut GpioPin<Output>,  bytes: u32, ) -> Result<(), Box<dyn std::error::Error>> {  
    
    let pinmsk = 1 << pin.pin();
    let pininitial = unsafe{ptr::read_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32)};
    let pinon = pininitial | pinmsk;
    let pinoff = pininitial & !pinmsk;

    for i in (0..24).rev() {
        // timens = 238 + 31.5ticks (see do_unsafe_timing_tests())
        // times from https://github.com/adafruit/circuitpython/blob/main/ports/espressif/common-hal/neopixel_write/__init__.c#L50
        const TICKS_T1H:u32 = 15;
        const TICKS_T1L:u32 = 10;
        const TICKS_T0H:u32 = 2;
        const TICKS_T0L:u32 = 23;

        if (bytes >> i) & 1 > 0 {
            // 1
            unsafe {
                ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, pinon);
                // wait T1H from https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
                for _i in 0..TICKS_T1H {nop();}

                ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, pinoff);
                // wait T1L 
                for _i in 0..TICKS_T1L {nop();}
            }
        } else {
            // 0
            unsafe {
                ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, pinon);
                // wait T0H
                for _i in 0..TICKS_T0H {nop();}

                ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, pinoff);
                // wait T0L
                for _i in 0..TICKS_T0L {nop();}
            }
        }

        // Assume clock is running at 160 MHz
        // 300 ns = 48 cycles
        // 600 ns = 96 cycles
        // 6900 ns = 144 cycles

        //But real timing test gives:
        //10 cycles took 25000 ns
        // 50 cycles took 19000 ns
        // 100 cycles took 21000 ns
        // 500 cycles took 32000 ns
        // 1000 cycles took 50000 ns
        // 5000 cycles took 173000 ns
        // 10000 cycles took 330000 ns
        // 50000 cycles took 1580000 ns
        // 100000 cycles took 3142000 ns
        // which is ~32 ns per cycle:
        // 300 ns = 9.375 cycles
        // 600 ns = 18.75 cycles
        // 900 ns = 28.125 cycles
        // But then measiring with an oscilloscope with those timings gave:
        // 9 ticks= 1.04,.96,1.12,1.04,.96,1.04 us -> 1027 avg
        // 18 ticks= 1.28,1.36,1.28,1.52,1.36,1.28 us -> 1312 avg
        // 28 ticks= 1.68,1.6,1.6,1.68,1.68 us -> 1648 avg
        // -> time = 32.7 * ticks +729 ns +/- a few ns

        // even with no noop you get 700 ns...
        // const TICKS300NS:u32 = 1; //-13
        // const TICKS600NS:u32 = 1; // -3
        // const TICKS900NS:u32 = 5;


        // if (byte >> i) & 1_u8 > 0 {
        //     // 1
        //     pin.set_high()?;
        //     //unsafe {nop();}
        //     // wait 300 ns
        //     unsafe {for _i in 0..TICKS300NS {nop();}}
        //     pin.set_low()?;
        //     // wait 900 ns
        //     unsafe {for _i in 0..TICKS900NS {nop();}}
        //     unsafe {nop();}
        // } else {
        //     // 0
        //     pin.set_high()?;
        //     // wait 600 ns
        //     unsafe {for _i in 0..TICKS600NS {nop();}}
        //     pin.set_low()?;
        //     // wait 600 ns
        //     unsafe {for _i in 0..TICKS600NS {nop();}}
        // }


        // this was way too slow

        //let start = EspSystemTime{}.now();
        //pin.set_high()?;
        // if (byte >> i) & 1_u8 > 0 {
        //     // 1
        //     // wait 600 ns
        //     while (EspSystemTime{}.now() - start).subsec_nanos() < 600 {} 
        // } else {
        //     // 0
        //     // wait 300 ns
        //     while (EspSystemTime{}.now() - start).subsec_nanos() < 300 {}
        // }

        // pin.set_low()?;
        // while (EspSystemTime{}.now() - start).subsec_nanos() < 1200 {} // wait 1200 ns
    }

    
    Ok(())
}

#[allow(dead_code)]
fn clock_timing_tests() {
    let start10 = EspSystemTime{}.now();
    unsafe {for _i in 0..10 {nop();}}
    let end10 = EspSystemTime{}.now();
    println!("10 cycles took {} ns", (end10 - start10).subsec_nanos());

    let start50 = EspSystemTime{}.now();
    unsafe {for _i in 0..50 {nop();}}
    let end50 = EspSystemTime{}.now();
    println!("50 cycles took {} ns", (end50 - start50).subsec_nanos());

    let start100 = EspSystemTime{}.now();
    unsafe {for _i in 0..100 {nop();}}
    let end100 = EspSystemTime{}.now();
    println!("100 cycles took {} ns", (end100 - start100).subsec_nanos());

    let start500 = EspSystemTime{}.now();
    unsafe {for _i in 0..500 {nop();}}
    let end500 = EspSystemTime{}.now();
    println!("500 cycles took {} ns", (end500 - start500).subsec_nanos());


    let start1000 = EspSystemTime{}.now();
    unsafe {for _i in 0..1000 {nop();}}
    let end1000 = EspSystemTime{}.now();
    println!("1000 cycles took {} ns", (end1000 - start1000).subsec_nanos());


    let start5000 = EspSystemTime{}.now();
    unsafe {for _i in 0..5000 {nop();}}
    let end5000 = EspSystemTime{}.now();
    println!("5000 cycles took {} ns", (end5000 - start5000).subsec_nanos());

    let start10000 = EspSystemTime{}.now();
    unsafe {for _i in 0..10000 {nop();}}
    let end10000 = EspSystemTime{}.now();
    println!("10000 cycles took {} ns", (end10000 - start10000).subsec_nanos());


    let start50000 = EspSystemTime{}.now();
    unsafe {for _i in 0..50000 {nop();}}
    let end50000 = EspSystemTime{}.now();
    println!("50000 cycles took {} ns", (end50000 - start50000).subsec_nanos());

    let start100000 = EspSystemTime{}.now();
    unsafe {for _i in 0..100000 {nop();}}
    let end100000 = EspSystemTime{}.now();
    println!("100000 cycles took {} ns", (end100000 - start100000).subsec_nanos());
}

#[allow(dead_code)]
fn do_unsafe_timing_tests() {
    loop {
        //g4d.set_high()?;
        //g4d.set_low()?;
         // the above loop yields 740 ns between state transitions = ~1 MHz, way too slow?
        unsafe{
            // ~250-260 ns with 0 noops
            ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, 0b10000);
            ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, 0b00000);
            ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, 0b10000);
            // ~540 ns with 10 noops
            for _i in 0..10 {nop();}
            ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, 0b00000);
            // 860~ ns with 20 noops
            for _i in 0..20 {nop();}
            ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, 0b10000);
            // 1820~ ns with 50 noops
            for _i in 0..50 {nop();}
            ptr::write_volatile(esp_idf_sys::GPIO_OUT_REG as *mut u32, 0b00000);
            for _i in 0..200 {nop();}
            // regressing this gives timens = 238 + 31.5ticks
        }
    }
}