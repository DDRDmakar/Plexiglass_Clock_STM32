//#![deny(unsafe_code)]
#![no_main]
#![no_std]

#![allow(dead_code)]


// Halt on panic
use panic_halt as _;

use core::str;
use core::fmt::Write;

use cortex_m_rt::entry;
use stm32f4xx_hal::{
    prelude::*,
	gpio::Edge,
	timer::Event,
	i2c::{I2c, Mode},
	rtc::Rtc,
    pac,
	serial,
};

use shared_bus;
use time::{
    macros::{date, time},
    PrimitiveDateTime,
};

use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use embedded_graphics::{
    prelude::*,
    pixelcolor::BinaryColor,
	text::Text,
	mono_font::{
		MonoTextStyle,
		iso_8859_5::{FONT_10X20, FONT_6X10},
	},
};
use bmp280_ehal::{self, PowerMode, Standby, Filter, Oversampling};

mod util;
mod irq;
use crate::util::*;
use crate::irq::*;


#[entry]
fn main() -> ! {
	//-------- Peripherals handler
	let mut dp = pac::Peripherals::take().unwrap();
	//let cp = cortex_m::peripheral::Peripherals::take().unwrap();

	let rcc = dp.RCC.constrain();
	let mut syscfg = dp.SYSCFG.constrain();
	
	//-------- Clocks
    let clocks = rcc.cfgr
		.use_hse(25.MHz()) // Using the high-speed external oscillator
		.hclk(50.MHz()) // Set the frequency for the AHB bus, which the root of every following clock peripheral
		.sysclk(50.MHz()) // System clock
		.pclk1(25.MHz()) // Peripheral clocks
		.pclk2(25.MHz()) // Peripheral clocks
		.freeze();
	
	//-------- Timers
	let mut timer2 = dp.TIM2.counter(&clocks);
    timer2.start(1.secs()).unwrap();
	timer2.listen(Event::Update);
	
	//let mut delay_sys = cp.SYST.delay(&clocks);
	//let mut delay_t4 = dp.TIM4.delay_us(&clocks);
	let mut delay_t5 = dp.TIM5.delay_us(&clocks);


	//-------- RTC
	let mut rtc = Rtc::new(dp.RTC, &mut dp.PWR);
	
	rtc.set_datetime(&PrimitiveDateTime::new(
        date!(1970 - 01 - 01),
        time!(23:59:50),
    )).unwrap();
	
	//-------- GPIO
	let gpioa = dp.GPIOA.split();
	let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();

    let mut led = gpioc.pc13.into_push_pull_output(); // Led in the module
	let mut but = gpioa.pa0.into_pull_up_input(); // Button on the module

	//-------- UART
	let uart_tx_pin = gpioa.pa2.into_alternate();
	let mut uart_tx: serial::Tx<pac::USART2, u8> = dp.USART2.tx(
        uart_tx_pin,
        serial::config::Config::default()
            .baudrate(115200.bps())
            .wordlength_8()
            .parity_none(),
        &clocks,
    ).unwrap();
	writeln!(&mut uart_tx, "UART is ready").unwrap();

	//-------- On-module button irq
	but.make_interrupt_source(&mut syscfg);
    but.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    but.enable_interrupt(&mut dp.EXTI);

	let scl = gpiob.pb8.into_open_drain_output();
	let sda = gpiob.pb9.into_open_drain_output();
	let i2c_inst = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard {
            frequency: 100.kHz(),
        },
        &clocks,
    );
	let i2c_shared = shared_bus::BusManagerSimple::new(i2c_inst);
	
	//-------- I2C display
	let display_interface = I2CDisplayInterface::new(i2c_shared.acquire_i2c());
	let mut displ = Ssd1306::new(display_interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    displ.init().unwrap();

	let style_6x10 = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
	let style_10x20 = MonoTextStyle::new(&FONT_10X20, BinaryColor::On);

	//-------- Thermometer and barometer
	let mut bmp = bmp280_ehal::BMP280::new(i2c_shared.acquire_i2c()).unwrap();
	let mut bmp_control = bmp.control();
	bmp.set_control(bmp280_ehal::Control {
        osrs_t: Oversampling::x1,
        osrs_p: Oversampling::x4,
        mode: PowerMode::Normal,
    });
	bmp.reset();
	bmp.set_config(bmp280_ehal::Config {
        t_sb: Standby::ms250,
        filter: Filter::c8,
    });

	//-------- Set global variables
	let global_regs: u8 = 0x55; // TODO
	cortex_m::interrupt::free(|cs| *G_REGS.borrow(cs).borrow_mut() = Some(global_regs));
	cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer2));
	cortex_m::interrupt::free(|cs| *G_BUT.borrow(cs).borrow_mut() = Some(but));

	//-------- Unmask IRQ
	unmask_irq();

	//--------------------------------------------------------------------------

	//let mut regs_copy: u8 = 0;
	//cortex_m::interrupt::free(|cs| {
	//	G_REGS.borrow(cs).replace(None).unwrap()
	//});
	
    loop {
		
		cortex_m::interrupt::free(|cs| {
			if let Some(r) = G_REGS.borrow(cs).borrow_mut().as_mut() {
				if *r != 0 {
					*r = 0;
					led.set_low();

					let dt = rtc.get_datetime();
					write!(uart_tx, "{}\t", dt).unwrap();

					let mut buf = [0u8; 19];

					uint_to_str10(dt.year() as u32, 4, &mut buf[0..4]);
					uint_to_str10(dt.month() as u32, 2, &mut buf[5..7]);
					uint_to_str10(dt.day() as u32, 2, &mut buf[8..10]);
					uint_to_str10(dt.hour() as u32, 2, &mut buf[11..13]);
					uint_to_str10(dt.minute() as u32, 2, &mut buf[14..16]);
					uint_to_str10(dt.second() as u32, 2, &mut buf[17..19]);
					buf[4] = b'-';
					buf[7] = b'-';
					buf[10] = b' ';
					buf[13] = b':';
					buf[16] = b':';

					let pressure = bmp.pressure();
					let temperature = bmp.temp();
					writeln!(uart_tx, "{} {}", pressure, temperature).unwrap();
					bmp.set_control(bmp280_ehal::Control {
						osrs_t: Oversampling::x1,
						osrs_p: Oversampling::x4,
						mode: PowerMode::Normal,
					});
					
					displ.clear_buffer();
					Text::new(
						str::from_utf8(&buf[0..10]).unwrap(),
						Point::new(0, 15),
						style_10x20
					).draw(&mut displ).unwrap();
					Text::new(
						str::from_utf8(&buf[11..19]).unwrap(),
						Point::new(0, 30),
						style_10x20
					).draw(&mut displ).unwrap();					
					displ.flush().unwrap();
				} else {
					led.set_high();
					delay_t5.delay_ms(100_u32);
				}
			}
		});
    }
}
