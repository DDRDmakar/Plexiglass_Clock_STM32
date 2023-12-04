//#![deny(unsafe_code)]
#![no_main]
#![no_std]

#![allow(dead_code)]


// Halt on panic
use panic_halt as _;


use cortex_m_rt::entry;
use stm32f4xx_hal::{
	gpio::{self, Edge},
    pac,
    prelude::*,
	pac::{interrupt, Interrupt, TIM2},
	timer::{CounterUs, Event},
	i2c::{I2c, Mode},
	serial
};
use core::cell::RefCell;
use core::fmt::Write;
use cortex_m::interrupt::Mutex;


// Globally available struct to work with interrupts
static G_REGS: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));

// Make timer interrupt registers globally available
static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));

static G_BUT: Mutex<RefCell<Option<gpio::gpioa::PA0>>> = Mutex::new(RefCell::new(None));

#[allow(clippy::empty_loop)]
#[entry]
fn main() -> ! {
	let (Some(mut dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) else { todo!() } ;

    let gpioa = dp.GPIOA.split();
	let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let mut led = gpioc.pc13.into_push_pull_output();
	let mut but = gpioa.pa0.into_pull_up_input();

	let regs: u8 = 0x55;
	cortex_m::interrupt::free(|cs| *G_REGS.borrow(cs).borrow_mut() = Some(regs));

	let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr
		.use_hse(25.MHz()) // Using the high-speed external oscillator
		.hclk(50.MHz()) // Set the frequency for the AHB bus, which the root of every following clock peripheral
		.sysclk(50.MHz())
		.pclk1(25.MHz()) // Peripheral clocks
		.pclk2(25.MHz()) // Peripheral clocks
		.freeze();
	//let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();
	let mut timer = dp.TIM2.counter(&clocks);
    timer.start(1.secs()).unwrap();

	timer.listen(Event::Update);



	//let mut delay = cp.SYST.delay(&clocks);
	let mut delay = dp.TIM5.delay_us(&clocks);

	//let mut rtc = Rtc::new(dp.RTC, &mut dp.PWR);
	//
	//rtc.set_datetime(&PrimitiveDateTime::new(
    //    date!(2022 - 02 - 07),
    //    time!(23:59:50),
    //)).unwrap();

	let mut syscfg = dp.SYSCFG.constrain();

	but.make_interrupt_source(&mut syscfg);
    but.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
    but.enable_interrupt(&mut dp.EXTI);

	let scl = gpiob.pb6;
	let sda = gpiob.pb7;
	let mut i2c = I2c::new(
        dp.I2C1,
        (scl, sda),
        Mode::Standard {
            frequency: 300.kHz(),
        },
        &clocks,
    );

	let tx_pin = gpioa.pa2.into_alternate();
	let mut uart_tx: serial::Tx<pac::USART2, u8> = dp.USART2.tx(
        tx_pin,
        serial::config::Config::default()
            .baudrate(115200.bps())
            .wordlength_8()
            .parity_none(),
        &clocks,
    ).unwrap();


	unsafe {
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
		cortex_m::peripheral::NVIC::unmask(but.interrupt());
    }
	
	cortex_m::interrupt::free(|cs| *G_TIM.borrow(cs).borrow_mut() = Some(timer));
	cortex_m::interrupt::free(|cs| *G_BUT.borrow(cs).borrow_mut() = Some(but));

    loop {
        //for _ in 0..100_000 {
        //    led.set_high();
        //}
        //for _ in 0..100_000 {
        //    led.set_low();
        //}

		//static mut REGS: Option<u8> = None;
		//let r = REGS.get_or_insert_with(|| {
		//	cortex_m::interrupt::free(|cs| {
		//		// Move LED pin here, leaving a None in its place
		//		G_REGS.borrow(cs).replace(None).unwrap()
		//	})
		//});

		//cortex_m::interrupt::free(|cs| *G_REGS.borrow(cs).borrow_mut() = Some(11));
		cortex_m::interrupt::free(|cs| {
			if let Some(r) = G_REGS.borrow(cs).borrow_mut().as_mut() {
				if *r != 0 {
					led.set_low();
					*r = 0;
					//println!("a");
					writeln!(uart_tx, "a").unwrap();
				} else {
					led.set_high();
				}	
			}
		});
		delay.delay_ms(100_u32);
    }
}

#[interrupt]
fn TIM2() {
    static mut TIM: Option<CounterUs<TIM2>> = None;

    //let regs = REGS.get_or_insert_with(|| {
    //    cortex_m::interrupt::free(|cs| {
    //        // Move LED pin here, leaving a None in its place
    //        G_REGS.borrow(cs).replace(None).unwrap()
    //    })
    //});

	cortex_m::interrupt::free(|cs| {
		if let Some(r) = G_REGS.borrow(cs).borrow_mut().as_mut() {
			*r = 11;
		}
	});

    let tim = TIM.get_or_insert_with(|| {
        cortex_m::interrupt::free(|cs| {
            // Move LED pin here, leaving a None in its place
            G_TIM.borrow(cs).replace(None).unwrap()
        })
    });

    let _ = tim.wait();
}

#[interrupt]
fn EXTI0() {
	cortex_m::interrupt::free(|cs| {
		if let Some(r) = G_REGS.borrow(cs).borrow_mut().as_mut() {
			*r = 11;
		}
	});
	cortex_m::interrupt::free(|cs| {
		if let Some(but) = G_BUT.borrow(cs).borrow_mut().as_mut() {
			but.clear_interrupt_pending_bit();
		}
	});

}
