
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
	pac::{interrupt, Interrupt, TIM2},
	timer::{Event, CounterUs},
	gpio::{self, ExtiPin},
};


// Global flags to work with interrupts
pub static G_FLAGS: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));
pub const FLAG_TIMER_1S: u8        = 1 << 0;
pub const FLAG_BUTTON_PRESSED: u8  = 1 << 1;
pub const FLAG_ENCODER_PRESSED: u8 = 1 << 2;

// Global timer interrupt registers
pub static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));

// Global button handler
pub static G_BUT: Mutex<RefCell<Option<gpio::gpioa::PA0>>> = Mutex::new(RefCell::new(None));

pub static G_ENCODER_PB: Mutex<RefCell<Option<gpio::gpioa::PA1>>> = Mutex::new(RefCell::new(None));

pub fn unmask_irq() {
	unsafe {
		cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
		cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0);
		cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI1);
	}
}

#[interrupt]
fn TIM2() {
	// Timer interrupt every second
	cortex_m::interrupt::free(|cs| {
		if let Some(r) = G_FLAGS.borrow(cs).borrow_mut().as_mut() {
			*r |= FLAG_TIMER_1S;
		}
	});

    cortex_m::interrupt::free(|cs| {
		let mut tim = G_TIM.borrow(cs).borrow_mut();
		tim.as_mut().unwrap().clear_interrupt(Event::Update);
	});
}

#[interrupt]
fn EXTI0() {
	// Push button pressed
	cortex_m::interrupt::free(|cs| {
		if let Some(r) = G_FLAGS.borrow(cs).borrow_mut().as_mut() {
			*r |= FLAG_BUTTON_PRESSED;
		}
	});
	
	cortex_m::interrupt::free(|cs| {
		if let Some(but) = G_BUT.borrow(cs).borrow_mut().as_mut() {
			but.clear_interrupt_pending_bit();
		}
	});

}

#[interrupt]
fn EXTI1() {
	// Button on rotary encoder pressed
	cortex_m::interrupt::free(|cs| {
		if let Some(r) = G_FLAGS.borrow(cs).borrow_mut().as_mut() {
			*r |= FLAG_ENCODER_PRESSED;
		}
	});
	
	cortex_m::interrupt::free(|cs| {
		if let Some(encoder_pb) = G_ENCODER_PB.borrow(cs).borrow_mut().as_mut() {
			encoder_pb.clear_interrupt_pending_bit();
		}
	});

}
