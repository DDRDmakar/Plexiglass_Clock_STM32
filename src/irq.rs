
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
	pac::{interrupt, Interrupt, TIM2},
	timer::{Event, CounterUs},
	gpio::{self, ExtiPin},
};


// Global struct to work with interrupts
pub static G_REGS: Mutex<RefCell<Option<u8>>> = Mutex::new(RefCell::new(None));

// Global timer interrupt registers
pub static G_TIM: Mutex<RefCell<Option<CounterUs<TIM2>>>> = Mutex::new(RefCell::new(None));

// Global button handler
pub static G_BUT: Mutex<RefCell<Option<gpio::gpioa::PA0>>> = Mutex::new(RefCell::new(None));

pub fn unmask_irq() {
	unsafe {
		cortex_m::peripheral::NVIC::unmask(Interrupt::TIM2);
		cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI0);
	}
}

#[interrupt]
fn TIM2() {
	cortex_m::interrupt::free(|cs| {
		if let Some(r) = G_REGS.borrow(cs).borrow_mut().as_mut() {
			*r = 11;
		}
	});

    cortex_m::interrupt::free(|cs| {
		let mut tim = G_TIM.borrow(cs).borrow_mut();
		tim.as_mut().unwrap().clear_interrupt(Event::Update);
	});
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

