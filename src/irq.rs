
use core::cell::RefCell;
use cortex_m::interrupt::Mutex;
use stm32f4xx_hal::{
	pac::{interrupt, Interrupt, TIM2},
	timer::CounterUs,
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
    static mut TIM: Option<CounterUs<TIM2>> = None;

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

