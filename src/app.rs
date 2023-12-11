
use core::fmt::Write;
use core::str;

use stm32f4xx_hal::{
	prelude::*,
	gpio::{Pin, Output, Input},
	timer::Delay,
	i2c::I2c,
	rtc::Rtc,
	pac,
	serial,
};

use shared_bus::{NullMutex, I2cProxy};
//use time::{
//	macros::{date, time},
//	PrimitiveDateTime,
//};
use ssd1306::{
	prelude::*,
	Ssd1306,
	mode::BufferedGraphicsMode,
};
use embedded_graphics::{
	prelude::*,
	pixelcolor::BinaryColor,
	text::Text,
	mono_font::MonoTextStyle,
	primitives::{Rectangle, PrimitiveStyleBuilder},
};
use bmp280_ehal::{self, PowerMode, Oversampling};
use rotary_encoder_embedded::{RotaryEncoder, Direction, standard::StandardMode};

use crate::irq::*;
use crate::util::*;

type OledDisplayType<'a> = Ssd1306<I2CInterface<I2cProxy<'a, NullMutex<I2c<pac::I2C1>>>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;
type RotaryEncoderType = RotaryEncoder<StandardMode, Pin<'B', 12, Input>, Pin<'B', 13, Input>>;
type BMP280Type<'a> = bmp280_ehal::BMP280<I2cProxy<'a, NullMutex<I2c<pac::I2C1>>>>;

pub const BMP_CONTROL: bmp280_ehal::Control = bmp280_ehal::Control {
	osrs_t: Oversampling::x1,
	osrs_p: Oversampling::x4,
	mode: PowerMode::Forced,
};

pub struct App<'a> {
	rtc: Rtc,
	led: Pin<'C', 13, Output>,
	delay_t5: Delay<pac::TIM5, 1_000_000>,
	uart_tx: serial::Tx<pac::USART2, u8>,
	rotary_encoder: RotaryEncoderType,
	displ: OledDisplayType<'a>,
	style_6x10: MonoTextStyle<'a, BinaryColor>,
	style_10x20: MonoTextStyle<'a, BinaryColor>,
	bmp: BMP280Type<'a>,

	pressure: f32,
	temperature: f32,
	cursor_position: i8,
}

impl<'b> App<'b> {
	pub fn new(
		rtc: Rtc,
		led: Pin<'C', 13, Output>,
		delay_t5: Delay<pac::TIM5, 1_000_000>,
		uart_tx: serial::Tx<pac::USART2, u8>,
		rotary_encoder: RotaryEncoderType,
		displ: OledDisplayType<'b>,
		style_6x10: MonoTextStyle<'b, BinaryColor>,
		style_10x20: MonoTextStyle<'b, BinaryColor>,
		bmp: BMP280Type<'b>,
	) -> Self {
		Self {
			rtc,
			led,
			delay_t5,
			uart_tx,
			rotary_encoder,
			displ,
			style_6x10,
			style_10x20,
			bmp,
			pressure: 0.0,
			temperature: 0.0,
			cursor_position: 0,
		}
	}
	
	pub fn main_loop(&mut self) {
		let mut loop_ctr = 0_u32;
		
		loop {
			loop_ctr += 1;

			let mut r = 0_u8;
			cortex_m::interrupt::free(|cs| {
				if let Some(r_) = G_REGS.borrow(cs).borrow_mut().as_mut() {
					r = *r_;
					*r_ = 0;
				}
			});

			// Update pressure and temperature
			if loop_ctr % 10000 == 0 {
				self.pressure = self.bmp.pressure() as f32;
				self.temperature = self.bmp.temp() as f32;
				self.bmp.set_control(BMP_CONTROL);
				writeln!(self.uart_tx, "Update temp").unwrap();
			}

			if r != 0 /*|| encoder_result != 0*/ {
				self.led.set_low();
				self.display_main_screen();
				self.draw_cursor();
				self.led.set_high();
			}

			self.rotary_encoder.update();
			match self.rotary_encoder.direction() {
				Direction::Clockwise => {
					self.cursor_position += 1;
					self.draw_cursor();
				}
				Direction::Anticlockwise => {
					self.cursor_position -= 1;
					self.draw_cursor();
				}
				Direction::None => { }
			}

			self.delay_t5.delay_ms(1_u32);
		}
	}

	pub fn display_main_screen(&mut self) {
		let dt = self.rtc.get_datetime();
		writeln!(self.uart_tx, "{} {:.2} {:.2}\t", dt, self.temperature, self.pressure).unwrap();

		self.displ.clear_buffer();

		let mut buf = [0u8; 19];
		uint_to_str10(dt.year() as u32, 4, &mut buf[0..4], true);
		uint_to_str10(dt.month() as u32, 2, &mut buf[5..7], true);
		uint_to_str10(dt.day() as u32, 2, &mut buf[8..10], true);
		uint_to_str10(dt.hour() as u32, 2, &mut buf[11..13], false);
		uint_to_str10(dt.minute() as u32, 2, &mut buf[14..16], true);
		uint_to_str10(dt.second() as u32, 2, &mut buf[17..19], true);
		buf[4] = b'-';
		buf[7] = b'-';
		buf[10] = b' ';
		buf[13] = b':';
		buf[16] = b':';

		Text::new(
			str::from_utf8(&buf[0..10]).unwrap(),
			Point::new(0, 15),
			self.style_10x20
		).draw(&mut self.displ).unwrap();
		Text::new(
			str::from_utf8(&buf[11..19]).unwrap(),
			Point::new(0, 30),
			self.style_10x20
		).draw(&mut self.displ).unwrap();

		let mmhg = self.pressure * 7.50062e-3;
		let is_above_zero = self.temperature >= 0.;
		let temperature_abs = if is_above_zero { self.temperature } else { -self.temperature };
		
		uint_to_str10(round_f32_i32(temperature_abs * 10.) as u32, 4, &mut buf[0..4], true);
		buf[0] = if is_above_zero { b'+' } else { b'-' };
		buf[4] = buf[3];
		buf[3] = b'.';
		buf[5] = b' ';
		buf[6] = b'C';
		Text::new(
			str::from_utf8(&buf[0..7]).unwrap(),
			Point::new(0, 40),
			self.style_6x10
		).draw(&mut self.displ).unwrap();

		uint_to_str10(round_f32_i32(self.pressure) as u32, 6, &mut buf[0..6], false);
		buf[6] = b' ';
		buf[7] = b'P';
		buf[8] = b'a';
		Text::new(
			str::from_utf8(&buf[0..9]).unwrap(),
			Point::new(0, 50),
			self.style_6x10
		).draw(&mut self.displ).unwrap();

		uint_to_str10(round_f32_i32(mmhg) as u32, 4, &mut buf[0..4], false);
		buf[4] = b' ';
		buf[5] = b'm';
		buf[6] = b'm';
		buf[7] = b'H';
		buf[8] = b'g';
		Text::new(
			str::from_utf8(&buf[0..9]).unwrap(),
			Point::new(0, 60),
			self.style_6x10
		).draw(&mut self.displ).unwrap();
		
		self.displ.flush().unwrap();
	}

	fn draw_cursor(&mut self) {
		let rectangle_style = PrimitiveStyleBuilder::new()
			.stroke_color(BinaryColor::On)
			.stroke_width(1)
			//.fill_color(BinaryColor::Off)
			.build();
		Rectangle::new(Point::new(60 + 10 * self.cursor_position as i32, 40), Size::new(10, 20))
			.into_styled(rectangle_style)
			.draw(&mut self.displ).unwrap();
		self.displ.flush().unwrap();
	}
}
