
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
use time::{
	macros::{date, time},
	PrimitiveDateTime,
	Duration,
};
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
	primitives::{Rectangle, Circle, PrimitiveStyle, PrimitiveStyleBuilder},
};
use bmp280_ehal::{self, PowerMode, Oversampling};
use rotary_encoder_embedded::{RotaryEncoder, Direction, standard::StandardMode};

use crate::irq::*;
use crate::util::*;

type OledDisplayType<'a> = Ssd1306<I2CInterface<I2cProxy<'a, NullMutex<I2c<pac::I2C1>>>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>;
type RotaryEncoderType = RotaryEncoder<StandardMode, Pin<'B', 13, Input>, Pin<'B', 12, Input>>;
type BMP280Type<'a> = bmp280_ehal::BMP280<I2cProxy<'a, NullMutex<I2c<pac::I2C1>>>>;

pub const BMP_CONTROL: bmp280_ehal::Control = bmp280_ehal::Control {
	osrs_t: Oversampling::x1,
	osrs_p: Oversampling::x4,
	mode: PowerMode::Forced,
};

const DATETIME_CURSOR_COORD: [(u8, u8); 14] = [
	(10 * 0, 1 + 0), // Y
	(10 * 1, 1 + 0), // Y
	(10 * 2, 1 + 0), // Y
	(10 * 3, 1 + 0), // Y
	(10 * 5, 1 + 0), // M
	(10 * 6, 1 + 0), // M
	(10 * 8, 1 + 0), // D
	(10 * 9, 1 + 0), // D
	
	(10 * 0, 1 + 20), // h
	(10 * 1, 1 + 20), // h
	(10 * 3, 1 + 20), // m
	(10 * 4, 1 + 20), // m
	(10 * 6, 1 + 20), // s
	(10 * 7, 1 + 20), // s
];

enum State {
	Idle,
	SetTime,
}

enum CursorRole {
	SelectDigit,
	SetTime,
}

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

	state: State,
	pressure: f32,
	temperature: f32,
	cursor_position: u8,
	encoder_value: i8,
	datetime: PrimitiveDateTime,
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
			state: State::Idle,
			pressure: 0.0,
			temperature: 0.0,
			cursor_position: 0,
			encoder_value: 0,
			datetime: PrimitiveDateTime::new(
				date!(1970-01-01),
				time!(00:00:00),
			),
		}
	}
	
	pub fn main_loop(&mut self) {
		self.state = State::Idle;
		let mut loop_ctr = 0_u32;
		let mut cursor_role = CursorRole::SelectDigit;
		let mut flags = 0_u8;
		
		loop {
			loop_ctr += 1;

			// Copy flags into local variable and clear
			cortex_m::interrupt::free(|cs| {
				if let Some(f) = G_FLAGS.borrow(cs).borrow_mut().as_mut() {
					flags |= *f;
					*f = 0;
				}
			});

			// Update pressure and temperature
			if loop_ctr % 10000 == 0 {
				self.pressure = self.bmp.pressure() as f32;
				self.temperature = self.bmp.temp() as f32;
				self.bmp.set_control(BMP_CONTROL);
				writeln!(self.uart_tx, "Update temp").unwrap();
			}

			self.rotary_encoder.update();
			match self.rotary_encoder.direction() {
				Direction::Clockwise => {
					self.encoder_value = self.encoder_value.saturating_add(1);
				}
				Direction::Anticlockwise => {
					self.encoder_value = self.encoder_value.saturating_sub(1);
				}
				Direction::None => { }
			}

			match self.state {
				State::Idle => {
					if flags & FLAG_TIMER_1S != 0 {
						flags &= !FLAG_TIMER_1S;
						self.led.set_low();
						self.display_screen_main();
						self.displ.flush().unwrap();
						self.led.set_high();
					}
					else if flags & FLAG_ENCODER_PRESSED != 0 {
						flags &= !FLAG_ENCODER_PRESSED;
						self.state = State::SetTime;
						self.datetime = self.rtc.get_datetime();
						self.display_screen_set_time();
						self.displ.flush().unwrap();
						// Clear button flag cause it's a return condition from State::SetTime
						flags &= !FLAG_BUTTON_PRESSED;
						self.encoder_value = 0;
					}
				},
				State::SetTime => {
					if flags & FLAG_BUTTON_PRESSED != 0 {
						flags &= !FLAG_BUTTON_PRESSED;
						self.state = State::Idle;
					}
					else if self.encoder_value != 0 {
						match cursor_role {
							CursorRole::SelectDigit => {
								self.cursor_position = self.cursor_position.saturating_add_signed(self.encoder_value);
								self.cursor_position = if self.cursor_position > 13 { 13 } else { self.cursor_position };
								self.display_screen_set_time();
								self.draw_cursor();
								self.displ.flush().unwrap();

								if flags & FLAG_ENCODER_PRESSED != 0 { cursor_role = CursorRole::SetTime; }
								flags &= !FLAG_ENCODER_PRESSED;
							},
							CursorRole::SetTime => {
								//self.change_time();
								self.datetime = self.datetime.saturating_add(Duration::days(1));
								self.display_screen_set_time();
								self.draw_cursor();
								self.displ.flush().unwrap();

								if flags & FLAG_ENCODER_PRESSED != 0 { cursor_role = CursorRole::SelectDigit; }
								flags &= !FLAG_ENCODER_PRESSED;
							},
						}
						self.encoder_value = 0;
					}
				},
			}
			
			self.delay_t5.delay_ms(1_u32);
		}
	}

	pub fn display_screen_main(&mut self) {
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
			Point::new(0, 35),
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
			Point::new(70, 50),
			self.style_6x10
		).draw(&mut self.displ).unwrap();
		Circle::new(Point::new(70 + (6 * 5) + 1, 42), 4)
			.into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
			.draw(&mut self.displ).unwrap();

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
	}

	pub fn display_screen_set_time(&mut self) {
		let dt = self.datetime;
		
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
			Point::new(0, 35),
			self.style_10x20
		).draw(&mut self.displ).unwrap();
	}

	fn draw_cursor(&mut self) {
		let rectangle_style = PrimitiveStyleBuilder::new()
			.stroke_color(BinaryColor::On)
			.stroke_width(1)
			//.fill_color(BinaryColor::Off)
			.build();
		Rectangle::new(
			Point::new(
				DATETIME_CURSOR_COORD[self.cursor_position as usize].0.into(),
				DATETIME_CURSOR_COORD[self.cursor_position as usize].1.into()
			),
			Size::new(10, 17))
			.into_styled(rectangle_style)
			.draw(&mut self.displ).unwrap();
	}
}
