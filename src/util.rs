
pub fn uint_to_str10(x: u32, n: usize, buf: &mut [u8], leading_zeros: bool) {
	let mut t = x;
	for i in 0..n {
		if t == 0 && i != 0 && !leading_zeros {
			buf[n-i-1] = b' ';
		} else {
			buf[n-i-1] = b'0' + (t % 10) as u8;
		}
		t /= 10;
	}
}

pub fn round_f32_i32(x: f32) -> i32 {
	(x + 0.5 - ((x >= 0.5) as u8 as f32)) as i32
}
