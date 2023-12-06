
pub fn uint_to_str10(x: u32, n: usize, buf: &mut [u8]) {
	let mut t = x;
	for i in 0..n {
		buf[n-i-1] = b'0' + (t % 10) as u8;
		t /= 10;
	}
}
