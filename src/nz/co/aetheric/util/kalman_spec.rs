
#[allow(unused_imports)]
use nz::co::aetheric::util::kalman::Kalman;

#[test]
fn default_kalman() {

	let kalman = Kalman::new(); {
		assert_eq!(0.001f32, kalman.q_angle);
		assert_eq!(0.003f32, kalman.q_bias);
		assert_eq!(0.03f32, kalman.r_measure);
		assert_eq!(0.0f32, kalman.angle);
		assert_eq!(0.0f32, kalman.bias);
		assert_eq!(0.0f32, kalman.bias);
		assert_eq!(0.0f32, kalman.p[0][0]);
		assert_eq!(0.0f32, kalman.p[0][1]);
		assert_eq!(0.0f32, kalman.p[1][0]);
		assert_eq!(0.0f32, kalman.p[1][1]);
	}

}

#[test]
fn mutable_kalman_angle() {

	let kalman = Kalman::new(); {
		assert_eq!(0.0f32, kalman.angle);
	}

	let result = kalman.angle(0.5f32); {
		assert_eq!(0.5f32, result.angle);
	}

}

#[test]
fn mutable_kalman_q_angle() {

	let kalman = Kalman::new(); {
		assert_eq!(0.001f32, kalman.q_angle);
	}

	let result = kalman.q_angle(0.5f32); {
		assert_eq!(0.5f32, result.q_angle);
	}

}

#[test]
fn mutable_kalman_q_bias() {

	let kalman = Kalman::new(); {
		assert_eq!(0.003f32, kalman.q_bias);
	}

	let result = kalman.q_bias(0.5f32); {
		assert_eq!(0.5f32, result.q_bias);
	}

}

#[test]
fn mutable_kalman_r_measure() {

	let kalman = Kalman::new(); {
		assert_eq!(0.03f32, kalman.r_measure);
	}

	let result = kalman.r_measure(0.5f32); {
		assert_eq!(0.5f32, result.r_measure);
	}


}
