
#[allow(unused_imports)]
use nz::co::aetheric::util::kalman::Kalman;

#[test]
fn expected_new() {

	let kalman = Kalman::new(); {
		assert_eq!(0.001f32, kalman.q_angle);
		assert_eq!(0.003f32, kalman.q_bias);
		assert_eq!(0.03f32, kalman.r_measure);
		assert_eq!(0.0f32, kalman.angle);
		assert_eq!(0.0f32, kalman.bias);
		assert_eq!(0.0f32, kalman.rate);
		assert_eq!(0.0f32, kalman.p[0][0]);
		assert_eq!(0.0f32, kalman.p[0][1]);
		assert_eq!(0.0f32, kalman.p[1][0]);
		assert_eq!(0.0f32, kalman.p[1][1]);
	}

}

#[test]
fn mutable_angle() {

	let kalman = Kalman::new(); {
		assert_eq!(0.0f32, kalman.angle);
	}

	let result = kalman.angle(0.5f32); {
		assert_eq!(0.5f32, result.angle);
	}

}

#[test]
fn mutable_q_angle() {

	let kalman = Kalman::new(); {
		assert_eq!(0.001f32, kalman.q_angle);
	}

	let result = kalman.q_angle(0.5f32); {
		assert_eq!(0.5f32, result.q_angle);
	}

}

#[test]
fn mutable_q_bias() {

	let kalman = Kalman::new(); {
		assert_eq!(0.003f32, kalman.q_bias);
	}

	let result = kalman.q_bias(0.5f32); {
		assert_eq!(0.5f32, result.q_bias);
	}

}

#[test]
fn mutable_r_measure() {

	let kalman = Kalman::new(); {
		assert_eq!(0.03f32, kalman.r_measure);
	}

	let result = kalman.r_measure(0.5f32); {
		assert_eq!(0.5f32, result.r_measure);
	}

}

#[test]
fn expected_update() {

	let kalman = Kalman::new(); {
		assert_eq!(0.001f32, kalman.q_angle);
		assert_eq!(0.003f32, kalman.q_bias);
		assert_eq!(0.03f32, kalman.r_measure);
		assert_eq!(0.0f32, kalman.angle);
		assert_eq!(0.0f32, kalman.bias);
		assert_eq!(0.0f32, kalman.rate);
		assert_eq!(0.0f32, kalman.p[0][0]);
		assert_eq!(0.0f32, kalman.p[0][1]);
		assert_eq!(0.0f32, kalman.p[1][0]);
		assert_eq!(0.0f32, kalman.p[1][1]);
	}

	let result = kalman.update(0.1f32, 0.2f32, 0.0001f32); {
		assert_eq!(0.001f32, result.q_angle);
		assert_eq!(0.003f32, result.q_bias);
		assert_eq!(0.03f32, result.r_measure);
		assert_eq!(0.000020333266f32, result.angle);
		assert_eq!(0.0f32, result.bias);
		assert_eq!(0.2f32, result.rate);
		assert_eq!(0.00000009999967f32, result.p[0][0]);
		assert_eq!(0.0f32, result.p[0][1]);
		assert_eq!(0.0f32, result.p[1][0]);
		assert_eq!(0.00000029999998f32, result.p[1][1]);
	}

}
