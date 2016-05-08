
#[path="mod.rs"]
mod kalman;

#[test]
fn defaultKalman() {
	let kalman = Kalman::new();

	// Test the properties
	assert_eq!(0.001f32, kalman.Q_angle);
	assert_eq!(0.003f32, kalman.Q_bias);
	assert_eq!(0.03f32, kalman.R_measure);
	assert_eq!(0.0f32, kalman.angle);
	assert_eq!(0.0f32, kalman.bias);
	assert_eq!(0.0f32, kalman.bias);
	assert_eq!(0.0f32, kalman.P[0][0]);
	assert_eq!(0.0f32, kalman.P[0][1]);
	assert_eq!(0.0f32, kalman.P[1][0]);
	assert_eq!(0.0f32, kalman.P[1][1]);

}

#[test]
fn mutableKalmanAngle() {
	let kalman = Kalman::new();

	assert_eq!(0.0f32, kalman.angle);

	kalman.setAngle(0.5f32);

	assert_eq!(0.5f32, kalman.angle);

}

#[test]
fn mutableKalmanQangle() {
	let kalman = Kalman::new();

	assert_eq!(0.0f32, kalman.Q_angle);

	kalman.setQangle(0.5f32);

	assert_eq!(0.5f32, kalman.Q_angle);

}

#[test]
fn mutableKalmanQbias() {
	let kalman = Kalman::new();

	assert_eq!(0.0f32, kalman.Q_bias);

	kalman.setQbias(0.5f32);

	assert_eq!(0.5f32, kalman.Q_bias);

}

#[test]
fn mutableKalmanRmeasure() {
	let kalman = Kalman::new();

	assert_eq!(0.0f32, kalman.R_measure);

	kalman.setRmeasure(0.5f32);

	assert_eq!(0.5f32, kalman.R_measure);

}
