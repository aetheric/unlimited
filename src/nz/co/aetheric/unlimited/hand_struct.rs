pub mod driver::hand;

pub struct UH {

	kalmanX: Kalman,
	kalmanY: Kalman,

	/* IMU Data */
	accX: f64, accY: f64, accZ: f64,
	gyroX: f64, gyroY: f64, gyroZ: f64,

	gyroXAngle: f64, gyroYAngle: f64, gyroZAngle: f64, // Angle calculated using the gyro only
	compAngleX: f64, compAngleY: f64, // Calculated angle using a complementary filter
	kalAngleX: f64, kalAngleY: f64, // Calculated angle using a Kalman filter

	timer: u32,
	roll: f32, pitch: f32,

	currentEMSChannel: i32,
	EMSTimeCount: i32,
	stimuHighWid: i32,
	stimuLowWid: i32,
	stimuTimeCount: i32,
	currentVol: i32

}
