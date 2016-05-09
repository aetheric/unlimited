
use nz::co::aetheric::util::kalman::Kalman;

#[allow(dead_code)]
pub struct UH {

	#[allow(dead_code)]
	kalman_x: Kalman,

	#[allow(dead_code)]
	kalman_y: Kalman,



	/* IMU Data */
	#[allow(dead_code)]
	acc_x: f64,

	#[allow(dead_code)]
	acc_y: f64,

	#[allow(dead_code)]
	acc_z: f64,


	#[allow(dead_code)]
	gyro_x: f64,

	#[allow(dead_code)]
	gyro_y: f64,

	#[allow(dead_code)]
	gyro_z: f64,



	#[allow(dead_code)]
	gyro_x_angle: f64,

	#[allow(dead_code)]
	gyro_y_angle: f64,

	#[allow(dead_code)]
	gyro_z_angle: f64, // Angle calculated using the gyro only


	#[allow(dead_code)]
	comp_angle_x: f64,

	#[allow(dead_code)]
	comp_angle_y: f64, // Calculated angle using a complementary filter


	#[allow(dead_code)]
	kal_angle_x: f64,

	#[allow(dead_code)]
	kal_angle_y: f64, // Calculated angle using a Kalman filter



	#[allow(dead_code)]
	timer: u32,


	#[allow(dead_code)]
	roll: f32,

	#[allow(dead_code)]
	pitch: f32,



	#[allow(dead_code)]
	current_ems_channel: i32,


	#[allow(dead_code)]
	ems_time_count: i32,


	#[allow(dead_code)]
	stimu_high_wid: i32,


	#[allow(dead_code)]
	stimu_low_wid: i32,


	#[allow(dead_code)]
	stimu_time_count: i32,


	#[allow(dead_code)]
	current_vol: i32

}
