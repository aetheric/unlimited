/**
 * Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.
 *
 * This software may be distributed and modified under the terms of the GNU
 * General Public License version 2 (GPL2) as published by the Free Software
 * Foundation and appearing in the file GPL2.TXT included in the packaging of
 * this file. Please note that GPL2 Section 2[b] requires that all works based
 * on this software must also be made publicly available under the terms of
 * the GPL2 ("Copyleft").
 *
 * Contact information
 * -------------------
 * Kristian Lauszus, TKJ Electronics
 * Web      :  http://www.tkjelectronics.com
 * e-mail   :  kristianl@tkjelectronics.com
 */

pub struct Kalman {

	/** Process noise variance for the accelerometer */
	pub q_angle: f32,

	/** Process noise variance for the gyro bias */
	pub q_bias: f32,

	/** Measurement noise variance - this is actually the variance of the measurement noise */
	pub r_measure: f32,


	/** The angle calculated by the Kalman filter - part of the 2x1 state vector */
	pub angle: f32,

	/** The gyro bias calculated by the Kalman filter - part of the 2x1 state vector */
	pub bias: f32,

	/** Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate */
	#[allow(dead_code)]
	pub rate: f32,


	/** Error covariance matrix - This is a 2x2 matrix */
	pub p: [ [ f32; 2 ]; 2 ],

}

impl Default for Kalman {

	#[allow(dead_code)]
	fn default() -> Kalman {
		Kalman {

			/* We will set the variables like so, these can also be tuned by the user */
			q_angle: 0.001f32,
			q_bias: 0.003f32,
			r_measure: 0.03f32,

			angle: 0.0f32, // Reset the angle
			bias: 0.0f32, // Reset bias
			rate: 0.0f32,

			// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
			p: [
				[ 0.0f32, 0.0f32 ],
				[ 0.0f32, 0.0f32 ],
			]

		}
	}

}

impl Kalman {

	/**
	 * KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	 * Modified by Kristian Lauszus
	 * See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
	 * The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
	 * @param angleUpdate degrees
	 * @param rateUpdate degrees / second
	 * @param dt (delta time) seconds
	 */
	#[allow(dead_code)]
	pub fn update(&self, angle_update: f32, rate_update: f32, dt: f32) -> Kalman {

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		let new_rate = rate_update - self.bias;
		let new_angle = self.angle + dt * new_rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		let new_p = [
			[
				self.p[0][0] + dt * ( dt * self.p[1][1] - self.p[0][1] - self.p[1][0] + self.q_angle),
				self.p[0][1] - dt * self.p[1][1]
			],
			[
				self.p[1][0] - dt * self.p[1][1],
				self.p[1][1] + self.q_bias * dt
			]
		];

		// Calculate angle and bias - Update estimate with measurement zk (angleUpdate)
		/* Step 3 */
		let y = angle_update - new_angle; // Angle difference

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		let s = new_p[0][0] + self.r_measure; // Estimate error

		/* Step 5 */
		let k = [ // Kalman gain - This is a 2x1 vector
			new_p[0][0] / s,
			new_p[1][0] / s
		];

		return Kalman {

			rate: new_rate,

			/* Step 6 */
			angle: new_angle + k[0] * y,
			bias: self.bias + k[1] * y,

			// Calculate estimation error covariance - Update the error covariance
			/* Step 7 */
			p: [
				[
					new_p[0][0] - k[0] * new_p[0][0],
					new_p[0][1] - k[0] * new_p[0][1]
				],
				[
					new_p[1][0] - k[1] * new_p[0][0],
					new_p[1][1] - k[1] * new_p[0][1]
				]
			],

			.. *self

		};
	}

	#[allow(dead_code)]
	pub fn angle(&self, update: f32) -> Kalman {
		return Kalman { angle: update, .. *self };
	}

	#[allow(dead_code)]
	pub fn q_angle(&self, update: f32) -> Kalman {
		return Kalman { q_angle: update, .. *self };
	}

	#[allow(dead_code)]
	pub fn q_bias(&self, update: f32) -> Kalman {
		return Kalman { q_bias: update, .. *self };
	}

	#[allow(dead_code)]
	pub fn r_measure(&self, update: f32) -> Kalman {
		return Kalman { r_measure: update, .. *self };
	}

}
