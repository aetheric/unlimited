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
	Q_angle: f32,

	/** Process noise variance for the gyro bias */
	Q_bias: f32,

	/** Measurement noise variance - this is actually the variance of the measurement noise */
	R_measure: f32,


	/** The angle calculated by the Kalman filter - part of the 2x1 state vector */
	angle: f32,

	/** The gyro bias calculated by the Kalman filter - part of the 2x1 state vector */
	bias: f32,

	/** Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate */
	rate: f32,


	/** Error covariance matrix - This is a 2x2 matrix */
	P: [ [ f32; 2 ]; 2 ],

}

impl Kalman {

	pub fn new() -> Kalman {
		Kalman {

			/* We will set the variables like so, these can also be tuned by the user */
			Q_angle: 0.001f32,
			Q_bias: 0.003f32,
			R_measure: 0.03f32,

			angle: 0.0f32, // Reset the angle
			bias: 0.0f32, // Reset bias
			rate: 0.0f32,

			// Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
			P: [
				[ 0.0f32, 0.0f32 ],
				[ 0.0f32, 0.0f32 ],
			]

		}
	}

	/**
	 * KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	 * Modified by Kristian Lauszus
	 * See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
	 * The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
	 * @param newAngle degrees
	 * @param newRate degrees / second
	 * @param dt (delta time) seconds
	 */
	pub fn getAngle(&self, newAngle: f32, newRate: f32, dt: f32) -> f32 {

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		self.rate = newRate - self.bias;
		self.angle += dt * self.rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		self.P[0][0] += dt * ( dt * self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle);
		self.P[0][1] -= dt * self.P[1][1];
		self.P[1][0] -= dt * self.P[1][1];
		self.P[1][1] += self.Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		let S: f32 = self.P[0][0] + self.R_measure; // Estimate error

		/* Step 5 */
		let K: [ f32; 2 ] = [ // Kalman gain - This is a 2x1 vector
			self.P[0][0] / S,
			self.P[1][0] / S
		];

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		let y: f32 = newAngle - self.angle; // Angle difference

		/* Step 6 */
		self.angle += K[0] * y;
		self.bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		let P00_temp: f32 = self.P[0][0];
		let P01_temp: f32 = self.P[0][1];

		self.P[0][0] -= K[0] * P00_temp;
		self.P[0][1] -= K[0] * P01_temp;
		self.P[1][0] -= K[1] * P00_temp;
		self.P[1][1] -= K[1] * P01_temp;

		return self.angle;
	}

}
