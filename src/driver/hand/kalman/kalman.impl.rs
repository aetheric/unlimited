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
pub impl Kalman {

	pub fn new() -> Kalman {
		Kalman {

			/* We will set the variables like so, these can also be tuned by the user */
			Q_angle = 0.001f;
			Q_bias = 0.003f;
			R_measure = 0.03f;

			angle = 0.0f; // Reset the angle
			bias = 0.0f; // Reset bias

			P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
			P[0][1] = 0.0f;
			P[1][0] = 0.0f;
			P[1][1] = 0.0f;

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
	pub fn getAngle(newAngle: f32, newRate: f32, dt: f32) -> f32 {

		// Discrete Kalman filter time update equations - Time Update ("Predict")
		// Update xhat - Project the state ahead
		/* Step 1 */
		rate = newRate - bias;
		angle += dt * rate;

		// Update estimation error covariance - Project the error covariance ahead
		/* Step 2 */
		P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;

		// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
		// Calculate Kalman gain - Compute the Kalman gain
		/* Step 4 */
		S: f32 = P[0][0] + R_measure; // Estimate error

		/* Step 5 */
		let K: [f32,2] = [ // Kalman gain - This is a 2x1 vector
			P[0][0] / S,
			P[1][0] / S
		];

		// Calculate angle and bias - Update estimate with measurement zk (newAngle)
		/* Step 3 */
		let y: f32 = newAngle - angle; // Angle difference

		/* Step 6 */
		angle += K[0] * y;
		bias += K[1] * y;

		// Calculate estimation error covariance - Update the error covariance
		/* Step 7 */
		let P00_temp: f32 = P[0][0];
		let P01_temp: f32 = P[0][1];

		P[0][0] -= K[0] * P00_temp;
		P[0][1] -= K[0] * P01_temp;
		P[1][0] -= K[1] * P00_temp;
		P[1][1] -= K[1] * P01_temp;

		return angle;
	}

	/**
	 * Used to set angle, this should be set as the starting angle
	 * @param angle
	 */
	pub fn setAngle(angle: f32) {
		this.angle = angle;
	}

	/**
	 * @return The unbiased rate
	 */
	pub fn getRate() -> f32 {
		return this.rate;
	}

	/**
	 * These are used to tune the Kalman filter
	 * @param Q_angle
	 */
	pub fn setQangle(Q_angle: f32) {
		this.Q_angle = Q_angle;
	};

	/**
	 * These are used to tune the Kalman filter
	 * @param Q_bias
	 */
	pub fn setQbias(Q_bias: f32) {
		this.Q_bias = Q_bias;
	};

	/**
	 * These are used to tune the Kalman filter
	 * @param R_measure
	 */
	pub fn setRmeasure(R_measure: f32) {
		this.R_measure = R_measure;
	};

	pub fn getQangle() -> f32 {
		return this.Q_angle;
	};

	pub fn getQbias() -> f32 {
		return this.Q_bias;
	};

	pub fn getRmeasure() -> f32 {
		return this.R_measure;
	};

}
