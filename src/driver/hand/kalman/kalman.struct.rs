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
	mut Q_angle: f32,

	/** Process noise variance for the gyro bias */
	mut Q_bias: f32,

	/** Measurement noise variance - this is actually the variance of the measurement noise */
	mut R_measure: f32,


	/** The angle calculated by the Kalman filter - part of the 2x1 state vector */
	mut angle: f32,

	/** The gyro bias calculated by the Kalman filter - part of the 2x1 state vector */
	mut bias: f32,

	/** Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate */
	mut rate: f32,

	/** Error covariance matrix - This is a 2x2 matrix */
	P: [ [ mut f32, 2 ], 2 ],

}
