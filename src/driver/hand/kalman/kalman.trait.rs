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
pub trait Kalman {

	// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
	pub fn getAngle(newAngle: f32, newRate: f32, dt: f32) -> f32;

	pub fn setAngle(angle: f32); // Used to set angle, this should be set as the starting angle
	pub fn getRate() -> f32; // Return the unbiased rate

	/* These are used to tune the Kalman filter */
	pub fn setQangle(Q_angle: f32);
	pub fn setQbias(Q_bias: f32);
	pub fn setRmeasure(R_measure: f32);

	pub fn getQangle() -> f32;
	pub fn getQbias() -> f32;
	pub fn getRmeasure() -> f32;

}
