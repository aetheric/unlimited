/** */

use nz::co::aetheric::util::kalman::Kalman;
use nz::co::aetheric::util::point::Point2;
use nz::co::aetheric::util::point::Point3;
use nz::co::aetheric::unlimited::consts::*;

pub struct UH {

	/** ??? */
	kalman: Point2<Kalman>,


	/** IMU Data */
	acc: Point3<f64>,


	/** ??? */
	gyro: Point3<f64>,

	/** Angle calculated using the gyro only */
	gyro_angle: Point3<f64>,

	/** Calculated angle using a complementary filter */
	comp_angle: Point2<f64>,

	/** Calculated angle using a Kalman filter */
	kal_angle: Point2<f64>,


	/** ??? */
	timer: u32,


	/** ??? */
	roll: f32,

	/** ??? */
	pitch: f32,


	/** ??? */
	current_ems_channel: i32,

	/** ??? */
	ems_time_count: i32,

	/** ??? */
	stimu_high_wid: i32,

	/** ??? */
	stimu_low_wid: i32,

	/** ??? */
	stimu_time_count: i32,

	/** ??? */
	current_vol: i32

}

impl Default for UH {

	pub fn default() -> UH {
		return UH {
			// stuff
		}
	}

}


impl UH {

	///////////////////////////////////////////////////////////////////////////////
	// MPU6050 FUNCTIONS for the Acceleration and the Gyro                        //
	///////////////////////////////////////////////////////////////////////////////
	/**  Function: MPU6050_read        ***************************/
	pub fn MPU6050_read(&self, regAddr: u8, &buffer: u8, size: i32) -> i32 {

		let n1 = i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE);
		let n2 = i2c_write(regAddr);
		i2c_stop();

		/*
		Serial.println (regAddr, HEX);
		Serial.print (n1);
		Serial.print ("___rd___");
		Serial.println (n2);
		*/

		delay (50);
		let mut r: i32 = 0;
		while (r < (size - 1)) {
			i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
			buffer[r] = i2c_read (false);
			i2c_stop();
			r += 1;
		}

		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		buffer[r] = i2c_read (true);
		i2c_stop();

		return (0);  // return : no error

	}

	/**  Function: MPU6050_read_one    ***************************/
	pub fn MPU6050_read_one(&self, regAddr: u8) -> int {

		//read high byte at first address
		let n1: i32 = i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE);
		let n2: i32 = i2c_write(regAddr);
		i2c_stop();

		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		let oneByte: i32 = i2c_read(false); //read one byte
		i2c_stop();

		return oneByte;  // return byte value

	}

	/**  Function: MPU6050_read_two    ***************************/
	pub fn MPU6050_read_two(&self, regAddr: u8) -> i32 {
		let i: i32;
		let n: i32;
		let n1: i32;
		let n2: i32;
		let error: i32;
		let hByte: i32;
		let lByte: i32;
		let buffer: i32;

		//read high byte at first address
		n1 = i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE);
		n2 = i2c_write(regAddr);
		i2c_stop();

		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		hByte = i2c_read (false); //read high byte
		i2c_stop();
		regAddr += 1;

		//read low byte at next address
		n1 = i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE);
		n2 = i2c_write(regAddr);
		i2c_stop();

		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		lByte = i2c_read (true); //read low byte
		i2c_stop();

		hByte = hByte << 8;
		buffer = hByte | lByte;

		return buffer;  // return
	}

	/**  Function: MPU6050_write_one    ***************************/
	pub fn MPU6050_write_one(&self, regAddr: i32, pData: i32) -> int {

		//Serial.println ("start write");
		//Wire.beginTransmission(MPU6050_I2C_ADDRESS);
		if(!self.i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE)){
			return false;
		}

		// write the start address
		if(!self.i2c_write(regAddr)) {
			return false;
		}

		if (!self.i2c_write(pData)) {
			return false;
		}

		self.i2c_stop();
		return (3);

	}

	/**  Function: MPU6050_write_one    ***************************/
	pub fn setupAcceleGyro(&self) {

		let TWBR = ((F_CPU / 400000f64) - 16) / 2; // Set I2C frequency to 400kHz

		if (!i2c_init()) {
			//Serial.println(F("Initialization error. SDA or SCL are low"));
		} else {
			//Serial.println(F("...done"));
		}

		//while (!Serial);  //according to docs this holds until serial is open, this does not appear to work

		let c: u8;
		let error1: i32 = self.MPU6050_read(MPU6050_WHO_AM_I, &c, 1);
		//Serial.print(F("WHO_AM_I : "));
		//Serial.print(c,HEX);

		//Serial.print(F(", error = "));
		//Serial.println(error,DEC);

		// According to the datasheet, the 'sleep' bit
		// should read a '1'.
		// That bit has to be cleared, since the sensor
		// is in sleep mode at power-up.
		let error2 = self.MPU6050_read(MPU6050_PWR_MGMT_1, &c, 1);
		/*
		Serial.print(F("PWR_MGMT_1 : "));
		Serial.print(c,HEX);
		Serial.print(F(", error = "));
		Serial.println(error,DEC);
		*/

		//from multiwii  set up 6050
		self.delay(100);
		self.MPU6050_write_one(0x6B, B10000000);             //PWR_MGMT_1    -- DEVICE_RESET 1
		self.delay(100);
		self.MPU6050_write_one(0x6B, 0);             //PWR_MGMT_1    -
		self.delay(100);

		let error3 = self.MPU6050_read(MPU6050_PWR_MGMT_1, &c, 1);
		/*
		Serial.print(F("PWR_MGMT_1 : "));
		Serial.print(c,HEX);
		Serial.print(F(", error = "));
		Serial.println(error,DEC);
		*/
	}

	/**  Function: initAccelGyroKalman    ***************************/
	pub fn initAccelGyroKalman(&self, initTime: u32, accelRawValues: Point3<i32>) {

		accX = accelRawValues[0];
		accY = accelRawValues[1];
		accZ = accelRawValues[2];

		if (RESTRICT_PITCH) { // Eq. 25 and 26
			roll  = atan2(accY, accZ) * RAD_TO_DEG;
			pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
		} else { // Eq. 28 and 29
			roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
			pitch = atan2(-accX, accZ) * RAD_TO_DEG;
		}

		self.kalmanX.setAngle(roll); // Set starting angle
		self.kalmanY.setAngle(pitch);
		self.gyroXangle = roll;
		self.gyroYangle = pitch;
		self.compAngleX = roll;
		self.compAngleY = pitch;
		self.timer = initTime;

	}

	/**  Function: readAccelGyro_XY_Kalman    ***************************/
	pub fn readAccelGyro_XYZ_Kalman(&self, accelRawValues: Point3<i32>, gyroRawValues: Point3<i32>) {

		/* Update all the values */
		accX = accelRawValues[0];
		accY = accelRawValues[1];
		accZ = accelRawValues[2];
		gyroX = gyroRawValues[0];
		gyroY = gyroRawValues[1];
		gyroZ = gyroRawValues[2];

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
		// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
		// It is then converted from radians to degrees
		if (RESTRICT_PITCH) {// Eq. 25 and 26
			roll  = atan2(accY, accZ) * RAD_TO_DEG;//+++++++++++++++++++++++
			pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
		} else // Eq. 28 and 29
			roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
			pitch = atan2(-accX, accZ) * RAD_TO_DEG;
		}

		double gyroXrate = gyroX / 131.0; // Convert to deg/s
		double gyroYrate = gyroY / 131.0; // Convert to deg/s
		double gyroZrate = gyroZ / 131.0; // Convert to deg/s

		if (RESTRICT_PITCH) {
			// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
			if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
				kalmanX.setAngle(roll);
				compAngleX = roll;
				kalAngleX = roll;
				gyroXangle = roll;
			} else {
				kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
			}

			if (abs(kalAngleX) > 90)
				gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
				kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
			} else {
				// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
				if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
					kalmanY.setAngle(pitch);
					compAngleY = pitch;
					kalAngleY = pitch;
					gyroYangle = pitch;
				} else {
					kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
				}

				if (abs(kalAngleY) > 90)
				gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
				kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
			}

			gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
			gyroYangle += gyroYrate * dt;
			gyroZangle += gyroZrate * dt;
			//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
			//gyroYangle += kalmanY.getRate() * dt;

			compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
			compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

			// Reset the gyro angle when it has drifted too much
			if (gyroXangle < -180 || gyroXangle > 180)
			gyroXangle = kalAngleX;
			if (gyroYangle < -180 || gyroYangle > 180)
			gyroYangle = kalAngleY;

			angleXYZ[0] =(int)kalAngleX;
			angleXYZ[1] =(int)kalAngleY;
			angleXYZ[2] =(int)gyroZangle;
			delay(1);

	}

	/**  Function: checkAccelGyro_XY_Kalman    ***************************/
	pub fn checkAccelGyro_XYZ_Kalman(&self) {

		let accelRaw = readRawAccelValues();
		let gyroRaw = readRawGyroValues();
		let temp = readTemperature();
		let angXYZ = readAccelGyro_XYZ_Kalman(accelRaw, gyroRaw);

		Serial.print(angXYZ.x, DEC);//AcX =
		Serial.print(", ");

		Serial.print(angXYZ.y, DEC);// | AcY =
		Serial.print(", ");

		Serial.print(angXYZ.z, DEC);// | AcZ =
		Serial.print(", ");

		Serial.print(temp, 3);  //temperature
		Serial.print(", ");

		Serial.print(gyroRaw.x, DEC);
		Serial.print(", ");

		Serial.print(gyroRaw.y, DEC);
		Serial.print(", ");

		Serial.println(gyroRaw.z, DEC);

		delay(1);

		/*
		accelRaw[0] = (int)kalAngleX;
		accelRaw[1] =(int)kalAngleY;
		accelRaw[2] =(int)gyroZangle;

		Serial.print(accelRaw[0], DEC);//AcX =
		  Serial.print(", "); Serial.print(accelRaw[1], DEC);// | AcY =
		  Serial.print(", "); Serial.print(accelRaw[2], DEC);// | AcZ =
		  Serial.print(", "); Serial.print(tempt, 3);  //temperature
		  Serial.print(", "); Serial.print(gyroRaw[0], DEC);
		  Serial.print(", "); Serial.print(gyroRaw[1], DEC);
		  Serial.print(", "); Serial.println(gyroRaw[2], DEC);
		  delay(1);
		  */

	}

	/**  Function: readRawAccelValues    ***************************/
	pub fn readRawAccelValues(&self) -> Point3<i32> {
		return = Point3<i32> {
			x: self.MPU6050_read_two(MPU6050_ACCEL_XOUT_H),
			y: self.MPU6050_read_two(MPU6050_ACCEL_YOUT_H),
			z: self.MPU6050_read_two(MPU6050_ACCEL_ZOUT_H)
		};
	};

	/**  Function: checkRawAccelValues    ***************************/
	pub fn checkRawAccelValues(&self) {

		let AccelRawVal = self.readRawAccelValues();

		Serial.print(AccelRawVal.x, DEC);//AcX =
		Serial.print(", ");

		Serial.print(AccelRawVal.y, DEC);// | AcY =
		Serial.print(", ");

		Serial.print(AccelRawVal.z, DEC);// | AcZ =

	}

	/**  Function: readRawGyroValues    ***************************/
	pub fn readRawGyroValues(&self) -> Point3<i32> {
		return Point3<i32> {
			x: self.MPU6050_read_two(MPU6050_GYRO_XOUT_H),
			y: self.MPU6050_read_two(MPU6050_GYRO_YOUT_H),
			z: self.MPU6050_read_two(MPU6050_GYRO_ZOUT_H)
		};
	};

	/**  Function: checkRawGyroValues    ***************************/
	pub fn checkRawGyroValues(&self) {

		let GyroRawVal = self.readRawGyroValues();

		Serial.print(GyroRawVal.x, DEC);
		Serial.print(", ");

		Serial.print(GyroRawVal.y, DEC);
		Serial.print(", ");

		Serial.print(GyroRawVal.z, DEC);

	}

	/**  Function: readTemperature    ***************************/
	pub fn readTemperature(&self) -> f64 {
		let tempRaw = self.MPU6050_read_two(MPU6050_TEMP_OUT_H) as f64;
		return (tempRaw + 12412.0f64) / 340.0f64;
	}

	/**  Function: checkTemperature    ***************************/
	pub fn checkTemperature(&self) {
		Serial.print(", ");
		Serial.print(self.readTemperature(), 3);  //temperature
	};


	///////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS: VIBRATION MOTOR						                         //
	///////////////////////////////////////////////////////////////////////////////

	/**  Function: setupVibrationMotor    ***************************/
	pub fn setupVibrationMotor(&self) {
		self.pinMode(vibrationMotor_PIN, OUTPUT);
		self.digitalWrite(vibrationMotor_PIN, HIGH);
	}

	/**  Function: moveVibrationMotor    ***************************/
	pub fn moveVibrationMotor(&self, mTime: i32) {
		self.digitalWrite(vibrationMotor_PIN, LOW);
		self.delay(mTime);
		self.digitalWrite(vibrationMotor_PIN, HIGH);
	}

	/**  Function: onVibrationMotor    ***************************/
	pub fn onVibrationMotor(&self) {
		self.digitalWrite(vibrationMotor_PIN, LOW);
	}

	/**  Function: offVibrationMotor    ***************************/
	pub fn offVibrationMotor(&self) {
		self.digitalWrite(vibrationMotor_PIN, HIGH);
	}


	///////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS: Photo-reflectors for Muscle Motion Sensors					 //
	///////////////////////////////////////////////////////////////////////////////

	/**  Function: readPR    ***************************/
	pub fn readPR(&self, channel: i32) -> i32 {
		return analogRead(channel);
	}

	/**  Function: readPR    ***************************/
	pub fn readPR(&self, PRValues: [ i32, PR_CH_NUM ]) {
		for (i: i32 = 0; i < PR_CH_NUM; i++) {
			PRValues[i] = readPR(i);
			delay(1);
		}
	}

	/**  Function: checkPR    ***************************/
	pub fn checkPR(&self) {

		//Loop through and read all 8 values
		//Reports back Value at channel 6 is: 346
		for(i: i32 = 0; i < PR_CH_NUM; i ++) {
			Serial.print(String(readPR(i)));
			Serial.print(",");
			delay(1);
		}

		Serial.println("");

	}

	/**  Function: checkPR    ***************************/
	pub fn checkPR_test(&self) {

		//Loop through and read all 8 values
		//Reports back Value at channel 6 is: 346
		for (i: i32 = 0; i < PR_CH_NUM; i ++) {
			Serial.print(String(readPR(i)));
			Serial.print('\t');
			delay(1);
		}

		Serial.println("");

	}

	pub fn initPR(&self) {
		pinMode(PR_LED_PIN, OUTPUT);
		digitalWrite(PR_LED_PIN, LOW);
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS: Controller for the Multiplexer of the EMS(Electric Muscle Stimuli			 //
	///////////////////////////////////////////////////////////////////////////////////////////


	//10進数をマルチプレクサの2進数に変換し，マルチプレクサに信号を送る関数
	/**  Function: connectMUX    ***************************/
	pub fn connectMUX(&self, num: i32){

		//objects for the function:connectMUX
		let mut dec2bin: [ i32; 4 ] = [ 0, 0, 0, 0 ];
		for (i: i32 = 0; num > 0; i++) {
			dec2bin[i] = num % 2;
			num = num/2;
		}

		digitalWrite(EMS_S0_PIN, HIGH if dec2bin[0] == 1 else LOW);
		digitalWrite(EMS_S1_PIN, HIGH if dec2bin[1] == 1 else LOW);
		digitalWrite(EMS_S2_PIN, HIGH if dec2bin[2] == 1 else LOW);

	}

	/**  Function: keepVoltage    ***************************/
	/*pub fn keepVoltage(volNum: i32) {
		if (MIN_VOL <= volNum && volNum <= MAX_VOL) {
			stockVolTime: i32 = f[volNum];
			loopNum: i32 = 1500 / stockVolTime;
			for (i: i32 = 0; i < loopNum; i++) {
				digitalWrite(BOOSTER_SWITCH_PIN, HIGH);
				delayMicroseconds(stockVolTime);
				digitalWrite(BOOSTER_SWITCH_PIN, LOW);
				delayMicroseconds(stockVolTime);
			}
		}
	}*/

	/**  Function: keepVoltage    ***************************/
	pub fn keepVoltage(&self, volNum: i32) {
		if (MIN_VOL <= volNum && volNum <= MAX_VOL) {
			let stockVolTime: i32 = relation_V_Delay[volNum];
			//loopNum: i32 = (relation_V_Delay[0] * 2) / stockVolTime;
			let loopNum: i32 = 100 / stockVolTime;
			for (i: i32 = 0; i < loopNum; i++) {

				digitalWrite(BOOSTER_SWITCH_PIN, HIGH);
				delayMicroseconds(stockVolTime * 20);
				//delay(stockVolTime);

				digitalWrite(BOOSTER_SWITCH_PIN, LOW);
				delayMicroseconds(stockVolTime * 20);
				//delay(stockVolTime);

			}
		}
	}

	/**  Function: initEMS    ***************************/
	pub fn initEMS(&self) {

		//set up the EMS MUX pins
		pinMode(EMS_EN_PIN, OUTPUT);
		pinMode(EMS_S0_PIN, OUTPUT);
		pinMode(EMS_S1_PIN, OUTPUT);
		pinMode(EMS_S2_PIN, OUTPUT);

		//set up the EMS booster pin and objects
		pinMode(BOOSTER_SWITCH_PIN, OUTPUT);

		//set up the EMS Pattern
		stimuTimeCount = DEF_STIMU_TIME_COUNT;
		stimuHighWid = DEF_STIMU_HIGH_WID;
		stimuLowWid = DEF_STIMU_LOW_WID;

		//set up
		currentEMSChannel = -1;
		EMSTimeCount = -1;
		currentVol = 11;

	}

	/**  Function: setStimulationTime    ***************************/
	pub fn setStimulationTime(&self) -> UH {
		return UH { ems_time_count: self.stimu_time_count, .. *self }
	}

	/**  Function: setStimulationTime    ***************************/
	pub fn setStimulationTime(&self, time: i32) -> UH {
		return UH { ems_time_count: time, .. *self }
	}

	/**  Function: setStimulationChannel    ***************************/
	pub fn setStimulationChannel(&self, channel: i32) {
		return *self if ( 8 <= channel || channel < -1 )
			else UH { current_ems_channel: channel, .. *self };
	}

	pub fn setStimulationVoltage(&self, vol: i32){
		currentVol = vol;
	}

	pub fn updateEMS(&self) {
		if (0<=EMSTimeCount) {//刺激する電極がある場合
			connectMUX(currentEMSChannel);
			digitalWrite(EMS_EN_PIN, HIGH);//マルチプレクサの準備
			delayMicroseconds(200);//HIGHパルス
			digitalWrite(EMS_EN_PIN, LOW);//パルスを切る
			keepVoltage(currentVol);
			keepVoltage(currentVol);
			//delayMicroseconds(stimuLowWid);//LOWパルス
			EMSTimeCount--;
		} else {              //刺激する電極が無い場合
			currentEMSChannel = -1;
			setEMS_LOW();
		}
	}

	pub fn updateEMS_HiVoltage(&self) {
		if (0<=EMSTimeCount) {//刺激する電極がある場合
			connectMUX(currentEMSChannel);
			digitalWrite(EMS_EN_PIN, HIGH);//マルチプレクサの準備
			delayMicroseconds(20);//HIGHパルス
			digitalWrite(EMS_EN_PIN, LOW);//パルスを切る
			delayMicroseconds(160);//LOWパルス
			digitalWrite(EMS_EN_PIN, HIGH);//マルチプレクサの準備
			delayMicroseconds(20);//HIGHパルス
			keepVoltage(currentVol);
			keepVoltage(currentVol);
			//delayMicroseconds(stimuLowWid);//LOWパルス
			EMSTimeCount--;
		} else {              //刺激する電極が無い場合
			currentEMSChannel = -1;
			setEMS_LOW();
		}
	}

	pub fn setEMS_LOW(&self) {
		digitalWrite(BOOSTER_SWITCH_PIN, LOW);
		digitalWrite(EMS_EN_PIN, LOW);
		digitalWrite(EMS_S0_PIN, LOW);
		digitalWrite(EMS_S1_PIN, LOW);
		digitalWrite(EMS_S2_PIN, LOW);
	}

}
