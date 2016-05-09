/** */

use nz::co::aetheric::util::kalman::Kalman;
use nz::co::aetheric::util::point::Point2;
use nz::co::aetheric::util::point::Point3;

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


impl UH {

	pub fn new() -> UH {
		UH {
			// stuff
		}
	}

	///////////////////////////////////////////////////////////////////////////////
	// MPU6050 FUNCTIONS for the Acceleration and the Gyro                        //
	///////////////////////////////////////////////////////////////////////////////
	/**  Function: MPU6050_read        ***************************/
	pub fn MPU6050_read(regAddr: u8, &buffer: u8, size: i32) -> i32 {

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
		let r: i32 = 0;
		while (r < (size - 1)) {
			i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
			buffer[r] = i2c_read (false);
			i2c_stop();
			r++;
		}
		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		buffer[r] = i2c_read (true);
		i2c_stop();
		return (0);  // return : no error
	};

	/**  Function: MPU6050_read_one    ***************************/
	pub fn MPU6050_read_one(uint8_t regAddr) -> int {

		//read high byte at first address
		let n1: i32 = i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE);
		let n2: i32 = i2c_write(regAddr);
		i2c_stop();

		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		let oneByte: i32 = i2c_read(false); //read one byte
		i2c_stop();

		return oneByte;  // return byte value

	};

	/**  Function: MPU6050_read_two    ***************************/
	pub fn MPU6050_read_two(uint8_t regAddr) -> int {
		i: i32, n, n1, n2, error, hByte, lByte, buffer;

		//read high byte at first address
		n1 = i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE);
		n2 = i2c_write(regAddr);
		i2c_stop();

		i2c_start(MPU6050_I2C_ADDRESS | I2C_READ);
		hByte = i2c_read (false); //read high byte
		i2c_stop();
		regAddr++;

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
	};

	/**  Function: MPU6050_write_one    ***************************/
	pub fn MPU6050_write_one(regAddr: i32, pData: i32) -> int {
		i: i32;
		n: i32;
		n1: i32;
		n2: i32
		error: i32;

		//Serial.println ("start write");
		//Wire.beginTransmission(MPU6050_I2C_ADDRESS);
		if(!i2c_start(MPU6050_I2C_ADDRESS | I2C_WRITE)){
			return false;
		}

		// write the start address
		if(!i2c_write(regAddr)){
			return false;
		}

		if (!i2c_write(pData)){
			return false;
		}

		i2c_stop();
		return (3);
	};

	/**  Function: MPU6050_write_one    ***************************/
	pub fn setupAcceleGyro(){
		TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
		if (!i2c_init()) {
			//Serial.println(F("Initialization error. SDA or SCL are low"));
		} else {
			//Serial.println(F("...done"));
		}

		while (!Serial);  //according to docs this holds until serial is open, this does not appear to work

		uint8_t c;
		error: i32 = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
		//Serial.print(F("WHO_AM_I : "));
		//Serial.print(c,HEX);

		//Serial.print(F(", error = "));
		//Serial.println(error,DEC);

		// According to the datasheet, the 'sleep' bit
		// should read a '1'.
		// That bit has to be cleared, since the sensor
		// is in sleep mode at power-up.
		error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
		/*
		Serial.print(F("PWR_MGMT_1 : "));
		Serial.print(c,HEX);
		Serial.print(F(", error = "));
		Serial.println(error,DEC);
		*/

		//from multiwii  set up 6050
		delay(100);
		MPU6050_write_one(0x6B, B10000000);             //PWR_MGMT_1    -- DEVICE_RESET 1
		delay(100);
		MPU6050_write_one(0x6B, 0);             //PWR_MGMT_1    -
		delay(100);

		error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
		/*
		Serial.print(F("PWR_MGMT_1 : "));
		Serial.print(c,HEX);
		Serial.print(F(", error = "));
		Serial.println(error,DEC);
		*/
	};

	/**  Function: initAccelGyroKalman    ***************************/
	pub fn initAccelGyroKalman(uint32_t initTime, accelRawValues: i32[]){
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

		kalmanX.setAngle(roll); // Set starting angle
		kalmanY.setAngle(pitch);
		gyroXangle = roll;
		gyroYangle = pitch;
		compAngleX = roll;
		compAngleY = pitch;
		timer = initTime;
	}

	/**  Function: readAccelGyro_XY_Kalman    ***************************/
	pub fn readAccelGyro_XYZ_Kalman(accelRawValues: i32[], gyroRawValues: i32[], angleXYZ: i32[]){
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
	pub fn checkAccelGyro_XYZ_Kalman(){
		accelRaw: [ i32, 3 ];
		readRawAccelValues(accelRaw);
		gyroRaw: [ i32, 3 ];
		readRawGyroValues(gyroRaw);
		double temp = readTemperature();
		angXYZ: [ i32, 3 ];
		readAccelGyro_XYZ_Kalman(accelRaw, gyroRaw, angXYZ);


		Serial.print(angXYZ[0], DEC);//AcX =
		Serial.print(", "); Serial.print(angXYZ[1], DEC);// | AcY =
		Serial.print(", "); Serial.print(angXYZ[2], DEC);// | AcZ =
		Serial.print(", "); Serial.print(temp, 3);  //temperature
		Serial.print(", "); Serial.print(gyroRaw[0], DEC);
		Serial.print(", "); Serial.print(gyroRaw[1], DEC);
		Serial.print(", "); Serial.println(gyroRaw[2], DEC);
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

	/**  Function: checkAccelGyro_XY_Kalman_test    ***************************/
	pub fn checkAccelGyro_XYZ_Kalman_test(){
		accelRaw: [ i32, 3 ];
		readRawAccelValues(accelRaw);
		gyroRaw: [ i32, 3 ];
		readRawGyroValues(gyroRaw);
		double temp = readTemperature();
		angXYZ: [ i32, 3 ];
		readAccelGyro_XYZ_Kalman(accelRaw, gyroRaw, angXYZ);


		Serial.print(angXYZ[0], DEC);//AcX =
		Serial.print('\t'); Serial.print(angXYZ[1], DEC);// | AcY =
		Serial.print('\t'); Serial.print(angXYZ[2], DEC);// | AcZ =
		Serial.print('\t'); Serial.print(temp, 3);  //temperature
		Serial.print('\t'); Serial.print(gyroRaw[0], DEC);
		Serial.print('\t'); Serial.print(gyroRaw[1], DEC);
		Serial.print('\t'); Serial.println(gyroRaw[2], DEC);
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
	pub fn readRawAccelValues(AccelRawValue: i32[]){
		AccelRawValue[0]= MPU6050_read_two (MPU6050_ACCEL_XOUT_H);
		AccelRawValue[1]= MPU6050_read_two (MPU6050_ACCEL_YOUT_H);
		AccelRawValue[2]= MPU6050_read_two (MPU6050_ACCEL_ZOUT_H);
	};

	/**  Function: checkRawAccelValues    ***************************/
	pub fn checkRawAccelValues(){
		AccelRawVal: [ i32, 3 ];
		AccelRawVal[0]= MPU6050_read_two (MPU6050_ACCEL_XOUT_H);
		AccelRawVal[1]= MPU6050_read_two (MPU6050_ACCEL_YOUT_H);
		AccelRawVal[2]= MPU6050_read_two (MPU6050_ACCEL_ZOUT_H);
		Serial.print(AccelRawVal[0], DEC);//AcX =
		Serial.print(", "); Serial.print(AccelRawVal[1], DEC);// | AcY =
		Serial.print(", "); Serial.print(AccelRawVal[2], DEC);// | AcZ =
	}

	/**  Function: readRawGyroValues    ***************************/
	pub fn readRawGyroValues(GyroRawValue: i32[]){
		GyroRawValue[0]= MPU6050_read_two (MPU6050_GYRO_XOUT_H);
		GyroRawValue[1]= MPU6050_read_two (MPU6050_GYRO_YOUT_H);
		GyroRawValue[2]= MPU6050_read_two (MPU6050_GYRO_ZOUT_H);
	};

	/**  Function: checkRawGyroValues    ***************************/
	pub fn checkRawGyroValues(){
		GyroRawVal: [ i32, 3 ];
		GyroRawVal[0]= MPU6050_read_two (MPU6050_GYRO_XOUT_H);
		GyroRawVal[1]= MPU6050_read_two (MPU6050_GYRO_YOUT_H);
		GyroRawVal[2]= MPU6050_read_two (MPU6050_GYRO_ZOUT_H);
		Serial.print(", "); Serial.print(GyroRawVal[0], DEC);
		Serial.print(", "); Serial.print(GyroRawVal[1], DEC);
		Serial.print(", "); Serial.print(GyroRawVal[2], DEC);
	};

	/**  Function: readTemperature    ***************************/
	double readTemperature(){
		tempRaw: i32 = MPU6050_read_two (MPU6050_TEMP_OUT_H);
		double calcTemp = ((double) tempRaw + 12412.0) / 340.0;
		return calcTemp;
	};

	/**  Function: checkTemperature    ***************************/
	pub fn checkTemperature(){
		tempRaw: i32 = MPU6050_read_two (MPU6050_TEMP_OUT_H);
		double calcTemp = ((double) tempRaw + 12412.0) / 340.0;
		Serial.print(", "); Serial.print(calcTemp, 3);  //temperature
	};


	///////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS: VIBRATION MOTOR						                         //
	///////////////////////////////////////////////////////////////////////////////

	/**  Function: setupVibrationMotor    ***************************/
	pub fn setupVibrationMotor(){
		pinMode(vibrationMotor_PIN, OUTPUT);
		digitalWrite(vibrationMotor_PIN, HIGH);
	}

	/**  Function: moveVibrationMotor    ***************************/
	pub fn moveVibrationMotor(mTime: i32){
		digitalWrite(vibrationMotor_PIN, LOW);
		delay(mTime);
		digitalWrite(vibrationMotor_PIN, HIGH);
	}

	/**  Function: onVibrationMotor    ***************************/
	pub fn onVibrationMotor(){
		digitalWrite(vibrationMotor_PIN, LOW);
	}

	/**  Function: offVibrationMotor    ***************************/
	pub fn offVibrationMotor(){
		digitalWrite(vibrationMotor_PIN, HIGH);
	}


	///////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS: Photo-reflectors for Muscle Motion Sensors					 //
	///////////////////////////////////////////////////////////////////////////////

	/**  Function: readPR    ***************************/
	readPR: i32(channel: i32){
		return analogRead(channel);
	}

	/**  Function: readPR    ***************************/
	pub fn readPR(PRValues: i32[]){
		for(i: i32 = 0; i < PR_CH_NUM; i ++){
			PRValues[i] = readPR(i);
			delay(1);
		}
	}

	/**  Function: checkPR    ***************************/
	pub fn checkPR(){
		//Loop through and read all 8 values
		//Reports back Value at channel 6 is: 346
		for(i: i32 = 0; i < PR_CH_NUM; i ++){
			Serial.print(String(readPR(i)));
			Serial.print(",");
			delay(1);
		}
		Serial.println("");
	}

	/**  Function: checkPR    ***************************/
	pub fn checkPR_test(){
		//Loop through and read all 8 values
		//Reports back Value at channel 6 is: 346
		for(i: i32 = 0; i < PR_CH_NUM; i ++){
			Serial.print(String(readPR(i)));
			Serial.print('\t');
			delay(1);
		}
		Serial.println("");
	}

	pub fn initPR(){
		pinMode(PR_LED_PIN, OUTPUT);
		digitalWrite(PR_LED_PIN, LOW);
	}

	///////////////////////////////////////////////////////////////////////////////////////////
	// FUNCTIONS: Controller for the Multiplexer of the EMS(Electric Muscle Stimuli			 //
	///////////////////////////////////////////////////////////////////////////////////////////


	//10進数をマルチプレクサの2進数に変換し，マルチプレクサに信号を送る関数
	/**  Function: connectMUX    ***************************/
	pub fn connectMUX(num: i32){
		dec2bin: i32[] = {0, 0, 0, 0};//objects for the function:connectMUX
		for(i: i32=0;num>0;i++){
			dec2bin[i] = num%2;
			num = num/2;
		}

		if(dec2bin[0]==1){
			digitalWrite(EMS_S0_PIN, HIGH);
		} else {
			digitalWrite(EMS_S0_PIN, LOW);
		}

		if(dec2bin[1]==1){
			digitalWrite(EMS_S1_PIN, HIGH);
		} else {
			digitalWrite(EMS_S1_PIN, LOW);
		}

		if(dec2bin[2]==1){
			digitalWrite(EMS_S2_PIN, HIGH);
		} else {
			digitalWrite(EMS_S2_PIN, LOW);
		}

	}

	/**  Function: keepVoltage    ***************************/
	/*void keepVoltage(volNum: i32) {
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
	pub fn keepVoltage(volNum: i32) {
		if (MIN_VOL <= volNum && volNum <= MAX_VOL) {
			stockVolTime: i32 = relation_V_Delay[volNum];
			//loopNum: i32 = (relation_V_Delay[0] * 2) / stockVolTime;
			loopNum: i32 = 100 / stockVolTime;
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
	pub fn initEMS(){
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
	pub fn setStimulationTime(){
		EMSTimeCount = stimuTimeCount;
	}

	/**  Function: setStimulationTime    ***************************/
	pub fn setStimulationTime(t: i32){
		EMSTimeCount = t;
	}

	/**  Function: setStimulationChannel    ***************************/
	pub fn setStimulationChannel(channel: i32){
		if(-1 <= channel && channel < 8){
			currentEMSChannel = channel;
		}
	}

	pub fn setStimulationVoltage(vol: i32){
		currentVol = vol;
	}

	pub fn updateEMS() {
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

	pub fn updateEMS_HiVoltage() {
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

	pub fn setEMS_LOW(){
		digitalWrite(BOOSTER_SWITCH_PIN, LOW);
		digitalWrite(EMS_EN_PIN, LOW);
		digitalWrite(EMS_S0_PIN, LOW);
		digitalWrite(EMS_S1_PIN, LOW);
		digitalWrite(EMS_S2_PIN, LOW);
	}

}
