#include "BNO055.hpp"

BNO055::BNO055(I2C_HandleTypeDef i2c_module,unsigned char device_address){
	i2c_module_ = i2c_module;
	device_address_ = device_address;
	uint8_t bno_mode_senddata[] = {0x3d,0x08};
	HAL_I2C_Master_Transmit(&i2c_module, device_address<<1, bno_mode_senddata, 2, 100);
}

QUATERNION BNO055::get_quaternion(){
	uint8_t bno_readquat_address = 0x20;
	uint8_t bno_receivedata[16];
	short quat[4];

	HAL_I2C_Master_Transmit(&i2c_module_, device_address_<<1, &bno_readquat_address, 1, 100);
	HAL_I2C_Master_Receive(&i2c_module_, device_address_<<1, bno_receivedata, 8, 100);
	quat[0] = bno_receivedata[1] << 8 | bno_receivedata[0];
	quat[1] = bno_receivedata[3] << 8 | bno_receivedata[2];
	quat[2] = bno_receivedata[5] << 8 | bno_receivedata[4];
	quat[3] = bno_receivedata[7] << 8 | bno_receivedata[6];

	QUATERNION q = { (float)quat[1]/16384.0,(float)quat[2]/16384.0,(float)quat[3]/16384.0,(float)quat[0]/16384.0 };
	return q;
}

EULAR BNO055::get_eular(){

	QUATERNION q = get_quaternion();
	EULAR e;
	// roll (x-axis rotation)
	double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
	double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
	e.x = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w * q.y - q.z * q.x);
	if (fabs(sinp) >= 1)
		e.y = copysign(3.1415926535 / 2, sinp); // use 90 degrees if out of range
	else
		e.y = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	e.z = atan2(siny_cosp, cosy_cosp);

	return e;
}

EULAR BNO055::get_accel(){
	uint8_t bno_readaccel_address = 0x08;
	uint8_t bno_receivedata[16];
	short e_raw[3];
	const float div = 100.0;

	HAL_I2C_Master_Transmit(&i2c_module_, device_address_<<1, &bno_readaccel_address, 1, 100);
	HAL_I2C_Master_Receive(&i2c_module_, device_address_<<1, bno_receivedata, 6, 100);
	e_raw[0] = bno_receivedata[1] << 8 | bno_receivedata[0];
	e_raw[1] = bno_receivedata[3] << 8 | bno_receivedata[2];
	e_raw[2] = bno_receivedata[5] << 8 | bno_receivedata[4];

	EULAR e = { (float)e_raw[0]/div,(float)e_raw[1]/div,(float)e_raw[2]/div };
	return e;
}

EULAR BNO055::get_gyro(){
	uint8_t bno_readgyro_address = 0x14;
	uint8_t bno_receivedata[16];
	short e_raw[3];
	const float div = 16.0;

	HAL_I2C_Master_Transmit(&i2c_module_, device_address_<<1, &bno_readgyro_address, 1, 100);
	HAL_I2C_Master_Receive(&i2c_module_, device_address_<<1, bno_receivedata, 6, 100);
	e_raw[0] = bno_receivedata[1] << 8 | bno_receivedata[0];
	e_raw[1] = bno_receivedata[3] << 8 | bno_receivedata[2];
	e_raw[2] = bno_receivedata[5] << 8 | bno_receivedata[4];

	EULAR e = { (float)e_raw[0]/div,(float)e_raw[1]/div,(float)e_raw[2]/div };
	return e;
}
