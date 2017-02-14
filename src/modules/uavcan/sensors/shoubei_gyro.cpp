/*
 * shoubei_gyro.cpp
 *  Created on: Sep 28, 2016
 *      Author: sin
 */

#include "shoubei_gyro.hpp"

#include <systemlib/err.h>


const char *const UavcanGyroBridge::NAME = "sb_gyro";
static float x;
static float y;
static float z;
UavcanGyroBridge::UavcanGyroBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_gyro", "/dev/uavcan/gyro", GYRO_BASE_DEVICE_PATH, ORB_ID(shoubei_sensor_gyro)),
	_sub_gyro(node),
	_rotation(ROTATION_NONE),
	_gyro_filter_x(GYRO_DEFAULT_RATE,GYRO_DEFAULT_FILTER_FREQ),
	_gyro_filter_y(GYRO_DEFAULT_RATE,GYRO_DEFAULT_FILTER_FREQ),
	_gyro_filter_z(GYRO_DEFAULT_RATE,GYRO_DEFAULT_FILTER_FREQ)
{
	_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_SHOUBEI;     // <-- Why?

	// default scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

}

int UavcanGyroBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_gyro.start(GyroCbBinder(this, &UavcanGyroBridge::gyro_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanGyroBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct gyro_report *gyro_buf = reinterpret_cast<struct gyro_report *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct gyro_report);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _report.timestamp) {
		/* copy report */
		lock();
		*gyro_buf = _report;
		last_read = _report.timestamp;
		unlock();
		return sizeof(struct gyro_report);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}


void UavcanGyroBridge::gyro_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Gyro>
		&msg)
{
	lock();
	struct shoubei_sensor_gyro_s gyro;
	float raw = M_PI / 180.0f;
	float new_range = 450;
//	float new_range_scale_dps_digit = 20e-3f;

	gyro.timestamp = hrt_absolute_time();//current time

	_gyro_range_rad_s = new_range / 180.0f * 3.1415926f;
	_gyro_range_scale = (0.02f * raw);
	gyro.x_raw = msg.gyro[0];
	gyro.y_raw = msg.gyro[1];
	gyro.z_raw = msg.gyro[2];

	gyro.scaling = _gyro_range_scale;
	gyro.range_rad_s = _gyro_range_rad_s;

	//test filter
#if 1
	float xraw_f = (float)gyro.x_raw;
	float yraw_f = (float)gyro.y_raw;
	float zraw_f = (float)gyro.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float xin = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float yin = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float zin = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	gyro.x = _gyro_filter_x.apply(xin);
	gyro.y = _gyro_filter_y.apply(yin);
	gyro.z = _gyro_filter_z.apply(zin);

	math::Vector<3> gval(xin, yin, zin);
	math::Vector < 3 > gval_integrated;

	bool gyro_notify = _gyro_int.put(gyro.timestamp, gval, gval_integrated,
			gyro.integral_dt);

	gyro.x_integral = gval_integrated(0);
	gyro.y_integral = gval_integrated(1);
	gyro.z_integral = gval_integrated(2);

	//test ----> nsh
	x = gyro.x;
	y = gyro.y;
	z = gyro.z;
	//young_test
	if(gyro_notify){
		if (_shoubei_gyro_pub == NULL) {
			_shoubei_gyro_pub = orb_advertise(ORB_ID(shoubei_sensor_gyro), &gyro);
		} else {
			orb_publish(ORB_ID(shoubei_sensor_gyro), _shoubei_gyro_pub, &gyro);
		}
	}

#endif
	//test success
#if 0

	if(gyro.x_raw < 32767){
		gyro.x = (float)msg.gyro[0] * (450.0000f / 22500.0f);
	}else{
		gyro.x = -(float)(~(msg.gyro[0])+1) * (450.0000f / 22500.0f);
	}
	gyro.x = gyro.x * raw;

	if(gyro.y_raw  < 32767){
		gyro.y = (float)msg.gyro[1] * (450.0000f / 22500.0f);
	}else{
		gyro.y= -(float)(~(msg.gyro[1])+1) * (450.0000f / 22500.0f);
	}
	gyro.y = gyro.y * raw;

	if(gyro.z_raw  < 32767){
		gyro.z = (float)msg.gyro[2] * (450.0000f / 22500.0f);
	}else{
		gyro.z = -(float)(~(msg.gyro[2])+1) * (450.0000f / 22500.0f);
	}
	gyro.z = gyro.z * raw;


	float xin = gyro.x;
	float yin = gyro.y;
	float zin = gyro.z;

	math::Vector<3> gval(xin, yin, zin);
	math::Vector<3> gval_integrated;

	_gyro_int.put(gyro.timestamp, gval, gval_integrated, gyro.integral_dt);

	gyro.x_integral = gval_integrated(0);
	gyro.y_integral = gval_integrated(1);
	gyro.z_integral = gval_integrated(2);

    if (_shoubei_gyro_pub == NULL) {
    	_shoubei_gyro_pub = orb_advertise(ORB_ID(shoubei_sensor_gyro), &gyro);
    } else {
        orb_publish(ORB_ID(shoubei_sensor_gyro), _shoubei_gyro_pub, &gyro);
    }
#endif
	unlock();

}
void UavcanGyroBridge::print_status() const
{
	printf("gyro.x: %.2f\n", (double)x);
	printf("gyro.y: %.2f\n", (double)y);
	printf("gyro.z: %.2f\n", (double)z);
}




