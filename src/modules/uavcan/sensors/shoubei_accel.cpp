/*
 * shoubei_accel.cpp
 *
 *  Created on: Sep 28, 2016
 *      Author: sin
 */
#include "shoubei_accel.hpp"

#include <systemlib/err.h>

const char *const UavcanAccelBridge::NAME = "sb_accel";
static int _count = 0;
UavcanAccelBridge::UavcanAccelBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_accel", "/dev/uavcan/accel", ACCEL_BASE_DEVICE_PATH, ORB_ID(shoubei_sensor_accel)),
	_sub_accel(node),
	_rotation(ROTATION_NONE),
	_accel_filter_x(800, 30),
	_accel_filter_y(800, 30),
	_accel_filter_z(800, 30)
{
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_SHOUBEI;     // <-- Why?

	// default scale factors
	_accel_scale.x_offset = 0.0f;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0.0f;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0.0f;
	_accel_scale.z_scale  = 1.0f;

	_count = 0;
}

int UavcanAccelBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_accel.start(AccelCbBinder(this, &UavcanAccelBridge::accel_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanAccelBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct accel_report *accel_buf = reinterpret_cast<struct accel_report *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct accel_report);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _report.timestamp) {
		/* copy report */
		lock();
		*accel_buf = _report;
		last_read = _report.timestamp;
		unlock();
		return sizeof(struct accel_report);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}

int UavcanAccelBridge::accel_set_range(unsigned max_g)
{
	float new_scale_g_digit = 0.0f;

	if (max_g == 0) {
		max_g = 16;
	}

	if (max_g <= 2) {
		_accel_range_m_s2 = 2.0f * ACCEL_ONE_G;
		new_scale_g_digit = 0.061e-3f;

	} else if (max_g <= 4) {
		_accel_range_m_s2 = 4.0f * ACCEL_ONE_G;
		new_scale_g_digit = 0.122e-3f;

	} else if (max_g <= 6) {
		_accel_range_m_s2 = 6.0f * ACCEL_ONE_G;
		new_scale_g_digit = 0.183e-3f;

	} else if (max_g <= 8) {
		_accel_range_m_s2 = 8.0f * ACCEL_ONE_G;
		new_scale_g_digit = 0.244e-3f;

	} else if (max_g <= 16) {
		_accel_range_m_s2 = 16.0f * ACCEL_ONE_G;
		new_scale_g_digit = 0.732e-3f;

	}else if(max_g <= 18){
		_accel_range_m_s2 = 18.0f * ACCEL_ONE_G;
		new_scale_g_digit = 0.800e-3f;
	}else {
		return -EINVAL;
	}

	_accel_range_scale = new_scale_g_digit * ACCEL_ONE_G;

	return OK;
}

void UavcanAccelBridge::accel_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Acc>
		&msg)
{
	struct shoubei_sensor_accel_s accel;
	accel.timestamp = hrt_absolute_time();//current time

	accel.scaling = (18.0f / 22500.0f) * 9.8066f;
	accel.range_m_s2 = 18.0f * 9.8066f;

	_accel_range_scale = (18.0f / 22500.0f) * 9.8066f;

	accel.x_raw = msg.acc[0];
	accel.y_raw = msg.acc[1];
	accel.z_raw = msg.acc[2];

#if 1
	float xraw_f = (float)accel.x_raw;
	float yraw_f = (float)accel.y_raw;
	float zraw_f = (float)accel.z_raw;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;


	/*
	  we have logs where the accelerometers get stuck at a fixed
	  large value. We want to detect this and mark the sensor as
	  being faulty
	 */
	if (fabsf(_last_accel[0] - x_in_new) < 0.001f &&
	    fabsf(_last_accel[1] - y_in_new) < 0.001f &&
	    fabsf(_last_accel[2] - z_in_new) < 0.001f &&
	    fabsf(x_in_new) > 20 &&
	    fabsf(y_in_new) > 20 &&
	    fabsf(z_in_new) > 20) {
		_constant_accel_count += 1;

	} else {
		_constant_accel_count = 0;
	}

	if (_constant_accel_count > 100) {
		// we've had 100 constant accel readings with large
		// values. The sensor is almost certainly dead. We
		// will raise the error_count so that the top level
		// flight code will know to avoid this sensor, but
		// we'll still give the data so that it can be logged
		// and viewed

		_constant_accel_count = 0;
	}

	_last_accel[0] = x_in_new;
	_last_accel[1] = y_in_new;
	_last_accel[2] = z_in_new;

	accel.x = _accel_filter_x.apply(x_in_new);
	accel.y = _accel_filter_y.apply(y_in_new);
	accel.z = _accel_filter_z.apply(z_in_new);

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	bool accel_notify = _accel_int.put(accel.timestamp, aval, aval_integrated, accel.integral_dt);
	accel.x_integral = aval_integrated(0);
	accel.y_integral = aval_integrated(1);
	accel.z_integral = aval_integrated(2);

	if(accel_notify){
		if (_accel_pub == NULL) {
			_accel_pub = orb_advertise(ORB_ID(shoubei_sensor_accel), &accel);
		} else {
			orb_publish(ORB_ID(shoubei_sensor_accel), _accel_pub, &accel);
		}
	}

#endif

	//test success
#if 0

	if(accel.x_raw  < 32767){
		accel.x= (float)msg.acc[0] * (18.0000f / 22500.0f);
	}else{
		accel.x = -(float)(~(msg.acc[0])+1) * (18.0000f / 22500.0f);
	}
	accel.x = accel.x * 9.80665f;

	if(accel.y_raw < 32767){
		accel.y = (float)msg.acc[1] * (18.0000f / 22500.0f);
	}else{
		accel.y = -(float)(~(msg.acc[1])+1) * (18.0000f / 22500.0f);
	}
	accel.y = accel.y* 9.80665f;

	if(accel.z_raw < 32767){
		accel.z = (float)msg.acc[2] * (18.0000f / 22500.0f);
	}else{
		accel.z = -(float)(~(msg.acc[2])+1) * (18.0000f / 22500.0f);
	}
	accel.z = accel.z * 9.80665f;

	_count++;
	if(_count > 20){
		_count = 0;
		if (_accel_pub == NULL) {
			_accel_pub = orb_advertise(ORB_ID(shoubei_sensor_accel), &accel);
		} else {
			orb_publish(ORB_ID(shoubei_sensor_accel), _accel_pub, &accel);
		}
	}
#endif

}

