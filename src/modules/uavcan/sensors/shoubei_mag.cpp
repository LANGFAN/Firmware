/*
 * shoubei_mag.cpp
 *
 *  Created on: Sep 28, 2016
 *      Author: sin
 */

#include "shoubei_mag.hpp"

#include <systemlib/err.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

const char *const UavcanMagBridge::NAME = "sb_mag";

static float x;
static float y;
static float z;
UavcanMagBridge::UavcanMagBridge(uavcan::INode &node) :
	UavcanCDevSensorBridgeBase("uavcan_mag", "/dev/uavcan/mag", MAG_BASE_DEVICE_PATH, ORB_ID(shoubei_sensor_mag)),
	_sub_mag(node),
	_rotation(ROTATION_NONE)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_SHOUBEI;     // <-- Why?

	_scale.x_scale = 1.0F;
	_scale.y_scale = 1.0F;
	_scale.z_scale = 1.0F;
	_scale.x_offset = 0.0F;
	_scale.y_offset = 0.0F;
	_scale.z_offset = 0.0F;
}

int UavcanMagBridge::init()
{
	int res = device::CDev::init();

	if (res < 0) {
		return res;
	}

	res = _sub_mag.start(MagCbBinder(this, &UavcanMagBridge::mag_sub_cb));

	if (res < 0) {
		DEVICE_LOG("failed to start uavcan sub: %d", res);
		return res;
	}

	return 0;
}

ssize_t UavcanMagBridge::read(struct file *filp, char *buffer, size_t buflen)
{
	static uint64_t last_read = 0;
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);

	/* buffer must be large enough */
	unsigned count = buflen / sizeof(struct mag_report);

	if (count < 1) {
		return -ENOSPC;
	}

	if (last_read < _report.timestamp) {
		/* copy report */
		lock();
		*mag_buf = _report;
		last_read = _report.timestamp;
		unlock();
		return sizeof(struct mag_report);

	} else {
		/* no new data available, warn caller */
		return -EAGAIN;
	}
}


void UavcanMagBridge::mag_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Mag>
		&msg)
{
	lock();
	struct shoubei_sensor_mag_s mag;

#if 1
	float _mag_range_scale = 3.27670000f / 32767.0f;

	mag.timestamp = hrt_absolute_time();//current time
	mag.scaling = _mag_range_scale;
	mag.range_ga = 3.2767F;

	int young_test1,young_test2,young_test3;
	young_test1  = msg.mag[0];
	young_test2  = msg.mag[1];
	young_test3  = msg.mag[2];


	if(young_test1 < 32767){
		mag.x = (float)msg.mag[0] * ((float)3.27670000 / 32767);
	}else{
		mag.x = -(float)(~(msg.mag[0])+1) * ((float)3.27670000 / 32767);
	}

	if(young_test2  < 32767){
		mag.y = (float)msg.mag[1] * ((float)3.27670000 / 32767);
	}else{
		mag.y = -(float)(~(msg.mag[1])+1) * ((float)3.27670000 / 32767);
	}

	if(young_test3  < 32767){
		mag.z = (float)msg.mag[2] * ((float)3.27670000 / 32767);
	}else{
		mag.z = -(float)(~(msg.mag[2])+1) * ((float)3.27670000 / 32767);
	}

#endif
	unlock();

	x = mag.x;
	y = mag.y;
	z = mag.z;

	if (_mag_pub == NULL) {
		_mag_pub = orb_advertise(ORB_ID(shoubei_sensor_mag), &mag);
	} else {
		orb_publish(ORB_ID(shoubei_sensor_mag), _mag_pub, &mag);
	}
}

void UavcanMagBridge::print_status() const
{
	int fd = ::open("/dev/uavcan/mag",O_RDONLY);
	ssize_t sz;
	struct shoubei_sensor_mag_s mag;

	if(fd < 0){
		printf("mag_fd: %d\n", fd);
	}
	printf("mag_fd: %d\n", fd);

	sz = ::read(fd, &mag, sizeof(mag));
	printf("mag_sz: %d\n", sz);

	printf("mag_x: %.8f\n", mag.x);
	printf("mag_y: %.8f\n", mag.y);
	printf("mag_z: %.8f\n", mag.z);
}








