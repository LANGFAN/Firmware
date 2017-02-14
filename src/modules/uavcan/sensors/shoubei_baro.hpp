#pragma once

#include "sensor_bridge.hpp"
#include <drivers/drv_baro.h>
#include <drivers/device/ringbuffer.h>

#include <uavcan/equipment/imu/Baro.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/shoubei_sensor_baro.h>


class RingBuffer;

class UavcanBaroBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanBaroBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:
	ssize_t read(struct file *filp, char *buffer, size_t buflen);
	int ioctl(struct file *filp, int cmd, unsigned long arg) override;

	void air_pressure_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Baro> &msg);

	typedef uavcan::MethodBinder < UavcanBaroBridge *,
		void (UavcanBaroBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Baro> &) >
		AirPressureCbBinder;

	uavcan::Subscriber<uavcan::equipment::imu::Baro, AirPressureCbBinder> _sub_air_pressure_data;

	ringbuffer::RingBuffer _reports;

	unsigned _msl_pressure = 101325;
	float last_temperature_kelvin = 0.0f;

	orb_advert_t _baro_pub = NULL;

};
