/*
 * shoubei_accel.hpp
 *
 *  Created on: Sep 28, 2016
 *      Author: sin
 */

#ifndef SHOUBEI_ACCEL_HPP_
#define SHOUBEI_ACCEL_HPP_

#include <math.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "sensor_bridge.hpp"
#include <drivers/drv_accel.h>

#include <uavcan/equipment/imu/Acc.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/shoubei_sensor_accel.h>

#define ACCEL_ONE_G						9.80665f
#define ACCEL_DEFAULT_RANGE_G			18
#define ACCEL_DPS                       (18.0000 / 22500)

class UavcanAccelBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanAccelBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

private:

	float			_accel_range_scale;
	float			 _accel_range_m_s2 ;

	struct accel_calibration_s	_accel_scale;
	enum Rotation		_rotation;

	uint8_t			_constant_accel_count;
	float			_last_accel[3];

	orb_advert_t _accel_pub = NULL;

	int	 accel_set_range(unsigned max_g);
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);


	void accel_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Acc> &msg);

	typedef uavcan::MethodBinder < UavcanAccelBridge *,
		void (UavcanAccelBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Acc> &) >
		AccelCbBinder;

	uavcan::Subscriber<uavcan::equipment::imu::Acc, AccelCbBinder> _sub_accel;

	math::LowPassFilter2p	_accel_filter_x;
	math::LowPassFilter2p	_accel_filter_y;
	math::LowPassFilter2p	_accel_filter_z;

	Integrator		_accel_int;

	accel_report _report =  {};
};





#endif /* SHOUBEI_ACCEL_HPP_ */
