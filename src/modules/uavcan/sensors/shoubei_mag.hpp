/*
 * shoubei_mag.hpp
 *
 *  Created on: Sep 28, 2016
 *      Author: sin
 */

#ifndef SHOUBEI_MAG_HPP_
#define SHOUBEI_MAG_HPP_
#pragma once
#include <math.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "sensor_bridge.hpp"
#include <drivers/drv_mag.h>

#include <sys/time.h>
#include <time.h>

#include <uavcan/equipment/imu/Mag.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/shoubei_sensor_mag.h>

class UavcanMagBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanMagBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);

	void print_status() const override;

private:
	orb_advert_t _mag_pub = NULL;

	enum Rotation		_rotation;

	void mag_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Mag> &msg);

	typedef uavcan::MethodBinder < UavcanMagBridge *,
		void (UavcanMagBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Mag> &) >
		MagCbBinder;

	uavcan::Subscriber<uavcan::equipment::imu::Mag, MagCbBinder> _sub_mag;
	struct mag_calibration_s _scale = {};

	mag_report _report =  {};
};





#endif /* SHOUBEI_MAG_HPP_ */
