/*
 * shoubei_gyro.hpp
 *
 *  Created on: Sep 28, 2016
 *      Author: sin
 */

#ifndef SHOUBEI_GYRO_HPP_
#define SHOUBEI_GYRO_HPP_
#pragma once

#include <math.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "sensor_bridge.hpp"
#include <drivers/drv_gyro.h>

#include <uavcan/equipment/imu/Gyro.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/shoubei_sensor_gyro.h>

#define GYRO_DEFAULT_RATE			760
#define GYRO_DEFAULT_FILTER_FREQ	30
#define GYRO_DPS 					(450.0000 / 22500)


class UavcanGyroBridge : public UavcanCDevSensorBridgeBase
{
public:
	static const char *const NAME;

	UavcanGyroBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

	void print_status() const override;

private:
	float			_gyro_range_scale;
	float 			_gyro_range_rad_s;

	struct gyro_calibration_s	_gyro_scale;
	enum Rotation		_rotation;


	orb_advert_t _shoubei_gyro_pub = NULL;
	ssize_t	read(struct file *filp, char *buffer, size_t buflen);
	void gyro_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Gyro> &msg);

	typedef uavcan::MethodBinder < UavcanGyroBridge *,
		void (UavcanGyroBridge::*)
		(const uavcan::ReceivedDataStructure<uavcan::equipment::imu::Gyro> &) >
		GyroCbBinder;

	uavcan::Subscriber<uavcan::equipment::imu::Gyro, GyroCbBinder> _sub_gyro;

	math::LowPassFilter2p	_gyro_filter_x;
	math::LowPassFilter2p	_gyro_filter_y;
	math::LowPassFilter2p	_gyro_filter_z;

	Integrator		_gyro_int;

	gyro_report _report =  {};
};

#endif /* SHOUBEI_GYRO_HPP_ */
