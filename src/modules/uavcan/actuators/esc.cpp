/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file esc.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#include "esc.hpp"
#include <systemlib/err.h>


#define MOTOR_BIT(x) (1<<(x))
#ifdef FOC_ESC_RPM_CTL_TYPE
	#define RAD_TO_DEG(x) (x * 57.29577951308232087685)
	// just for compile test, and it will be deleteed later
	// -------------------------------------------
	// T-Motor U10 + KV170 / 30inch propeller
	// #define OMEGA_LINEAR_SCALE 250.99F
	// #define OMEGA_BASE 73.44F
	// ------------------------------------------
	// T-Motor MN5212 KV 340 / 18x60 propeller
	#define OMEGA_LINEAR_SCALE 528.97F
	// // it should be 528.97,for test, change it to be 200 temporily
	// #define OMEGA_LINEAR_SCALE 100.0F
	#define OMEGA_BASE 140.16F
	// ------------------------------------------
	#define PWM_TO_RPM_RPS(x) (x * OMEGA_LINEAR_SCALE + OMEGA_BASE)	// rotation speed unit: rad/s
#endif

UavcanEscController::UavcanEscController(uavcan::INode &node) :
	_node(node),
	#ifdef FOC_ESC_RPM_CTL_TYPE
		_uavcan_pub_rpm_cmd(node),
	#else
		_uavcan_pub_raw_cmd(node),
	#endif
	#ifdef PROPELLER_RELOCATE_ENABLE
		_uavcan_pub_prop_reloc_cmd(node),
	#endif
	#ifdef ESC_RGBLED_ENABLE
		_uavcan_pub_rgb_status_cmd(node),
	#endif
	_uavcan_sub_status(node),
	_orb_timer(node)
{
	#ifdef FOC_ESC_RPM_CTL_TYPE
		_uavcan_pub_rpm_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	#else
		_uavcan_pub_raw_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	#endif

	#ifdef PROPELLER_RELOCATE_ENABLE
		_uavcan_pub_prop_reloc_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	#endif

	#ifdef ESC_RGBLED_ENABLE
		_uavcan_pub_rgb_status_cmd.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	#endif

	if (_perfcnt_invalid_input == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_invalid_input");
	}

	if (_perfcnt_scaling_error == nullptr) {
		errx(1, "uavcan: couldn't allocate _perfcnt_scaling_error");
	}
}

UavcanEscController::~UavcanEscController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanEscController::init()
{
	// ESC status subscription
	int res = _uavcan_sub_status.start(StatusCbBinder(this, &UavcanEscController::esc_status_sub_cb));

	if (res < 0) {
		warnx("ESC status sub failed %i", res);
		return res;
	}

	// ESC status will be relayed from UAVCAN bus into ORB at this rate
	_orb_timer.setCallback(TimerCbBinder(this, &UavcanEscController::orb_pub_timer_cb));
	_orb_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / ESC_STATUS_UPDATE_RATE_HZ));

	return res;
}

void UavcanEscController::update_rgb_status(uint8_t rgb_status)
{
	uavcan::equipment::esc::VehicleRGB msg;
	msg.e_rgb = rgb_status;
	_uavcan_pub_rgb_status_cmd.broadcast(msg);

}

void UavcanEscController::update_outputs(float *outputs, unsigned num_outputs)
{
	if ((outputs == nullptr) ||
	    (
				#ifdef FOC_ESC_RPM_CTL_TYPE
					num_outputs > uavcan::equipment::esc::RPMCommand::FieldTypes::rpm::MaxSize
				#else
			num_outputs > uavcan::equipment::esc::RawCommand::FieldTypes::cmd::MaxSize
				#endif
			)	||
	    (num_outputs > esc_status_s::CONNECTED_ESC_MAX)) {
		perf_count(_perfcnt_invalid_input);
		return;
	}

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = _node.getMonotonicTime();

	if ((timestamp - _prev_cmd_pub).toUSec() < (1000000 / MAX_RATE_HZ)) {
		return;
	}

	_prev_cmd_pub = timestamp;

	/*
	 * Fill the command message
	 * If unarmed, we publish an empty message anyway
	 */
	// #ifdef FOC_ESC_RPM_CTL_TYPE
			uavcan::equipment::esc::RPMCommand msg;
			// static const uint16_t cmd_max = uavcan::equipment::esc::RPMCommand::FieldTypes::rpm::RawValueType::max();
	// #else
	// 	uavcan::equipment::esc::RawCommand msg;
	// 	static const int cmd_max = uavcan::equipment::esc::RawCommand::FieldTypes::cmd::RawValueType::max();
	// #endif

	#ifdef COPTER_USED
	// send 19 bytes totally
	/***
	AXIS1_LOWBYTE | AXIS1_HIGHBYTE | AXIS2_LOWBYTE | AXIS2_HIGHBYTE | AXIS3_LOWBYTE | AXIS3_HIGHBYTE | FLAG | TAILBYTE
	AXIS4_LOWBYTE | AXIS4_HIGHBYTE | AXIS5_LOWBYTE | AXIS5_HIGHBYTE | AXIS6_LOWBYTE | AXIS6_HIGHBYTE | FLAG | TAILBYTE
	AXIS7_LOWBYTE | AXIS7_HIGHBYTE | AXIS8_LOWBYTE | AXIS8_HIGHBYTE | FE | FE | FLAG | TAILBYTE
	***/
	// for (unsigned j = 0; j <= 18; j++) {
		// unsigned i = j / 2;
		// if(i < num_outputs) {

	for (unsigned i = 0; i < num_outputs; i++) {
	#else
	// plane
	for (unsigned i = 0; i < 1; i++) {
	#endif
	// #ifndef FOC_ESC_RPM_CTL_TYPE
	// 	if (_armed_mask & MOTOR_BIT(i)) {
	// 		float scaled = (outputs[i] + 1.0F) * 0.5F * cmd_max;
	//
	// 		// trim negative values back to 0. Previously
	// 		// we set this to 0.1, which meant motors kept
	// 		// spinning when armed, but that should be a
	// 		// policy decision for a specific vehicle
	// 		// type, as it is not appropriate for all
	// 		// types of vehicles (eg. fixed wing).
	// 		if (scaled < 0.0F) {
	// 			scaled = 0.0F;
	// 		}
	//
	// 		if (scaled > cmd_max) {
	// 			scaled = cmd_max;
	// 			perf_count(_perfcnt_scaling_error);
	// 		}
	//
	// 		msg.cmd.push_back(static_cast<int>(scaled));
	//
	// 		_esc_status.esc[i].esc_setpoint_raw = abs(static_cast<int>(scaled));
	// 	} else {
	// 		msg.cmd.push_back(static_cast<unsigned>(0));
	// 	}
	// #else
		if (_armed_mask & MOTOR_BIT(i)) {
		// for test without IMU connected
		// if (1) {
			/*
			 * convert pwm range [-1,1] to FOC ESC RPM CMD
			 * rpm_cmd = ((outputs[i] + 1.0F) * 0.5F * C_R + OMEGA_BASE) * RAD_TO_DEG / 360 * 60; UNIT: r/min
			 * rpm_cmd = ((outputs[i] + 1.0F) * 0.5F * C_R + OMEGA_BASE) * 10; UNIT: r/min
			 * output --> actuator_outputs_s.output, it's [-1,1]
			 * C_R --> linear scale factor of throttle percent to rotation speed; UNIT: rad/s
			 * OMEGA_BASE --> linar distance of y axis when 0 point; UNIT: rad/s
			*/
			// RPM CMD, motor stable state rotation speed
			uint16_t rpm_cmd = 0;
			uint8_t rpm_cmd_low_byte = 0;
			uint8_t rpm_cmd_high_byte = 0;

			#ifdef COPTER_USED
				// copter
				float scaled = (outputs[i] + 1.0F) * 0.5F;
			#else
				// plane only throttle need be sent by uavcan
				float scaled = (outputs[2] + 1.0F) * 0.5F;
			#endif
				if(scaled <= 0.1F) {
					scaled = 0.0F;
					rpm_cmd = 0;
				}

				if(scaled >= 1.0F) {
					scaled = 1.0F;
				}

				if(scaled >= 0.1F && scaled <= 1.0F) {
					float rpm_dps = RAD_TO_DEG(PWM_TO_RPM_RPS(scaled));
					rpm_cmd = static_cast<uint16_t>(rpm_dps / 6.0F);
				}

				// For test
				// rpm_cmd = 1031;
				// if(rpm_cmd > 2000) {
				// 	rpm_cmd = 2000;
				// }

				if (rpm_cmd > 8000) {
					rpm_cmd = 8000;
					perf_count(_perfcnt_scaling_error);
				}

				rpm_cmd_low_byte = (rpm_cmd & 0xFF);
				rpm_cmd_high_byte = (rpm_cmd >> 8);

				#ifdef COPTER_USED
					// copter
					msg.rpm.push_back(rpm_cmd_low_byte);
					msg.rpm.push_back(rpm_cmd_high_byte);

					if (1 == i || 4 == i) {
						msg.rpm.push_back(0xFE);
					}

				#else
					// plane
					msg.rpm.push_back(rpm_cmd_low_byte);
					msg.rpm.push_back(rpm_cmd_high_byte);

					msg.rpm.push_back(rpm_cmd_low_byte);
					msg.rpm.push_back(rpm_cmd_high_byte);
				#endif

				_esc_status.esc[i].esc_setpoint_raw = rpm_cmd;

				// for test
				// printf("motor speed(rpm):%u\n", rpm_cmd);
		} else {
			#ifdef COPTER_USED
			// copter
			msg.rpm.push_back(static_cast<uint8_t>(0));
			msg.rpm.push_back(static_cast<uint8_t>(0));

			if (1 == i || 4 == i) {
				msg.rpm.push_back(0xFE);
			}

			#else
			// plane
			msg.rpm.push_back(static_cast<uint8_t>(0));
			msg.rpm.push_back(static_cast<uint8_t>(0));
			msg.rpm.push_back(static_cast<uint8_t>(0));
			msg.rpm.push_back(static_cast<uint8_t>(0));
			#endif
		} // armed
		// #ifdef COPTER_USED
		// 	j++;
		// 	continue;
		// } // if < num_outputs

		// msg.rpm.push_back(0xFE);
		// #endif
	// #endif
	} // FOR LOOP

	#ifdef COPTER_USED
	unsigned rest_byte_num = 0;
	if (4 == num_outputs) {
		rest_byte_num = num_outputs * 2 + 1;
	} else {
		rest_byte_num = num_outputs * 2 + 2;
	}

	for (unsigned j = 0; j <= 18 - rest_byte_num; j++) {
		msg.rpm.push_back(0xFE);
	}
	#endif
	/*
	 * Publish the command message to the bus
	 * Note that for a quadrotor it takes one CAN frame
	 */
	//  #ifdef FOC_ESC_RPM_CTL_TYPE
		(void)_uavcan_pub_rpm_cmd.broadcast(msg);
	//  #else
	// 	(void)_uavcan_pub_raw_cmd.broadcast(msg);
	//  #endif
}

void UavcanEscController::arm_all_escs(bool arm)
{
	if (arm) {
		_armed_mask = -1;

	} else {
		_armed_mask = 0;
	}
}

void UavcanEscController::arm_single_esc(int num, bool arm)
{
	if (arm) {
		_armed_mask = MOTOR_BIT(num);

	} else {
		_armed_mask = 0;
	}
}

void UavcanEscController::esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg)
{
	if (msg.esc_index < esc_status_s::CONNECTED_ESC_MAX) {
		_esc_status.esc_count = uavcan::max<int>(_esc_status.esc_count, msg.esc_index + 1);
		_esc_status.timestamp = msg.getMonotonicTimestamp().toUSec();

		auto &ref = _esc_status.esc[msg.esc_index];

		ref.esc_address = msg.getSrcNodeID().get();

		ref.esc_voltage     = msg.voltage;
		ref.esc_current     = msg.current;
		ref.esc_temperature = msg.temperature;
		ref.esc_setpoint    = msg.power_rating_pct;
		ref.esc_rpm         = msg.rpm;
		ref.esc_errorcount  = msg.error_count;
	}
}

void UavcanEscController::orb_pub_timer_cb(const uavcan::TimerEvent &)
{
	_esc_status.counter += 1;
	_esc_status.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_CAN;

	if (_esc_status_pub != nullptr) {
		(void)orb_publish(ORB_ID(esc_status), _esc_status_pub, &_esc_status);

	} else {
		_esc_status_pub = orb_advertise(ORB_ID(esc_status), &_esc_status);
	}
}
