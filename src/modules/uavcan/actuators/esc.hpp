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
 * @file esc.hpp
 *
 * UAVCAN <--> ORB bridge for ESC messages:
 *     uavcan.equipment.esc.RawCommand
 *     uavcan.equipment.esc.RPMCommand
 *     uavcan.equipment.esc.Status
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>

#define FOC_ESC_USED 1

#if FIRMWARENAME == ArduCopter
#define COPTER_USED 1
#endif

#ifdef FOC_ESC_USED
#define FOC_ESC_RPM_CTL_TYPE 1
#define PROPELLER_RELOCATE_ENABLE 1
#define ESC_RGBLED_ENABLE 1
#endif

#ifdef FOC_ESC_RPM_CTL_TYPE
#include <uavcan/equipment/esc/RPMCommand.hpp>
#else
#include <uavcan/equipment/esc/RawCommand.hpp>
#endif
#ifdef PROPELLER_RELOCATE_ENABLE
#include <uavcan/equipment/esc/PropRelocCommand.hpp>
#endif
#ifdef ESC_RGBLED_ENABLE
#include <uavcan/equipment/esc/VehicleRGB.hpp>
#endif
#include <uavcan/equipment/esc/Status.hpp>
#include <systemlib/perf_counter.h>
#include <uORB/topics/esc_status.h>


class UavcanEscController
{
public:
	UavcanEscController(uavcan::INode &node);
	~UavcanEscController();

	int init();

	void update_outputs(float *outputs, unsigned num_outputs);
	void update_rgb_status(uint8_t rgb_status);

	void arm_all_escs(bool arm);
	void arm_single_esc(int num, bool arm);

private:
	/**
	 * ESC status message reception will be reported via this callback.
	 */
	void esc_status_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status> &msg);

	/**
	 * ESC status will be published to ORB from this callback (fixed rate).
	 */
	void orb_pub_timer_cb(const uavcan::TimerEvent &event);


	// static constexpr unsigned MAX_RATE_HZ = 200;			///< XXX make this configurable
	static constexpr unsigned MAX_RATE_HZ = 30;			///< XXX make this configurable
	static constexpr unsigned ESC_STATUS_UPDATE_RATE_HZ = 10;
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY = 5;	///< 0..31, inclusive, 0 - highest, 31 - lowest

	typedef uavcan::MethodBinder<UavcanEscController *,
		void (UavcanEscController::*)(const uavcan::ReceivedDataStructure<uavcan::equipment::esc::Status>&)>
		StatusCbBinder;

	typedef uavcan::MethodBinder<UavcanEscController *, void (UavcanEscController::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;

	bool		_armed = false;
	esc_status_s	_esc_status = {};
	orb_advert_t	_esc_status_pub = nullptr;

	/*
	 * libuavcan related things
	 */
	uavcan::MonotonicTime							_prev_cmd_pub;   ///< rate limiting
	uavcan::INode								&_node;
	#ifdef FOC_ESC_RPM_CTL_TYPE
		uavcan::Publisher<uavcan::equipment::esc::RPMCommand>			_uavcan_pub_rpm_cmd;
	#else
		uavcan::Publisher<uavcan::equipment::esc::RawCommand>			_uavcan_pub_raw_cmd;
	#endif
	#ifdef PROPELLER_RELOCATE_ENABLE
		uavcan::Publisher<uavcan::equipment::esc::PropRelocCommand>			_uavcan_pub_prop_reloc_cmd;
	#endif
	#ifdef ESC_RGBLED_ENABLE
	uavcan::Publisher<uavcan::equipment::esc::VehicleRGB>			_uavcan_pub_rgb_status_cmd;
	#endif
	uavcan::Subscriber<uavcan::equipment::esc::Status, StatusCbBinder>	_uavcan_sub_status;
	uavcan::TimerEventForwarder<TimerCbBinder>				_orb_timer;

	/*
	 * ESC states
	 */
	uint32_t 			_armed_mask = 0;

	/*
	 * Perf counters
	 */
	perf_counter_t _perfcnt_invalid_input = perf_alloc(PC_COUNT, "uavcan_esc_invalid_input");
	perf_counter_t _perfcnt_scaling_error = perf_alloc(PC_COUNT, "uavcan_esc_scaling_error");
};
