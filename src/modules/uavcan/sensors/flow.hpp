/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file flow.hpp
 *
 * UAVCAN --> ORB bridge for Legacy Flow messages:
 *     threedr.equipment.flow.optical_flow.LegacyRawSample
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 *
 */

#pragma once

#include <uORB/uORB.h>
#include <uORB/topics/optical_flow.h>

#include <uavcan/uavcan.hpp>
#include <threedr/equipment/flow/optical_flow/LegacyRawSample.hpp>
#include <conversion/rotation.h>


#include "sensor_bridge.hpp"

class UavcanFlowBridge : public IUavcanSensorBridge
{
public:
	static const char *const NAME;

	UavcanFlowBridge(uavcan::INode &node);

	const char *get_name() const override { return NAME; }

	int init() override;

	unsigned get_num_redundant_channels() const override;

	void print_status() const override;

private:
	/**
	 * Flow message will be reported via this callback.
	 */
	void flow_sub_cb(const uavcan::ReceivedDataStructure<threedr::equipment::flow::optical_flow::LegacyRawSample> &msg);

	typedef uavcan::MethodBinder < UavcanFlowBridge *,
		void (UavcanFlowBridge::*)(const uavcan::ReceivedDataStructure<threedr::equipment::flow::optical_flow::LegacyRawSample> &) >
		FlowCbBinder;

	uavcan::INode &_node;
	uavcan::Subscriber<threedr::equipment::flow::optical_flow::LegacyRawSample, FlowCbBinder> _sub_flow;
	int _receiver_node_id = -1;

	enum Rotation _sensor_rotation;

	orb_advert_t _report_pub;                ///< uORB pub for Flow data
	uint64_t last_packet_usec;
	float flow_x_integral, flow_y_integral;
};
