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
 * @file flow.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 *
 */

#include "flow.hpp"
#include <systemlib/err.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/subsystem_info.h>

const char *const UavcanFlowBridge::NAME = "flow";

UavcanFlowBridge::UavcanFlowBridge(uavcan::INode &node) :
	_node(node),
	_sub_flow(node),
        _sensor_rotation(ROTATION_NONE),
	_report_pub(-1),
	last_packet_usec(0),
	flow_x_integral(0),
	flow_y_integral(0)
{
}

int UavcanFlowBridge::init()
{
	int res = _sub_flow.start(FlowCbBinder(this, &UavcanFlowBridge::flow_sub_cb));

	if (res < 0) {
		warnx("Flow sub failed %i", res);
		return res;
	}

        /* notify about state change */
        struct subsystem_info_s info = {
                true,
                true,
                true,
                SUBSYSTEM_TYPE_OPTICALFLOW
        };
        static orb_advert_t pub = -1;

        if (pub > 0) {
                orb_publish(ORB_ID(subsystem_info), pub, &info);

        } else {
                pub = orb_advertise(ORB_ID(subsystem_info), &info);
        }

	return res;
}

unsigned UavcanFlowBridge::get_num_redundant_channels() const
{
	return (_receiver_node_id < 0) ? 0 : 1;
}

void UavcanFlowBridge::print_status() const
{
	printf("RX errors: %d, receiver node id: ", _sub_flow.getFailureCount());

	if (_receiver_node_id < 0) {
		printf("N/A\n");

	} else {
		printf("%d -- (%g, %g) @ %lld\n", _receiver_node_id, flow_x_integral, flow_y_integral, last_packet_usec);
	}
}

void UavcanFlowBridge::flow_sub_cb(const uavcan::ReceivedDataStructure<threedr::equipment::flow::optical_flow::LegacyRawSample> &msg)
{
	// This bridge does not support redundant Flow Devices yet.
	if (_receiver_node_id < 0) {
		_receiver_node_id = msg.getSrcNodeID().get();
		warnx("Flow receiver node ID: %d", _receiver_node_id);

	} else {
		if (_receiver_node_id != msg.getSrcNodeID().get()) {
			return;  // This Flow device is the redundant one, ignore it.
		}
	}

	auto report = ::optical_flow_s();

	report.timestamp = msg.time.usec;
        report.pixel_flow_x_integral = static_cast<float>(msg.integral.pixel_flow_x_integral) / 10000.0f;//convert to radians
        report.pixel_flow_y_integral = static_cast<float>(msg.integral.pixel_flow_y_integral) / 10000.0f;//convert to radians
        report.frame_count_since_last_readout = msg.integral.frame_count_since_last_readout;
        report.ground_distance_m = static_cast<float>(msg.integral.ground_distance) / 1000.0f;//convert to meters
        report.quality = msg.integral.qual; //0:bad ; 255 max quality
        report.gyro_x_rate_integral = static_cast<float>(msg.integral.gyro_x_rate_integral) / 10000.0f; //convert to radians
        report.gyro_y_rate_integral = static_cast<float>(msg.integral.gyro_y_rate_integral) / 10000.0f; //convert to radians
        report.gyro_z_rate_integral = static_cast<float>(msg.integral.gyro_z_rate_integral) / 10000.0f; //convert to radians
        report.integration_timespan = msg.integral.integration_timespan; //microseconds
        report.time_since_last_sonar_update = msg.integral.distance_timestamp;//microseconds
        report.gyro_temperature = msg.integral.gyro_temperature;//Temperature * 100 in centi-degrees Celsius

        report.sensor_id = 0;

        /* rotate measurements according to parameter */
        float zeroval = 0.0f;
        rotate_3f(_sensor_rotation, report.pixel_flow_x_integral, report.pixel_flow_y_integral, zeroval);

	if (_report_pub > 0) {
		orb_publish(ORB_ID(optical_flow), _report_pub, &report);

	} else {
		_report_pub = orb_advertise(ORB_ID(optical_flow), &report);
	}

	// for print_status
	last_packet_usec = report.timestamp;
	flow_x_integral = report.pixel_flow_x_integral;
	flow_y_integral = report.pixel_flow_y_integral;
}
