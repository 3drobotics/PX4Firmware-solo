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
 * @file range.cpp
 *
 * @author Pavel Kirienko <pavel.kirienko@gmail.com>
 * @author Andrew Chambers <achamber@gmail.com>
 * @author David Sidrane <david_s5@nscdg.com>
 *
 */

#include "range.hpp"
#include <systemlib/err.h>
#include <mathlib/mathlib.h>
#include <uORB/topics/subsystem_info.h>

const char *const UavcanRangeBridge::NAME = "range";

#define PX4FLOW_MAX_DISTANCE 5.0f
#define PX4FLOW_MIN_DISTANCE 0.3f

UavcanRangeBridge::UavcanRangeBridge(uavcan::INode &node) :
	_node(node),
	_sub_range(node),
	_report_pub(-1),
	last_packet_usec(0),
	distance_cm(0)
{
}

int UavcanRangeBridge::init()
{
	int res = _sub_range.start(RangeCbBinder(this, &UavcanRangeBridge::range_sub_cb));

	if (res < 0) {
		warnx("Range sub failed %i", res);
		return res;
	}
        /* notify about state change */
        struct subsystem_info_s info = {
                true,
                true,
                true,
                SUBSYSTEM_TYPE_RANGEFINDER
        };
        static orb_advert_t pub = -1;

        if (pub > 0) {
                orb_publish(ORB_ID(subsystem_info), pub, &info);

        } else {
                pub = orb_advertise(ORB_ID(subsystem_info), &info);
        }
	return res;
}

unsigned UavcanRangeBridge::get_num_redundant_channels() const
{
	return (_receiver_node_id < 0) ? 0 : 1;
}

void UavcanRangeBridge::print_status() const
{
	printf("RX errors: %d, receiver node id: ", _sub_range.getFailureCount());

	if (_receiver_node_id < 0) {
		printf("N/A\n");

	} else {
		printf("%d -- (%g) @ %lld\n", _receiver_node_id, distance_cm, last_packet_usec);
	}
}

void UavcanRangeBridge::range_sub_cb(const uavcan::ReceivedDataStructure<uavcan::equipment::range_sensor::Measurement> &msg)
{
	// This bridge does not support redundant Range Devices yet.
	if (_receiver_node_id < 0) {
		_receiver_node_id = msg.getSrcNodeID().get();
		warnx("Range receiver node ID: %d", _receiver_node_id);

	} else {
		if (_receiver_node_id != msg.getSrcNodeID().get()) {
			return;  // This Range device is the redundant one, ignore it.
		}
	}

	auto report = ::distance_sensor_s();

	report.timestamp = msg.timestamp.usec;
        report.min_distance = PX4FLOW_MIN_DISTANCE;
        report.max_distance = PX4FLOW_MAX_DISTANCE;
        report.current_distance = msg.range;
        report.covariance = 0.0f;
        report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
        report.id = 0;
        report.orientation = 8;

	if (_report_pub > 0) {
		orb_publish(ORB_ID(distance_sensor), _report_pub, &report);

	} else {
		_report_pub = orb_advertise(ORB_ID(distance_sensor), &report);
	}

	// for print_status
	last_packet_usec = report.timestamp;
	distance_cm = report.current_distance;
}
