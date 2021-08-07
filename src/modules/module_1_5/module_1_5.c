/****************************************************************************
 *
 *   Copyright (c) 2012-2019 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * Minimal application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>
#include <iostream>

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>

struct coords //input structure for coordinates from GPS data from sim
{
	float lat;
	float lon;
};

/* do we need a structure for distances or can it just be an array?
struct distances
{
	float distance_to_node
	float node_ID
};
*/

#define pi 3.14159265358979323846
float calculateDistances(struct current_location, struct nodes) // current location should be coords struct and nodes should be array of coords structs
{
	int num_nodes = sizeof(nodes)
	float dist[num_nodes]
	for (i = 0; i < num_nodes; i++)
	{
	dist[i] = sin(nodes[i].lat) * sin(current_location.lat) + cos(nodes[i].lat) * cos(current_location.lat) * cos(nodes[i].lat - current_location.lat);
	dist[i] = acos(dist);
	dist[i] = (6371 * pi * dist) / 180;
	} //check this formula - see haversine formula
	return dist
}

int main()
{
	struct coords[4] anchor_nodes; //anchor nodes coordinates structure

	//enter coordinates in apostrophes when found later - should be in switzerland?
	//node 1
	strcopy(anchor_nodes[1].lat, "");
	strcopy(anchor_nodes[1].lon, "");

	//node 2
	strcopy(anchor_nodes[2].lat, "");
	strcopy(anchor_nodes[2].lon, "");

	//node 3
	strcopy(anchor_nodes[3].lat, "");
	strcopy(anchor_nodes[3].lon, "");

	//node 4
	strcopy(anchor_nodes[4].lat, "");
	strcopy(anchor_nodes[4].lon, "");

	float distances[4]
}