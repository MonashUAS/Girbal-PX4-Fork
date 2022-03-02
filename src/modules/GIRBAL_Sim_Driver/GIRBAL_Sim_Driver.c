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

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/GIRBAL_anchor_distances.h>


__EXPORT int GIRBAL_Sim_Driver_main(int argc, char *argv[]);

// Instance variables
struct coordinates_gps // struct for passing 3D coords around the module
{
int lat;
int lon;
int alt;
};

struct coordinates_xyz // struct for passing 3D coords around the module
{
int x;
int y;
int z;
};

struct coordinates_gps anchor_nodes_gps[4]; // coords arrays to store our anchors (in both GPS and XYZ systems)
struct coordinates_xyz anchor_nodes_xyz[4];

double distances[4];

struct coordinates_xyz gps2ecef(struct coordinates_gps gps)
{
    // https://stackoverflow.com/questions/18759601/converting-lla-to-xyz
    // assuumes WSG-84 complicit implementation of jmavsim GPS
    double R = 6378137; // radius of the earth
    double f_inv = 298.257224; // magic (some sort of flattening ratio to translate 3D lat/lon onto a 2D plane)
    double f = 1.0 / f_inv;
    double f1 = 1.0 - f;
    //double e2 = 1.0 - f1 * f1;

    double cosLat = cos(gps.lat / 1e7 * M_PI / 180);
    double sinLat = sin(gps.lat / 1e7 * M_PI / 180);

    double cosLon = cos(gps.lon / 1e7 * M_PI / 180);
    double sinLon = sin(gps.lon / 1e7 * M_PI / 180);

    double c = 1 / sqrt(cosLat * cosLat + f1 * f1 * sinLat * sinLat);
    double s = f1 * f1 * c;

    struct coordinates_xyz xyzPos;

    xyzPos.x = ((R*c + gps.alt/1000) * cosLat * cosLon);
    xyzPos.y = ((R*c + gps.alt/1000) * cosLat * sinLon);
    xyzPos.z = ((R*s + gps.alt/1000) * sinLat);

    return xyzPos;
}

// NOTE: This function takes current location as GPS coords, and the node location as XYZ coords!
/*void calculateDistances(struct coordinates_gps current_location, struct coordinates_xyz nodes[], double *dist_in[4])
{
    for (int i = 0; i < 4; i++) //hardcoded for 4 base stations
    {
	struct coordinates_xyz currentLocXYZ;
        currentLocXYZ = gps2ecef(current_location);

        // formula for finding distance between 2 xyz coordinates
        *dist_in = sqrt(pow(currentLocXYZ.x - nodes[i].x, 2) + pow(currentLocXYZ.y - nodes[i].y, 2) + pow(currentLocXYZ.z - nodes[i].z, 2));
	dist_in++;
    }
    //return dist_in;
}*/

int GIRBAL_Sim_Driver_main(int argc, char *argv[])
{
	PX4_INFO("Hello Sky!");

	// hard coded coords are choosen from Irchelpark, Zurich as this is where jmavsim defaults to
	// as gps publisher uses int, coords are 1e7 greater than reality
	anchor_nodes_gps[0].lat = 473963450; // anchor0, bottom right
	anchor_nodes_gps[0].lon = 85452000;
	anchor_nodes_gps[0].alt = 490506;
	anchor_nodes_gps[1].lat = 473969840; // anchor1, bottom left
	anchor_nodes_gps[1].lon = 85413730;
	anchor_nodes_gps[1].alt = 490506;
	anchor_nodes_gps[2].lat = 473992560; // anchor2, top left
	anchor_nodes_gps[2].lon = 85427100;
	anchor_nodes_gps[2].alt = 490506;
	anchor_nodes_gps[3].lat = 473999640; // anchor3, top right
	anchor_nodes_gps[3].lon = 85474340;
	anchor_nodes_gps[3].alt = 490506;

	for (int i = 0; i < 4; i++) // convert GPS base stations to XYZ in the constructor (I could definitely pre-calculate this but I'm lazy)
	{
		anchor_nodes_xyz[i] = gps2ecef(anchor_nodes_gps[i]);
		distances[i] = 0;
	}

	/* subscribe to vehicle_acceleration topic */
	int sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_gps_position));
	/* limit the update rate to 5 Hz */
	orb_set_interval(sensor_sub_fd, 200);

	//advertise attitude topic
	struct GIRBAL_anchor_distances_s dist;
	memset(&dist, 0, sizeof(dist));
	orb_advert_t dist_pub = orb_advertise(ORB_ID(GIRBAL_anchor_distances), &dist);

	/* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = sensor_sub_fd,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};

	int error_counter = 0;

	for (int i = 0; i < 5; i++) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			PX4_ERR("Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("ERROR return value from poll(): %d", poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {
				/* obtained data for the first file descriptor */
				struct vehicle_gps_position_s accel;
				/* copy sensors raw data into local buffer */
				orb_copy(ORB_ID(vehicle_gps_position), sensor_sub_fd, &accel);
				PX4_INFO("Position:\t%8.4f\t%8.4f\t%8.4f",
					 (double)accel.lat,
					 (double)accel.lon,
					 (double)accel.alt);

				struct coordinates_gps currentPos;

				// put coords subscriber receives into struct
				currentPos.lat = accel.lat;
				currentPos.lon = accel.lon;
				currentPos.alt = accel.alt;

				// run calculateDistances on that struct
				// the func will edit the distances array with the calculated distances
				//calculateDistances(currentPos, anchor_nodes_xyz, &distances);
				struct coordinates_xyz currentLocXYZ;
				currentLocXYZ = gps2ecef(currentPos);
				for (int k = 0; k < 4; k++) //hardcoded for 4 base stations
				{
					// formula for finding distance between 2 xyz coordinates
					distances[k] = sqrt(pow(currentLocXYZ.x - anchor_nodes_xyz[k].x, 2) + pow(currentLocXYZ.y - anchor_nodes_xyz[k].y, 2) + pow(currentLocXYZ.z - anchor_nodes_xyz[k].z, 2));
				}


				dist.timestamp = hrt_absolute_time();
				for (int j = 0; j < 4; j++)
				{
					dist.anchor_id[j] = j;
					dist.anchor_pos_x[j] = anchor_nodes_xyz[j].x;
					dist.anchor_pos_y[j] = anchor_nodes_xyz[j].y;
					dist.anchor_pos_z[j] = anchor_nodes_xyz[j].z;
					dist.distance[j] = distances[j];
					PX4_INFO("GIRBAL message:\t%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f",
						(double)dist.anchor_id[j],
						(double)dist.anchor_pos_x[j],
						(double)dist.anchor_pos_y[j],
						(double)dist.anchor_pos_z[j],
						(double)dist.distance[j]);
				}
				orb_publish(ORB_ID(GIRBAL_anchor_distances), dist_pub, &dist);
			}

			/* there could be more file descriptors here, in the form like:
			 * if (fds[1..n].revents & POLLIN) {}
			 */
		}
	}

	PX4_INFO("exiting");

	return 0;
}
