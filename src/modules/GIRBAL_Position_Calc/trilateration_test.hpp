#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/GIRBAL_anchor_distances.h> // these are auto generated when the program is built (based on our msg definitions)
#include <uORB/topics/GIRBAL_vehicle_pos.h>
//#include <uORB/topics/orb_test.h>
//#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/vehicle_gps_position.h>
//#include <uORB/topics/vehicle_status.h>

#include <cmath>
#include <iostream>

using namespace time_literals;
using namespace std;


class GIRBAL_Position_Calc : public ModuleBase<GIRBAL_Position_Calc>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    // Instance variables

    // definitions
    struct COORDS {
        double x = 0;
        double y = 0;
        double z = 0;
        double r = 0;
    };

    double dotProduct(COORDS* A, COORDS* B);                    // returns a number
    void crossProduct(COORDS* A, COORDS* B, COORDS* P)          // returns a vector with structure COORDS
    COORDS trilateration(COORDS* P1, COORDS* P2, COORDS* P3)    // performs trilateration using x,y,z coordinates of 3 nodes, and the respective distances from each node to the drone
                                                                // calls dotProduct and crossProduct functions
}



