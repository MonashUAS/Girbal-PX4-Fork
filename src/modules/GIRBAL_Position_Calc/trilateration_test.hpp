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

using namespace time_literals;

class GIRBAL_Position_Calc : public ModuleBase<GIRBAL_Position_Calc>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    // Instance variables

    // definitions
    struct coordinates // struct for passing 3D coords
    {
        double x;
        double y;
        double z;
    };
    typedef struct coordinates COORDS;

    struct coordinates_dist // struct for passing 3D coords with distance
    {
        COORDS coords;
        double radius; // i.e. distance
    };
    typedef struct coordinates_dist COORDS_DIST;

    COORDS polygonCalcCentre(COORDS[] vertices);

    COORDS calculateIntersection(double x1, double y1, double r1, double x2, double y2, double r2);

    COORDS* calculateIntersections(COORDS_DIST[] circles);
    COORDS trilateration(&anchor_coords,&anchor_distances )
}



