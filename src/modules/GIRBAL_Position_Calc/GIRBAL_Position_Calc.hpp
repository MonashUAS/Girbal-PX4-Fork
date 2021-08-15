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
    struct coordinates_xyz // struct for passing 3D coords around the module
    {
        int x;
        int y;
        int z;
    };
    typedef struct coordinates_xyz xyzCOORDS;


    GIRBAL_Position_Calc(); // constructor

    GIRBAL_Position_Calc() override; // destructor

    bool init(); // unsure how this differs from the constructor

private:
    void Run() override; // callback func

    // Publications
	uORB::Publication<GIRBAL_vehicle_pos_s> _anchor_distance_pub{ORB_ID(GIRBAL_vehicle_pos)}; //

    // Subscriptions
    uORB::SubscriptionCallbackWorkItem GIRBAL_anchor_distances_s{this, ORB_ID(GIRBAL_anchor_distances)};        // subscription that schedules WorkItemExample when updated
    //uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
    //uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

};
