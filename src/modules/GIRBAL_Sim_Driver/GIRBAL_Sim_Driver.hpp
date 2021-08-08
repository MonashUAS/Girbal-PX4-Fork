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
#include <uORB/topics/GIRBAL_anchor_distances.h>
//#include <uORB/topics/orb_test.h>
//#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_gps_position.h>
//#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class GIRBAL_Sim_Driver : public ModuleBase<GIRBAL_Sim_Driver>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    struct coordinates //input structure for coordinates from GPS data from sim
    {
        float lat;
        float lon;
    };
    typedef struct coordinates COORDS;

    GIRBAL_Sim_Driver(); // constructor

    ~GIRBAL_Sim_Driver() override; // destructor

    void calculateDistances(COORDS current_location, COORDS nodes[], float *distances[]); // current location should be coords struct and nodes should be array of coords structs

    bool init();

private:
    void Run() override;

    // Publications
	uORB::Publication<GIRBAL_anchor_distances_s> _anchor_distance_pub{ORB_ID(GIRBAL_anchor_distances)};

    // Subscriptions
    uORB::SubscriptionCallbackWorkItem sensor_gps_s{this, ORB_ID(vehicle_gps_position)};        // subscription that schedules WorkItemExample when updated
    //uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
    //uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

};
