#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>
#include <math.h>


#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/GIRBAL_anchor_distances.h>
//#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class GIRBAL_Sim_Driver : public ModuleBase<GIRBAL_Sim_Driver>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
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

    coordinates_gps anchor_nodes_gps[4]; // coords arrays to store our anchors (in both GPS and XYZ systems)
    coordinates_xyz anchor_nodes_xyz[4];

    int distances[4];

    GIRBAL_Sim_Driver(); // constructor

    ~GIRBAL_Sim_Driver() override; // destructor

    void publishDistances(int distances[]);

    void calculateDistances(coordinates_gps current_location, coordinates_xyz nodes[], int distances[]);

    coordinates_xyz gps2ecef(coordinates_gps gps);

    /** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

    bool init(); // unsure how this differs from the constructor

    int print_status() override;

private:
    void Run() override; // callback func

    // Publications
	uORB::Publication<GIRBAL_anchor_distances_s> _GIRBAL_anchor_distances_pub{ORB_ID(GIRBAL_anchor_distances)}; //

    // Subscriptions
    uORB::SubscriptionCallbackWorkItem _vehicle_gps_position_sub{this, ORB_ID(vehicle_gps_position)};        // subscription that schedules WorkItemExample when updated

    uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
    uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data

    // Performance (perf) counters
	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

    // Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

    bool _armed{false};
};
