#include "GIRBAL_Sim_Driver.hpp"

GIRBAL_Sim_Driver::GIRBAL_Sim_Driver() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
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
}

GIRBAL_Sim_Driver::~GIRBAL_Sim_Driver()
{
    perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool GIRBAL_Sim_Driver::init()
{
    // execute Run() on every gps publication
    if (!_vehicle_gps_position_sub.registerCallback()) {
        PX4_ERR("GPS data callback registration failed");
        return false;
    }

    // alternatively, Run on fixed interval
    //ScheduleOnInterval(1000000_us); // 1Hz // 5000 us interval, 200 Hz rate

    return true;
}

void GIRBAL_Sim_Driver::Run()
{
    PX4_INFO("RUN FUNCTION ENTERED");

    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

    	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}


	// Example
	//  update vehicle_status to check arming state
	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {

			const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

			if (armed && !_armed) {
				PX4_WARN("vehicle armed due to %d", vehicle_status.latest_arming_reason);

			} else if (!armed && _armed) {
				PX4_INFO("vehicle disarmed due to %d", vehicle_status.latest_disarming_reason);
			}

			_armed = armed;
		}
	}

    // "work" happens here on GPS callback
    if (_vehicle_gps_position_sub.updated()) {
        PX4_INFO("ENTERED FIRST LOOP");

        struct vehicle_gps_position_s gpsPtr;//accel in simple app
        struct coordinates_gps currentPos;

		if (_vehicle_gps_position_sub.copy(&gpsPtr)) {
            PX4_INFO("ENTERED SECOND LOOP");

			// put coords subscriber receives into struct
            currentPos.lat = gpsPtr.lat;
            currentPos.lon = gpsPtr.lon;
            currentPos.alt = gpsPtr.alt;

            // run calculateDistances on that struct
            // the func will edit the distances array with the calculated distances
            calculateDistances(currentPos, anchor_nodes_xyz, distances);
            // broadcast distances over publisher
            // ** must ensure that ALL node distances are published when this func is called, either by publishing 4 separate messages, or converting the message definition
            // to use arrays and publishing the lot in one message **
            publishDistances(distances);

		}
	}
    perf_end(_loop_perf);
}

void GIRBAL_Sim_Driver::publishDistances(int dist_in[])
{
    struct GIRBAL_anchor_distances_s dist; //from msg definition
    PX4_INFO("ENTERED PUBLISH FUNCTION");

	memset(&dist, 0, sizeof(dist));
	orb_advert_t dist_pub = orb_advertise(ORB_ID(GIRBAL_anchor_distances), &dist);

    dist.timestamp = hrt_absolute_time();
    for (int i = 0; i < 4; i++)
    {
        dist.anchor_id[i] = i;
        dist.anchor_pos_x[i] = anchor_nodes_xyz[i].x;
        dist.anchor_pos_y[i] = anchor_nodes_xyz[i].y;
        dist.anchor_pos_z[i] = anchor_nodes_xyz[i].z;
        dist.distance[i] = dist_in[i];
    }

    orb_publish(ORB_ID(GIRBAL_anchor_distances), dist_pub, &dist);
    //_GIRBAL_anchor_distances_pub.publish(dist);
}

// NOTE: This function takes current location as GPS coords, and the node location as XYZ coords!
void GIRBAL_Sim_Driver::calculateDistances(coordinates_gps current_location, coordinates_xyz nodes[], int dist_in[])
{
    for (int i = 0; i < 4; i++) //hardcoded for 4 base stations
    {
        coordinates_xyz currentLocXYZ = gps2ecef(current_location);

        // formula for finding distance between 2 xyz coordinates
        dist_in[i] = sqrt(pow(currentLocXYZ.x - nodes[i].x, 2) + pow(currentLocXYZ.y - nodes[i].y, 2) + pow(currentLocXYZ.z - nodes[i].z, 2));
    }
}

GIRBAL_Sim_Driver::coordinates_xyz GIRBAL_Sim_Driver::gps2ecef(coordinates_gps gps)
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

    coordinates_xyz xyzPos;

    xyzPos.x = ((R*c + gps.alt/1000) * cosLat * cosLon);
    xyzPos.y = ((R*c + gps.alt/1000) * cosLat * sinLon);
    xyzPos.z = ((R*s + gps.alt/1000) * sinLat);

    return xyzPos;
}

int GIRBAL_Sim_Driver::task_spawn(int argc, char *argv[])
{
	//WorkItemExample *instance = new WorkItemExample();
    GIRBAL_Sim_Driver *instance  = new GIRBAL_Sim_Driver();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int GIRBAL_Sim_Driver::print_status()
{
    PX4_INFO("Running GIRBAL_Sim_Driver");
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int GIRBAL_Sim_Driver::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GIRBAL_Sim_Driver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GIRBAL simulated module 1 (module 1.5).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("GIRBAL_Sim_Driver", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int GIRBAL_Sim_Driver_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Sim_Driver::main(argc, argv);
    //PX4_INFO("NO CLUE WHAT THE FUCK THIS DOES")
    //return 0;
}
