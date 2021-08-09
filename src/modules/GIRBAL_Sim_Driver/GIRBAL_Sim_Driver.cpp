#include "GIRBAL_Sim_Driver.hpp"

GIRBAL_Sim_Driver::GIRBAL_Sim_Driver() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
    // hard coded coords are choosen from Irchelpark, Zurich as this is where jmavsim defaults to
    anchor_nodes[1].lat = 473969840; // anchor1, bottom left
    anchor_nodes[1].lon = 85413730;
    anchor_nodes[1].alt = 490506;
    anchor_nodes[2].lat = 473992560; // anchor2, top left
    anchor_nodes[2].lon = 85427100;
    anchor_nodes[2].alt = 490506;
    anchor_nodes[3].lat = 473999640; // anchor3, top right
    anchor_nodes[3].lon = 8.5474340;
    anchor_nodes[3].alt = 490506;
    anchor_nodes[4].lat = 473963450; // anchor4, bottom right
    anchor_nodes[4].lon = 85452000;
    anchor_nodes[4].alt = 490506;
}

bool GIRBAL_Sim_Driver::init()
{
    // execute Run() on every sensor_accel publication
    if (!sensor_gps_s.registerCallback()) {
        PX4_ERR("GPS data callback registration failed");
        return false;
    }

    // alternatively, Run on fixed interval
    // ScheduleOnInterval(5000_us); // 2000 us interval, 200 Hz rate

    return true;
}

void GIRBAL_Sim_Driver::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // "work" happens here on GPS callback
    if (sensor_gps_s.updated()) {
		sensor_gps_s gpsPtr;
        COORDS currentPos;

		if (sensor_gps_s.copy(&gpsPtr)) {
			// put coords subscriber receives into
            currentPos.lat = gpsPtr->lat;
            currentPos.lon = gpsPtr->lon;
            currentPos.alt = gpsPtr->alt;

            // run calculateDistances on that struct
            // the func will edit the distances array with the calculated distances
            calculateDistances(currentPos, anchor_nodes, distances);
            // TODO: broadcast distances


		}
	}
}

void GIRBAL_Sim_Driver::calculateDistances(COORDS current_location, COORDS nodes[], int *distances[])
{
    for (int i = 0; i < 4; i++) //hardcoded for 4 base stations
    {
        // formula for finding distance between 2 xyz coordinates
        // Haversine formula not needed as curvature of earth negligible at this scale
        *distances[i] = sqrt(pow(current_location.lat - nodes[i].lat, 2) + pow(current_location.lon - nodes[i].lon, 2) + pow(current_location.alt - nodes[i].alt, 2));
    }
}


extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Sim_Driver::main(argc, argv);
}
