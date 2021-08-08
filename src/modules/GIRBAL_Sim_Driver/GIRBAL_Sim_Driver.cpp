#include "GIRBAL_Sim_Driver.hpp"

GIRBAL_Sim_Driver::GIRBAL_Sim_Driver() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
}

bool GIRBAL_Sim_Driver::init()
{
    COORDS anchor_nodes[4]; //anchor nodes coordinates structure

    // hard coded coords are choosen from Irchelpark, Zurich as this is where jmavsim defaults to
    anchor_nodes[1].lat = 47.396984; // anchor1, bottom left
    anchor_nodes[1].lon = 8.541373;
    anchor_nodes[2].lat = 47.399256; // anchor2, top left
    anchor_nodes[2].lon = 8.542710;
    anchor_nodes[3].lat = 47.399964; // anchor3, top right
    anchor_nodes[3].lon = 8.547434;
    anchor_nodes[4].lat = 47.396345; // anchor4, bottom right
    anchor_nodes[4].lon = 8.545200;

    float distances[4];

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
			// TODO: take the bits of GPS we care about (lat & lon) and store them in a nice COORDS struct


            // TODO: run calculateDistances on that struct

            // TODO: broadcast distances

		}
	}
}

void GIRBAL_Sim_Driver::calculateDistances(COORDS current_location, COORDS nodes[], float *distances[]) // current location should be coords struct and nodes should be array of coords structs
{
    int num_nodes = 4; // imo we may as well hardcode this as 4 for this module
    for (int i = 0; i < num_nodes; i++)
    {
        *distances[i] = sin(nodes[i].lat) * sin(current_location.lat) + cos(nodes[i].lat) * cos(current_location.lat) * cos(nodes[i].lat - current_location.lat);
        *distances[i] = acos(*distances[i]);
        *distances[i] = (6371 * M_PI * *distances[i]) / 180;
    } // TODO: check this formula - see haversine formula
}


extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Sim_Driver::main(argc, argv);
}
