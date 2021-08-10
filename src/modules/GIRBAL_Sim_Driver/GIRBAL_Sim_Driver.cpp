#include "GIRBAL_Sim_Driver.hpp"

GIRBAL_Sim_Driver::GIRBAL_Sim_Driver() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
    // hard coded coords are choosen from Irchelpark, Zurich as this is where jmavsim defaults to
    // as gps publisher uses int, coords are 1e7 greater than reality
    anchor_nodes_gps[1].lat = 473969840; // anchor1, bottom left
    anchor_nodes_gps[1].lon = 85413730;
    anchor_nodes_gps[1].alt = 490506;
    anchor_nodes_gps[2].lat = 473992560; // anchor2, top left
    anchor_nodes_gps[2].lon = 85427100;
    anchor_nodes_gps[2].alt = 490506;
    anchor_nodes_gps[3].lat = 473999640; // anchor3, top right
    anchor_nodes_gps[3].lon = 85474340;
    anchor_nodes_gps[3].alt = 490506;
    anchor_nodes_gps[4].lat = 473963450; // anchor4, bottom right
    anchor_nodes_gps[4].lon = 85452000;
    anchor_nodes_gps[4].alt = 490506;

    for (int i = 0; i < 4; i++) // convert GPS base stations to XYZ in the constructor (I could definitely pre-calculate this but I'm lazy)
    {
        anchor_nodes_xyz[i] = gps2ecef(anchor_nodes_gps[i]);
    }
}

bool GIRBAL_Sim_Driver::init()
{
    // execute Run() on every gps publication
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
        gpsCOORDS currentPos;

		if (sensor_gps_s.copy(&gpsPtr)) {
			// put coords subscriber receives into struct
            currentPos.lat = gpsPtr->lat;
            currentPos.lon = gpsPtr->lon;
            currentPos.alt = gpsPtr->alt;

            // run calculateDistances on that struct
            // the func will edit the distances array with the calculated distances
            calculateDistances(currentPos, anchor_nodes_xyz, distances);
            // TODO: broadcast distances over publisher


		}
	}
}

void GIRBAL_Sim_Driver::calculateDistances(gpsCOORDS current_location, xyzCOORDS nodes[], int *distances[])
{
    for (int i = 0; i < 4; i++) //hardcoded for 4 base stations
    {
        xyzCOORDS currentLocXYZ = gps2ecef(current_location);

        // formula for finding distance between 2 xyz coordinates
        *distances[i] = sqrt(pow(currentLocXYZ.x - nodes[i].x, 2) + pow(currentLocXYZ.y - nodes[i].y, 2) + pow(currentLocXYZ.z - nodes[i].z, 2));
    }
}

GIRBAL_Sim_Driver::xyzCOORDS GIRBAL_Sim_Driver::gps2ecef(gpsCOORDS gps)
{
    // https://stackoverflow.com/questions/18759601/converting-lla-to-xyz
    // assuumes WSG-84 complicit implementation of jmavsim GPS
    float R = 6378137*1e7; // radius of the earth
    float f_inv = 298.257224*1e7; // magic (some sort of flattening ratio to translate 3D lat/lon onto a 2D plane)
    float f = 1.0 / f_inv;
    float e2 = 1 - (1 - f) * (1 - f);

    float cosLat = cos(gps.lat * M_PI / 180);
    float sinLat = sin(gps.lat * M_PI / 180);

    float cosLon = cos(gps.lon * M_PI / 180);
    float sinLon = sin(gps.lon * M_PI / 180);

    float c = 1 / sqrt(cosLat * cosLat + (1 - f) * (1 - f) * sinLat * sinLat);
    float s = (1 - f) * (1 - f) * c;

    xyzCOORDS xyzPos;

    xyzPos.x = (int)((R*c + gps.alt) * cosLat * cosLon);
    xyzPos.y = (int)((R*c + gps.alt) * cosLat * sinLon);
    xyzPos.z = (int)((R*s + gps.alt) * sinLat);

    return xyzPos;
}


extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Sim_Driver::main(argc, argv);
}
