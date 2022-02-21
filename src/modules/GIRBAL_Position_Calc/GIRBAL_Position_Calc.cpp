#include "GIRBAL_Position_Calc.hpp"

GIRBAL_Position_Calc::GIRBAL_Position_Calc() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1) // no idea what this is and can't find documentation for it
{

}

// I assume this is the destructor?
GIRBAL_Position_Calc::~GIRBAL_Position_Calc()
{
    perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool GIRBAL_Position_Calc::init()
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

void GIRBAL_Position_Calc::Run()
{
    PX4_INFO("RUN FUNCTION ENTERED");
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // defined in destructor(?) near the start
    perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);


    // "work" happens here on distances callback
    if (GIRBAL_anchor_distances_sub.updated()) {
        PX4_INFO("UPDATING ANCHOR DISTANCES");

        // initialising distPtr using structure defined in GIRBAL_anchor_distances.msg
        // int32[4] anchor_id
        // float32[4] anchor_pos_x
        // float32[4] anchor_pos_y
        // float32[4] anchor_pos_z
        // float32[4] distance
        struct GIRBAL_anchor_distances_s distPtr; // still can't find documentation on how _s affects code/copies inputs

        // initialising 3 COORDS structures to copy info into, and enter into function
        struct COORDS p1,p2,p3,*P1,*P2,*P3;
        P1 = &p1; P2 = &p2; P3 = &p3;

        // initialising COORDS structure to output result
        struct COORDS pt, *PT;
        PT = &pt;

		if (GIRBAL_anchor_distances_s.copy(&distPtr)) {
            PX4_INFO("COPYING NEW DATA INTO STRUCTURES");
            // copying input into the new COORDS structures
            P1->x = distPtr.anchor_pos_x[0];
            P1->y = distPtr.anchor_pos_y[0];
            P1->z = distPtr.anchor_pos_z[0];
            P1->r = distPtr.distance[0];

            P2->x = distPtr.anchor_pos_x[1];
            P2->y = distPtr.anchor_pos_y[1];
            P2->z = distPtr.anchor_pos_z[1];
            P2->r = distPtr.distance[1];

            P3->x = distPtr.anchor_pos_x[2];
            P3->y = distPtr.anchor_pos_y[2];
            P3->z = distPtr.anchor_pos_z[2];
            P3->r = distPtr.distance[2];

            *PT = trilateration(P1,P2,P3);
            // WRITE A FUNCTION TO PUBLISH COORDS, same as line 119 in GIRBAL_Sim_Driver.cpp
            publishCoords(PT);
		}
	}
    perf_end(_loop_perf);
}

void GIRBAL_Position_Calc::publishCoords(COORDS* PT) {
    // followed code in GIRBAL_Sim_Driver by adding _s after the message name, but can't find documentation
    // don't know if this works
    struct GIRBAL_vehicle_pos_s pos;
    PX4_INFO("PUBLISHING VEHICLE POSITION");

    memset(&pos,0,sizeof(pos));
    // can't find documentation of orb_advert_t structure and orb_advertise function
    // copied from GIRBAL_Sim_Driver
    orb_advert_t pos_pub = orb_advertise(ORB_ID(GIRBAL_vehicle_pos), &pos);

    // copying over information
    pos.timestamp = hrt_absolute_time(); // log start time
    pos.vehicle_pos_x = PT->x;
    pos.vehicle_pos_y = PT->y;
    pos.vehicle_pos_z = PT->z;

    orb_publish(ORB_ID(GIRBAL_vehicle_pos), pos_pub, &pos); // can't seem to find documentation on orb_publish
    //_GIRBAL_vehicle_pos_pub.publish(pos);
    // ^^ from line 124 in https://github.com/PX4/PX4-Autopilot/blob/master/src/examples/work_item/WorkItemExample.cpp
    // these lines were copied from GIRBAL_Sim_Driver, but not sure which one's supposed to work
}

// function that calculates the dot product of two COORD structures, used in 'trilateration' function
double dotProduct(COORDS* A, COORDS* B) {
    double product = (A->x) * (B->x) + (A->y) * (B->y) + (A->z) * (B->z);
    return product;
}

// function that calculates the cross product of two COORD structures, used in 'trilateration' function
void crossProduct(COORDS* A, COORDS* B, COORDS* P) {
    P->x = A->y * B->z - A->z * B->y;
    P->y = A->z * B->x - A->x * B->z;
    P->z = A->x * B->y - A->y * B->x;
}

// takes the coordinates of 3 nodes and performs trilateration to locate the drone
COORDS GIRBAL_Position_Calc::trilateration(COORDS* P1, COORDS* P2, COORDS* P3) {

    // Redefine anchor node coordinates with P1 (the first anchor node) as the origin
    COORDS p1a, p2a, p3a, *P1a, *P2a, *P3a;
    P1a = &p1a; P2a = &p2a; P3a = &p3a;

    P1a->x = P1->x - P1->x;
    P1a->y = P1->y - P1->y;
    P1a->z = P1->z - P1->z;

    P2a->x = P2->x - P1->x;
    P2a->y = P2->y - P1->y;
    P2a->z = P2->z - P1->z;

    P3a->x = P3->x - P1->x;
    P3a->y = P3->y - P1->y;
    P3a->z = P3->z - P1->z;

    // 2. rotate P2 and P3 so that P2 only has an x component
    //    new x coordinate of P2 can be found via pythagoras - sqrt( (p2x-p1x)^2 + (p2y-p1y)^2 + (p2z-p1z)^2 ),
    //    modified source code from https://gis.stackexchange.com/questions/66/trilateration-using-3-latitude-longitude-points-and-3-distances

    double d  = sqrt(pow(P2a->x,2) + pow(P2a->y,2) + pow(P2a->z,2));  // distance from origin to P2a
    COORDS ex, *EX;
    EX = &ex;                                                         // unit x vector (in direction of P2a)
    EX->x = (P2a->x) / d;
    EX->y = (P2a->y) / d;
    EX->z = (P2a->z) / d;
    double i  = dotProduct(EX, P3a);                                  // x coordinate for rotated P3
    double ey_denom = sqrt( pow(P3a->x - i*(EX->x),2) + pow(P3a->y - i*(EX->y),2) + pow(P3a->z - i*(EX->z),2) );
    COORDS ey, *EY;                                                   // unit y vector i think?? not sure what the fancy i*ex is for
    EY = &ey;
    EY->x = (P3a->x - i*EX->x) / ey_denom;
    EY->y = (P3a->y - i*EX->y) / ey_denom;
    EY->z = (P3a->z - i*EX->z) / ey_denom;

    double j  = dotProduct(EY, P3a);                                  // y coordinate for rotated P3
    COORDS ez, *EZ;
    crossProduct(EX, EY, EZ);                                         // unit z vector

    // 3. using previous calculations to work out x,y,z coordinates of the drone, in our newly defined coordinate system
    //    modified source code from https://en.wikipedia.org/wiki/True-range_multilateration#Three_Cartesian_dimensions,_three_measured_slant_ranges
    double x = ( pow(P1->r,2) - pow(P2->r,2) + pow(d,2) ) / (2*d);
    double y = ( pow(P1->r,2) - pow(P3->r,2) + pow(i,2) + pow(j,2) )/(2*j) - ((i/j)*x);
    double z = sqrt(pow(P1->r,2) - pow(x,2) - pow(y,2)); // only one case shown here - could be the negative square root??

    // 4. return triPt - a COORDS structure with ECEF x,y,z of trilateration point
    //    converts from the newly defined coordinate system back to the original coordinate system
    COORDS tri_pt, *triPt;
    triPt = &tri_pt;
    triPt->x = P1->x + x*(EX->x) + y*(EY->x) + z*(EZ->x);
    triPt->y = P1->y + x*(EX->y) + y*(EY->y) + z*(EZ->y);
    triPt->z = P1->z + x*(EX->z) + y*(EY->z) + z*(EZ->z);

    // USED TO CHECK RESULTS (not passed on through the function) convert back to lat/long from ECEF
    // int earthR = 6371;
    // double pi = 2*acos(0.0);

    // double lat = (asin(triPt->z / earthR)) * (180/pi);
    // double lon = (atan2(triPt->y, triPt->x)) * (180/pi);

    return *triPt;
}

extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Position_Calc::main(argc, argv);
}
