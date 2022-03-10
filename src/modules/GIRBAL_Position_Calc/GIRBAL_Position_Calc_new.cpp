#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/GIRBAL_anchor_distances.h>
#include <uORB/topics/GIRBAL_Position_Calc.h>

__EXPORT int GIRBAL_Sim_Driver_main(int argc, char *argv[]);

//////////// FUNCTIONS USED IN TRILATERATION //////////
double dotProduct(COORDS* A, COORDS* B) 
{
    double product = (A->x) * (B->x) + (A->y) * (B->y) + (A->z) * (B->z);
    return product;
}

// function that calculates the cross product of two COORD structures, used in 'trilateration' function
void crossProduct(COORDS* A, COORDS* B, COORDS* P)
{
    P->x = A->y * B->z - A->z * B->y;
    P->y = A->z * B->x - A->x * B->z;
    P->z = A->x * B->y - A->y * B->x;
}

COORDS GIRBAL_Position_Calc::trilateration COORDS* P1, COORDS* P2, COORDS* P3) 
{

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




int GIRBAL_Sim_Driver_main(int argc, char *argv[]) //Main function
{
    /* subscribe to vehicle_acceleration topic */
	int anchor_distance_sub = orb_subscribe(ORB_ID(GIRBAL_anchor_distances));
	/* limit the update rate to 5 Hz */
	orb_set_interval(anchor_distance_sub, 200);

	//advertise attitude topic
	struct GIRBAL_vehichle_pos_s dist;
	memset(&dist, 0, sizeof(dist));
	orb_advert_t dist_pub = orb_advertise(ORB_ID(GIRBAL_vehichle_pos), &dist);


    /* one could wait for multiple topics with this technique, just using one here */
	px4_pollfd_struct_t fds[] = {
		{ .fd = anchor_distance_sub,   .events = POLLIN },
		/* there could be more file descriptors here, in the form like:
		 * { .fd = other_sub_fd,   .events = POLLIN },
		 */
	};
    




    
}

