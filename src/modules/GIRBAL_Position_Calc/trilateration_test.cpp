#include <cmath>
#include <iostream>
using namespace std;

// struct for passing 3D coords
// removed COORDS_DIST struct and instead just gave a radius attribute to the COORDS struct
struct COORDS {
    double x = 0;
    double y = 0;
    double z = 0;
    double r = 0;
};

// // structure for outputting gps coordinates
// struct COORDS_gps {
//     double lat;
//     double lon;
// };

double dotProduct(COORDS* A, COORDS* B) {
    double product = (A->x) * (B->x) + (A->y) * (B->y) + (A->z) * (B->z);
    return product;
}

void crossProduct(COORDS* A, COORDS* B, COORDS* P) {
    P->x = A->y * B->z - A->z * B->y;
    P->y = A->z * B->x - A->x * B->z;
    P->z = A->x * B->y - A->y * B->x;
}

COORDS trilateration(COORDS* P1, COORDS* P2, COORDS* P3) {

 // Redefined anchor node coordinates with P1 (the first anchor node) as the origin
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
    // new x coordinate of P2 can be found via pythagoras - sqrt( (p2x-p1x)^2 + (p2y-p1y)^2 + (p2z-p1z)^2 ),
    // modified source code from https://gis.stackexchange.com/questions/66/trilateration-using-3-latitude-longitude-points-and-3-distances

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

    // from https://en.wikipedia.org/wiki/True-range_multilateration#Three_Cartesian_dimensions,_three_measured_slant_ranges
    double x = ( pow(P1->r,2) - pow(P2->r,2) + pow(d,2) ) / (2*d);
    double y = ( pow(P1->r,2) - pow(P3->r,2) + pow(i,2) + pow(j,2) )/(2*j) - ((i/j)*x);
    double z = sqrt(pow(P1->r,2) - pow(x,2) - pow(y,2)); // only one case shown here - could be the negative square root??

    // triPt is an array with ECEF x,y,z of trilateration point
    COORDS tri_pt, *triPt;
    triPt = &tri_pt;
    triPt->x = P1->x + x*(EX->x) + y*(EY->x) + z*(EZ->x);
    triPt->y = P1->y + x*(EX->y) + y*(EY->y) + z*(EZ->y);
    triPt->z = P1->z + x*(EX->z) + y*(EY->z) + z*(EZ->z);

    // convert back to lat/long from ECEF
    // convert to degrees
    int earthR = 6371;
    double pi = 2*acos(0.0);

    double lat = (asin(triPt->z / earthR)) * (180/pi);
    double lon = (atan2(triPt->y, triPt->x)) * (180/pi);

    return *triPt;
}

int main() {
    // test coordinates
    COORDS p1 = { .x = -2678.6397797164627, .y = -4292.806432214039, .z = 3871.219816293349, .r = 0.265710701754};
    COORDS p2 = { .x = -2678.5634684441684, .y = -4292.949056245459, .z = 3871.1144578787075, .r = 0.234592423446};
    COORDS p3 = { .x = -2678.3846453910214, .y = -4292.945234069091, .z = 3871.2424244184303, .r = 0.0548954278262};
    COORDS *P1, *P2, *P3;
    P1 = &p1; P2 = &p2; P3 = &p3;

    COORDS pt, *PT;
    PT = &pt;

    // // test gps output
    // COORDS_gps pt;
    // COORDS_gps *PT;
    // PT = &pt;

    *PT = trilateration(P1,P2,P3);
    cout << "x = " << PT->x << endl;
    cout << "y = " << PT->y << endl;
    cout << "z = " << PT->z << endl;
    // cout << "lat = " << PT->lat << endl;
    // cout << "lon = " << PT->lon << endl;
    // lat = 37.419102373825389
    // lon = -121.96057920839233
    int earthR = 6371;
    double pi = 2*acos(0.0);
    double lat = (asin(PT->z / earthR)) * (180/pi);
    double lon = (atan2(PT->y, PT->x)) * (180/pi);

    cout << "lat = " << lat << endl;
    cout << "lon = " << lon << endl;

    return 0;
}


