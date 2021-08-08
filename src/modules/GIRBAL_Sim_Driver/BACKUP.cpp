/*

Fake UWB driver for simulator testing
Takes simulator GPS data and calculates distances to a series of hardcoded anchor nodes
Transmits anchors and their respective distances over uORB for further processing

This document exists as an easy-access archive of Josh's initial commit (that I started to edit)
*/

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <math.h>

#include <uORB/uORB.h>

struct coordinates //input structure for coordinates from GPS data from sim
{
    float lat;
    float lon;
};
typedef struct coordinates COORDS;

/* do we need a structure for distances or can it just be an array?
struct distances
{
    float distance_to_node
    float node_ID
};
*/

void calculateDistances(COORDS current_location, COORDS nodes[], float distances[]) // current location should be coords struct and nodes should be array of coords structs
{
    int num_nodes = 4; // imo we may as well hardcode this as 4 for this module
    for (int i = 0; i < num_nodes; i++)
    {
        distances[i] = sin(nodes[i].lat) * sin(current_location.lat) + cos(nodes[i].lat) * cos(current_location.lat) * cos(nodes[i].lat - current_location.lat);
        distances[i] = acos(distances[i]);
        distances[i] = (6371 * M_PI * distances[i]) / 180;
    } //check this formula - see haversine formula

}

int main()
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


}
