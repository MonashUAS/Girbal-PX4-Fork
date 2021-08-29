#include "GIRBAL_Position_Calc.hpp"

GIRBAL_Position_Calc::GIRBAL_Position_Calc() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{

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
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // "work" happens here on distances callback
    if (GIRBAL_anchor_distances_s.updated()) {
		GIRBAL_anchor_distances_s distPtr;

		if (GIRBAL_anchor_distances_s.copy(&distPtr)) {


		}
	}
}

// modified from: https://stackoverflow.com/questions/2792443/finding-the-centroid-of-a-polygon
// formula can be found: https://en.wikipedia.org/wiki/Centroid#Of_a_polygon
COORDS GIRBAL_Position_Calc::polygonCalcCentre(COORDS vertices[])
{
    int vertexCount = sizeof(vertices)/sizeof(vertices[0]);

    COORDS centroid = {0, 0};
    double signedArea = 0.0;
    double x0 = 0.0; // Current vertex X
    double y0 = 0.0; // Current vertex Y
    double x1 = 0.0; // Next vertex X
    double y1 = 0.0; // Next vertex Y
    double a = 0.0;  // Partial signed area

    // For all vertices except last
    int i=0;
    for (i=0; i<vertexCount-1; ++i)
    {
        x0 = vertices[i].x;
        y0 = vertices[i].y;
        x1 = vertices[i+1].x;
        y1 = vertices[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroid.x += (x0 + x1)*a;
        centroid.y += (y0 + y1)*a;
    }

    // Do last vertex separately to avoid performing an expensive modulus operation in each iteration.
    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[0].x;
    y1 = vertices[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroid.x += (x0 + x1)*a;
    centroid.y += (y0 + y1)*a;

    signedArea *= 0.5;
    centroid.x /= (6.0*signedArea);
    centroid.y /= (6.0*signedArea);
    centroid.z = 0; // not using 3D in this implementation, so set Z to zero

    return centroid;
}


extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Position_Calc::main(argc, argv);
}
