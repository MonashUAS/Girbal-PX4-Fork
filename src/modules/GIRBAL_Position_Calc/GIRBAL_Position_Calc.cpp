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



COORDS GIRBAL_Position_Calc::calculateIntersection(double x1, double y1, double r1, double x2, double y2, double r2)
{
    COORDS intersection = { -0, -0 };
    
    //1. same circle
    if (x1 == x2 && y1 == y2 && r1 == r2)
    {
        return intersection;
    }

    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    //2. too far away
    if (distance > r1 + r2)
    {
        return intersection;
    }

    double xx = (pow(r1, 2) - pow(r2, 2) + pow(distance, 2)) / (2 * distance);
    double yy = sqrt(pow(r1, 2) - pow(xx, 2));

    //3. one circle inside the other
    if (r1 < abs(xx) || distance < abs(r2 - r1))
    {
        return intersection;
    }


    Vector2d v;
    v(0) = x2 - x1;
    v(1) = y2 - y1;

    Vector2d A;
    A(0) = x1;
    A(1) = y1;

    Vector2d e1 = (1 / distance) * v;

    //4a. 1 solution
    if (yy == 0)
    {
        Vector2d p = A + xx * e1;
        
        intersection = { p(0), p(1) };

        return intersection;
    }

    //4b. 2 solutions
    Matrix2d rot;
    rot(0, 0) = 0;
    rot(0, 1) = -1;
    rot(1, 0) = 1;
    rot(1, 1) = 0;

    Vector2d e2 = rot * e1;

    Vector2d p1 = A + xx * e1 + yy * e2;
    Vector2d p2 = A + xx * e1 - yy * e2;


    return answer.pushback()

}



// formula can be found: https://www.xarg.org/2016/07/calculate-the-intersection-points-of-two-circles/
COORDS[] GIRBAL_Position_Calc::calculateIntersections(COORDS_DIST circles[])
{
    int circleCount = sizeof(circles);
    
    for (int i = 0; i < circleCount - 1;++i)
    {
        COORDS_DIST circle1 = circles[i];

        for (int j = i + 1; j < circleCount;++j)
        {
            COORDS_DIST circle2 = circles[j];
            calculateIntersection(circle1.coords.x, circle1.coords.y, circle1.radius, circle2.coords.x, circle2.coords.y, circle2.radius);
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

Pos2dVec Trilateration::CalculateIntersection(PosAndDistance2d c1, PosAndDistance2d c2)
{
    double x1 = c1.m_pos(0);
    double y1 = c1.m_pos(1);
    double r1 = c1.m_distance;

    double x2 = c2.m_pos(0);
    double y2 = c2.m_pos(1);
    double r2 = c2.m_distance;

    Pos2dVec a;


    //1. same circle
    if (x1 == x2 && y1 == y2 && r1 == r2)
    {
        return a;
    }

    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    //2. too far away
    if (distance > r1 + r2)
    {
        return a;
    }

    double xx = (pow(r1, 2) - pow(r2, 2) + pow(distance, 2)) / (2 * distance);
    double yy = sqrt(pow(r1, 2) - pow(xx, 2));

    //3. one circle inside the other
    if (r1 < abs(xx) || distance < abs(r2 - r1))
    {
        return a;
    }


    Vector2d v;
    v(0) = x2 - x1;
    v(1) = y2 - y1;

    Vector2d A;
    A(0) = x1;
    A(1) = y1;

    Vector2d e1 = (1 / distance) * v;

    //4a. 1 solution
    if (yy == 0)
    {
        Vector2d p = A + xx * e1;
        a.push_back(p);
        return a;
    }

    //4b. 2 solutions
    Matrix2d rot;
    rot(0, 0) = 0;
    rot(0, 1) = -1;
    rot(1, 0) = 1;
    rot(1, 1) = 0;

    Vector2d e2 = rot * e1;

    Vector2d p1 = A + xx * e1 + yy * e2;
    Vector2d p2 = A + xx * e1 - yy * e2;

    a.push_back(p1);
    a.push_back(p2);

    return a;


}

Pos2dVec Trilateration::CalculateIntersections(PosAndDistance2dVec circles)
{
    int count = circles.size();
    Pos2dVec ans;

    for (int i = 0; i < count - 1;++i)
    {
        PosAndDistance2d circle1 = circles[i];

        for (int j = i + 1; j < count; ++j)
        {
            PosAndDistance2d circle2 = circles[j];

            Pos2dVec solution = CalculateIntersection(circle1, circle2);

            if (solution.size() == 1)
            {
                Vector2d row = CalculateIntersection(circle1, circle2)[0];
                ans.push_back(row);
            }

            if (solution.size() == 2)
            {
                Vector2d row1 = CalculateIntersection(circle1, circle2)[0];
                Vector2d row2 = CalculateIntersection(circle1, circle2)[1];

                ans.push_back(row1);
                ans.push_back(row2);
            }

        }
    }
    return ans;
}



extern "C" __EXPORT int work_item_example_main(int argc, char *argv[]) // not really sure what this func does tbh
{
    return GIRBAL_Position_Calc::main(argc, argv);
}
