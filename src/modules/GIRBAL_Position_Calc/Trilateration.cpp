//
//  Trilateration.cpp
//
//  Code modified from https://github.com/Wayne82/Trilateration/blob/master/source/Trilateration.cpp
//  Copyright (c) 2014 waynewang. All rights reserved.
//
#pragma warning(push)
#pragma warning(disable : 4244) // Disable warnings from external toolkit.
#include "Trilateration.h"
#include <Eigen/Dense>
#include <Eigen/LU>
#include <math.h>
#include <iostream>
#pragma warning(pop)

Pos3d Trilateration::CalculateLocation3d(const PosAndDistance3dVec& anchors, Pos3d& location)
{
    // To locate position in a 3D space, have to get at least 4 becaons,
    // otherwise return false.
    // if (anchors.size() < 3)
    //    return false;

    // Define the matrix that we are going to use
    size_t count = anchors.size();
    size_t rows = count * (count - 1) / 2;
    Eigen::MatrixXd m(rows, 3);
    Eigen::VectorXd b(rows);

    // Fill in matrix according to the equations
    size_t row = 0;
    double x1, x2, y1, y2, z1, z2, r1, r2;
    PosAndDistance3d anchor1, anchor2;
    for (size_t i = 0; i < count; ++i) {
        anchor1 = anchors[i];
        for (size_t j = i + 1; j < count; ++j) {
            anchor2 = anchors[j];

            x1 = anchor1.m_pos(0), y1 = anchor1.m_pos(1), z1 = anchor1.m_pos(2);
            x2 = anchor2.m_pos(0), y2 = anchor2.m_pos(1), z2 = anchor2.m_pos(2);
            r1 = anchor1.m_distance;
            r2 = anchor2.m_distance;
            m(row, 0) = x1 - x2;
            m(row, 1) = y1 - y2;
            m(row, 2) = z1 - z2;
            b(row) = ((pow(x1, 2) - pow(x2, 2)) +
                (pow(y1, 2) - pow(y2, 2)) +
                (pow(z1, 2) - pow(z2, 2)) -
                (pow(r1, 2) - pow(r2, 2))) / 2;
            row++;
        }
    }

    // Uses least square solution to calculate solution
    //location = m.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    
    // Uses LU decomposition to calculate solution

    location = m.fullPivLu().solve(b);
    
    return location;
}


double ErrorModel(const double posd)
{

    if (posd < 4)
    {
        return 0.0125 * posd;
    }
    else
    {
        return 0.3;
    }
}

Pos3d Trilateration::CalculateLocation3dWithError(const PosAndDistance3dVec& anchors, Pos3d& location)
{
    // To locate position in a 3D space, have to get at least 4 becaons,
    // otherwise return false.
    // if (anchors.size() < 3)
    //    return false;

    // Define the matrix that we are going to use
    size_t count = anchors.size();
    size_t rows = count * (count - 1) / 2;
    Eigen::MatrixXd m(rows, 3);
    Eigen::VectorXd b(rows);

    // Fill in matrix according to the equations
    size_t row = 0;
    double x1, x2, y1, y2, z1, z2, r1, r2;
    PosAndDistance3d anchor1, anchor2;
    for (size_t i = 0; i < count; ++i) {
        anchor1 = anchors[i];
        for (size_t j = i + 1; j < count; ++j) {
            anchor2 = anchors[j];

            x1 = anchor1.m_pos(0), y1 = anchor1.m_pos(1), z1 = anchor1.m_pos(2);
            x2 = anchor2.m_pos(0), y2 = anchor2.m_pos(1), z2 = anchor2.m_pos(2);
            r1 = anchor1.m_distance;
            r2 = anchor2.m_distance;
            m(row, 0) = x1 - x2;
            m(row, 1) = y1 - y2;
            m(row, 2) = z1 - z2;
            b(row) = ((pow(x1, 2) - pow(x2, 2)) +
                (pow(y1, 2) - pow(y2, 2)) +
                (pow(z1, 2) - pow(z2, 2)) -
                (pow(r1, 2) - pow(r2, 2))) / 2;
            row++;
        }
    }

    // Uses least square solution to calculate solution
    //location = m.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

    // Uses LU decomposition to calculate solution

    location = m.fullPivLu().solve(b);

    return location;
}



int main() {
    Trilateration drone;
    PosAndDistance3dVec anchors;
    PosAndDistance3dVec anchorerror;
    Pos3d location;

    // initialise the anchors

    //EXAMPLE 1 - ANS = (-2, 8.01208, 5.82569)
    //anchors.push_back(PosAndDistance3d(Pos3d(-5, 3, 2), 6.982610536361024));
    //anchors.push_back(PosAndDistance3d(Pos3d(-3, -6, 0), 15.207795695053901));
    //anchors.push_back(PosAndDistance3d(Pos3d(4, -2, -1), 13.521530605020274));
    //anchors.push_back(PosAndDistance3d(Pos3d(3, 6, 0), 7.936443151847054));


    //EXAMPLE 2 - Need more than 4 anchor nodes. ANS = (0, 4, 0)
    //anchors.push_back(PosAndDistance3d(Pos3d(-1, 0, 0), 4.123105625617661));
    //anchors.push_back(PosAndDistance3d(Pos3d(1, 0, 0), 4.123105625617661));
    //anchors.push_back(PosAndDistance3d(Pos3d(0, 0, 1), 4.123105625617661));
    //anchors.push_back(PosAndDistance3d(Pos3d(0, 0, -1), 4.123105625617661));
    //anchors.push_back(PosAndDistance3d(Pos3d(5, 5, 2), 115.477225575051661));

    //EXAMPLE 3 - With errors
    anchors.push_back(PosAndDistance3d(Pos3d(-5, 3, 2), 6.982610536361024 + ErrorModel(6.982610536361024)));
    anchors.push_back(PosAndDistance3d(Pos3d(-3, -6, 0), 15.207795695053901 + ErrorModel(15.207795695053901)));
    anchors.push_back(PosAndDistance3d(Pos3d(4, -2, -1), 13.521530605020274 + ErrorModel(13.521530605020274)));
    anchors.push_back(PosAndDistance3d(Pos3d(3, 6, 0), 7.936443151847054 + + ErrorModel(7.936443151847054)));

    

    std::cout <<drone.CalculateLocation3d(anchors, location);
    return 0;

}