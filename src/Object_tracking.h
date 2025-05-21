#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

#include <BasicLinearAlgebra.h>
#include <Pixy2.h>

namespace BLA {
    // Déclaration de la fonction
    BLA::Matrix<18,1> trackObject_robot(float t1, float t2, float t3, float Zc, const BLA::Matrix<8,1>& ref, float ref_cx);
    bool isObjectWithinTolerance(const BLA::Matrix<8,1>& points_robot, float tolerance);
}

#endif // OBJECT_TRACKING_H
