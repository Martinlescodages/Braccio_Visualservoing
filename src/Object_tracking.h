#ifndef OBJECT_TRACKING_H
#define OBJECT_TRACKING_H

#include <BasicLinearAlgebra.h>
#include <Pixy2.h>

extern Pixy2 pixy;

namespace BLA {

    BLA::Matrix<18,1> trackObject_robot(float t1, float t2, float t3, float Zc, const BLA::Matrix<8,1>& ref, float ref_cx);
    BLA::Matrix<18,1> applyPIDToErrors(const BLA::Matrix<18,1>& f, float dt);
    bool isObjectWithinTolerance(const BLA::Matrix<8,1>& points_robot, const BLA::Matrix<8,1>& ref, float tolerance);
}
#endif // OBJECT_TRACKING_H
