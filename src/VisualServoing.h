#ifndef VISUAL_SERVOING_H
#define VISUAL_SERVOING_H

#include <BasicLinearAlgebra.h>

namespace BLA {
    // Déclaration de la fonction
    void computeArticularVelocityFromVisualServoing(const BLA::Matrix<4,1>& r, float& t1, float& t2, float& t3, float& t4, float dt);
}

#endif // VISUAL_SERVOING_H
