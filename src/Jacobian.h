#ifndef JACOBIAN_H
#define JACOBIAN_H

#include <BasicLinearAlgebra.h>

namespace BLA {
    // Déclaration de la fonction
    BLA::Matrix<4,1> jvplus(const BLA::Matrix<18,1>& f);
}

#endif // JACOBIAN_H
