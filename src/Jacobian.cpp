#include "Jacobian.h"
#include <BasicLinearAlgebra.h>


// Fonction jvplus : calcule le torseur cinématique r à partir de f (erreurs + primitives + centrageX)

BLA::Matrix<4,1> BLA::jvplus(const BLA::Matrix<18,1>& f) {
    const float Gx = 1000.0;
    const float Gy = 1000.0;
    const float z  = 0.5;

    BLA::Matrix<8,3> J;

    // Construction du Jacobien image (8 lignes = 4 coins x (x,y))
    for (int i = 0; i < 4; i++) {
        float x = f(8 + 2*i);     // f(8), f(10), f(12), f(14)
        float y = f(8 + 2*i + 1); // f(9), f(11), f(13), f(15)

        int r = 2 * i;

        // Ligne paire (X)
        J(r, 0) = 0;
        J(r, 1) = x / z;
        J(r, 2) = (x * y) / Gy;

        // Ligne impaire (Y)
        J(r+1, 0) = -Gy / z;
        J(r+1, 1) = y / z;
        J(r+1, 2) = (Gy * Gy + y * y) / Gy;
    }

    // Pseudo-inverse : Jvp = (Jᵗ * J + λI)⁻¹ * Jᵗ
    BLA::Matrix<3,8> J_T = ~J;
    BLA::Matrix<3,3> JTJ = J_T * J;

    float lambda = 0.01;
    BLA::Matrix<3,3> I = {1,0,0, 0,1,0, 0,0,1};
    BLA::Matrix<3,3> JTJ_reg = JTJ + lambda * I;
    BLA::Matrix<3,3> JTJ_inv = Inverse(JTJ_reg);

    BLA::Matrix<3,8> Jvp = JTJ_inv * J_T;

    // Partie erreur d'image (f(0) à f(7))
    BLA::Matrix<8,1> err;
    for (int i = 0; i < 8; i++) {
        err(i) = f(i);
    }

    BLA::Matrix<3,1> r_visuel = Jvp * err;

    // Erreur centrage X (f(16)) → associée à t4
    float err_centre_x = f(16);

    // Torseur cinématique final [vy, vz, dθ, correction_rotation_base]
    BLA::Matrix<4,1> r;
    r(0) = r_visuel(0);
    r(1) = r_visuel(1);
    r(2) = r_visuel(2);
    r(3) = err_centre_x;  // t4 (centrage en X de l’objet)

    return r;
}
