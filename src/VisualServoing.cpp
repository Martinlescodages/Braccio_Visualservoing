#include "VisualServoing.h"
#include <Arduino.h> // Pour Serial.print

void BLA::computeArticularVelocityFromVisualServoing(const BLA::Matrix<4,1>& r, float& t1, float& t2, float& t3, float& t4, float dt) {
    // 1. Extraction du torseur visuel r = [vy, vz, dθ, erreur_centrage_x]
    BLA::Matrix<3,1> v;
    v(0) = r(0); // vy
    v(1) = r(1); // vz
    v(2) = r(2); // dθ
    float centrageX = r(3); // erreur de centrage en X de l'objet

    // 2. Constantes géométriques
    float a1 = 125.0;
    float a2 = 125.0;
    float a3 = 170.0;

    // 3. Jacobien géométrique dans le repère de la base
    BLA::Matrix<3,3> J_T0 = {
        -a1 * sin(t1) - a2 * sin(t1 + t2) - a3 * sin(t1 + t2 + t3),
        -a2 * sin(t1 + t2) - a3 * sin(t1 + t2 + t3),
        -a3 * sin(t1 + t2 + t3),

         a1 * cos(t1) + a2 * cos(t1 + t2) + a3 * cos(t1 + t2 + t3),
         a2 * cos(t1 + t2) + a3 * cos(t1 + t2 + t3),
         a3 * cos(t1 + t2 + t3),

         1, 1, 1
    };

    BLA::Matrix<3,3> J_R0 = {
        0, 0, 0,
        0, 0, 0,
        1, 1, 1
    };

    // 4. Matrices de rotation
    BLA::Matrix<3,3> R01 = {
        cos(t1), -sin(t1), 0,
        sin(t1),  cos(t1), 0,
        0, 0, 1
    };
    BLA::Matrix<3,3> R12 = {
        cos(t2), -sin(t2), 0,
        sin(t2),  cos(t2), 0,
        0, 0, 1
    };
    BLA::Matrix<3,3> R23 = {
        cos(t3), -sin(t3), 0,
        sin(t3),  cos(t3), 0,
        0, 0, 1
    };
    BLA::Matrix<3,3> R3c = {
        0,  0, 1,
        0, -1, 0,
        1,  0, 0
    };

    // 5. Transformation totale repère base → caméra
    BLA::Matrix<3,3> R0c = R01 * R12 * R23 * R3c;
    BLA::Matrix<3,3> R0c_inv = Inverse(R0c);

    // 6. Changement de repère du Jacobien
    BLA::Matrix<3,3> J_Tc = R0c_inv * J_T0;
    BLA::Matrix<3,3> J_Rc = R0c_inv * J_R0;

    // 7. Construction du Jacobien visuel (plan y/z + rotation autour x)
    BLA::Matrix<3,3> J;
    J(0,0) = J_Tc(1,0); J(0,1) = J_Tc(1,1); J(0,2) = J_Tc(1,2); // y
    J(1,0) = J_Tc(2,0); J(1,1) = J_Tc(2,1); J(1,2) = J_Tc(2,2); // z
    J(2,0) = J_Rc(0,0); J(2,1) = J_Rc(0,1); J(2,2) = J_Rc(0,2); // rotation x

    // 8. Inversion du Jacobien
    BLA::Matrix<3,3> J_inv = Inverse(J);

    // 9. Calcul de la vitesse articulaire (t1, t2, t3)
    BLA::Matrix<3,1> q_dot = J_inv * v;

    // 10. Intégration des angles articulaires
    t1 += q_dot(0) * dt;
    t2 += q_dot(1) * dt;
    t3 += q_dot(2) * dt;

    // 11. Contrôle de t4 (centrage de l’objet sur l’axe X caméra)
    t4 += centrageX * dt;
/*
    // Debug
    Serial.print("θ1: "); Serial.print(t1, 4);
    Serial.print("  θ2: "); Serial.print(t2, 4);
    Serial.print("  θ3: "); Serial.print(t3, 4);
    Serial.print("  θ4 (centrage X): "); Serial.println(t4, 4);
    */
}
