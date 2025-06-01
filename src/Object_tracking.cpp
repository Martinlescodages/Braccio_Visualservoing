#include "Object_tracking.h"
#include <Arduino.h> // Pour Serial.print

Pixy2 pixy;


// ---------------------------------------------------------------------------
// PID controller for visual servoing (4D error vector)
// e(t) = [vy_err, vz_err, dθ_err, centerX_err] (typically from jvplus output)

#include <BasicLinearAlgebra.h>
using namespace BLA;

// PID Gains 
float Kp = 0.8;
float Ki = 5;
float Kd = 0.4;

// PID state (erreur intégrale)
BLA::Matrix<8,1> integral_error = {0};  // à placer en global ou static
BLA::Matrix<8,1> error_prev = {0};      // pour le calcul de la dérivée

float integral_cx = 0;
float previous_cx = 0;

BLA::Matrix<18,1> applyPIDToErrors(const BLA::Matrix<18,1>& f, float dt) {
    static float Kp = 0.8;
    static float Ki = 5;
    static float Kd = 0.4;

    BLA::Matrix<18,1> f_corrected = f;

    for (int i = 0; i < 8; i++) {
        float e = f(i);  // erreur actuelle
        integral_error(i) += e * dt;
        float de = (e - error_prev(i)) / dt;
        f_corrected(i) = Kp * e + Ki * integral_error(i)+ Kd * de;
        error_prev(i) = e;  // mise à jour de l'erreur précédente
    }

    // Optionnel : PID aussi sur f(16) (centrage X)
    static float integral_cx = 0;
    integral_cx += f(16) * dt;
    float derivative_cx = (f(16) - previous_cx) / dt;
    f_corrected(16) = Kp * f(16) + Ki * integral_cx + Kd * derivative_cx;
    previous_cx = f_corrected(16);

    return f_corrected;
}

BLA::Matrix<18,1> BLA::trackObject_robot(float t1, float t2, float t3, float Zc, const BLA::Matrix<8,1>& ref, float ref_cx) {
    //pixel :
    BLA::Matrix<18,1> f;

    pixy.ccc.getBlocks();
    if (pixy.ccc.numBlocks == 0) {
        Serial.println("Aucun objet détecté !");
        f.Fill(0);
        return f;
    }

    uint16_t cx = pixy.ccc.blocks[0].m_x;
    uint16_t cy = pixy.ccc.blocks[0].m_y;
    uint16_t w  = pixy.ccc.blocks[0].m_width;
    uint16_t h  = pixy.ccc.blocks[0].m_height;

    // Calcul des coins image (ordre : HG, HD, BD, BG)
    float img_x[4] = {
        cx - w / 2.0,  // HG
        cx + w / 2.0,  // HD
        cx + w / 2.0,  // BD
        cx - w / 2.0   // BG
    };

    float img_y[4] = {
        cy + h / 2.0,
        cy + h / 2.0,
        cy - h / 2.0,
        cy - h / 2.0
    };

    // Calcul du centre de l’objet en X
    float x_center = (img_x[0] + img_x[1] + img_x[2] + img_x[3]) / 4.0;

    // Construction de Primitives et erreurs
    BLA::Matrix<8,1> Primitives;
    for (int i = 0; i < 4; i++) {
        Primitives(2*i)     = img_x[i];
        Primitives(2*i + 1) = img_y[i];

        // Calcul des erreurs pondérées
        f(2*i)     = (img_x[i] - ref(2*i))     * Kp;
        f(2*i + 1) = (img_y[i] - ref(2*i + 1)) * Kp;
    }

    // Ajout des primitives observées
    for (int i = 0; i < 8; i++) {
        f(i + 8) = Primitives(i);
    }

    // Erreur de centrage en X
    f(16) = (x_center - ref_cx) * Kp;

    // Référence du centre
    f(17) = ref_cx;

    // Debug
    Serial.println("Primitives observées :");
    for (int i = 0; i < 8; i += 2) {
        Serial.print("Point "); Serial.print(i/2 + 1); Serial.print(" : ");
        Serial.print("x = "); Serial.print(Primitives(i));
        Serial.print(" ; y = "); Serial.println(Primitives(i+1));
    }

    Serial.print("Erreur de centrage X : ");
    Serial.println(f(16));

    return f;
}

bool isObjectWithinTolerance(const BLA::Matrix<8,1>& points_robot, const BLA::Matrix<8,1>& ref, float tolerance) {
    float weightedError = 0.0;
    float totalWeight = 0.0;

    // Poids pour chaque point (vous pouvez ajuster ces poids selon l'importance de chaque point)
    float weights[4] = {1.0, 1.0, 1.0, 1.0};

    // Calculer la moyenne pondérée des erreurs
    for (int i = 0; i < 4; i++) {
        float x_error = abs(points_robot(2*i) - ref(2*i));
        float y_error = abs(points_robot(2*i + 1) - ref(2*i + 1));

        weightedError += (x_error + y_error) * weights[i];
        totalWeight += 2 * weights[i]; // 2 pour x et y
    }

    // Calculer la moyenne pondérée
    float averageWeightedError = weightedError / totalWeight;

    // Vérifier si la moyenne pondérée des erreurs est inférieure à la tolérance
    if (averageWeightedError < tolerance) {
        return true;
    } else {
        return false;
    }
}
