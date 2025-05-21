#include "Object_tracking.h"
#include <Arduino.h> // Pour Serial.print

// Paramètres intrinsèques de la caméra (à calibrer)
const float fx = 320.0, fy = 200.0; // Focale en pixels
const float cx = 160.0, cy = 100.0;  // Centre de l'image

// Initialisation de la caméra Pixy2
Pixy2 pixy;

BLA::Matrix<18,1> BLA::trackObject_robot(float t1, float t2, float t3, float Zc, const BLA::Matrix<8,1>& ref, float ref_cx) {
    //pixel :
    const float Gx = 320.0;
    const float Gy = 200.0;
    const float Kp = 1;

    BLA::Matrix<18,1> f;  // 8 erreurs, 8 primitives, 1 erreur centrage, 1 référence centre

    if (pixy.ccc.numBlocks == 0) {
        Serial.println("Aucun objet détecté !");
        f.Fill(0); 
        return f;
    }

    uint16_t cx = pixy.ccc.blocks[0].m_x;
    uint16_t cy = pixy.ccc.blocks[0].m_y;
    uint16_t w  = pixy.ccc.blocks[0].m_width;
    uint16_t h  = pixy.ccc.blocks[0].m_height;

    float img_x[4] = {
        cx - w / 2.0,
        cx + w / 2.0,
        cx + w / 2.0,
        cx - w / 2.0
    };

    float img_y[4] = {
        cy + h / 2.0,
        cy + h / 2.0,
        cy - h / 2.0,
        cy - h / 2.0
    };

    float x_center = (img_x[0] + img_x[1] + img_x[2] + img_x[3]) / 4.0;

    // Transformation géométrique : repère robot
    BLA::Matrix<4,4> M01 = {
        cos(t1), -sin(t1), 0, 0.4 * cos(t1),
        sin(t1),  cos(t1), 0, 0.4 * sin(t1),
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    BLA::Matrix<4,4> M12 = {
        cos(t2), -sin(t2), 0, 0.4 * cos(t2),
        sin(t2),  cos(t2), 0, 0.4 * sin(t2),
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    BLA::Matrix<4,4> M23 = {
        cos(t3), -sin(t3), 0, 0.1 * cos(t3),
        sin(t3),  cos(t3), 0, 0.1 * sin(t3),
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    BLA::Matrix<4,4> M3c = {
        0, 0, 1, 0,
        0, -1, 0, 0,
        1, 0, 0, 0,
        0, 0, 0, 1
    };

    auto M0c = M01 * M12 * M23 * M3c;
    BLA::Matrix<4,4> M0c_inv = Inverse(M0c);

    // Object points in object frame in meters
    BLA::Matrix<4,1> P1 = {0.1, 0.1, 0, 1};
    BLA::Matrix<4,1> P2 = {0.1, -0.1, 0, 1};
    BLA::Matrix<4,1> P3 = {-0.1, -0.1, 0, 1};
    BLA::Matrix<4,1> P4 = {-0.1, 0.1, 0, 1};

    // Transform to camera frame
    auto p1m = M0c_inv * P1;
    auto p2m = M0c_inv * P2;
    auto p3m = M0c_inv * P3;
    auto p4m = M0c_inv * P4;

    // Project to image
    float p1x = Gx * p1m(0) / p1m(2);
    float p1y = Gy * p1m(1) / p1m(2);
    float p2x = Gx * p2m(0) / p2m(2);
    float p2y = Gy * p2m(1) / p2m(2);
    float p3x = Gx * p3m(0) / p3m(2);
    float p3y = Gy * p3m(1) / p3m(2);
    float p4x = Gx * p4m(0) / p4m(2);
    float p4y = Gy * p4m(1) / p4m(2);

    BLA::Matrix<8,1> Primitives = {p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y};



    // f[0:7] = erreurs image (coins), f[8:15] = primitives actuelles
    for (int i = 0; i < 8; i++) {
        f(i)     = (Primitives(i) - ref(i)) * Kp;
        f(i + 8) = Primitives(i);
    }

    // f[16] = erreur de centrage X
    f(16) = (ref_cx - x_center) * Kp;

    // f[17] = x_ref
    f(17) = ref_cx;

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
