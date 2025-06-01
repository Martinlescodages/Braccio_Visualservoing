#include <Braccio.h>
#include <Servo.h>  //TIMER 1
#include <math.h>
#include <Pixy2.h>
#include "VisualServoing.h"
#include "Jacobian.h"
#include "Object_tracking.h"
#include "moveArm.h"
using namespace BLA;


// Initialisation de la caméra Pixy2

int compt = 0; // Compteur pour l'acquisition d'images

int dt = 100; //ms

const float tolerance = 1.0;//en cm 

bool objectGrabbed = false;

bool PID_ON = false; // Activer ou désactiver le PID


// Variables d'angles (en radians) initialisées avec des valeurs de départ
float t1 = 0.261799, t2 = 2.140, t3 = 2.96706, t4= 0;

int Zc =40;

// Fonction pour commander le robot (déplacement par correction d'erreur)
volatile bool lect = false;

BLA::Matrix<18,1> f;                       // Vecteur visuel global

float ref_centerX = 160.00;                   // Centre X de l’objet

BLA::Matrix<8,1> Im_ref = {
  110,  193,
  208,  193,
  208, 110,
  110, 110
};

BLA::Matrix<3, 3> imgPoints ;
BLA::Matrix<3, 3> J;
Pixy2 pixy2;
void setup() {
    Braccio.begin();
    pixy2.init();
    Serial.begin(115200); 

    // Timer2 en mode CTC, interruption toutes les ~10ms
    TCCR2A = 0b00000010; // Mode CTC
    TCCR2B = 0b00000111; // Prescaler 1024
    OCR2A = 155;         // (16MHz/1024)/100Hz - 1 ≈ 155 pour 10ms
    TIMSK2 = 0b00000010; // Interruption sur OCR2A
}

ISR(TIMER2_COMPA_vect) {
    static int tick = 0;
    tick++;
    if (tick >= 10) { // 10 x 10ms = 100ms
        lect = true;
        tick = 0;
    }
}

int lostCount = 0; // à déclarer en global

// -------------------------------------------------------------------
// Boucle principale
void loop() {
    if (lect) {
        pixy2.ccc.getBlocks();
        Serial.print("Nombre de blocs détectés : ");
        Serial.println(pixy2.ccc.numBlocks);

        if (pixy2.ccc.numBlocks == 0) {
            lostCount++;
            if (lostCount >= 3) { // 3 cycles sans détection = objet perdu
                Serial.println("Aucun objet !");
                moveArm(t1, t2, t3, t4,dt);
            }
            lect = false;
            return;
        } else {
            lostCount = 0; // reset si objet détecté
        }

        // --- Suite du code servoing ---
        f = trackObject_robot(t1,t2,t3,Zc, Im_ref, ref_centerX);
        Serial.println("f:");
        Serial.print(f);
        Serial.println();
        if (PID_ON) {
            f = applyPIDToErrors(f, dt); // Appliquer le PID
        }

        BLA::Matrix<4,1> r = jvplus(f);
        computeArticularVelocityFromVisualServoing(r, t1, t2, t3, t4, dt / 1000.0);
        moveArm(t1, t2, t3, t4, dt );
        lect = false;
    }
}
