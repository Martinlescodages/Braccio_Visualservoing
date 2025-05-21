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


//delay de la boucle
int dt = 100; //ms

const float tolerance = 1.0;//en cm 

bool objectGrabbed = false;


// Variables d'angles (en radians) initialisées avec des valeurs de départ
float t1 = 0.261799, t2 = 3.140, t3 = 2.96706, t4= 0;

int Zc =40;

// Fonction pour commander le robot (déplacement par correction d'erreur)
bool lect = false;

BLA::Matrix<18,1> f;                       // Vecteur visuel global

int ref_centerX = 160;                   // Centre X de l’objet

BLA::Matrix<8,1> Im_ref = {
    -40,  25,
     40,  25,
     40, -25,
    -40, -25
};

BLA::Matrix<3, 3> imgPoints ;
BLA::Matrix<3, 3> J;
Pixy2 pixy2;
void setup() {
    Braccio.begin();
    pixy2.init();
    Serial.begin(9600); 
    

    TCCR0A = 0b00000000;
    TCCR0B = 0b00000011; //  prescaler 64
    TIMSK0 = 0b00000001; // Activer interruption
}


ISR(TIMER0_COMPA_vect) {
    TCNT0=40536;
    lect = true; //acquisition
}

// -------------------------------------------------------------------
// Boucle principale
void loop() {
    // Vérifie si un objet est détecté au moment de l'acquisition
    if (lect) {
        pixy2.ccc.getBlocks();
        if (pixy2.ccc.numBlocks == 0) {
            Serial.println("Aucun objet détecté !");
            moveArm(t1, t2, t3, t4);
            return;
        }

        // Acquisition de f toutes les 100 ms
        f = trackObject_robot(t1, t2, t3, Zc, Im_ref, ref_centerX);
        lect = false;  // Réinitialisation du bool
    }

    // --- Calcul et mise à jour continue ---
    BLA::Matrix<4,1> r = jvplus(f); // vy, vz, dθ, erreur_centreX
    computeArticularVelocityFromVisualServoing(r, t1, t2, t3, t4, dt / 1000.0);
    moveArm(t1, t2, t3, t4);

}

