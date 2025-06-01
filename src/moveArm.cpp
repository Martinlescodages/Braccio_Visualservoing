#include "moveArm.h"
#include <Arduino.h> // Pour Serial.print

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

void moveArm(float t1, float t2, float t3, float t4,float dt) {

    float deg1 = t1 * 180.0 / PI;
    float deg2 = t2 * 180.0 / PI;
    float deg3 = t3 * 180.0 / PI;
    float deg4 = t4 * 180.0 / PI;

    // Convert velocity (or delta angle) to servo angles
    int baseAngle     = constrain(90 + deg1, 0, 180);  // t4: base rotation
    int shoulderAngle = constrain(90 + deg2, 15, 165);
    int elbowAngle    = constrain(90 + deg3, 0, 180);
    int wristAngle    = constrain(90 + deg4, 0, 180);

    // Command all servos, fixed wrist_rot and gripper for now
    Braccio.ServoMovement(dt, baseAngle, shoulderAngle, elbowAngle, wristAngle, 90, 73);
/*
    // Debug
    Serial.print("Base Angle: "); Serial.print(baseAngle);
    Serial.print("  Shoulder Angle: "); Serial.print(shoulderAngle);
    Serial.print("  Elbow Angle: "); Serial.print(elbowAngle);
    Serial.print("  Wrist Angle: "); Serial.println(wristAngle);
    */
}
