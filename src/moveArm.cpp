#include "moveArm.h"
#include <Arduino.h> // Pour Serial.print

Servo base;
Servo shoulder;
Servo elbow;
Servo wrist_ver;
Servo wrist_rot;
Servo gripper;

void moveArm(float t1, float t2, float t3, float t4) {
    float scaleFactor = 50.0;  // Scale velocity to angle variation

    // Convert velocity (or delta angle) to servo angles
    int baseAngle     = constrain(90 + t4 * scaleFactor, 0, 180);  // t4: base rotation
    int shoulderAngle = constrain(90 + t1 * scaleFactor, 15, 165);
    int elbowAngle    = constrain(90 + t2 * scaleFactor, 0, 180);
    int wristAngle    = constrain(90 + t3 * scaleFactor, 0, 180);

    // Command all servos, fixed wrist_rot and gripper for now
    Braccio.ServoMovement(10, baseAngle, shoulderAngle, elbowAngle, wristAngle, 90, 73);
/*
    // Debug
    Serial.print("Base Angle: "); Serial.print(baseAngle);
    Serial.print("  Shoulder Angle: "); Serial.print(shoulderAngle);
    Serial.print("  Elbow Angle: "); Serial.print(elbowAngle);
    Serial.print("  Wrist Angle: "); Serial.println(wristAngle);
    */
}
