#include <Servo.h>

// Motori
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

static int throttle = 1120;
const int minThrottle = 1120;
const int maxThrottle = 1500;