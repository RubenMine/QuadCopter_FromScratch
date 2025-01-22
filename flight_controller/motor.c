#include "header/motor.h"

void init_esc(){
  motor1.attach(3);
  motor2.attach(5);
  motor3.attach(6);
  motor4.attach(9);

  Serial.println("[DEBUG] Calibrazione ESC Completata!");
  // Segnale massimo (2000 µs) per iniziare la calibrazione
  /*
  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
  Serial.println("Velocità Max Calibrazione");
  delay(3000); // Attesa per calibrazione
  */
  
  // Segnale minimo (1000 µs) per completare la calibrazione
  motor1.writeMicroseconds(1000);
  motor2.writeMicroseconds(1000);
  motor3.writeMicroseconds(1000);
  motor4.writeMicroseconds(1000);
  delay(10000); // Attesa per completare

  Serial.println("[DEBUG] Calibrazione ESC Completata!");
}

void motor_power() { 
  motor1.writeMicroseconds(throttle);
  motor2.writeMicroseconds(throttle);
  motor3.writeMicroseconds(throttle);
  motor4.writeMicroseconds(throttle);
  Serial.println("[DEBUG] Motor Throttle: %d", throttle);
}
