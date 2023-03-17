#include "stdio.h"
#include <Servo.h>

//Servo motor;
Servo steer;

//pin declarations
#define servoPin 9
#define motorPin 8
#define encoderPin A1

void setup() {
  Serial.begin(9600);
//motor.attach(8);
  steer.attach(servoPin);
  pinMode(motorPin, OUTPUT);
  pinMode(encoderPin, INPUT);
}

void steering(angle) {
  servoPosition = map(angle, -60, 60, -60, 60);
  servoPosition = constrain(servoPosition, -60, 60);
  steer.write(servoPosition);
}

void motorPower(velocity)
  dcSpeed = map(velocity, 0, 255, 0, 255);
  dcSpeed = constrain(dcSpeed, 0, 255);
  analogWrite(motorPin, dcSpeed);
}
  
void loop() {
  angle, velocity = data.read();
  steering(angle);
  motorPower(velocity);
}
