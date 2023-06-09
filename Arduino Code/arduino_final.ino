//libraries
#include <Servo.h>
#include "stdio.h"

//servo declarations
Servo dirControl;
Servo pwrControl;

//pin declarations
#define servoPin 8
#define motorPin 9

void setup() {
  Serial.begin(9600);
  dirControl.attach(servoPin);
  pwrControl.attach(motorPin);
}

void loop() {

  if(Serial.available() > 0) {
    int angle = Serial.parseInt();
    int power = Serial.parseInt();  
    dirControl.write(map(angle, -180, 180, 30, 150));
    pwrControl.write(map(power, 0, 100, 30, 150));
  }
  
}
