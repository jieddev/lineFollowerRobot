#include <Arduino.h>

#define BLACK LOW
#define WHITE HIGH

#define ENA  5   // PWM for Motor A speed
#define IN1  18  // Motor A forward
#define IN2  19  // Motor A backward

#define ENB  23   // PWM for Motor B speed
#define IN3  21  // Motor B forward
#define IN4  22  // Motor B backward

#define LED_PIN 2   // D2 LED on ESP32

#define MIN_SPEED 45
#define MAX_SPEED 70

#define IR_SENSOR_LEFT 33
#define IR_SENSOR_RIGHT 32

unsigned long lastSensorChange = 0;

int lastLeftState = WHITE;

int lastRightState = WHITE;

void normalSpeed() {
  int speed = applySpeedConstraint(69);
  analogWrite(ENA, speed);  // Set speed for Motor A (0-255)
  analogWrite(ENB, speed);  // Set speed for Motor B (0-255)
}

void turnSpeed() {
  int speed = applySpeedConstraint(46); 
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void stopValkiri() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void forwardValkiri() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

}

void turnLeftValkiri() {

  // Right Motor Forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  // Left Motor Stops
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  digitalWrite(LED_PIN, HIGH);

}

void turnRightValkiri() {

  // Right Motor Stops
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  // Left Motor Forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  digitalWrite(LED_PIN, HIGH);

}

int applySpeedConstraint(int speed) {
  if (speed == 0) return 0;

  return constrain(speed, MIN_SPEED, MAX_SPEED);
}

void setup() {
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    pinMode(LED_PIN, OUTPUT);

    pinMode(IR_SENSOR_LEFT, INPUT);
    pinMode(IR_SENSOR_RIGHT, INPUT);

    stopValkiri();

    delay(3000);
}

void loop() {
  int leftSensorValue = digitalRead(IR_SENSOR_LEFT);
  int rightSensorValue = digitalRead(IR_SENSOR_RIGHT);

  if (leftSensorValue != lastLeftState || rightSensorValue != lastRightState) {
    lastSensorChange = millis();

    lastLeftState = leftSensorValue;

    lastRightState = rightSensorValue;
  } else {
      if (millis() - lastSensorChange > 3500) {
        if (lastLeftState == BLACK && lastRightState == WHITE) {
          normalSpeed();
          turnLeftValkiri();
        } 
        if (lastLeftState == WHITE && lastRightState == BLACK) {
          normalSpeed();
          turnRightValkiri();
        }
      }
  }

  if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    normalSpeed();
    delay(10);

    turnSpeed();
    turnRightValkiri();
    digitalWrite(LED_PIN, HIGH);

  } else if (leftSensorValue == LOW && rightSensorValue == HIGH) {
    normalSpeed();
    delay(10);
    
    turnSpeed();
    turnLeftValkiri();
    digitalWrite(LED_PIN, HIGH);
    
  } else if (leftSensorValue == LOW && rightSensorValue == LOW) { 
    normalSpeed();
    forwardValkiri();
    digitalWrite(LED_PIN, LOW);
    
     
  }


  
}