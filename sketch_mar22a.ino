#include <Arduino.h>

// Pin Definitions
// Motor A (Left Motor)
#define ENA 5    // PWM pin for left motor speed
#define IN1 18   // Left motor forward
#define IN2 19   // Left motor backward

// Motor B (Right Motor)
#define ENB 23   // PWM pin for right motor speed
#define IN3 21   // Right motor forward
#define IN4 22   // Right motor backward

// IR Sensors
#define IR_LEFT 33
#define IR_RIGHT 32

// LED for debugging
#define LED_PIN 2

// Speed settings (0-255)
#define BASE_SPEED 50
#define MAX_SPEED 100

// PID Constants
#define KP 0.5    // Proportional constant
#define KI 0.1    // Integral constant
#define KD 0.2    // Derivative constant

// PID Variables
float lastError = 0;
float integral = 0;

// Basic motor control functions
void stopRobot() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
    // Ensure speeds are within valid range (0-255)
    leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);
    
    // Set motor directions
    digitalWrite(IN1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN2, leftSpeed > 0 ? LOW : HIGH);
    digitalWrite(IN3, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(IN4, rightSpeed > 0 ? LOW : HIGH);
    
    // Set motor speeds
    analogWrite(ENA, abs(leftSpeed));
    analogWrite(ENB, abs(rightSpeed));
}

void setup() {
    // Initialize motor pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    // Initialize sensor pins
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);
    
    // Initialize LED
    pinMode(LED_PIN, OUTPUT);
    
    // Initial stop
    stopRobot();
    
    // Wait before starting
    delay(3000);
}

void loop() {
    // Read sensor values
    int leftSensor = digitalRead(IR_LEFT);
    int rightSensor = digitalRead(IR_RIGHT);
    
    // Calculate error (-100 to 100)
    float error = (rightSensor - leftSensor) * 50;  // Swapped leftSensor and rightSensor
    
    // Calculate PID terms
    float proportional = error * KP;
    integral += error * KI;
    float derivative = (error - lastError) * KD;
    
    // Calculate motor speeds
    float leftSpeed = BASE_SPEED + proportional + integral + derivative;
    float rightSpeed = BASE_SPEED - proportional - integral - derivative;
    
    // Update last error
    lastError = error;
    
    // Set motor speeds
    setMotorSpeed(leftSpeed, rightSpeed);
    
    // LED feedback
    if (abs(error) > 30) {  // If error is significant
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
    
    // Reset integral if error is small (to prevent integral windup)
    if (abs(error) < 10) {
        integral = 0;
    }
} 