#include <Arduino.h>

#define BLACK 1

#define WHITE 0

// Constants for pins and speeds

const int MOT1 = D8; // Left motor forward

const int MOT2 = D7; // Left motor backward

const int MOT3 = D6; // Right motor forward

const int MOT4 = D5; // Right motor backward

const int SENSOR_LEFT = D1;

const int SENSOR_RIGHT = D2;

// Speed settings

const int MAX_SPEED = 90;

const int INITIAL_SPEED = 67;

const int TURN_SPEED = 75; // Adjusted for tight corners

const int BACKWARD_SPEED = 65;

// State variables

int leftSensorState = WHITE;

int rightSensorState = WHITE;

bool isRunning = false;

// Timer variables for stuck detection

unsigned long lastSensorChange = 0;

int lastLeftState = WHITE;

int lastRightState = WHITE;

void turnRight();

void turnLeft();

void stop();

void forward();

void readSensors();

void spinRightUntilBlack();

void spinLeftUntilBlack();

void setup()

{

  pinMode(MOT1, OUTPUT);

  pinMode(MOT2, OUTPUT);

  pinMode(MOT3, OUTPUT);

  pinMode(MOT4, OUTPUT);

  pinMode(SENSOR_LEFT, INPUT_PULLUP);

  pinMode(SENSOR_RIGHT, INPUT_PULLUP);

  Serial.begin(9600);
}

void stop()

{

  analogWrite(MOT1, 0);

  analogWrite(MOT2, 0);

  analogWrite(MOT3, 0);

  analogWrite(MOT4, 0);
}

void forward()

{

  analogWrite(MOT1, INITIAL_SPEED);

  analogWrite(MOT2, 0);

  analogWrite(MOT3, INITIAL_SPEED);

  analogWrite(MOT4, 0);
}

void spinRightUntilBlack()

{

  // Spin the right motor until the right sensor detects black

  analogWrite(MOT1, 0); // Stop left motor

  analogWrite(MOT3, TURN_SPEED); // Spin right motor

  while (digitalRead(SENSOR_RIGHT) == WHITE)
  {

    delay(10); // Wait until the right sensor detects black
  }

  stop(); // Stop after detecting black
}

void spinLeftUntilBlack()

{

  // Spin the left motor until the left sensor detects black

  analogWrite(MOT3, 0); // Stop right motor

  analogWrite(MOT1, TURN_SPEED); // Spin left motor

  while (digitalRead(SENSOR_LEFT) == WHITE)
  {

    delay(10); // Wait until the left sensor detects black
  }

  stop(); // Stop after detecting black
}

void turnLeft()

{

  int speed = TURN_SPEED; // Start with TURN_SPEED

  unsigned long startTime = millis();

  // Right motor forward, left motor stopped

  analogWrite(MOT1, 0);

  analogWrite(MOT2, 0);

  analogWrite(MOT3, speed);

  analogWrite(MOT4, 0);

  // Wait until right sensor sees white

  while (digitalRead(SENSOR_RIGHT) == BLACK)

  {

    // Increase speed if taking too long

    if (millis() - startTime > 500 && speed < MAX_SPEED)

    {

      speed = min(MAX_SPEED, speed + 5);

      analogWrite(MOT3, speed);
    }

    delay(10);
  }

  analogWrite(MOT3, 0); // Stop turning
}

void turnRight()

{

  int speed = TURN_SPEED; // Start with TURN_SPEED

  unsigned long startTime = millis();

  // Left motor forward, right motor stopped

  analogWrite(MOT1, speed);

  analogWrite(MOT2, 0);

  analogWrite(MOT3, 0);

  analogWrite(MOT4, 0);

  // Wait until left sensor sees white

  while (digitalRead(SENSOR_LEFT) == BLACK)

  {

    // Increase speed if taking too long

    if (millis() - startTime > 500 && speed < MAX_SPEED)

    {

      speed = min(MAX_SPEED, speed + 5);

      analogWrite(MOT1, speed);
    }

    delay(10);
  }

  analogWrite(MOT1, 0); // Stop turning
}

void readSensors()

{

  leftSensorState = digitalRead(SENSOR_LEFT);

  rightSensorState = digitalRead(SENSOR_RIGHT);
}

void loop()

{

  readSensors();

  // Check if sensors have changed

  if (leftSensorState != lastLeftState || rightSensorState != lastRightState)

  {

    lastSensorChange = millis();

    lastLeftState = leftSensorState;

    lastRightState = rightSensorState;
  }

  else

  {

    // Check if stuck

    if (millis() - lastSensorChange > 2000)

    { // 2 seconds without change

      // Perform recovery

      // Turn 45 degrees at speed 95

      analogWrite(MOT1, 0);

      analogWrite(MOT3, 95); // Turn right at speed 95

      delay(500); // Adjust this delay to achieve 45 degrees

      stop(); // Stop after turning

      lastSensorChange = millis(); // Reset the timer
    }
  }

  if (leftSensorState == BLACK && rightSensorState == WHITE)

  {

    spinRightUntilBlack(); // Spin right until the right sensor detects black
  }

  else if (rightSensorState == BLACK && leftSensorState == WHITE)

  {

    spinLeftUntilBlack(); // Spin left until the left sensor detects black
  }

  else if (leftSensorState == WHITE && rightSensorState == WHITE)

  {

    forward();
  }

  else

  {

    // Both sensors on black - handle intersection

    forward();

    delay(500); // Cross the intersection
  }
}