#include <Arduino.h>
#include <Servo.h>

// Stepper pins (DRV8825 instances 1..4)
const uint8_t STEP_PINS[4] = {2, 4, 6, 8};
const uint8_t DIR_PINS[4]  = {3, 5, 7, 9};

// Limit switches (COM -> D30..D33, NO -> GND) -> use INPUT_PULLUP (active LOW)
const uint8_t LIMIT_PINS[4] = {30, 31, 32, 33};

// Servo
const uint8_t SERVO_PIN = 10;
Servo mechServo;

// non-blocking stepper state
volatile unsigned long remSteps[4];
volatile bool dirState[4];
unsigned long stepIntervalUs = 1000;
unsigned long lastStepMicros = 0;

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Configure stepper pins
  for (uint8_t i = 0; i < 4; ++i)
  {
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
  }

  // Configure limit switches as INPUT_PULLUP (NO wired to GND, COM to pin)
  for (uint8_t i = 0; i < 4; ++i)
  {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
  }

  // Attach servo and center it
  mechServo.attach(SERVO_PIN);
  mechServo.write(90);
  Serial.print("READY");
}

void loop()
{
  // put your main code here, to run repeatedly:
}

