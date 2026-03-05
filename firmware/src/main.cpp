#include <Arduino.h>
#include <Servo.h>

// Stepper pins (DRV8825 instances 1..4)
const uint8_t NO_OF_JOINTS = 4;
const uint8_t STEP_PINS[NO_OF_JOINTS] = {2, 4, 6, 8};
const uint8_t DIR_PINS[NO_OF_JOINTS] = {3, 5, 7, 9};

// Limit switches (J1..J4)
const uint8_t LIMIT_PINS[NO_OF_JOINTS] = {30, 31, 32, 33};

// Servo
const uint8_t SERVO_PIN = 10;
const uint8_t SERVO_CENTER_ANGLE = 90;
Servo mechServo; 

// non-blocking stepper state
volatile unsigned long remSteps[NO_OF_JOINTS];
volatile bool dirState[NO_OF_JOINTS];
unsigned long stepIntervalUs = 1000;
unsigned long lastStepMicros = 0;

void gripperOpen();
void gripperClose();
void selfTest();
bool limitTriggered(int pin);
void stuck_switch_detection_onboot();
void homeAll();

void setup()
{
  // Initialize serial communication for debugging
  Serial.begin(115200);
  // Configure stepper pins
  for (uint8_t i = 0; i < NO_OF_JOINTS; ++i)
  {
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
  }

  // Configure limit switches as INPUT_PULLUP (NO wired to GND, COM to pin)
  for (uint8_t i = 0; i < NO_OF_JOINTS; ++i)
  {
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
  }

  // Attach servo and center it
  mechServo.attach(SERVO_PIN);
  mechServo.write(SERVO_CENTER_ANGLE);
  Serial.print("READY");
}

void loop()
{
  // put your main code here, to run repeatedly:
  gripperClose();
  delay(2000);
  gripperOpen();
}

// Tests gripper motion, axis movement, and that no home switch is already stuck.
void selfTest()
{
}

bool limitTriggered(int pin)
{
  if (digitalRead(pin) == HIGH)
  {                          // NC: HIGH = open = triggered
    delayMicroseconds(3000); // 3 ms debounce
    return digitalRead(pin) == HIGH;
  }
  return false;
}

void stuck_switch_detection_onboot()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    if (limitTriggered(LIMIT_PINS[i]))
    {
      Serial.print("ERR:SELFTEST:SWITCH_STUCK on J");
      Serial.println(i);
    
    }
  }
}

void homeAll()
{
  for (uint8_t i = 0; i < 4; ++i)
  {
    // Move towards the limit switch until triggered
    digitalWrite(DIR_PINS[i], LOW); // Assuming LOW moves towards the switch
    while (!limitTriggered(LIMIT_PINS[i]))
    {
      digitalWrite(STEP_PINS[i], HIGH);
      delayMicroseconds(stepIntervalUs);
      digitalWrite(STEP_PINS[i], LOW);
      delayMicroseconds(stepIntervalUs);
    }
    Serial.print("HOMED J");
    Serial.println(i);
  }
}

void gripperOpen()
{
  mechServo.write(SERVO_CENTER_ANGLE + 30); // Open by moving 30 degrees from center
}

void gripperClose()
{
  mechServo.write(SERVO_CENTER_ANGLE - 30); // Close by moving 30 degrees from center
}
