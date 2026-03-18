#include <Arduino.h>
// Servo library removed — SG90 driven directly, no timer conflict

// ─── CONFIG ──────────────────────────────────────────────────────────────────
const uint8_t NO_OF_JOINTS = 4;
const uint8_t J1_IDX = 0;
const uint8_t J2_IDX = 1;
const uint8_t J3_IDX = 2;
const uint8_t Z_IDX = 3;

const uint8_t STEP_PINS[NO_OF_JOINTS] = {2, 4, 6, 8};
const uint8_t DIR_PINS[NO_OF_JOINTS] = {3, 5, 7, 9};
const uint8_t LIMIT_PINS[NO_OF_JOINTS] = {18, 19, 20, 21};

const uint8_t GRIPPER_PIN = 10;
const uint8_t SERVO_OPEN_ANGLE = 180;
const uint8_t SERVO_CLOSE_ANGLE = 60;
const uint8_t LED_PIN = 13;

const unsigned long HOME_SPEED_US = 900;
const float DEFAULT_MAX_SPEED = 1200.0f;
const float DEFAULT_ACCEL = 800.0f;
const float MIN_SPEED = 80.0f;
const unsigned long SELFTEST_BLINK_MS = 150;

// ── BRESENHAM TOGGLE ─────────────────────────────────────────────────────────
// 1 = axes synchronised (smooth coordinated motion)
// 0 = axes independent (good for testing each axis individually)
#define USE_BRESENHAM 1

// ─── DEMO MODE ───────────────────────────────────────────────────────────────
// 0 = all four axes back and forth continuously
// 1 = single axis only — set DEMO_AXIS below
// 2 = production serial command mode
#define DEMO_MODE 0
#define DEMO_AXIS J1_IDX // change to J2_IDX, J3_IDX, or Z_IDX
#define DEMO_STEPS 1000 // steps per direction
#define DEMO_PAUSE 300  // ms pause between moves

// 520 steps and 180 degree in steps to degree = 1 revolution = 360 degrees
// 350 steps and 180 degree in steps to degree = 1 revolution = 180 degrees
//  ─── UNIT CONVERSION ─────────────────────────────────────────────────────────
const float STEPS_PER_REV = 200.0f;
const float STEPS_PER_DEG = STEPS_PER_REV / 360.0f;
const float STEPS_PER_MM_Z = 25.0f;

inline long degToSteps(float deg) { return (long)(deg * STEPS_PER_DEG); }
inline long mmToSteps(float mm) { return (long)(mm * STEPS_PER_MM_Z); }

// ─── GLOBALS ─────────────────────────────────────────────────────────────────
char rxBuf[64];
uint8_t rxIdx = 0;
long jointPos[NO_OF_JOINTS] = {0, 0, 0, 0};
float maxSpeed = DEFAULT_MAX_SPEED;
float accel = DEFAULT_ACCEL;

// ─── ISR MOTION STATE ────────────────────────────────────────────────────────
struct AxisState
{
  volatile long stepsRemaining;
  volatile long accumulator;
  volatile long delta;
  volatile bool active;
};

volatile AxisState axes[NO_OF_JOINTS];
volatile long masterSteps = 0;
volatile long stepsDone = 0;
volatile float currentSpeed = MIN_SPEED;
volatile bool motionRunning = false;
volatile long rampSteps = 0;
volatile long decelStart = 0;

// ─── HOMING INTERRUPT FLAGS ──────────────────────────────────────────────────
volatile bool limitHit[NO_OF_JOINTS] = {false, false, false, false};

void LIMIT_ISR_0() { limitHit[0] = true; }
void LIMIT_ISR_1() { limitHit[1] = true; }
void LIMIT_ISR_2() { limitHit[2] = true; }
void LIMIT_ISR_3() { limitHit[3] = true; }

const uint8_t INT_NUMS[NO_OF_JOINTS] = {5, 4, 3, 2};
void (*const LIMIT_ISRS[NO_OF_JOINTS])() = {
    LIMIT_ISR_0, LIMIT_ISR_1, LIMIT_ISR_2, LIMIT_ISR_3};

// ─── PROTOTYPES ──────────────────────────────────────────────────────────────
void servoInit();
void servoWriteAngle(uint8_t angle);
void gripperOpen();
void gripperClose();
void stuck_switch_detection_onboot();
bool homeJoint(uint8_t j);
void homeAll();
void startMove(long tJ1, long tJ2, long tJ3, long tZ);
void waitForMove();
void rehomeZ(long layerZsteps);
void handleCommand(const char *cmd);
void setupTimer5();
void stopTimer5();

// ─── SERIAL PROTOCOL ─────────────────────────────────────────────────────────
//  HOME                 → OK:HOME
//  MOVE:j1,j2,j3,z_mm  → OK:MOVE
//  GRIP                 → OK:GRIP
//  RELEASE              → OK:RELEASE
//  REHOME_Z:z_mm        → OK:REHOME_Z
//  SET_SPEED:v          → OK:SPEED
//  SET_ACCEL:a          → OK:ACCEL
//  STATUS               → STATUS:IDLE | STATUS:RUNNING
//  (boot)               → READY
//  Errors  → ERR:<CODE>:<DETAIL>
//  Bad cmd → NACK:<REASON>

// ─── TIMER 5 ISR ─────────────────────────────────────────────────────────────
ISR(TIMER5_COMPA_vect)
{
  if (!motionRunning)
    return;

  long i = stepsDone;
  float v;

  if (i < rampSteps)
  {
    v = sqrtf(MIN_SPEED * MIN_SPEED + 2.0f * accel * (float)(i + 1));
    if (v > maxSpeed)
      v = maxSpeed;
  }
  else if (i >= decelStart)
  {
    long left = masterSteps - i;
    v = sqrtf(MIN_SPEED * MIN_SPEED + 2.0f * accel * (float)left);
    if (v > maxSpeed)
      v = maxSpeed;
  }
  else
  {
    v = maxSpeed;
  }
  currentSpeed = v;

  uint16_t ticks = (uint16_t)(2000000.0f / v);
  if (ticks < 200)
    ticks = 200;
  OCR5A = ticks;

#if USE_BRESENHAM
  for (uint8_t j = 0; j < NO_OF_JOINTS; j++)
  {
    if (!axes[j].active)
      continue;
    axes[j].accumulator += axes[j].delta;
    if (axes[j].accumulator >= masterSteps)
    {
      digitalWrite(STEP_PINS[j], HIGH);
      axes[j].accumulator -= masterSteps;
      axes[j].stepsRemaining--;
    }
  }
#else
  for (uint8_t j = 0; j < NO_OF_JOINTS; j++)
  {
    if (axes[j].active && axes[j].stepsRemaining > 0)
    {
      digitalWrite(STEP_PINS[j], HIGH);
      axes[j].stepsRemaining--;
    }
  }
#endif

  delayMicroseconds(2);
  for (uint8_t j = 0; j < NO_OF_JOINTS; j++)
    digitalWrite(STEP_PINS[j], LOW);

  stepsDone++;
  if (stepsDone >= masterSteps)
  {
    motionRunning = false;
    stopTimer5();
  }
}

// ─── TIMER 5 ─────────────────────────────────────────────────────────────────
void setupTimer5()
{
  TCCR5A = 0;
  TCCR5B = 0;
  TCNT5 = 0;
  TCCR5B |= (1 << WGM12) | (1 << CS11);
  OCR5A = (uint16_t)(2000000.0f / MIN_SPEED);
  TIMSK5 |= (1 << OCIE5A);
}

void stopTimer5()
{
  TIMSK5 &= ~(1 << OCIE5A);
  TCCR5B = 0;
}

// ─── COORDINATED MOVE ────────────────────────────────────────────────────────
void startMove(long tJ1, long tJ2, long tJ3, long tZ)
{
  long d[NO_OF_JOINTS];
  d[J1_IDX] = labs(tJ1 - jointPos[J1_IDX]);
  d[J2_IDX] = labs(tJ2 - jointPos[J2_IDX]);
  d[J3_IDX] = labs(tJ3 - jointPos[J3_IDX]);
  d[Z_IDX] = labs(tZ - jointPos[Z_IDX]);

  masterSteps = max(max(d[J1_IDX], d[J2_IDX]), max(d[J3_IDX], d[Z_IDX]));
  if (masterSteps == 0)
    return;

  digitalWrite(DIR_PINS[J1_IDX], (tJ1 >= jointPos[J1_IDX]) ? HIGH : LOW);
  digitalWrite(DIR_PINS[J2_IDX], (tJ2 >= jointPos[J2_IDX]) ? HIGH : LOW);
  digitalWrite(DIR_PINS[J3_IDX], (tJ3 >= jointPos[J3_IDX]) ? HIGH : LOW);
  digitalWrite(DIR_PINS[Z_IDX], (tZ >= jointPos[Z_IDX]) ? HIGH : LOW);

  rampSteps = (long)((maxSpeed * maxSpeed - MIN_SPEED * MIN_SPEED) / (2.0f * accel));
  if (rampSteps > masterSteps / 2)
    rampSteps = masterSteps / 2;
  decelStart = masterSteps - rampSteps;

  for (uint8_t j = 0; j < NO_OF_JOINTS; j++)
  {
    axes[j].delta = d[j];
    axes[j].stepsRemaining = d[j];
    axes[j].accumulator = 0;
    axes[j].active = (d[j] > 0);
  }

  stepsDone = 0;
  currentSpeed = MIN_SPEED;
  motionRunning = true;

  jointPos[J1_IDX] = tJ1;
  jointPos[J2_IDX] = tJ2;
  jointPos[J3_IDX] = tJ3;
  jointPos[Z_IDX] = tZ;

  setupTimer5();
}

void waitForMove()
{
  while (motionRunning)
  {
  }
}

// ─── LIMIT SWITCH ────────────────────────────────────────────────────────────
bool limitTriggered(uint8_t pin)
{
  if (digitalRead(pin) == HIGH)
  {
    delayMicroseconds(3000);
    return (digitalRead(pin) == HIGH);
  }
  return false;
}

// ─── STUCK SWITCH DETECTION ──────────────────────────────────────────────────
void stuck_switch_detection_onboot()
{
  for (uint8_t i = 0; i < NO_OF_JOINTS; ++i)
  {
    if (limitTriggered(LIMIT_PINS[i]))
    {
      Serial.print("ERR:SELFTEST:SWITCH_STUCK:J");
      Serial.println(i + 1);
      while (true)
      {
        digitalWrite(LED_PIN, HIGH);
        delay(SELFTEST_BLINK_MS);
        digitalWrite(LED_PIN, LOW);
        delay(SELFTEST_BLINK_MS);
      }
    }
  }
}


void test_limit_switches()
{
  for (uint8_t j = 0; j < NO_OF_JOINTS; ++j)
  {
    Serial.print("Testing limit switch for joint ");
    Serial.println(j + 1);
    Serial.println("Please trigger the switch now...");
    while (!limitTriggered(LIMIT_PINS[j]))
    {
      // Wait for the user to trigger the switch
    }
    Serial.println("Switch triggered successfully!");
  }
}

// ─── HOMING ──────────────────────────────────────────────────────────────────
bool homeJoint(uint8_t j)
{
  limitHit[j] = false;
  attachInterrupt(INT_NUMS[j], LIMIT_ISRS[j], RISING);

  if (limitTriggered(LIMIT_PINS[j]))
  {
    detachInterrupt(INT_NUMS[j]);
    Serial.print("ERR:HOMING:STUCK_AT_START:J");
    Serial.println(j + 1);
    return false;
  }

  digitalWrite(DIR_PINS[j], LOW);
  unsigned long start = millis();

  while (!limitHit[j])
  {
    digitalWrite(STEP_PINS[j], HIGH);
    delayMicroseconds(HOME_SPEED_US);
    digitalWrite(STEP_PINS[j], LOW);
    delayMicroseconds(HOME_SPEED_US);

    if (millis() - start > 4000)
    {
      detachInterrupt(INT_NUMS[j]);
      Serial.print("ERR:HOMING:TIMEOUT:J");
      Serial.println(j + 1);
      return false;
    }
  }

  detachInterrupt(INT_NUMS[j]);
  jointPos[j] = 0;
  return true;
}

void homeAll()
{
  for (uint8_t j = 0; j < NO_OF_JOINTS; ++j)
  {
    if (!homeJoint(j))
      return;
  }
}

// ─── Z REHOME ────────────────────────────────────────────────────────────────
void rehomeZ(long layerZsteps)
{
  if (!homeJoint(Z_IDX))
    return;
  if (layerZsteps > 0)
  {
    startMove(jointPos[J1_IDX], jointPos[J2_IDX],
              jointPos[J3_IDX], layerZsteps);
    waitForMove();
  }
}

// ─── GRIPPER ─────────────────────────────────────────────────────────────────
void servoWriteAngle(uint8_t angle)
{
  uint16_t pulseUs = 500 + (uint16_t)((uint32_t)angle * 2000 / 180);
  for (uint8_t i = 0; i < 30; i++)
  {
    digitalWrite(GRIPPER_PIN, HIGH);
    delayMicroseconds(pulseUs);
    digitalWrite(GRIPPER_PIN, LOW);
    delayMicroseconds(20000 - pulseUs);
  }
}

void servoInit()
{
  pinMode(GRIPPER_PIN, OUTPUT);
  digitalWrite(GRIPPER_PIN, LOW);
}

void gripperOpen() { servoWriteAngle(SERVO_OPEN_ANGLE); }
void gripperClose() { servoWriteAngle(SERVO_CLOSE_ANGLE); }

// ─── COMMAND PARSING ─────────────────────────────────────────────────────────
enum CommandType
{
  CMD_MOVE,
  CMD_GRIP,
  CMD_RELEASE,
  CMD_REHOME_Z,
  CMD_HOME_ALL,
  CMD_SET_SPEED,
  CMD_SET_ACCEL,
  CMD_STATUS,
  CMD_UNKNOWN
};

CommandType parseCommand(const char *cmd)
{
  if (strncmp(cmd, "MOVE:", 5) == 0)
    return CMD_MOVE;
  if (strcmp(cmd, "GRIP") == 0)
    return CMD_GRIP;
  if (strcmp(cmd, "RELEASE") == 0)
    return CMD_RELEASE;
  if (strncmp(cmd, "REHOME_Z:", 9) == 0)
    return CMD_REHOME_Z;
  if (strcmp(cmd, "HOME") == 0)
    return CMD_HOME_ALL;
  if (strncmp(cmd, "SET_SPEED:", 10) == 0)
    return CMD_SET_SPEED;
  if (strncmp(cmd, "SET_ACCEL:", 10) == 0)
    return CMD_SET_ACCEL;
  if (strcmp(cmd, "STATUS") == 0)
    return CMD_STATUS;
  return CMD_UNKNOWN;
}

// ─── COMMAND HANDLER ─────────────────────────────────────────────────────────
void handleCommand(const char *cmd)
{
  digitalWrite(LED_PIN, HIGH);

  switch (parseCommand(cmd))
  {

  case CMD_MOVE:
  {
    float j1, j2, j3, z;
    if (sscanf(cmd + 5, "%f,%f,%f,%f", &j1, &j2, &j3, &z) == 4)
    {
      startMove(degToSteps(j1), degToSteps(j2),
                degToSteps(j3), mmToSteps(z));
      waitForMove();
      Serial.println("OK:MOVE");
    }
    else
    {
      Serial.println("NACK:BAD_ARGS:MOVE");
    }
  }
  break;

  case CMD_GRIP:
    gripperClose();
    Serial.println("OK:GRIP");
    break;

  case CMD_RELEASE:
    gripperOpen();
    Serial.println("OK:RELEASE");
    break;

  case CMD_REHOME_Z:
  {
    float z_mm = atof(cmd + 9);
    rehomeZ(mmToSteps(z_mm));
    Serial.println("OK:REHOME_Z");
  }
  break;

  case CMD_HOME_ALL:
    homeAll();
    Serial.println("OK:HOME");
    break;

  case CMD_SET_SPEED:
  {
    float v = atof(cmd + 10);
    if (v > MIN_SPEED)
    {
      maxSpeed = v;
      Serial.println("OK:SPEED");
    }
    else
      Serial.println("NACK:SPEED_TOO_LOW");
  }
  break;

  case CMD_SET_ACCEL:
  {
    float a = atof(cmd + 10);
    if (a > 0)
    {
      accel = a;
      Serial.println("OK:ACCEL");
    }
    else
      Serial.println("NACK:ACCEL_INVALID");
  }
  break;

  case CMD_STATUS:
    Serial.println(motionRunning ? "STATUS:RUNNING" : "STATUS:IDLE");
    break;

  default:
    Serial.print("NACK:UNKNOWN:");
    Serial.println(cmd);
    break;
  }

  digitalWrite(LED_PIN, LOW);
}

// ─── SETUP ───────────────────────────────────────────────────────────────────
void setup()
{
  Serial.begin(115200);

  for (uint8_t i = 0; i < NO_OF_JOINTS; ++i)
  {
    pinMode(STEP_PINS[i], OUTPUT);
    digitalWrite(STEP_PINS[i], LOW);
    pinMode(DIR_PINS[i], OUTPUT);
    digitalWrite(DIR_PINS[i], LOW);
    pinMode(LIMIT_PINS[i], INPUT_PULLUP);
  }
  pinMode(LED_PIN, OUTPUT);

  servoInit();
  gripperOpen();

#if DEMO_MODE == 2
  //stuck_switch_detection_onboot();
  //homeAll();
#endif

  Serial.println("READY");
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
void loop()
{

#if DEMO_MODE == 0
  // ── All four axes back and forth continuously ─────────────────────────────
  //startMove(DEMO_STEPS, DEMO_STEPS, DEMO_STEPS, DEMO_STEPS);
 startMove(degToSteps(90), degToSteps(90),
             degToSteps(90), degToSteps(90));

  waitForMove();
  delay(DEMO_PAUSE);

  startMove(0, 0, 0, 0);
  waitForMove();
  delay(DEMO_PAUSE);

  gripperClose();
  delay(DEMO_PAUSE);
   gripperOpen();

#elif DEMO_MODE == 1
  // ── Single axis — only DEMO_AXIS moves, all others stay at 0 ─────────────
  long fwd[NO_OF_JOINTS] = {0, 0, 0, 0};
  fwd[DEMO_AXIS] = degToSteps(180); // or mmToSteps(50) for Z axis

  startMove(fwd[0], fwd[1], fwd[2], fwd[3]);
  waitForMove();
  delay(DEMO_PAUSE);

  startMove(0, 0, 0, 0);
  waitForMove();
  delay(DEMO_PAUSE);

#else
  // ── Production: serial command mode ──────────────────────────────────────
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || rxIdx >= 62)
    {
      rxBuf[rxIdx] = '\0';
      rxIdx = 0;
      if (!motionRunning)
        handleCommand(rxBuf);
    }
    else if (c != '\r')
    {
      rxBuf[rxIdx++] = c;
    }
  }
#endif
}
