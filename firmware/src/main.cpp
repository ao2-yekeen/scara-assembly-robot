#include <Arduino.h>

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
const float DEFAULT_ACCEL = 400.0f;
const float MIN_SPEED = 80.0f;
const unsigned long SELFTEST_BLINK_MS = 150;

// ─── JOINT LIMITS ─────────────────────────────────────────────────────────────
const float JOINT_MIN_DEG = -160.0f;
const float JOINT_MAX_DEG = 160.0f;
const float Z_MIN_MM = 0.0f;
const float Z_MAX_MM = 250.0f;

// ── BRESENHAM TOGGLE ─────────────────────────────────────────────────────────
#define USE_BRESENHAM 1

// ─── DEMO MODE ───────────────────────────────────────────────────────────────
#define DEMO_MODE 2
#define DEMO_PAUSE 300

// ─── UNIT CONVERSION ─────────────────────────────────────────────────────────
const float STEPS_PER_REV_J1 = 1800.0f;
const float STEPS_PER_REV_J2 = 400.0f;
const float STEPS_PER_REV_J3 = 200.0f;
const float STEPS_PER_DEG_ARR[NO_OF_JOINTS] = {
    STEPS_PER_REV_J1 / 360.0f,
    STEPS_PER_REV_J2 / 360.0f,
    STEPS_PER_REV_J3 / 360.0f,
    0.0f};
const float STEPS_PER_MM_Z = 25.0f;
const uint8_t Z_HOME_DIR = LOW;
const uint8_t Z_UP_DIR = HIGH;

inline long degToSteps(float deg, uint8_t joint) { return (long)(deg * STEPS_PER_DEG_ARR[joint]); }
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
volatile uint16_t masterSteps = 0;
volatile uint16_t stepsDone = 0;
volatile float currentSpeed = MIN_SPEED;
volatile bool motionRunning = false;
volatile uint16_t rampSteps = 0;
volatile uint16_t decelStart = 0;

// ─── HOMING INTERRUPT FLAGS ──────────────────────────────────────────────────
volatile bool limitHit[NO_OF_JOINTS] = {false, false, false, false};

void LIMIT_ISR_0() { limitHit[0] = true; }
void LIMIT_ISR_1() { limitHit[1] = true; }
void LIMIT_ISR_2() { limitHit[2] = true; }
void LIMIT_ISR_3() { limitHit[3] = true; }

const uint8_t INT_NUMS[NO_OF_JOINTS] = {5, 4, 3, 2};
void (*const LIMIT_ISRS[NO_OF_JOINTS])() = {LIMIT_ISR_0, LIMIT_ISR_1, LIMIT_ISR_2, LIMIT_ISR_3};

// ─── PROTOTYPES ──────────────────────────────────────────────────────────────
void servoInit();
void servoWriteAngle(uint8_t angle);
void gripperOpen();
void gripperClose();
void stuck_switch_detection_onboot();
bool homeJoint(uint8_t j);
void homeAll();
void startMoveJoints(long tJ1, long tJ2, long tJ3);
void moveZ(long tZ);
void startMove(long tJ1, long tJ2, long tJ3, long tZ);
void waitForMove();
void rehomeZ(long layerZsteps);
void handleCommand(const char *cmd);
void setupTimer5();
void stopTimer5();
bool validateMoveArgs(float j1_deg, float j2_deg, float j3_deg, float z_mm);

// ─── SERIAL PROTOCOL ─────────────────────────────────────────────────────────
//  HOME                 → OK:HOME
//  MOVE:j1,j2,j3,z_mm  → OK:MOVE    (angles deg signed ±160; z mm 0–250)
//  GRIP                 → OK:GRIP
//  RELEASE              → OK:RELEASE
//  REHOME_Z:z_mm        → OK:REHOME_Z
//  SET_SPEED:v          → OK:SPEED
//  SET_ACCEL:a          → OK:ACCEL
//  STATUS               → STATUS:IDLE | STATUS:RUNNING
//  (boot)               → READY

// ─── TIMER 5 ISR — rotational joints only ────────────────────────────────────
// Exactly as original, drives J1/J2/J3 with Bresenham + trapezoidal ramp.
// Z is NOT driven from here — it uses a separate blocking loop (moveZ).
ISR(TIMER5_COMPA_vect)
{
    if (!motionRunning)
        return;

    long i = (long)stepsDone;
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
        if (axes[j].active && axes[j].stepsRemaining > 0)
        {
            digitalWrite(STEP_PINS[j], HIGH);
            axes[j].stepsRemaining--;
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
    sei(); // guarantee global interrupts enabled — ISR cannot fire without this
}

void stopTimer5()
{
    TIMSK5 &= ~(1 << OCIE5A);
    TCCR5B = 0;
}

// ─── ROTATIONAL JOINTS MOVE (Timer5 ISR) ─────────────────────────────────────
void startMoveJoints(long tJ1, long tJ2, long tJ3)
{
    // --- SHORTEST-PATH ANGLE WRAPPING (fix dangerous long rotations) ---
    auto wrapTarget = [&](long target, long current, int jointIdx) -> long
    {
        long delta = target - current; // in steps

        // Convert delta to degrees for wrapping logic
        float degDelta = (float)delta / STEPS_PER_DEG_ARR[jointIdx];

        if (degDelta > 180.0f)
            delta -= (long)(360.0f * STEPS_PER_DEG_ARR[jointIdx]);
        else if (degDelta < -180.0f)
            delta += (long)(360.0f * STEPS_PER_DEG_ARR[jointIdx]);

        return current + delta; // new wrapped target
    };

    // Apply wrapping to each rotational joint
    tJ1 = wrapTarget(tJ1, jointPos[J1_IDX], J1_IDX);
    tJ2 = wrapTarget(tJ2, jointPos[J2_IDX], J2_IDX);
    tJ3 = wrapTarget(tJ3, jointPos[J3_IDX], J3_IDX);
    // Z is linear → no wrapping needed


    long d[NO_OF_JOINTS];
    d[J1_IDX] = labs(tJ1 - jointPos[J1_IDX]);
    d[J2_IDX] = labs(tJ2 - jointPos[J2_IDX]);
    d[J3_IDX] = labs(tJ3 - jointPos[J3_IDX]);
    d[Z_IDX] = 0; // Z never driven from ISR

    masterSteps = (uint16_t)max(max(d[J1_IDX], d[J2_IDX]), d[J3_IDX]);
    if (masterSteps == 0)
        return;

    digitalWrite(DIR_PINS[J1_IDX], (tJ1 >= jointPos[J1_IDX]) ? HIGH : LOW);
    digitalWrite(DIR_PINS[J2_IDX], (tJ2 >= jointPos[J2_IDX]) ? HIGH : LOW);
    digitalWrite(DIR_PINS[J3_IDX], (tJ3 >= jointPos[J3_IDX]) ? HIGH : LOW);

    delayMicroseconds(10); // DIR setup time

    long ramp = (long)((maxSpeed * maxSpeed - MIN_SPEED * MIN_SPEED) / (2.0f * accel));
    if (ramp > (long)masterSteps / 2)
        ramp = (long)masterSteps / 2;
    if (ramp < 1)
        ramp = 1;
    rampSteps = (uint16_t)ramp;
    decelStart = (uint16_t)((long)masterSteps - ramp);
    if (decelStart < rampSteps)
        decelStart = rampSteps;

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

    setupTimer5();
}

// ─── Z AXIS MOVE (blocking, constant speed) ───────────────────────────────────
// Identical mechanism to the working test_z_movement() — pure bit-bang, no ISR.
void moveZ(long tZ)
{
    long steps = labs(tZ - jointPos[Z_IDX]);
    if (steps == 0)
        return;

    digitalWrite(DIR_PINS[Z_IDX], (tZ >= jointPos[Z_IDX]) ? Z_UP_DIR : Z_HOME_DIR);
    delayMicroseconds(10); // DIR setup time

    for (long s = 0; s < steps; s++)
    {
        digitalWrite(STEP_PINS[Z_IDX], HIGH);
        delayMicroseconds(HOME_SPEED_US);
        digitalWrite(STEP_PINS[Z_IDX], LOW);
        delayMicroseconds(HOME_SPEED_US);
    }

    jointPos[Z_IDX] = tZ;
}

// ─── COMBINED MOVE ────────────────────────────────────────────────────────────
// Joints move via Timer5 ISR (ramp), Z moves after via blocking loop.
void startMove(long tJ1, long tJ2, long tJ3, long tZ)
{
    // Move rotational joints first (ISR-driven, non-blocking until waitForMove)
    startMoveJoints(tJ1, tJ2, tJ3);
    waitForMove();

    // Then move Z (blocking, constant speed)
    moveZ(tZ);
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

// ─── HOMING ──────────────────────────────────────────────────────────────────
bool homeJoint(uint8_t j)
{
    limitHit[j] = false;
    digitalWrite(DIR_PINS[j], LOW);
    delayMicroseconds(10);

    unsigned long start = millis();
    const unsigned long timeout = 6000;
    const unsigned long RAMP_START_US = 3000;
    const int RAMP_STEPS = 40;
    int stepCount = 0;

    while (true)
    {
        if (limitTriggered(LIMIT_PINS[j]))
        {
            limitHit[j] = true;
            break;
        }

        unsigned long stepUs = HOME_SPEED_US;
        if (stepCount < RAMP_STEPS)
            stepUs = RAMP_START_US - ((RAMP_START_US - HOME_SPEED_US) * stepCount / RAMP_STEPS);

        digitalWrite(STEP_PINS[j], HIGH);
        delayMicroseconds(stepUs);
        digitalWrite(STEP_PINS[j], LOW);
        delayMicroseconds(stepUs);
        stepCount++;

        if (millis() - start > timeout)
        {
            Serial.print("ERR:HOMING:TIMEOUT:J");
            Serial.println(j + 1);
            return false;
        }
    }

    jointPos[j] = 0;
    Serial.print("HOMED: Joint ");
    Serial.println(j + 1);
    return true;
}

void homeAll()
{
    for (uint8_t j = 0; j < NO_OF_JOINTS; ++j)
        if (!homeJoint(j))
            return;
}

// ─── Z REHOME ────────────────────────────────────────────────────────────────
void rehomeZ(long layerZsteps)
{
    if (!homeJoint(Z_IDX))
        return;
    if (layerZsteps > 0)
        moveZ(layerZsteps);
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

// ─── INPUT VALIDATION ────────────────────────────────────────────────────────
bool validateMoveArgs(float j1_deg, float j2_deg, float j3_deg, float z_mm)
{
    struct
    {
        float val;
        const char *name;
        float lo;
        float hi;
        const char *unit;
    } checks[] = {
        {j1_deg, "J1", JOINT_MIN_DEG, JOINT_MAX_DEG, "deg"},
        {j2_deg, "J2", JOINT_MIN_DEG, JOINT_MAX_DEG, "deg"},
        {j3_deg, "J3", JOINT_MIN_DEG, JOINT_MAX_DEG, "deg"},
        {z_mm, "Z", Z_MIN_MM, Z_MAX_MM, "mm"},
    };
    for (uint8_t i = 0; i < 4; i++)
    {
        if (checks[i].val < checks[i].lo || checks[i].val > checks[i].hi)
        {
            Serial.print("ERR:LIMIT:");
            Serial.print(checks[i].name);
            Serial.print("_OUT_OF_RANGE:");
            Serial.print(checks[i].val, 1);
            Serial.print(" (allowed ");
            Serial.print(checks[i].lo, 0);
            Serial.print(" to ");
            Serial.print(checks[i].hi, 0);
            Serial.print(" ");
            Serial.print(checks[i].unit);
            Serial.println(")");
            return false;
        }
    }
    return true;
}

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
            if (!validateMoveArgs(j1, j2, j3, z))
                break;
            startMove(degToSteps(j1, J1_IDX), degToSteps(j2, J2_IDX),
                      degToSteps(j3, J3_IDX), mmToSteps(z));
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
        if (z_mm < Z_MIN_MM || z_mm > Z_MAX_MM)
        {
            Serial.print("ERR:LIMIT:REHOME_Z_OUT_OF_RANGE:");
            Serial.print(z_mm, 1);
            Serial.print(" (allowed ");
            Serial.print(Z_MIN_MM, 0);
            Serial.print(" to ");
            Serial.print(Z_MAX_MM, 0);
            Serial.println(" mm)");
            break;
        }
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

    // homeJoint(J1_IDX);

    homeJoint(Z_IDX);
    // ── Test moves — uncomment one at a time ─────────────────────────────────
    // startMoveJoints(degToSteps(180.0f, J1_IDX), 0, 0);  // J1 only, 90 deg
    startMove(0, 0, 0, mmToSteps(100));
    waitForMove();

    Serial.println("READY");
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
void loop()
{
#if DEMO_MODE == 0
    startMove(degToSteps(90, J1_IDX), degToSteps(90, J2_IDX),
              degToSteps(90, J3_IDX), mmToSteps(90));
    waitForMove();
    delay(DEMO_PAUSE);
    startMove(0, 0, 0, 0);
    waitForMove();
    delay(DEMO_PAUSE);
    gripperClose();
    delay(DEMO_PAUSE);
    gripperOpen();

#elif DEMO_MODE == 1
    // startMoveJoints(degToSteps(90, J1_IDX), 0, 0); waitForMove(); delay(DEMO_PAUSE);
    // startMoveJoints(0, 0, 0);                       waitForMove(); delay(DEMO_PAUSE);

#else
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
