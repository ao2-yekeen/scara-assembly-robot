#include <Arduino.h>

// ─── PER-JOINT CONFIGURATION ─────────────────────────────────────────────────
// All properties for one joint in one place.
// To change any joint: edit only its entry in JOINTS[] below.

struct JointConfig
{
    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t limitPin;
    uint8_t intNum;            // hardware interrupt number for limit switch
    float stepsPerUnit;        // steps/deg for rotational, steps/mm for Z
    float minPos;              // minimum position (deg or mm)
    float maxPos;              // maximum position (deg or mm)
    unsigned long homeSpeedUs; // half-period delay during homing (µs)
    bool isLinear;             // true = Z axis (mm), false = rotational (deg)
};

// ─── JOINT TABLE ─────────────────────────────────────────────────────────────
// Edit values here only — nothing else needs changing when hardware changes.
//
//           step  dir  lim  int   steps/unit  min     max    homeUs  linear
const JointConfig JOINTS[] = {
    /* J1 */ {2, 3, 21, 5, 1800.0f / 360.0f, 0.0f, 160.0f, 2000, false},
    /* J2 */ {4, 5, 18, 4, 400.0f / 360.0f, 0.0f, 330.0f, 5000, false},
    /* J3 */ {6, 7, 19, 3, 200.0f / 360.0f, 0.0f, 330.0f, 5000, false},
    /* Z  */ {8, 9, 20, 2, 25.0f, 0.0f, 300.0f, 800, true},
};

const uint8_t NO_OF_JOINTS = sizeof(JOINTS) / sizeof(JOINTS[0]);
const uint8_t J1_IDX = 0;
const uint8_t J2_IDX = 1;
const uint8_t J3_IDX = 2;
const uint8_t Z_IDX = 3;

// ─── OTHER CONFIG ─────────────────────────────────────────────────────────────
const uint8_t GRIPPER_PIN = 10;
const uint8_t SERVO_OPEN_ANGLE = 40;
const uint8_t SERVO_CLOSE_ANGLE = 100;
const uint8_t LED_PIN = 13;

const float DEFAULT_MAX_SPEED = 1200.0f;
const float DEFAULT_ACCEL = 400.0f;
const float MIN_SPEED = 80.0f;
const unsigned long SELFTEST_BLINK_MS = 150;

// ── BRESENHAM TOGGLE ─────────────────────────────────────────────────────────
#define USE_BRESENHAM 1

// ─── DEMO MODE ───────────────────────────────────────────────────────────────
#define DEMO_MODE 2
#define DEMO_PAUSE 100

// ─── UNIT CONVERSION ─────────────────────────────────────────────────────────
// Reads from JOINTS[] — no separate arrays needed.

inline long toSteps(float value, uint8_t j)
{
    return (long)(value * JOINTS[j].stepsPerUnit);
}

inline long degToSteps(float deg, uint8_t j)
{
    return toSteps(deg, j);
}

inline long mmToSteps(float mm)
{
    return toSteps(mm, Z_IDX);
}

// ─── DIRECTION CONSTANTS FOR Z ────────────────────────────────────────────────
const uint8_t Z_HOME_DIR = LOW;
const uint8_t Z_UP_DIR = HIGH;

// ─── GLOBALS ─────────────────────────────────────────────────────────────────
char rxBuf[64];
uint8_t rxIdx = 0;
long jointPos[4] = {0, 0, 0, 0}; // current position in steps
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

volatile AxisState axes[4];
volatile uint16_t masterSteps = 0;
volatile uint16_t stepsDone = 0;
volatile float currentSpeed = MIN_SPEED;
volatile bool motionRunning = false;
volatile uint16_t rampSteps = 0;
volatile uint16_t decelStart = 0;

// ─── HOMING INTERRUPT FLAGS ──────────────────────────────────────────────────
volatile bool limitHit[4] = {false, false, false, false};

void LIMIT_ISR_0() { limitHit[0] = true; }
void LIMIT_ISR_1() { limitHit[1] = true; }
void LIMIT_ISR_2() { limitHit[2] = true; }
void LIMIT_ISR_3() { limitHit[3] = true; }

void (*const LIMIT_ISRS[4])() = {
    LIMIT_ISR_0, LIMIT_ISR_1, LIMIT_ISR_2, LIMIT_ISR_3};

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
bool validateMoveArgs(float j1, float j2, float j3, float z);

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



// ─── TIMER 5 ISR ─────────────────────────────────────────────────────────────
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
            digitalWrite(JOINTS[j].stepPin, HIGH);
            axes[j].accumulator -= masterSteps;
            axes[j].stepsRemaining--;
        }
    }
#else
    for (uint8_t j = 0; j < NO_OF_JOINTS; j++)
        if (axes[j].active && axes[j].stepsRemaining > 0)
        {
            digitalWrite(JOINTS[j].stepPin, HIGH);
            axes[j].stepsRemaining--;
        }
#endif

    delayMicroseconds(2);
    for (uint8_t j = 0; j < NO_OF_JOINTS; j++)
        digitalWrite(JOINTS[j].stepPin, LOW);

    stepsDone++;
    if (stepsDone >= masterSteps)
    {
        motionRunning = false;
        stopTimer5();
    }
}

// ─── SERVO ISR — Timer 4 two-state ───────────────────────────────────────────
volatile uint16_t servoTargetPulseUs = 1500;
volatile bool     servoPulseHigh     = false;
static   uint8_t  servoCurrentAngle  = 90;

ISR(TIMER4_COMPA_vect)
{
    if (!servoPulseHigh)
    {
        // Start of pulse — pull HIGH, set timer to pulse width
        digitalWrite(GRIPPER_PIN, HIGH);
        OCR4A         = servoTargetPulseUs * 2;  // ticks at 0.5µs each (prescaler 8)
        servoPulseHigh = true;
    }
    else
    {
        // End of pulse — pull LOW, reset timer for 20ms period
        digitalWrite(GRIPPER_PIN, LOW);
        OCR4A          = 40000 - servoTargetPulseUs * 2;  // remainder of 20ms
        servoPulseHigh = false;
    }
}

void setupTimer4()
{
    TCCR4A = 0;
    TCCR4B = 0;
    TCNT4  = 0;

    // CTC mode, prescaler 8
    // 16MHz / 8 = 2MHz → 0.5µs per tick
    // 20ms = 40000 ticks total
    // Split: pulse ticks HIGH + (40000 - pulse ticks) LOW
    TCCR4B |= (1 << WGM12) | (1 << CS41);
    OCR4A   = 40000 - servoTargetPulseUs * 2;  // start with LOW phase
    TIMSK4 |= (1 << OCIE4A);
    sei();
}

void stopTimer4()
{
    TIMSK4 &= ~(1 << OCIE4A);
    TCCR4B  = 0;
    digitalWrite(GRIPPER_PIN, LOW);
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
    sei();
}

void stopTimer5()
{
    TIMSK5 &= ~(1 << OCIE5A);
    TCCR5B = 0;
}

// ─── ROTATIONAL JOINTS MOVE ───────────────────────────────────────────────────
void startMoveJoints(long tJ1, long tJ2, long tJ3)
{
    // Shortest-path angle wrapping for rotational joints
    auto wrapTarget = [&](long target, long current, uint8_t j) -> long
    {
        float degDelta = (float)(target - current) / JOINTS[j].stepsPerUnit;
        long delta = target - current;
        if (degDelta > 180.0f)
            delta -= (long)(360.0f * JOINTS[j].stepsPerUnit);
        if (degDelta < -180.0f)
            delta += (long)(360.0f * JOINTS[j].stepsPerUnit);
        return current + delta;
    };

    tJ1 = wrapTarget(tJ1, jointPos[J1_IDX], J1_IDX);
    tJ2 = wrapTarget(tJ2, jointPos[J2_IDX], J2_IDX);
    tJ3 = wrapTarget(tJ3, jointPos[J3_IDX], J3_IDX);

    long d[4];
    d[J1_IDX] = labs(tJ1 - jointPos[J1_IDX]);
    d[J2_IDX] = labs(tJ2 - jointPos[J2_IDX]);
    d[J3_IDX] = labs(tJ3 - jointPos[J3_IDX]);
    d[Z_IDX] = 0;

    masterSteps = (uint16_t)max(max(d[J1_IDX], d[J2_IDX]), d[J3_IDX]);
    if (masterSteps == 0)
        return;

    digitalWrite(JOINTS[J1_IDX].dirPin, (tJ1 >= jointPos[J1_IDX]) ? HIGH : LOW);
    digitalWrite(JOINTS[J2_IDX].dirPin, (tJ2 >= jointPos[J2_IDX]) ? HIGH : LOW);
    digitalWrite(JOINTS[J3_IDX].dirPin, (tJ3 >= jointPos[J3_IDX]) ? HIGH : LOW);
    delayMicroseconds(10);

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

// ─── Z AXIS MOVE (blocking) ───────────────────────────────────────────────────
void moveZ(long tZ)
{
    long steps = labs(tZ - jointPos[Z_IDX]);
    if (steps == 0)
        return;

    digitalWrite(JOINTS[Z_IDX].dirPin,
                 (tZ >= jointPos[Z_IDX]) ? Z_UP_DIR : Z_HOME_DIR);
    delayMicroseconds(10);

    for (long s = 0; s < steps; s++)
    {
        digitalWrite(JOINTS[Z_IDX].stepPin, HIGH);
        delayMicroseconds(JOINTS[Z_IDX].homeSpeedUs);
        digitalWrite(JOINTS[Z_IDX].stepPin, LOW);
        delayMicroseconds(JOINTS[Z_IDX].homeSpeedUs);
    }

    jointPos[Z_IDX] = tZ;
}

// ─── COMBINED MOVE ────────────────────────────────────────────────────────────
void startMove(long tJ1, long tJ2, long tJ3, long tZ)
{
    startMoveJoints(tJ1, tJ2, tJ3);
    waitForMove();
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
        if (limitTriggered(JOINTS[i].limitPin))
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

    digitalWrite(JOINTS[j].dirPin, LOW);
    delayMicroseconds(10);

    unsigned long start = millis();
    const unsigned long timeout = 100000UL;

    while (true)
    {
        if (limitTriggered(JOINTS[j].limitPin))
        {
            limitHit[j] = true;
            digitalWrite(JOINTS[j].stepPin, LOW);
            Serial.print("LIMIT_HIT:J");
            break;
        }

        digitalWrite(JOINTS[j].stepPin, HIGH);
        delayMicroseconds(JOINTS[j].homeSpeedUs);
        digitalWrite(JOINTS[j].stepPin, LOW);
        delayMicroseconds(JOINTS[j].homeSpeedUs);

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


void servoWriteAngle(uint8_t targetAngle)
{
    uint8_t degsPerSec = 60;
    targetAngle = constrain(targetAngle, 0, 180);

    // Stop the hold-ISR so it doesn't conflict with manual bit-bang below
    stopTimer4();

    // How many 20ms frames to wait between each degree step
    uint8_t framesPerDeg = (1000 / degsPerSec) / 20;
    if (framesPerDeg < 1) framesPerDeg = 1;

    int8_t dir = (targetAngle > servoCurrentAngle) ? 1 : -1;

    while (servoCurrentAngle != targetAngle)
    {
        servoCurrentAngle += dir;
        uint16_t pulseUs = 500 + (uint16_t)((uint32_t)servoCurrentAngle * 2000 / 180);

        // Send framesPerDeg pulses at this angle before moving to next degree
        for (uint8_t f = 0; f < framesPerDeg; f++)
        {
            digitalWrite(GRIPPER_PIN, HIGH);
            delayMicroseconds(pulseUs);
            digitalWrite(GRIPPER_PIN, LOW);
            delay(20);
        }
    }

    // Update ISR target and restart Timer 4 to hold the position indefinitely
    servoTargetPulseUs = 500 + (uint16_t)((uint32_t)servoCurrentAngle * 2000 / 180);
    setupTimer4();
}
void servoInit()
{
    pinMode(GRIPPER_PIN, OUTPUT);
    digitalWrite(GRIPPER_PIN, LOW);
    servoCurrentAngle  = 90;
    servoTargetPulseUs = 500 + (uint16_t)((uint32_t)servoCurrentAngle * 2000 / 180);
}

void gripperOpen() { servoWriteAngle(SERVO_OPEN_ANGLE); }
void gripperClose() { servoWriteAngle(SERVO_CLOSE_ANGLE); }

// ─── INPUT VALIDATION ────────────────────────────────────────────────────────
bool validateMoveArgs(float j1, float j2, float j3, float z)
{
    struct
    {
        float val;
        uint8_t idx;
        const char *name;
    } checks[] = {
        {j1, J1_IDX, "J1"},
        {j2, J2_IDX, "J2"},
        {j3, J3_IDX, "J3"},
        {z, Z_IDX, "Z"},
    };
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t j = checks[i].idx;
        if (checks[i].val < JOINTS[j].minPos || checks[i].val > JOINTS[j].maxPos)
        {
            Serial.print("ERR:LIMIT:");
            Serial.print(checks[i].name);
            Serial.print("_OUT_OF_RANGE:");
            Serial.print(checks[i].val, 1);
            Serial.print(" (allowed ");
            Serial.print(JOINTS[j].minPos, 0);
            Serial.print(" to ");
            Serial.print(JOINTS[j].maxPos, 0);
            Serial.println(JOINTS[j].isLinear ? " mm)" : " deg)");
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
        if (z_mm < JOINTS[Z_IDX].minPos || z_mm > JOINTS[Z_IDX].maxPos)
        {
            Serial.print("ERR:LIMIT:REHOME_Z_OUT_OF_RANGE:");
            Serial.println(z_mm, 1);
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
        pinMode(JOINTS[i].stepPin, OUTPUT);
        digitalWrite(JOINTS[i].stepPin, LOW);
        pinMode(JOINTS[i].dirPin, OUTPUT);
        digitalWrite(JOINTS[i].dirPin, LOW);
        pinMode(JOINTS[i].limitPin, INPUT_PULLUP);
    }
    pinMode(LED_PIN, OUTPUT);

    servoInit();

 
gripperOpen();
// delay(100);
gripperClose();
   // homeAll();
    //homeJoint(J3_IDX);

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
    // single axis tests here

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
