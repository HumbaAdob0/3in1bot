// ============================================================
// HYBRID 3-IN-1 COMPETITION BOT
// MCU: Arduino Nano | Serial Monitor: 115200 baud
//
// DIP modes, INPUT_PULLUP logic:
//   SW2 OFF, SW1 OFF -> Mode 0: Standby
//   SW2 OFF, SW1 ON  -> Mode 1: Sumo
//   SW2 ON,  SW1 OFF -> Mode 2: Line follower
//   SW2 ON,  SW1 ON  -> Mode 3: Balloon hunt / pop
//
// Tune the constants in the CONFIG section before a match.
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>

// ============================================================
// PIN MAP
// ============================================================

// -------------------- DIP SWITCH PINS --------------------
#define SW1_PIN 12 // LSB
#define SW2_PIN 4  // MSB

// With INPUT_PULLUP wiring, switch ON should connect the pin to GND.
// If your DIP module outputs HIGH when ON, change this to HIGH.
#define DIP_SWITCH_ON_LEVEL LOW

// -------------------- MOTOR DRIVER PINS --------------------
#define AIN1_L 2
#define AIN2_L 3
#define PWMA_L 5
#define AIN1_R 8
#define AIN2_R 9
#define PWMA_R 10

// -------------------- LINE SENSOR / QTR PINS --------------------
// A6 and A7 are analog-input only on an Arduino Nano.
// A4 and A5 are reserved for I2C ToF sensors.
// QTR-8D has 8 sensors (OUT0-OUT7). We use 6 of them.
// NOTE: OUT4 sensor is defective AND pin A2/OUT6 also reads stuck HIGH.
// SOLUTION: Use 6 sensors - OUT0,1,2,3,5,7 (skip OUT4 and OUT6)
// Result: 6 QTR sensors + 3 ToF sensors - excellent coverage!
#define USE_FULL_8_QTR_ARRAY 0

// Balloon popping is passive by default: a fixed needle on the front of the bot.
// Set this to 1 only if you later add an electronic popper actuator.
#define USE_ELECTRIC_POPPER 0

// WORKING MAPPING: Use 6 out of 8 QTR sensors (skip OUT4 and OUT6)
// QTR-8D Physical Layout (left to right):
// [OUT0] [OUT1] [OUT2] [OUT3] [OUT4] [OUT5] [OUT6] [OUT7]
//   ↓      ↓      ↓      ↓      X      ↓      X      ↓
//  Use    Use    Use    Use   SKIP   Use   SKIP    Use
//
// Arduino mapping (left to right on bot):
// Position 0 (leftmost)  -> OUT0 -> D7
// Position 1             -> OUT1 -> A6
// Position 2             -> OUT2 -> A0
// Position 3 (center-left) -> OUT3 -> A1
// Position 4 (center-right) -> OUT5 -> A3  (skip OUT4 - defective!)
// Position 5 (rightmost) -> OUT7 -> A7  (skip OUT6 - A2 stuck HIGH!)

#define QTR_S0 7  // OUT0 (leftmost outer sensor) -> D7
#define QTR_S1 A6 // OUT1 -> A6

#if USE_FULL_8_QTR_ARRAY
#define QTR_S2 6
#define QTR_S3 7
#define QTR_S4 A1
#define QTR_S5 A2
#define QTR_S6 A3
#define QTR_S7 A7
#else
#define QTR_S2 A0 // OUT2 -> A0
#define QTR_S3 A1 // OUT3 -> A1
#define QTR_S4 A3 // OUT5 -> A3 (skip defective OUT4!)
#define QTR_S5 A7 // OUT7 (rightmost outer sensor) -> A7 (skip OUT6/A2!)
#endif

// -------------------- ToF XSHUT PINS --------------------
// Best setup: each VL53L0X has its own XSHUT pin.
#define XSHUT_L 13
#define XSHUT_C 11
#define XSHUT_R 6 // Keep Right ToF active!

// -------------------- OPTIONAL ELECTRIC POPPER PIN --------------------
// Static-needle balloon mode does not use this pin.
// If enabled, wire this to a MOSFET/servo/driver signal, never directly to a load.
#if USE_ELECTRIC_POPPER && !USE_FULL_8_QTR_ARRAY
#define POPPER_PIN 7
#else
#define POPPER_PIN -1
#endif

// ============================================================
// CONFIG
// ============================================================

const bool DEBUG_SERIAL = true; // Set false before final match.
const uint16_t TELEMETRY_INTERVAL_MS = 300;
const uint16_t SENSOR_STATUS_INTERVAL_MS = 500;
const uint16_t START_DELAY_MS = 5000; // Common sumo delay. Set 0 if not needed.
const uint16_t MODE_SWITCH_DEBOUNCE_MS = 40;

const uint8_t MAX_MOTOR_PWM = 255;

// If your motors run backward, flip one or both signs to -1.
const int8_t LEFT_MOTOR_SIGN = 1;
const int8_t RIGHT_MOTOR_SIGN = 1;

// If sumo/balloon turn toward the wrong side, flip this between 1 and -1.
// This is separate from LINE_STEERING_SIGN so line mode can stay tuned.
const int8_t MODE_TURN_SIGN = -1;

// VL53L0X default address is 0x29. The code re-addresses each sensor.
const uint8_t TOF_ADDR_LEFT = 0x30;
const uint8_t TOF_ADDR_CENTER = 0x31;
const uint8_t TOF_ADDR_RIGHT = 0x32;
const uint16_t TOF_TIMEOUT_MS = 120;
const uint32_t I2C_CLOCK_HZ = 100000UL; // Safer for robot wiring.
const uint16_t TOF_NO_READING_MM = 8190;
const uint16_t TOF_REINIT_INTERVAL_MS = 3000;

// ToF sensors are angled 20 degrees outward from center.
// This creates a wider detection field but affects distance calculations.
const float TOF_ANGLE_DEGREES = 20.0;
const float TOF_ANGLE_COS = 0.94; // cos(20°) ≈ 0.94 for forward distance compensation

// Line sensor polarity and threshold.
const uint16_t QTR_DARK_THRESHOLD = 500;
// Your JET/QTR-8D behavior: red LED ON on the light floor, LED OFF on black.
// That means the Arduino reads the light floor as HIGH and the black line as LOW.
// BLACK PLATFORM: QTR reads LOW (0) on black platform, HIGH (1023) when off platform (light floor)
// IMPORTANT: If edge detection doesn't work, the polarity might be inverted!
// Test: Place bot on black platform and check Serial Monitor QTR values.
// If QTR shows 1023,1023,1023... on black, change QTR_DARK_IS_HIGH to true.
const bool QTR_DARK_IS_HIGH = false; // FIXED: Black platform reads LOW (0)
const bool QTR_OUTPUT_IS_DIGITAL = true;
const bool LINE_AUTO_DETECT_POLARITY = false;

// Sumo edge config for BLACK PLATFORM with LIGHT FLOOR outside.
// When bot is ON the black platform: QTR reads LOW (0)
// When bot reaches EDGE/outside: QTR reads HIGH (1023) - light floor
// So edge = HIGH = light, which means SUMO_EDGE_IS_DARK = false
const bool SUMO_EDGE_IS_DARK = false;
// Keep this enabled for matches so the bot protects itself from the ring edge.
// If testing on a light table/floor, this may trigger constantly; test on the
// actual dark platform or temporarily set it false for bench testing only.
const bool SUMO_EDGE_ESCAPE_ENABLED = true;
// Fight-mode front is the side with the ToF sensors / sumo wedge.
// DIAGNOSTIC: Test which direction is "forward" for your bot:
// 1. Set to standby mode (SW2=OFF, SW1=OFF)
// 2. Manually test: setMotors(200, 200) should drive toward ToF sensors
// 3. If it drives away from ToF sensors, flip LEFT_MOTOR_SIGN and RIGHT_MOTOR_SIGN
// 4. If motors are correct but sumo drives backward, flip SUMO_FORWARD_SIGN
const int8_t SUMO_FORWARD_SIGN = 1; // CHANGED: Bot was going backward, flip to 1
// Keep edge escape independent from attack direction. If the bot sees the edge
// but still drives farther out, flip this sign only.
const int8_t SUMO_ESCAPE_FORWARD_SIGN = 1;      // Should match SUMO_FORWARD_SIGN usually
const uint8_t SUMO_ATTACK_PWM = 255;            // Max speed for aggressive push
const uint8_t SUMO_CHASE_PWM = 245;             // High speed chase - small arena needs speed
const uint8_t SUMO_SIDE_CHASE_PWM = 230;        // Fast side approach
const uint8_t SUMO_SIDE_STEER_PWM = 140;        // Moderate steering for side targets
const uint8_t SUMO_SEARCH_PWM = 105;            // Slower search for 77cm platform - don't miss opponent
const uint8_t SUMO_REVERSE_PWM = 255;           // Max speed reverse - 2cm white edge warning
const uint8_t SUMO_ESCAPE_TURN_PWM = 250;       // Fast turn to get back to center
const uint16_t SUMO_DETECT_MM = 350;            // CALIBRATED: 77cm open arena, sensors see edges at ~200mm, detect opponents at 350mm
const uint16_t SUMO_ATTACK_MM = 250;            // Start aggressive attack when close
const uint16_t SUMO_SIDE_DETECT_MM = 320;       // Side detection - slightly shorter due to 20° angle
const uint16_t SUMO_REVERSE_MS = 280;           // REDUCED: Shorter reverse - small platform, get back fast
const uint16_t SUMO_TURN_MS = 220;              // REDUCED: Shorter turn - need to face center quickly
const uint16_t SUMO_AGGRESSIVE_SEARCH_MS = 600; // REDUCED: Change direction faster for 77cm platform
const uint16_t SUMO_SEARCH_PAUSE_MS = 80;       // REDUCED: Shorter pause - small arena needs faster search

// Line follower tuning.
// ==================== EASY CALIBRATION SECTION ====================
// OPTIMIZED FOR NARROW LINE (only 1-2 sensors see line at a time)

// SPEED SETTINGS - Race tune for the CTU track's straights, zig-zags, and 4WD scrub.
const uint8_t LINE_BASE_PWM = 105;         // Normal bend speed
const uint8_t LINE_MAX_PWM = 155;          // Stable straight speed
const uint8_t LINE_CURVE_SLOW_PWM = 62;    // Hard apex / U-turn entry speed
const uint8_t LINE_APEX_PWM = 44;          // Brief brake for the middle zig-zag V apexes
const uint8_t LINE_APEX_OUTER_PWM = 125;   // Limit forward lunge while pivoting at V apexes
const uint8_t LINE_SEARCH_PWM = 85;        // Gentle forward recovery while the line was just lost
const uint8_t LINE_REVERSE_PWM = 120;      // Allow inner-wheel reverse pivots on steep turns
const uint8_t LINE_PIVOT_SEARCH_PWM = 110; // Spin recovery after the line has been lost briefly

// STEERING AGGRESSIVENESS - Softer high-speed gain, higher cap for real pivots.
const uint8_t LINE_KP_DIV = 5;      // Kp ~= 0.20 in the current integer controller
const uint8_t LINE_KD_DIV = 3;      // Kd ~= 0.33; less digital-sensor wobble at speed
const uint8_t LINE_APEX_KP_DIV = 3; // Stronger local steering while apex braking
const uint8_t LINE_APEX_KD_DIV = 2;
const uint8_t LINE_MAX_CORRECTION = 210; // High enough to pivot, capped by wheel PWM limits

// CURVE DETECTION SENSITIVITY - A one-sensor offset is not a hard turn on a 2.1cm line.
const uint16_t LINE_SHARP_CURVE_THRESHOLD = 1050; // Hard zig-zag apex or U-turn entry
const uint16_t LINE_MEDIUM_CURVE_THRESHOLD = 520; // One-sensor offset / medium curve
const uint16_t LINE_APEX_ERROR_THRESHOLD = 560;   // Earlier trigger for V-corner overshoot
const uint16_t LINE_STRAIGHT_THRESHOLD = 260;     // Centered enough to start straight boost
const uint16_t LINE_STRAIGHT_DERIVATIVE_THRESHOLD = 220;
const uint16_t LINE_APEX_DERIVATIVE_THRESHOLD = 360;
const uint16_t LINE_BRAKE_DERIVATIVE_THRESHOLD = 850;
const uint8_t LINE_STRAIGHT_CONFIRM_COUNT = 5;
const uint16_t LINE_APEX_BRAKE_HOLD_MS = 190;

// RECOVERY SETTINGS - Steep track corners need fast reacquisition.
const uint16_t LINE_LOST_TIMEOUT_MS = 120;    // Pivot if the line is still missing after this
const uint8_t LINE_RECOVERY_SPEED_BOOST = 30; // Extra speed on outside wheel during immediate recovery

// LINE END DETECTION - For T-junction or end-of-line turnaround
const uint16_t LINE_END_TIMEOUT_MS = 900; // Time lost before assuming line end
const uint8_t LINE_END_TURN_PWM = 95;     // Speed for 180° turnaround at line end

// ADVANCED SETTINGS
const uint16_t LINE_SIGNAL_FLOOR = 40;
const uint16_t LINE_MIN_TOTAL = 80;   // REDUCED: was 100, narrow line has less total signal
const int8_t LINE_STEERING_SIGN = -1; // Flip between 1 and -1 if the bot steers away from the line.
const bool LINE_ALL_DARK_GO_STRAIGHT = false;
// ==================== END CALIBRATION SECTION ====================

// Balloon mode tuning.
const bool BALLOON_BOUNDARY_IS_DARK = false;
// Static needle is on the same competition-front side as the ToF sensors.
// If bot drives backward when chasing balloon, flip this between 1 and -1.
const int8_t BALLOON_FORWARD_SIGN = 1;         // Should match SUMO_FORWARD_SIGN
const int8_t BALLOON_ESCAPE_FORWARD_SIGN = 1;  // Should match BALLOON_FORWARD_SIGN
const uint8_t BALLOON_RAM_PWM = 255;           // Max speed for balloon pop
const uint8_t BALLOON_CHASE_PWM = 240;         // High speed chase - small arena
const uint8_t BALLOON_SIDE_CHASE_PWM = 225;    // Fast side approach
const uint8_t BALLOON_SIDE_STEER_PWM = 120;    // Moderate steering for side targets
const uint8_t BALLOON_CENTER_STEER_PWM = 95;   // Gentle center correction
const uint8_t BALLOON_SEARCH_PWM = 110;        // Slower search for 77cm platform - don't miss balloon
const uint8_t BALLOON_REVERSE_PWM = 255;       // Max speed reverse - 2cm white edge
const uint8_t BALLOON_ESCAPE_TURN_PWM = 250;   // Fast turn to get back to center
const uint16_t BALLOON_DETECT_MM = 350;        // CALIBRATED: 77cm open arena, sensors see edges at ~200mm, detect balloons at 350mm
const uint16_t BALLOON_POP_MM = 200;           // Close approach for needle pop - be aggressive
const uint16_t BALLOON_SIDE_DETECT_MM = 320;   // Side detection - slightly shorter due to 20° angle
const uint16_t BALLOON_REVERSE_MS = 280;       // REDUCED: Shorter reverse - small platform
const uint16_t BALLOON_TURN_MS = 220;          // REDUCED: Shorter turn - get back to center fast
const uint16_t BALLOON_SEARCH_CHANGE_MS = 700; // REDUCED: Change direction faster for 77cm platform
const uint16_t BALLOON_SEARCH_PAUSE_MS = 80;   // REDUCED: Shorter pause - small arena needs faster search

const bool POPPER_ACTIVE_HIGH = true;
const uint16_t POPPER_FIRE_MS = 350;
const uint16_t POPPER_COOLDOWN_MS = 900;

// ============================================================
// GLOBALS
// ============================================================

VL53L0X tofLeft;
VL53L0X tofCenter;
VL53L0X tofRight;

bool tofLeftReady = false;
bool tofCenterReady = false;
bool tofRightReady = false;

#if USE_FULL_8_QTR_ARRAY
const uint8_t QTR_COUNT = 8;
const uint8_t QTR_PINS[QTR_COUNT] = {
    QTR_S0, QTR_S1, QTR_S2, QTR_S3,
    QTR_S4, QTR_S5, QTR_S6, QTR_S7};
#else
const uint8_t QTR_COUNT = 6; // Using 6 sensors (skip OUT4 and OUT6)
const uint8_t QTR_PINS[QTR_COUNT] = {
    QTR_S0, QTR_S1, QTR_S2,
    QTR_S3, QTR_S4, QTR_S5}; // 6 sensors total
#endif

const uint16_t QTR_POSITION_SCALE = 1000;
const uint16_t QTR_CENTER_POSITION = ((QTR_COUNT - 1) * QTR_POSITION_SCALE) / 2;

enum BotMode : uint8_t
{
  MODE_STANDBY = 0,
  MODE_SUMO = 1,
  MODE_LINE = 2,
  MODE_BALLOON = 3
};

enum EscapePhase : uint8_t
{
  ESCAPE_NONE = 0,
  ESCAPE_REVERSE = 1,
  ESCAPE_TURN = 2
};

BotMode currentMode = MODE_STANDBY;
BotMode stableSwitchMode = MODE_STANDBY;
BotMode lastRawSwitchMode = MODE_STANDBY;
uint32_t modeArmedAt = 0;
uint32_t rawSwitchChangedAt = 0;
uint32_t lastTelemetryAt = 0;
uint32_t lastSensorStatusAt = 0;
uint32_t lastTofRetryAt = 0;

EscapePhase sumoEscapePhase = ESCAPE_NONE;
uint32_t sumoEscapeUntil = 0;
int8_t sumoEscapeTurnDir = 1; // 1 = turn right, -1 = turn left.
int8_t sumoLastTargetDir = 1;
uint32_t sumoLastTargetSeenAt = 0;
uint32_t sumoSearchStartedAt = 0;
uint32_t sumoLastSearchPauseAt = 0; // NEW: Track search pause timing

EscapePhase balloonEscapePhase = ESCAPE_NONE;
uint32_t balloonEscapeUntil = 0;
int8_t balloonEscapeTurnDir = 1;
int8_t balloonLastTargetDir = 1;
uint32_t balloonLastTargetSeenAt = 0;
uint32_t balloonSearchStartedAt = 0;
uint32_t balloonLastSearchPauseAt = 0; // NEW: Track search pause timing

int16_t lineLastError = 0;
int8_t lineLastDirection = 1;
bool lineLastFound = false;
bool lineLastDarkIsHigh = QTR_DARK_IS_HIGH;
uint16_t lineLastSignalTotal = 0;
int16_t lineLastPosition = QTR_CENTER_POSITION;
int16_t lineLastLeftSpeed = 0;
int16_t lineLastRightSpeed = 0;
uint8_t lineStraightConfidence = 0;
uint32_t lineApexBrakeUntil = 0;
uint8_t lineLastLowCount = 0;
uint8_t lineLastHighCount = 0;
bool lineLastAllSame = false;
bool lineLastAllDark = false;
uint32_t lineLastSeenAt = 0;

bool popperActive = false;
uint32_t popperOffAt = 0;
uint32_t popperReadyAt = 0;

// ============================================================
// SERIAL HELPERS
// ============================================================

void debugPrint(const char *text)
{
  if (DEBUG_SERIAL)
  {
    Serial.print(text);
  }
}

void debugPrintln(const char *text)
{
  if (DEBUG_SERIAL)
  {
    Serial.println(text);
  }
}

void printHexByte(uint8_t value)
{
  Serial.print("0x");
  if (value < 16)
  {
    Serial.print('0');
  }
  Serial.print(value, HEX);
}

void printModeName(BotMode mode)
{
  switch (mode)
  {
  case MODE_STANDBY:
    Serial.print("Standby");
    break;
  case MODE_SUMO:
    Serial.print("Sumo");
    break;
  case MODE_LINE:
    Serial.print("Line");
    break;
  case MODE_BALLOON:
    Serial.print("Balloon");
    break;
  }
}

// ============================================================
// PIN / SENSOR HELPERS
// ============================================================

bool isAnalogOnlyPin(uint8_t pin)
{
  return pin == A6 || pin == A7;
}

bool isAnalogReadablePin(uint8_t pin)
{
  return pin == A0 || pin == A1 || pin == A2 || pin == A3 ||
         pin == A4 || pin == A5 || pin == A6 || pin == A7;
}

uint16_t readQtrRaw(uint8_t pin)
{
  if (QTR_OUTPUT_IS_DIGITAL && !isAnalogOnlyPin(pin))
  {
    return digitalRead(pin) == HIGH ? 1023 : 0;
  }

  if (isAnalogReadablePin(pin))
  {
    uint16_t value = analogRead(pin);
    if (QTR_OUTPUT_IS_DIGITAL)
    {
      return value > QTR_DARK_THRESHOLD ? 1023 : 0;
    }
    return value;
  }

  return digitalRead(pin) == HIGH ? 1023 : 0;
}

void readQtr(uint16_t rawValues[QTR_COUNT])
{
  for (uint8_t i = 0; i < QTR_COUNT; i++)
  {
    rawValues[i] = readQtrRaw(QTR_PINS[i]);
  }
}

bool qtrSeesDark(uint16_t rawValue)
{
  if (QTR_DARK_IS_HIGH)
  {
    return rawValue > QTR_DARK_THRESHOLD;
  }

  return rawValue < QTR_DARK_THRESHOLD;
}

bool qtrIsConfiguredEdge(uint16_t rawValue, bool edgeIsDark)
{
  bool dark = qtrSeesDark(rawValue);
  return edgeIsDark ? dark : !dark;
}

void holdTofInReset(int8_t xshutPin)
{
  if (xshutPin < 0)
  {
    return;
  }

  pinMode((uint8_t)xshutPin, OUTPUT);
  digitalWrite((uint8_t)xshutPin, LOW);
}

void releaseTofFromReset(int8_t xshutPin)
{
  if (xshutPin < 0)
  {
    return;
  }

  pinMode((uint8_t)xshutPin, OUTPUT);
  digitalWrite((uint8_t)xshutPin, HIGH);
}

bool targetSeen(uint16_t distanceMm, uint16_t thresholdMm)
{
  return distanceMm > 0 && distanceMm < thresholdMm;
}

// ============================================================
// MOTOR HELPERS
// ============================================================

void setOneMotor(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin, int speed)
{
  speed = constrain(speed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

  if (speed > 0)
  {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, -speed);
  }
  else
  {
    analogWrite(pwmPin, 0);
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
}

void setMotors(int leftSpeed, int rightSpeed)
{
  setOneMotor(AIN1_L, AIN2_L, PWMA_L, leftSpeed * LEFT_MOTOR_SIGN);
  setOneMotor(AIN1_R, AIN2_R, PWMA_R, rightSpeed * RIGHT_MOTOR_SIGN);
}

void stopMotors()
{
  setMotors(0, 0);
}

void setSteeredMotion(int baseSpeed, int steer, int8_t forwardSign)
{
  setMotors((baseSpeed - steer) * forwardSign,
            (baseSpeed + steer) * forwardSign);
}

void setStraightMotion(int speed, int8_t forwardSign)
{
  setSteeredMotion(speed, 0, forwardSign);
}

void setForwardArc(int baseSpeed, int8_t dir, uint8_t steerPwm, int8_t forwardSign)
{
  int8_t motorDir = dir * MODE_TURN_SIGN;
  int insideSpeed = constrain(baseSpeed - steerPwm, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);
  int outsideSpeed = constrain(baseSpeed, -MAX_MOTOR_PWM, MAX_MOTOR_PWM);

  if (motorDir >= 0)
  {
    setMotors(outsideSpeed * forwardSign, insideSpeed * forwardSign);
  }
  else
  {
    setMotors(insideSpeed * forwardSign, outsideSpeed * forwardSign);
  }
}

void turnInPlace(int8_t dir, uint8_t pwm)
{
  int8_t motorDir = dir * MODE_TURN_SIGN;
  if (motorDir >= 0)
  {
    setMotors(pwm, -pwm);
  }
  else
  {
    setMotors(-pwm, pwm);
  }
}

// ============================================================
// ToF HELPERS
// ============================================================

void configureTofForOpponentDetect(VL53L0X &sensor)
{
  // Long-range profile from the Pololu VL53L0X examples. The extra timing
  // budget is worth it in sumo/balloon modes because missed targets are worse
  // than a slightly slower distance update.
  sensor.setSignalRateLimit(0.1);
  sensor.setMeasurementTimingBudget(33000);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
}

bool initTofAtDefaultAddress(VL53L0X &sensor, const char *name, uint8_t newAddress)
{
  if (DEBUG_SERIAL)
  {
    Serial.print("Init ToF ");
    Serial.print(name);
    Serial.print(" -> ");
    printHexByte(newAddress);
    Serial.print(" ... ");
  }

  sensor.setTimeout(TOF_TIMEOUT_MS);

  if (!sensor.init())
  {
    debugPrintln("FAILED");
    return false;
  }

  sensor.setAddress(newAddress);
  sensor.setTimeout(TOF_TIMEOUT_MS);
  configureTofForOpponentDetect(sensor);
  sensor.startContinuous(0);
  delay(5);

  debugPrintln("OK");
  return true;
}

bool initAlwaysOnTof(VL53L0X &sensor, const char *name, uint8_t targetAddress)
{
  if (DEBUG_SERIAL)
  {
    Serial.print("Init ToF ");
    Serial.print(name);
    Serial.print(" -> ");
    printHexByte(targetAddress);
    Serial.print(" ... ");
  }

  sensor.setTimeout(TOF_TIMEOUT_MS);

  if (sensor.init())
  {
    sensor.setAddress(targetAddress);
    sensor.setTimeout(TOF_TIMEOUT_MS);
    configureTofForOpponentDetect(sensor);
    sensor.startContinuous(0);
    delay(5);
    debugPrintln("OK");
    return true;
  }

  // If only the Arduino reset, the always-on sensor may still be at targetAddress.
  sensor.setAddress(targetAddress);
  sensor.setTimeout(TOF_TIMEOUT_MS);

  if (sensor.init())
  {
    configureTofForOpponentDetect(sensor);
    sensor.startContinuous(0);
    debugPrintln("OK (retained address)");
    return true;
  }

  debugPrintln("FAILED");
  return false;
}

void setupTofSensors()
{
  holdTofInReset(XSHUT_L);
  holdTofInReset(XSHUT_C);
  holdTofInReset(XSHUT_R);
  delay(20);

  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);

  releaseTofFromReset(XSHUT_R);
  delay(20);
#if XSHUT_R < 0
  tofRightReady = initAlwaysOnTof(tofRight, "Right", TOF_ADDR_RIGHT);
#else
  tofRightReady = initTofAtDefaultAddress(tofRight, "Right", TOF_ADDR_RIGHT);
#endif

  releaseTofFromReset(XSHUT_L);
  delay(20);
  tofLeftReady = initTofAtDefaultAddress(tofLeft, "Left", TOF_ADDR_LEFT);

  releaseTofFromReset(XSHUT_C);
  delay(20);
  tofCenterReady = initTofAtDefaultAddress(tofCenter, "Center", TOF_ADDR_CENTER);
}

void retryTofInitIfNeeded(uint32_t now)
{
  if (tofLeftReady && tofCenterReady && tofRightReady)
  {
    return;
  }

  if (now - lastTofRetryAt < TOF_REINIT_INTERVAL_MS)
  {
    return;
  }

  lastTofRetryAt = now;
  stopMotors();
  debugPrintln("Retrying ToF init...");
  setupTofSensors();
}

uint16_t readTofContinuous(VL53L0X &sensor, bool ready)
{
  if (!ready)
  {
    return TOF_NO_READING_MM;
  }

  uint16_t distanceMm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred() || distanceMm == 0 || distanceMm > TOF_NO_READING_MM)
  {
    return TOF_NO_READING_MM;
  }

  return distanceMm;
}

// ============================================================
// POPPER HELPERS
// ============================================================

void setPopperOutput(bool active)
{
  if (POPPER_PIN < 0)
  {
    return;
  }

  bool level = POPPER_ACTIVE_HIGH ? active : !active;
  digitalWrite((uint8_t)POPPER_PIN, level ? HIGH : LOW);
}

void setupPopper()
{
  if (POPPER_PIN < 0)
  {
    return;
  }

  pinMode((uint8_t)POPPER_PIN, OUTPUT);
  setPopperOutput(false);
}

void firePopper(uint32_t now)
{
  if (POPPER_PIN < 0 || popperActive || now < popperReadyAt)
  {
    return;
  }

  popperActive = true;
  popperOffAt = now + POPPER_FIRE_MS;
  popperReadyAt = now + POPPER_FIRE_MS + POPPER_COOLDOWN_MS;
  setPopperOutput(true);
}

void updatePopper(uint32_t now)
{
  if (popperActive && now >= popperOffAt)
  {
    popperActive = false;
    setPopperOutput(false);
  }
}

void disarmPopper()
{
  popperActive = false;
  setPopperOutput(false);
}

// ============================================================
// MODE HELPERS
// ============================================================

bool switchIsOn(uint8_t pin)
{
  return digitalRead(pin) == DIP_SWITCH_ON_LEVEL;
}

BotMode readRawModeSwitches()
{
  uint8_t b0 = switchIsOn(SW1_PIN);
  uint8_t b1 = switchIsOn(SW2_PIN);
  return (BotMode)((b1 << 1) | b0);
}

BotMode readModeSwitches(uint32_t now)
{
  BotMode rawMode = readRawModeSwitches();

  // Standby is the safety state, so accept it immediately.
  if (rawMode == MODE_STANDBY)
  {
    lastRawSwitchMode = rawMode;
    stableSwitchMode = rawMode;
    rawSwitchChangedAt = now;
    return stableSwitchMode;
  }

  if (rawMode != lastRawSwitchMode)
  {
    lastRawSwitchMode = rawMode;
    rawSwitchChangedAt = now;
  }

  if (now - rawSwitchChangedAt >= MODE_SWITCH_DEBOUNCE_MS)
  {
    stableSwitchMode = rawMode;
  }

  return stableSwitchMode;
}

void printSwitchState(const char *label, uint8_t pin)
{
  Serial.print(label);
  Serial.print('=');
  Serial.print(switchIsOn(pin) ? "ON" : "OFF");
}

void resetModeState(BotMode nextMode, uint32_t now)
{
  stopMotors();
  disarmPopper();

  sumoEscapePhase = ESCAPE_NONE;
  sumoLastTargetSeenAt = now;
  sumoSearchStartedAt = now;
  sumoLastSearchPauseAt = now;
  balloonEscapePhase = ESCAPE_NONE;
  balloonLastTargetSeenAt = now;
  balloonSearchStartedAt = now;
  balloonLastSearchPauseAt = now;
  lineLastError = 0;
  lineLastDirection = 1;
  lineLastFound = false;
  lineLastDarkIsHigh = QTR_DARK_IS_HIGH;
  lineLastSignalTotal = 0;
  lineLastPosition = QTR_CENTER_POSITION;
  lineLastLeftSpeed = 0;
  lineLastRightSpeed = 0;
  lineStraightConfidence = 0;
  lineApexBrakeUntil = 0;
  lineLastLowCount = 0;
  lineLastHighCount = 0;
  lineLastAllSame = false;
  lineLastAllDark = false;
  lineLastSeenAt = now;

  currentMode = nextMode;
  modeArmedAt = (nextMode == MODE_STANDBY) ? now : now + START_DELAY_MS;

  if (DEBUG_SERIAL)
  {
    Serial.print("Mode -> ");
    printModeName(currentMode);
    Serial.print(" | ");
    printSwitchState("SW1", SW1_PIN);
    Serial.print(' ');
    printSwitchState("SW2", SW2_PIN);
    if (currentMode != MODE_STANDBY && START_DELAY_MS > 0)
    {
      Serial.print(" | start in ");
      Serial.print(START_DELAY_MS);
      Serial.print(" ms");
    }
    Serial.println();
  }
}

bool modeIsArmed(uint32_t now)
{
  return currentMode == MODE_STANDBY || now >= modeArmedAt;
}

// ============================================================
// EDGE / BOUNDARY HELPERS
// ============================================================

bool findEdge(const uint16_t raw[QTR_COUNT], bool edgeIsDark, bool &leftEdge, bool &rightEdge)
{
  // For 6-sensor array, check outer 3 sensors on each side for better edge detection
  // This gives earlier warning before bot goes off platform
  // IMPORTANT: Require at least 2 sensors to trigger edge (avoid false positives from single sensor)
  uint8_t sideCount = (QTR_COUNT + 1) / 2; // 3 sensors per side for 6-sensor array
  if (sideCount < 2)
  {
    sideCount = 2;
  }

  bool anyEdge = false;
  leftEdge = false;
  rightEdge = false;
  uint8_t leftEdgeCount = 0;
  uint8_t rightEdgeCount = 0;

  for (uint8_t i = 0; i < QTR_COUNT; i++)
  {
    if (!qtrIsConfiguredEdge(raw[i], edgeIsDark))
    {
      continue;
    }

    if (i < sideCount)
    {
      leftEdgeCount++;
    }
    if (i >= QTR_COUNT - sideCount)
    {
      rightEdgeCount++;
    }
  }

  // Require at least 2 sensors to confirm edge (reduces false positives)
  if (leftEdgeCount >= 2)
  {
    leftEdge = true;
    anyEdge = true;
  }
  if (rightEdgeCount >= 2)
  {
    rightEdge = true;
    anyEdge = true;
  }

  return anyEdge;
}

void printDistanceValue(uint16_t distanceMm)
{
  if (distanceMm == TOF_NO_READING_MM)
  {
    Serial.print("----");
    return;
  }

  Serial.print(distanceMm);
}

void printSensorStatus(uint32_t now)
{
  if (!DEBUG_SERIAL || now - lastSensorStatusAt < SENSOR_STATUS_INTERVAL_MS)
  {
    return;
  }

  lastSensorStatusAt = now;

  uint16_t qtrRaw[QTR_COUNT];
  readQtr(qtrRaw);

  uint16_t leftMm = readTofContinuous(tofLeft, tofLeftReady);
  uint16_t centerMm = readTofContinuous(tofCenter, tofCenterReady);
  uint16_t rightMm = readTofContinuous(tofRight, tofRightReady);

  uint16_t detectMm = currentMode == MODE_BALLOON ? BALLOON_DETECT_MM : SUMO_DETECT_MM;
  bool leftSeen = targetSeen(leftMm, detectMm);
  bool centerSeen = targetSeen(centerMm, detectMm);
  bool rightSeen = targetSeen(rightMm, detectMm);

  bool leftEdge = false;
  bool rightEdge = false;
  bool edgeSeen = findEdge(qtrRaw,
                           currentMode == MODE_BALLOON ? BALLOON_BOUNDARY_IS_DARK : SUMO_EDGE_IS_DARK,
                           leftEdge, rightEdge);

  Serial.print("STATUS mode=");
  printModeName(currentMode);
  Serial.print(modeIsArmed(now) ? " armed" : " waiting");
  Serial.print(" | ");
  printSwitchState("SW1", SW1_PIN);
  Serial.print(' ');
  printSwitchState("SW2", SW2_PIN);
  Serial.print(" | ToF L/C/R=");
  printDistanceValue(leftMm);
  Serial.print('/');
  printDistanceValue(centerMm);
  Serial.print('/');
  printDistanceValue(rightMm);
  Serial.print("mm ready=");
  Serial.print(tofLeftReady ? '1' : '0');
  Serial.print(tofCenterReady ? '1' : '0');
  Serial.print(tofRightReady ? '1' : '0');
  Serial.print(" seen=");
  Serial.print(leftSeen ? '1' : '0');
  Serial.print(centerSeen ? '1' : '0');
  Serial.print(rightSeen ? '1' : '0');
  Serial.print(" limit=");
  Serial.print(detectMm);
  Serial.print(" | QTR=");
  for (uint8_t i = 0; i < QTR_COUNT; i++)
  {
    if (i > 0)
    {
      Serial.print(',');
    }
    Serial.print(qtrRaw[i]);
  }
  Serial.print(" edge=");
  Serial.print(edgeSeen ? '1' : '0');
  Serial.print(" L=");
  Serial.print(leftEdge ? '1' : '0');
  Serial.print(" R=");
  Serial.print(rightEdge ? '1' : '0');

  if (currentMode == MODE_LINE)
  {
    Serial.print(" | line=");
    Serial.print(lineLastFound ? "FOUND" : "LOST");
    Serial.print(" polarity=");
    Serial.print(lineLastDarkIsHigh ? "HIGH" : "LOW");
    Serial.print(" pos=");
    Serial.print(lineLastPosition);
    Serial.print(" err=");
    Serial.print(lineLastError);
    Serial.print(" total=");
    Serial.print(lineLastSignalTotal);
    Serial.print(" low/high=");
    Serial.print(lineLastLowCount);
    Serial.print('/');
    Serial.print(lineLastHighCount);
    Serial.print(" allSame=");
    Serial.print(lineLastAllSame ? '1' : '0');
    Serial.print(" allDark=");
    Serial.print(lineLastAllDark ? '1' : '0');
    Serial.print(" motor=");
    Serial.print(lineLastLeftSpeed);
    Serial.print('/');
    Serial.print(lineLastRightSpeed);
  }

  Serial.println();
}

int8_t chooseEscapeTurn(bool leftEdge, bool rightEdge, int8_t fallbackDir)
{
  if (leftEdge && !rightEdge)
  {
    return 1; // Edge on left, turn right.
  }
  if (rightEdge && !leftEdge)
  {
    return -1; // Edge on right, turn left.
  }
  return fallbackDir >= 0 ? 1 : -1;
}

bool runEscape(EscapePhase &phase, uint32_t &until, int8_t turnDir,
               uint16_t reverseMs, uint16_t turnMs,
               uint8_t reversePwm, uint8_t turnPwm,
               int8_t forwardSign,
               uint32_t now)
{
  if (phase == ESCAPE_NONE)
  {
    return false;
  }

  if (phase == ESCAPE_REVERSE)
  {
    setStraightMotion(-reversePwm, forwardSign);
    if (now >= until)
    {
      phase = ESCAPE_TURN;
      until = now + turnMs;
    }
    return true;
  }

  turnInPlace(turnDir, turnPwm);
  if (now >= until)
  {
    phase = ESCAPE_NONE;
  }
  return true;
}

// ============================================================
// MODE 1: SUMO
// ============================================================

void runSumo(uint32_t now)
{
  // PRIORITY 1: Check edge FIRST before any other action
  uint16_t qtrRaw[QTR_COUNT];
  readQtr(qtrRaw);

  bool leftEdge = false;
  bool rightEdge = false;
  bool edgeSeen = findEdge(qtrRaw, SUMO_EDGE_IS_DARK, leftEdge, rightEdge);

  // Immediately trigger escape if edge detected and not already escaping
  if (SUMO_EDGE_ESCAPE_ENABLED && edgeSeen && sumoEscapePhase == ESCAPE_NONE)
  {
    sumoEscapeTurnDir = chooseEscapeTurn(leftEdge, rightEdge, -sumoLastTargetDir);
    sumoEscapePhase = ESCAPE_REVERSE;
    sumoEscapeUntil = now + SUMO_REVERSE_MS;
  }

  // PRIORITY 2: Execute escape sequence if active
  if (runEscape(sumoEscapePhase, sumoEscapeUntil, sumoEscapeTurnDir,
                SUMO_REVERSE_MS, SUMO_TURN_MS,
                SUMO_REVERSE_PWM, SUMO_ESCAPE_TURN_PWM,
                SUMO_ESCAPE_FORWARD_SIGN, now))
  {
    return; // Don't do anything else while escaping
  }

  // PRIORITY 3: Normal attack/search behavior only if no edge detected
  uint16_t leftMm = readTofContinuous(tofLeft, tofLeftReady);
  uint16_t centerMm = readTofContinuous(tofCenter, tofCenterReady);
  uint16_t rightMm = readTofContinuous(tofRight, tofRightReady);

  // Side sensors are angled 20° outward, so they detect at different effective ranges
  bool leftSeen = targetSeen(leftMm, SUMO_SIDE_DETECT_MM);
  bool centerSeen = targetSeen(centerMm, SUMO_DETECT_MM);
  bool rightSeen = targetSeen(rightMm, SUMO_SIDE_DETECT_MM);

  bool anyTargetSeen = leftSeen || centerSeen || rightSeen;
  if (anyTargetSeen)
  {
    sumoLastTargetSeenAt = now;
  }

  if (centerSeen)
  {
    sumoLastTargetDir = 1;
    if (centerMm < SUMO_ATTACK_MM)
    {
      setStraightMotion(SUMO_ATTACK_PWM, SUMO_FORWARD_SIGN);
    }
    else
    {
      setStraightMotion(SUMO_CHASE_PWM, SUMO_FORWARD_SIGN);
    }
  }
  else if (leftSeen && rightSeen)
  {
    // Both side sensors see target - opponent is wide or close.
    // Drive toward the closer side with aggressive arc.
    sumoLastTargetDir = leftMm <= rightMm ? -1 : 1;
    uint8_t steer = constrain(abs((int16_t)rightMm - (int16_t)leftMm) / 4,
                              60, SUMO_SIDE_STEER_PWM);
    setForwardArc(SUMO_SIDE_CHASE_PWM, sumoLastTargetDir, steer, SUMO_FORWARD_SIGN);
  }
  else if (leftSeen && (!rightSeen || leftMm <= rightMm))
  {
    sumoLastTargetDir = -1;
    // Angled sensor sees target - use aggressive arc to intercept
    setForwardArc(SUMO_SIDE_CHASE_PWM, -1, SUMO_SIDE_STEER_PWM, SUMO_FORWARD_SIGN);
  }
  else if (rightSeen)
  {
    sumoLastTargetDir = 1;
    setForwardArc(SUMO_SIDE_CHASE_PWM, 1, SUMO_SIDE_STEER_PWM, SUMO_FORWARD_SIGN);
  }
  else
  {
    // No target seen - slower search with pauses for better ToF detection on 1m platform
    uint32_t timeSinceLastSeen = now - sumoLastTargetSeenAt;
    if (timeSinceLastSeen > SUMO_AGGRESSIVE_SEARCH_MS)
    {
      // Switch search direction periodically for better coverage
      if ((timeSinceLastSeen / SUMO_AGGRESSIVE_SEARCH_MS) % 2 == 0)
      {
        sumoLastTargetDir = -sumoLastTargetDir;
      }
    }

    // Pulsed search: spin slowly with brief pauses to let ToF sensors stabilize
    uint32_t searchCycleTime = (now - sumoLastSearchPauseAt) % (SUMO_SEARCH_PAUSE_MS * 4);
    if (searchCycleTime < SUMO_SEARCH_PAUSE_MS)
    {
      // Brief pause - ToF sensors can get clean reading
      stopMotors();
    }
    else
    {
      // Slow spin search - won't miss opponent
      turnInPlace(sumoLastTargetDir, SUMO_SEARCH_PWM);
    }
  }

  if (DEBUG_SERIAL && now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryAt = now;
    Serial.print("SUMO ToF L/C/R=");
    printDistanceValue(leftMm);
    Serial.print('/');
    printDistanceValue(centerMm);
    Serial.print('/');
    printDistanceValue(rightMm);
    Serial.print(" ready=");
    Serial.print(tofLeftReady ? '1' : '0');
    Serial.print(tofCenterReady ? '1' : '0');
    Serial.print(tofRightReady ? '1' : '0');
    Serial.print(" seen=");
    Serial.print(leftSeen ? '1' : '0');
    Serial.print(centerSeen ? '1' : '0');
    Serial.print(rightSeen ? '1' : '0');
    Serial.print(" edge=");
    Serial.print(edgeSeen ? '1' : '0');
    Serial.print(" escaping=");
    Serial.print(sumoEscapePhase != ESCAPE_NONE ? '1' : '0');
    Serial.print(" edgeEsc=");
    Serial.print(SUMO_EDGE_ESCAPE_ENABLED ? '1' : '0');
    Serial.print(" limit=");
    Serial.print(SUMO_DETECT_MM);
    Serial.print(" QTR=");
    for (uint8_t i = 0; i < QTR_COUNT; i++)
    {
      if (i > 0)
        Serial.print(',');
      Serial.print(qtrRaw[i]);
    }
    Serial.println();
  }
}

// ============================================================
// MODE 2: LINE FOLLOWER
// ============================================================

bool readLineError(int16_t &error)
{
  uint16_t rawValues[QTR_COUNT];
  uint32_t weighted = 0;
  uint16_t total = 0;
  uint8_t lowCount = 0;
  uint8_t highCount = 0;

  for (uint8_t i = 0; i < QTR_COUNT; i++)
  {
    rawValues[i] = readQtrRaw(QTR_PINS[i]);
    if (rawValues[i] < QTR_DARK_THRESHOLD)
    {
      lowCount++;
    }
    else
    {
      highCount++;
    }
  }

  lineLastLowCount = lowCount;
  lineLastHighCount = highCount;
  lineLastAllSame = lowCount == QTR_COUNT || highCount == QTR_COUNT;

  bool darkIsHigh = lineLastDarkIsHigh;
  if (LINE_AUTO_DETECT_POLARITY && lowCount > 0 && highCount > 0 && lowCount != highCount)
  {
    // On a black-line course, the line usually covers fewer sensors than the floor.
    darkIsHigh = highCount < lowCount;
  }

  lineLastDarkIsHigh = darkIsHigh;
  lineLastAllDark = darkIsHigh ? (highCount == QTR_COUNT) : (lowCount == QTR_COUNT);

  if (LINE_ALL_DARK_GO_STRAIGHT && lineLastAllDark)
  {
    error = 0;
    lineLastFound = true;
    lineLastSignalTotal = QTR_COUNT * 1023U;
    lineLastPosition = QTR_CENTER_POSITION;
    return true;
  }

  if (lineLastAllSame)
  {
    lineLastFound = false;
    lineLastSignalTotal = 0;
    return false;
  }

  for (uint8_t i = 0; i < QTR_COUNT; i++)
  {
    int signal = darkIsHigh ? rawValues[i] : (1023 - rawValues[i]);
    signal -= LINE_SIGNAL_FLOOR;
    signal = constrain(signal, 0, 1023);

    weighted += (uint32_t)signal * (uint32_t)i * QTR_POSITION_SCALE;
    total += signal;
  }

  if (total < LINE_MIN_TOTAL)
  {
    lineLastFound = false;
    lineLastSignalTotal = total;
    return false;
  }

  int16_t position = weighted / total; // 0 to the rightmost sensor position.
  error = position - (int16_t)QTR_CENTER_POSITION;
  lineLastFound = true;
  lineLastSignalTotal = total;
  lineLastPosition = position;
  return true;
}

void runLineFollower(uint32_t now)
{
  int16_t error = 0;
  bool lineFound = readLineError(error);

  if (lineFound)
  {
    lineLastSeenAt = now;
  }

  // ========== LINE LOST RECOVERY ==========
  if (!lineFound)
  {
    uint32_t timeSinceLine = now - lineLastSeenAt;
    lineLastError = 0;
    lineStraightConfidence = 0;
    lineApexBrakeUntil = 0;

    int8_t recoverDirection = lineLastDirection * LINE_STEERING_SIGN;

    // Check if line has been lost for a long time (line end / T-junction)
    if (timeSinceLine > LINE_END_TIMEOUT_MS)
    {
      // LINE END DETECTED - Do 180° turnaround
      // Spin in place to find line going back the other way
      if (recoverDirection >= 0)
      {
        lineLastLeftSpeed = LINE_END_TURN_PWM;
        lineLastRightSpeed = -LINE_END_TURN_PWM;
      }
      else
      {
        lineLastLeftSpeed = -LINE_END_TURN_PWM;
        lineLastRightSpeed = LINE_END_TURN_PWM;
      }
    }
    else if (timeSinceLine > LINE_LOST_TIMEOUT_MS)
    {
      // STEEP-CORNER RECOVERY - the CTU track has abrupt apexes where the
      // forward sensor array can briefly leave the line. Pivot back toward
      // the last known side instead of continuing to drift forward.
      if (recoverDirection >= 0)
      {
        lineLastLeftSpeed = LINE_PIVOT_SEARCH_PWM;
        lineLastRightSpeed = -LINE_PIVOT_SEARCH_PWM;
      }
      else
      {
        lineLastLeftSpeed = -LINE_PIVOT_SEARCH_PWM;
        lineLastRightSpeed = LINE_PIVOT_SEARCH_PWM;
      }
    }
    else
    {
      // NORMAL RECOVERY - Gentle forward arc for narrow line
      // Keep both wheels moving forward to maintain momentum
      if (recoverDirection >= 0)
      {
        // Turn right - left wheel faster, right wheel much slower
        lineLastLeftSpeed = LINE_SEARCH_PWM + LINE_RECOVERY_SPEED_BOOST;
        lineLastRightSpeed = LINE_SEARCH_PWM / 4; // Very slow inside wheel for narrow line
      }
      else
      {
        // Turn left - right wheel faster, left wheel much slower
        lineLastLeftSpeed = LINE_SEARCH_PWM / 4; // Very slow inside wheel for narrow line
        lineLastRightSpeed = LINE_SEARCH_PWM + LINE_RECOVERY_SPEED_BOOST;
      }
    }

    setMotors(lineLastLeftSpeed, lineLastRightSpeed);
    return;
  }

  // ========== LINE FOUND - NORMAL FOLLOWING ==========

  // Calculate derivative for damping
  int16_t derivative = error - lineLastError;
  lineLastError = error;

  // Track which side of the line we're on for recovery
  // Use smaller threshold for more sensitive direction tracking
  if (error > 80)
  {
    lineLastDirection = 1;
  }
  else if (error < -80)
  {
    lineLastDirection = -1;
  }

  // Calculate absolute error for curve detection
  int absError = abs(error);
  int absDerivative = abs(derivative);
  bool apexDetected = absError > LINE_APEX_ERROR_THRESHOLD ||
                      (absDerivative > LINE_APEX_DERIVATIVE_THRESHOLD &&
                       absError > LINE_STRAIGHT_THRESHOLD) ||
                      (lineLastLowCount >= 3 &&
                       absError > LINE_STRAIGHT_THRESHOLD);

  if (apexDetected)
  {
    lineApexBrakeUntil = now + LINE_APEX_BRAKE_HOLD_MS;
    lineStraightConfidence = 0;
  }

  bool apexBrakeActive = now < lineApexBrakeUntil;

  bool stableStraight = absError < LINE_STRAIGHT_THRESHOLD &&
                        absDerivative < LINE_STRAIGHT_DERIVATIVE_THRESHOLD &&
                        !apexBrakeActive;
  if (stableStraight)
  {
    if (lineStraightConfidence < LINE_STRAIGHT_CONFIRM_COUNT)
    {
      lineStraightConfidence++;
    }
  }
  else
  {
    lineStraightConfidence = 0;
  }

  // ========== ADAPTIVE SPEED BASED ON CURVE SHARPNESS ==========
  int baseSpeed;
  if (apexBrakeActive)
  {
    // Briefly hold a lower speed through zig-zag apexes so 4WD scrub does not
    // carry the bot past the next segment.
    baseSpeed = LINE_APEX_PWM;
  }
  else if (absError > LINE_SHARP_CURVE_THRESHOLD ||
           absDerivative > LINE_BRAKE_DERIVATIVE_THRESHOLD)
  {
    // VERY SHARP CURVE (>90° turn) - slow down significantly
    baseSpeed = LINE_CURVE_SLOW_PWM;
  }
  else if (absError > LINE_MEDIUM_CURVE_THRESHOLD)
  {
    // MEDIUM CURVE (45-90° turn) - gradual slowdown
    // Smooth interpolation between curve speed and base speed
    int speedRange = LINE_BASE_PWM - LINE_CURVE_SLOW_PWM;
    int errorRange = LINE_SHARP_CURVE_THRESHOLD - LINE_MEDIUM_CURVE_THRESHOLD;
    int errorDiff = absError - LINE_MEDIUM_CURVE_THRESHOLD;
    int speedReduction = (speedRange * errorDiff) / errorRange;
    baseSpeed = LINE_BASE_PWM - speedReduction;
  }
  else if (lineStraightConfidence >= LINE_STRAIGHT_CONFIRM_COUNT)
  {
    // STABLE STRAIGHT - use full race speed only after the line stays calm.
    baseSpeed = LINE_MAX_PWM;
  }
  else
  {
    // GENTLE CURVE - use base speed
    baseSpeed = LINE_BASE_PWM;
  }

  // ========== PD CONTROL FOR STEERING ==========
  // Calculate correction with proportional and derivative terms
  uint8_t kpDiv = apexBrakeActive ? LINE_APEX_KP_DIV : LINE_KP_DIV;
  uint8_t kdDiv = apexBrakeActive ? LINE_APEX_KD_DIV : LINE_KD_DIV;
  int proportional = error / kpDiv;
  int derivativeTerm = derivative / kdDiv;
  int correction = (proportional + derivativeTerm) * LINE_STEERING_SIGN;

  // Constrain correction to prevent excessive steering
  correction = constrain(correction, -LINE_MAX_CORRECTION, LINE_MAX_CORRECTION);

  // ========== APPLY SPEED AND STEERING ==========
  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  // Constrain to motor limits
  int forwardLimit = apexBrakeActive ? LINE_APEX_OUTER_PWM : LINE_MAX_PWM;
  leftSpeed = constrain(leftSpeed, -LINE_REVERSE_PWM, forwardLimit);
  rightSpeed = constrain(rightSpeed, -LINE_REVERSE_PWM, forwardLimit);

  lineLastLeftSpeed = leftSpeed;
  lineLastRightSpeed = rightSpeed;
  setMotors(leftSpeed, rightSpeed);
}

// ============================================================
// MODE 3: BALLOON HUNT / POP
// ============================================================

void runBalloon(uint32_t now)
{
  updatePopper(now);

  // PRIORITY 1: Check boundary FIRST before any other action
  uint16_t qtrRaw[QTR_COUNT];
  readQtr(qtrRaw);

  bool leftBoundary = false;
  bool rightBoundary = false;
  bool boundarySeen = findEdge(qtrRaw, BALLOON_BOUNDARY_IS_DARK, leftBoundary, rightBoundary);

  // Immediately trigger escape if boundary detected and not already escaping
  if (boundarySeen && balloonEscapePhase == ESCAPE_NONE)
  {
    balloonEscapeTurnDir = chooseEscapeTurn(leftBoundary, rightBoundary, -balloonLastTargetDir);
    balloonEscapePhase = ESCAPE_REVERSE;
    balloonEscapeUntil = now + BALLOON_REVERSE_MS;
  }

  // PRIORITY 2: Execute escape sequence if active
  if (runEscape(balloonEscapePhase, balloonEscapeUntil, balloonEscapeTurnDir,
                BALLOON_REVERSE_MS, BALLOON_TURN_MS,
                BALLOON_REVERSE_PWM, BALLOON_ESCAPE_TURN_PWM,
                BALLOON_ESCAPE_FORWARD_SIGN, now))
  {
    return; // Don't do anything else while escaping
  }

  // PRIORITY 3: Normal balloon hunting only if no boundary detected
  uint16_t leftMm = readTofContinuous(tofLeft, tofLeftReady);
  uint16_t centerMm = readTofContinuous(tofCenter, tofCenterReady);
  uint16_t rightMm = readTofContinuous(tofRight, tofRightReady);

  // Side sensors angled 20° - use different thresholds
  bool leftSeen = targetSeen(leftMm, BALLOON_SIDE_DETECT_MM);
  bool centerSeen = targetSeen(centerMm, BALLOON_DETECT_MM);
  bool rightSeen = targetSeen(rightMm, BALLOON_SIDE_DETECT_MM);

  bool anyTargetSeen = leftSeen || centerSeen || rightSeen;
  if (anyTargetSeen)
  {
    balloonLastTargetSeenAt = now;
  }

  if (centerSeen && centerMm <= BALLOON_POP_MM)
  {
#if USE_ELECTRIC_POPPER
    firePopper(now);
#endif
    // Full ram with static needle
    setStraightMotion(BALLOON_RAM_PWM, BALLOON_FORWARD_SIGN);
  }
  else if (centerSeen)
  {
    // Center sensor has target - check sides for fine adjustment
    if (leftSeen && rightSeen)
    {
      // All three sensors see balloon - it's centered and close
      int8_t targetDir = leftMm <= rightMm ? -1 : 1;
      uint8_t steer = constrain(abs((int16_t)rightMm - (int16_t)leftMm) / 6,
                                20, BALLOON_CENTER_STEER_PWM);
      setForwardArc(BALLOON_CHASE_PWM, targetDir, steer, BALLOON_FORWARD_SIGN);
    }
    else if (leftSeen)
    {
      // Center and left see it - steer slightly left
      setForwardArc(BALLOON_CHASE_PWM, -1, BALLOON_CENTER_STEER_PWM / 2, BALLOON_FORWARD_SIGN);
    }
    else if (rightSeen)
    {
      // Center and right see it - steer slightly right
      setForwardArc(BALLOON_CHASE_PWM, 1, BALLOON_CENTER_STEER_PWM / 2, BALLOON_FORWARD_SIGN);
    }
    else
    {
      // Only center sees it - drive straight
      setStraightMotion(BALLOON_CHASE_PWM, BALLOON_FORWARD_SIGN);
    }
  }
  else if (leftSeen && rightSeen)
  {
    // Both sides see balloon but center doesn't - balloon is close and wide
    balloonLastTargetDir = leftMm <= rightMm ? -1 : 1;
    uint8_t steer = constrain(abs((int16_t)rightMm - (int16_t)leftMm) / 5,
                              50, BALLOON_SIDE_STEER_PWM);
    setForwardArc(BALLOON_SIDE_CHASE_PWM, balloonLastTargetDir, steer, BALLOON_FORWARD_SIGN);
  }
  else if (leftSeen && (!rightSeen || leftMm <= rightMm))
  {
    balloonLastTargetDir = -1;
    // Left angled sensor sees balloon - aggressive arc to intercept
    setForwardArc(BALLOON_SIDE_CHASE_PWM, -1, BALLOON_SIDE_STEER_PWM, BALLOON_FORWARD_SIGN);
  }
  else if (rightSeen)
  {
    balloonLastTargetDir = 1;
    setForwardArc(BALLOON_SIDE_CHASE_PWM, 1, BALLOON_SIDE_STEER_PWM, BALLOON_FORWARD_SIGN);
  }
  else
  {
    // No target - slower search with pauses for better ToF detection on 1m platform
    uint32_t timeSinceLastSeen = now - balloonLastTargetSeenAt;
    if (timeSinceLastSeen > BALLOON_SEARCH_CHANGE_MS)
    {
      if ((timeSinceLastSeen / BALLOON_SEARCH_CHANGE_MS) % 2 == 0)
      {
        balloonLastTargetDir = -balloonLastTargetDir;
      }
    }

    // Pulsed search: spin slowly with brief pauses to let ToF sensors stabilize
    uint32_t searchCycleTime = (now - balloonLastSearchPauseAt) % (BALLOON_SEARCH_PAUSE_MS * 4);
    if (searchCycleTime < BALLOON_SEARCH_PAUSE_MS)
    {
      // Brief pause - ToF sensors can get clean reading
      stopMotors();
    }
    else
    {
      // Slow spin search - won't miss balloon
      turnInPlace(balloonLastTargetDir, BALLOON_SEARCH_PWM);
    }
  }

  if (DEBUG_SERIAL && now - lastTelemetryAt >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryAt = now;
    Serial.print("BALLOON ToF L/C/R=");
    printDistanceValue(leftMm);
    Serial.print('/');
    printDistanceValue(centerMm);
    Serial.print('/');
    printDistanceValue(rightMm);
    Serial.print(" ready=");
    Serial.print(tofLeftReady ? '1' : '0');
    Serial.print(tofCenterReady ? '1' : '0');
    Serial.print(tofRightReady ? '1' : '0');
    Serial.print(" seen=");
    Serial.print(leftSeen ? '1' : '0');
    Serial.print(centerSeen ? '1' : '0');
    Serial.print(rightSeen ? '1' : '0');
    Serial.print(" boundary=");
    Serial.print(boundarySeen ? '1' : '0');
    Serial.print(" limit=");
    Serial.print(BALLOON_DETECT_MM);
#if USE_ELECTRIC_POPPER
    Serial.print(" popper=");
    Serial.println(POPPER_PIN < 0 ? "OFF" : (popperActive ? "FIRE" : "READY"));
#else
    Serial.println(" needle=STATIC");
#endif
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(200);

  pinMode(AIN1_L, OUTPUT);
  pinMode(AIN2_L, OUTPUT);
  pinMode(PWMA_L, OUTPUT);
  pinMode(AIN1_R, OUTPUT);
  pinMode(AIN2_R, OUTPUT);
  pinMode(PWMA_R, OUTPUT);
  stopMotors();

  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);

  for (uint8_t i = 0; i < QTR_COUNT; i++)
  {
    if (!isAnalogOnlyPin(QTR_PINS[i]))
    {
      pinMode(QTR_PINS[i], QTR_OUTPUT_IS_DIGITAL ? INPUT_PULLUP : INPUT);
    }
  }

  setupPopper();
  setupTofSensors();

  uint32_t now = millis();
  currentMode = readRawModeSwitches();
  stableSwitchMode = currentMode;
  lastRawSwitchMode = currentMode;
  rawSwitchChangedAt = now;
  resetModeState(currentMode, now);

  if (DEBUG_SERIAL)
  {
    Serial.println("Hybrid bot competition code ready.");
    Serial.print("QTR sensors used: ");
    Serial.println(QTR_COUNT);
    Serial.print("Right ToF XSHUT pin: ");
    Serial.println(XSHUT_R);
#if USE_ELECTRIC_POPPER
    Serial.print("Popper pin: ");
    Serial.println(POPPER_PIN);
#else
    Serial.println("Balloon popper: static front needle");
#endif

    // QTR Pin Diagnostic
    Serial.println("\n=== QTR PIN DIAGNOSTIC ===");
    Serial.println("Testing each QTR sensor pin...");
    for (uint8_t i = 0; i < QTR_COUNT; i++)
    {
      Serial.print("Sensor ");
      Serial.print(i);
      Serial.print(" (Pin ");
      if (QTR_PINS[i] >= A0)
      {
        Serial.print("A");
        Serial.print(QTR_PINS[i] - A0);
      }
      else
      {
        Serial.print(QTR_PINS[i]);
      }
      Serial.print("): ");
      uint16_t val = readQtrRaw(QTR_PINS[i]);
      Serial.print(val);
      if (val == 1023)
      {
        Serial.print(" <- HIGH (check wiring if on dark surface!)");
      }
      Serial.println();
    }
    Serial.println("=========================\n");
  }
}

void loop()
{
  uint32_t now = millis();
  BotMode selectedMode = readModeSwitches(now);

  if (selectedMode != currentMode)
  {
    resetModeState(selectedMode, now);
  }

  updatePopper(now);

  if (currentMode == MODE_SUMO || currentMode == MODE_BALLOON)
  {
    retryTofInitIfNeeded(now);
  }

  if (!modeIsArmed(now))
  {
    stopMotors();
    printSensorStatus(now);
    return;
  }

  switch (currentMode)
  {
  case MODE_STANDBY:
    stopMotors();
    disarmPopper();
    break;

  case MODE_SUMO:
    runSumo(now);
    break;

  case MODE_LINE:
    runLineFollower(now);
    break;

  case MODE_BALLOON:
    runBalloon(now);
    break;
  }

  printSensorStatus(now);
}
