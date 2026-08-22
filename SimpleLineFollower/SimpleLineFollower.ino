/* ============================================================================
 *  SimpleLineFollower  —  a clean, reusable PID line follower
 *  Derived from the SUIVEUR LASSAAD firmware, stripped down to ONE file.
 *
 *  WHY THIS EXISTS
 *  ---------------
 *  The full firmware drives a *programmed* path ("B1CCCCCs" + distances) and is
 *  great for a known maze. This sketch instead just FOLLOWS THE LINE under it,
 *  whatever its shape — so the SAME code runs on ANY track with no editing.
 *
 *  HOW TO USE (3 steps)
 *  --------------------
 *   1. Set LINE_IS_BLACK below (black line on white floor = true).
 *   2. Upload. On power-up it calibrates: sweep the sensor bar left<->right
 *      across the line for ~8 s while the on-board LED blinks.
 *   3. Place robot on the line, press the button -> it follows.
 *
 *  TUNING (only these three usually matter)
 *  ----------------------------------------
 *   BASE_SPEED : higher = faster but harder to keep on line.
 *   KP         : how hard it steers toward the line. Too low = cuts corners;
 *                too high = wobbles/oscillates.
 *   KD         : damping. Raise it if KP makes it wobble.
 *
 *  Hardware: Arduino Mega 2560, 16-ch QTR analog array, L298-style dual driver.
 *  Library : QTRSensors (Pololu) — install via Library Manager. Nothing else.
 * ========================================================================== */

#include <QTRSensors.h>

/* ======================  USER SETTINGS  ================================== */

const bool LINE_IS_BLACK = true;   // true: dark line / light floor.  false: invert.

int   BASE_SPEED = 200;            // cruise PWM 0..255. Fastest laps come from a
//   HIGH base + corner braking (TURN_SLOW), not from raising TURN_SLOW. Sim says
//   240-255 is safe on the real-world side keep ~15-20% margin -> 200 is a fast,
//   safe start; push toward 240-255 once you trust it on your track.
float KP         = 0.030;          // proportional gain   (sim-tuned)
float KI         = 0.0;            // integral gain (leave 0 unless drifting)
float KD         = 1.2;            // derivative gain     (sim-tuned)

// ^ KP/KD were tuned in robot_sim.py for THIS controller form (per-loop
//   derivative) at a ~2 ms loop period. Because the derivative here is NOT
//   divided by dt, KD's strength scales with the loop period:
//        KD_for_your_robot = 1.2 * (2 ms / your_measured_loop_ms)
//   The sketch prints its average loop time over Serial once a second so you
//   can check it (set REPORT_LOOP_TIME=false and disable Serial for racing).
const bool REPORT_LOOP_TIME = true;

int   MAX_SPEED  = 255;            // never command more than this
float TURN_SLOW  = 1.0;            // corner braking. SPEED<->SMOOTHNESS trade, NOT
//   a speed boost: higher = smoother corners but SLOWER laps; lower = faster but
//   more wobble. Sim sweet spot = 1.0 (fastest lap that stays <=2cm on tight R18
//   corners). Use 1.5 for very smooth / 0.7 for aggressive.

// Line-recovery: what to do when ALL sensors leave the line (sharp corner/gap).
int   SEARCH_SPEED = 130;          // PWM used while pivoting to re-find the line
int   ON_LINE_MIN  = 200;          // a sensor reading (0..1000) above this = "sees line"

const uint8_t SensorCount = 16;
// Analog pins, LEFTmost sensor first .. RIGHTmost last (matches the robot's bar).
const uint8_t SENSOR_PINS[SensorCount] =
  {A15, A14, A13, A12, A11, A10, A9, A8, A7, A6, A5, A4, A3, A2, A1, A0};

/* ======================  MOTOR PINS (do not change — matches wiring) ===== */
const uint8_t ENA = 2, IN1 = 3, IN2 = 4;   // LEFT  motor
const uint8_t ENB = 7, IN3 = 6, IN4 = 5;   // RIGHT motor
const uint8_t BUTTON_PIN = 13;             // start button (to GND, uses pull-up)

/* ======================  INTERNALS  ===================================== */
QTRSensors qtr;
uint16_t sensors[SensorCount];

const float CENTER = (SensorCount - 1) * 500.0;   // ideal position (7500 for 16)
float integral = 0;
int   lastError = 0;
int   lastDir   = 1;                              // +1 line was last seen to the right

/* ----------  low-level motor helpers (same convention as the firmware) ---- */
void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// Drive each wheel with a SIGNED speed (-255..255). Negative = reverse.
void drive(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // LEFT motor
  if (leftSpeed >= 0) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  else                { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
  analogWrite(ENA, abs(leftSpeed));

  // RIGHT motor
  if (rightSpeed >= 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
  else                 { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }
  analogWrite(ENB, abs(rightSpeed));
}

/* ----------  read line position, and tell if we're still on it  ---------- */
bool onLine() {
  for (uint8_t i = 0; i < SensorCount; i++)
    if (sensors[i] > ON_LINE_MIN) return true;
  return false;
}

uint16_t readPosition() {
  return LINE_IS_BLACK ? qtr.readLineBlack(sensors)
                       : qtr.readLineWhite(sensors);
}

/* ============================  SETUP  ==================================== */
void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  stopMotors();

  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SensorCount);

  // ---- Calibration: sweep the bar across the line while this runs (~8 s) ----
  Serial.println(F("CALIBRATING - sweep sensor bar across the line..."));
  for (uint16_t i = 0; i < 400; i++) {       // 400 * ~20ms = ~8 s
    qtr.calibrate();
    digitalWrite(LED_BUILTIN, (i / 25) & 1); // blink while calibrating
  }
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("CALIBRATION DONE. Press button to start."));

  // ---- Wait for the start button (LED solid ON = ready) ----
  digitalWrite(LED_BUILTIN, HIGH);
  while (digitalRead(BUTTON_PIN) == HIGH) { /* wait for press (active LOW) */ }
  delay(50);                                  // debounce
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println(F("GO"));
}

/* ============================  MAIN LOOP  =============================== */
void reportLoopTime() {
  static unsigned long lastMicros = 0, accum = 0, count = 0, lastPrint = 0;
  unsigned long now = micros();
  if (lastMicros) { accum += now - lastMicros; count++; }
  lastMicros = now;
  if (millis() - lastPrint >= 1000 && count) {
    Serial.print(F("avg loop = ")); Serial.print(accum / (float)count / 1000.0, 2);
    Serial.println(F(" ms  (rescale KD = 1.2 * 2 / this)"));
    accum = 0; count = 0; lastPrint = millis();
  }
}

void loop() {
  if (REPORT_LOOP_TIME) reportLoopTime();

  uint16_t position = readPosition();

  // --- Recovery: line lost -> pivot toward the side we last saw it ---------
  if (!onLine()) {
    drive(SEARCH_SPEED * lastDir, -SEARCH_SPEED * lastDir);
    return;
  }

  // --- PID on the position error ------------------------------------------
  int error = (int)position - (int)CENTER;
  integral += error;
  integral = constrain(integral, -20000, 20000);     // anti-windup
  int derivative = error - lastError;

  float pid = KP * error + KI * integral + KD * derivative;

  lastError = error;
  lastDir   = (error >= 0) ? 1 : -1;                  // remember which side

  // Corner braking: the harder we steer, the more we slow the base speed.
  int base = BASE_SPEED - (int)(TURN_SLOW * fabs(pid));
  if (base < 0) base = 0;

  // pid > 0 means line is to the RIGHT -> speed up left wheel, slow right.
  int leftSpeed  = base + (int)pid;
  int rightSpeed = base - (int)pid;

  drive(leftSpeed, rightSpeed);
}
