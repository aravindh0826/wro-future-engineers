/*
  WRO 2026 Future Engineers -- Arduino Motor/Servo Controller
  ==============================================================
  Runs on the Arduino (SBM). Receives steering + speed setpoints from the
  Raspberry Pi (SBC) over a WIRED serial link and drives:
    - the front steering servo (Ackermann, single servo -- Rule 11.3)
    - the rear drive axle via a NEMA-14 stepper (STH-39D219, 1.8 deg/step
      family) + DRV8825 driver + GT2 pulley/belt reduction (Rules 11.3 /
      11.5, single shared axle -- Rule 11.13 governs max 2 driving
      motors, not used here since this build uses one)

  *** RULES CAVEAT -- READ BEFORE COMPETITION ***
  The rulebook's motor clause is worded "Teams can use any electrical DC
  motors and/or servo motors of their choice." It does not explicitly
  say "any motor type." A stepper is commonly accepted in practice, but
  this wording does not unambiguously cover it. Confirm with your
  organizer/judges before relying on this at competition -- swapping
  back to a DC motor + L298N is a straightforward revert if needed
  (previous version of this file used MOTOR_IN1/IN2/EN with L298N;
  see git history / CHANGES_STEPPER_DRIVE.md).

  WHY THIS EXISTS (vs. driving the servo/L298N straight from the Pi):
    - Servo.write() / analogWrite() use the Arduino's own hardware timers.
      Once set, the output pulse train keeps running on its own -- the Pi
      does NOT need to keep servicing it every loop iteration the way
      RPi.GPIO software PWM does. This removes a real-time timing burden
      from the Pi and eliminates the jitter that comes from Linux
      scheduling contention with the CV pipeline.
    - Setpoints only need to arrive whenever the Pi has a new vision-derived
      target (steering + speed), not at a guaranteed fixed rate. This
      Arduino keeps outputting the last good setpoint continuously in the
      meantime, so actuation is smooth even if a CV frame takes longer
      than usual.
    - A watchdog here means a stalled/dropped Pi<->Arduino link fails to a
      safe stop instead of "coast on the last command forever" -- required
      given Rule 11.6 (vehicle must be autonomous; no external/remote
      control while running -- a hung link must not act like uncommanded
      autopilot).

  RULES THIS DESIGN IS BUILT AROUND:
    - Rule 11.8 / 11.9 : more than one SBC/SBM is explicitly permitted,
                          no restriction on brand.
    - Rule 11.10        : no wireless communication components may be used
                          during competition rounds -- this link MUST stay
                          a physical wire (USB-serial / UART), never
                          Bluetooth/WiFi/RF.
    - Rule 11.17         : "Only wire connections are permitted for
                          communication between vehicle electromechanical
                          components." Same constraint, restated for the
                          Pi<->Arduino link specifically.
    - Rule 12.6          : at vehicle check, ALL controllers must be
                          powered off -- remember to power down both the
                          Pi and this Arduino, not just one.
    - Rule 11.6          : autonomous operation only -- see watchdog above.

  PROTOCOL (Pi -> Arduino, plain ASCII line, one setpoint update per line):
      "C,<steer>,<speed>\n"
        steer : float in [-1.0, 1.0]   (-1 = full left, 1 = full right,
                                         0 = straight -- same convention as
                                         the Pi's steering_value_to_duty())
        speed : int, mm/s, signed      (positive = forward, negative =
                                         reverse, 0 = stop)
      Example: "C,0.35,220\n"

  PROTOCOL (Arduino -> Pi, unsolicited telemetry line, Rule 11.11 permits
  any sensor of the team's choice, no restriction on brand/function/number):
      "U,<left_mm>,<right_mm>\n"
        left_mm/right_mm : float, distance in mm from the rear-left /
                            rear-right ultrasonic sensors. -1 means "no
                            echo" (out of range / nothing in front of that
                            sensor within ULTRASONIC_TIMEOUT_US).
      Sent every ULTRASONIC_REPORT_MS once both sensors have a fresh
      reading. Used by parking (ultrasonic_parking_controller.py) as the
      contact-avoidance and wheel-to-wall parallelism measurement during
      the blind-spot reverse-parking manoeuvre, since the forward-facing
      camera loses sight of the magenta parking markers once the vehicle
      is mid-reverse. This is the same measurement the rules use to judge
      "parked parallel" -- distance from the wheels on one side to the
      wall, difference <= 2cm (see "Parking in the parking lot", rules
      13.25-13.27) -- so it doubles as a rule-accurate alignment check,
      not just a proxy.

  CALIBRATION -- OPEN VARIABLES, NOT FIXED BY RULE:
    SERVO_CENTER_DEG / SERVO_LEFT_DEG / SERVO_RIGHT_DEG are placeholders
    -- a standard +/-30 deg travel around center, per your instruction to
    use a standard range until the real Ackermann linkage geometry is
    finalised. WHEEL_DIAMETER_MM is also a placeholder. These depend on
    your physical servo horn travel and drive wheel, and must be measured
    on the real hardware (matching how camera height/tilt in
    focal_calibrator.py/hsv_calibrator.py are resolved empirically, not
    assumed). Do not trust these numbers -- recalibrate on your chassis.
    Same applies to the ultrasonic mounting position/angle at the rear
    corners -- verify with the sensors actually bolted to the chassis,
    not before.

  STEPPER DRIVE MATH (see speedToStepIntervalUs()):
    Pi sends a target speed in mm/s (same protocol as before -- nothing
    changed on the Pi side). This file converts that to a STEP pulse
    interval using:
      wheel_rev_per_s  = speed_mm_s / (PI * WHEEL_DIAMETER_MM)
      motor_rev_per_s  = wheel_rev_per_s * GEAR_RATIO      (motor pulley
                         is smaller than the axle pulley, so the motor
                         spins GEAR_RATIO x faster than the wheel --
                         GEAR_RATIO = axle_teeth / motor_teeth = 60/20 = 3)
      steps_per_s      = motor_rev_per_s * STEPS_PER_REV * MICROSTEPPING
    STEPS_PER_REV=200 assumes the STH-39D219 is a standard 1.8 deg/step
    Shinano-Kenshi 39mm-frame motor (the whole STH-39D family is
    overwhelmingly 1.8 deg/step, a few variants are 0.9 deg/step/400
    steps) -- CONFIRM against the motor's datasheet/nameplate before
    trusting this. MICROSTEPPING=1 assumes DRV8825 MS1/MS2/MS3 are left
    at their default (full step) -- most DRV8825 breakout boards pull
    these low by default. If you wire MS1-3 for microstepping, update
    MICROSTEPPING to match (2/4/8/16/32) or the car will drive at the
    wrong speed for a given step rate.
*/

#include <Servo.h>

// ── Pin configuration -- change to match your wiring ───────────────────────
const uint8_t SERVO_PIN     = 9;   // PWM-capable pin, steering servo signal
const uint8_t STEPPER_STEP  = 6;   // DRV8825 STEP
const uint8_t STEPPER_DIR   = 7;   // DRV8825 DIR
const uint8_t STEPPER_EN    = 5;   // DRV8825 nENABLE -- active LOW (LOW = driver enabled/outputs on)

// Rear-corner ultrasonic sensors (HC-SR04), used only for parking
// (contact avoidance + wheel-to-wall parallelism). Mount facing straight
// back, as close to each rear wheel as the chassis allows, so the
// reading approximates the rule's "distance between the wheel and the
// wall" measurement.
const uint8_t ULTRA_LEFT_TRIG   = 2;
const uint8_t ULTRA_LEFT_ECHO   = 4;
const uint8_t ULTRA_RIGHT_TRIG  = 3;
const uint8_t ULTRA_RIGHT_ECHO  = 8;

// ── Servo calibration (degrees) -- standard placeholder, Ackermann
// geometry/linkage not finalised yet. +/-30 deg around center is a
// generic safe range for a hobby-servo Ackermann front axle -- narrow or
// widen once the real linkage's lock-to-lock travel is measured, or the
// servo will bind against its own horn/linkage at one or both extremes.
const int SERVO_CENTER_DEG = 90;
const int SERVO_LEFT_DEG   = 60;
const int SERVO_RIGHT_DEG  = 120;

// ── Stepper drive-train constants ───────────────────────────────────────────
// STH-39D219 -- CONFIRM against datasheet/nameplate, see header note above.
const float STEPS_PER_REV   = 200.0;   // 1.8 deg/step
const float MICROSTEPPING   = 1.0;     // DRV8825 MS1/MS2/MS3 default (full step)
// GT2 pulley/belt reduction: 20T motor pulley -> 60T axle pulley (202mm belt).
const float PULLEY_MOTOR_TEETH = 20.0;
const float PULLEY_AXLE_TEETH  = 60.0;
const float GEAR_RATIO = PULLEY_AXLE_TEETH / PULLEY_MOTOR_TEETH;   // 3.0
// PLACEHOLDER -- measure the actual drive wheel diameter (mm) and update.
// Wrong value = every mm/s target from speed_controller.py is scaled
// wrong on the real car, even though the Pi-side number looks correct.
const float WHEEL_DIAMETER_MM = 64.0;
// Hard cap on step rate regardless of computed value, so a bad
// WHEEL_DIAMETER_MM/GEAR_RATIO edit (or a stray huge speed command)
// can't ask the driver for a step rate the motor can't physically
// track (steppers lose torque and can stall/skip silently at high step
// rates -- there is no encoder feedback here to detect that). 4000
// steps/s is conservative for a NEMA-14-class motor; lower it if the
// motor stalls audibly at MAX_SPEED_MM once you test on the bench.
const float MAX_STEP_RATE_HZ = 4000.0;
// DRV8825 minimum STEP pulse width is 1.9us per the datasheet; 3us
// leaves margin without adding meaningful latency at these step rates.
const unsigned int STEP_PULSE_US = 3;

// ── Ultrasonic timing -- TUNE_ME, but timeout math is fixed by sound speed ─
// pulseIn timeout bounds worst-case blocking time per sensor read. 6000us
// caps range at ~103cm (343 m/s / 2 * 6ms), plenty for close-range parking
// and keeps the main loop responsive during normal 3-lap driving, where
// these sensors are read but not acted on.
const unsigned long ULTRA_TIMEOUT_US    = 6000UL;
const unsigned long ULTRA_INTERVAL_MS   = 60UL;   // one sensor read per interval, alternating L/R
const unsigned long ULTRA_REPORT_MS     = 120UL;  // send "U,.." at most this often

// ── Safety ───────────────────────────────────────────────────────────────
const unsigned long WATCHDOG_MS = 200;   // stop if no valid command in this window
const unsigned long BAUD_RATE   = 115200;

Servo steeringServo;
unsigned long lastCommandMillis = 0;
float currentSteer = 0.0;   // [-1, 1]
int   currentSpeed = 0;     // mm/s, signed

// Stepper pulse-generation state. Step pulses are generated in loop() via
// micros() comparison rather than blocking delay() or a hardware timer --
// deliberately NOT using Timer1 for this, because the Servo library
// already owns Timer1 on an ATmega328 (Uno/Nano); a competing Timer1
// config would break the steering servo. The step rates this drivetrain
// needs (well under MAX_STEP_RATE_HZ) are comfortably served by a
// micros()-polled loop -- loop() runs orders of magnitude faster than
// that on a 16MHz AVR.
unsigned long lastStepMicros      = 0;
unsigned long currentStepIntervalUs = 0;   // 0 = motor stopped, no stepping

String inputLine;

unsigned long lastUltraReadMillis   = 0;
unsigned long lastUltraReportMillis = 0;
bool  ultraTurnIsLeft  = true;
float ultraLeftMm      = -1.0;
float ultraRightMm     = -1.0;
bool  ultraLeftFresh    = false;
bool  ultraRightFresh   = false;

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR,  OUTPUT);
  pinMode(STEPPER_EN,   OUTPUT);
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_EN, LOW);   // active LOW -- enable the DRV8825 outputs

  pinMode(ULTRA_LEFT_TRIG,  OUTPUT);
  pinMode(ULTRA_LEFT_ECHO,  INPUT);
  pinMode(ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(ULTRA_RIGHT_ECHO, INPUT);
  digitalWrite(ULTRA_LEFT_TRIG,  LOW);
  digitalWrite(ULTRA_RIGHT_TRIG, LOW);

  steeringServo.attach(SERVO_PIN);

  applySteering(0.0);
  applySpeed(0);
  lastCommandMillis = millis();

  inputLine.reserve(32);
}

void loop() {
  readSerialCommands();
  updateUltrasonic();
  updateStepper();   // non-blocking; must run every loop() iteration

  // Watchdog: if we haven't heard from the Pi recently, fail safe.
  if (millis() - lastCommandMillis > WATCHDOG_MS) {
    applySteering(0.0);
    applySpeed(0);
  }
}

// ── Ultrasonic (rear-left / rear-right, parking only) ───────────────────────
void updateUltrasonic() {
  unsigned long nowMs = millis();
  if (nowMs - lastUltraReadMillis >= ULTRA_INTERVAL_MS) {
    lastUltraReadMillis = nowMs;
    if (ultraTurnIsLeft) {
      ultraLeftMm  = readUltrasonicMm(ULTRA_LEFT_TRIG,  ULTRA_LEFT_ECHO);
      ultraLeftFresh = true;
    } else {
      ultraRightMm = readUltrasonicMm(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO);
      ultraRightFresh = true;
    }
    ultraTurnIsLeft = !ultraTurnIsLeft;
  }

  if (ultraLeftFresh && ultraRightFresh &&
      (nowMs - lastUltraReportMillis >= ULTRA_REPORT_MS)) {
    lastUltraReportMillis = nowMs;
    Serial.print("U,");
    Serial.print(ultraLeftMm, 1);
    Serial.print(",");
    Serial.println(ultraRightMm, 1);
  }
}

float readUltrasonicMm(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long durationUs = pulseIn(echoPin, HIGH, ULTRA_TIMEOUT_US);
  if (durationUs == 0) {
    return -1.0;   // no echo within timeout -- out of range / nothing there
  }
  // distance_cm = duration_us / 58.0  ->  distance_mm = duration_us / 5.8
  return (float)durationUs / 5.8;
}

// ── Serial parsing ───────────────────────────────────────────────────────
void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      parseLine(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
      if (inputLine.length() > 64) {
        // Malformed / overlong line -- drop it rather than let it grow forever.
        inputLine = "";
      }
    }
  }
}

void parseLine(const String &line) {
  // Expect: "C,<steer>,<speed>"
  if (line.length() < 5 || line.charAt(0) != 'C' || line.charAt(1) != ',') {
    return;   // ignore anything we don't recognise
  }

  int secondComma = line.indexOf(',', 2);
  if (secondComma == -1) return;

  String steerStr = line.substring(2, secondComma);
  String speedStr = line.substring(secondComma + 1);

  float steer = steerStr.toFloat();
  int   speed = speedStr.toInt();

  steer = constrain(steer, -1.0, 1.0);

  currentSteer = steer;
  currentSpeed = speed;
  lastCommandMillis = millis();

  applySteering(currentSteer);
  applySpeed(currentSpeed);
}

// ── Actuation ────────────────────────────────────────────────────────────
void applySteering(float value) {
  // value: -1 = full left, 1 = full right, 0 = straight
  int angle;
  if (value >= 0) {
    angle = SERVO_CENTER_DEG + (int)((SERVO_RIGHT_DEG - SERVO_CENTER_DEG) * value);
  } else {
    angle = SERVO_CENTER_DEG + (int)((SERVO_CENTER_DEG - SERVO_LEFT_DEG) * value);
  }
  steeringServo.write(angle);
}

void applySpeed(int speedMmS) {
  currentSpeed = speedMmS;

  // TUNE_ME: if "forward" (positive speedMmS) drives the wheels backwards
  // on the real car, swap HIGH/LOW here -- it's a wiring-orientation
  // question with no way to know in advance which way DIR maps on your
  // build.
  digitalWrite(STEPPER_DIR, speedMmS >= 0 ? HIGH : LOW);

  currentStepIntervalUs = speedToStepIntervalUs(speedMmS);
}

// Converts a target speed (mm/s) into a STEP pulse interval (us). Returns
// 0 to mean "stopped, do not step" (both for a literal 0 mm/s command and
// for any speed so small it would compute to zero/negative interval).
unsigned long speedToStepIntervalUs(int speedMmS) {
  float speed = abs(speedMmS);
  if (speed < 1.0) return 0;

  float wheelRevPerSec = speed / (PI * WHEEL_DIAMETER_MM);
  float motorRevPerSec = wheelRevPerSec * GEAR_RATIO;
  float stepsPerSec    = motorRevPerSec * STEPS_PER_REV * MICROSTEPPING;

  if (stepsPerSec < 1.0) return 0;
  if (stepsPerSec > MAX_STEP_RATE_HZ) stepsPerSec = MAX_STEP_RATE_HZ;

  return (unsigned long)(1000000.0 / stepsPerSec);
}

// Non-blocking STEP pulse generator -- call every loop() iteration.
// Emits one STEP pulse whenever currentStepIntervalUs has elapsed since
// the last one; does nothing when currentStepIntervalUs is 0 (stopped).
// The only blocking call here is the STEP_PULSE_US (3us) HIGH hold,
// which is negligible next to WATCHDOG_MS/serial/ultrasonic timing.
void updateStepper() {
  if (currentStepIntervalUs == 0) return;

  unsigned long nowUs = micros();
  if (nowUs - lastStepMicros >= currentStepIntervalUs) {
    lastStepMicros = nowUs;
    digitalWrite(STEPPER_STEP, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEPPER_STEP, LOW);
  }
}
