#include <Wire.h>
#include <AS5600.h>
const char* myversion = "SimMotion4.ino";
// -------------------- SENSORS --------------------
AMS_5600 pitchSensor;
AMS_5600 rollSensor;

// -------------------- MOTOR PINS -----------------
const int STEP_PITCH = 2;
const int DIR_PITCH  = 3;
const int STEP_ROLL  = 4;
const int DIR_ROLL   = 5;
const int STEP_YAW   = 6;
const int DIR_YAW    = 9;

// -------------------- MECHANICS ------------------
// deg/step at shaft
const float RES_PITCH_ROLL = 0.0045;  // deg/step at 4000 stp/rev
const float RES_YAW        = 0.0045;  // deg/step

// lever ratio: encoder moves 3.2× more than platform
const float LEVER_RATIO = 3.2f;

// yaw tracking
const float DEG_PER_YAW_STEP = RES_YAW;
long yawStepCount = (long)(180 / DEG_PER_YAW_STEP);

// max frequencies
const int MAX_FREQ_PITCH_ROLL = 14000;
const int MAX_FREQ_YAW        = 5000;

// platform limits (deg)
const float LIMIT_PITCH = 17.0;
const float LIMIT_ROLL  = 15.0;

// encoder zero offsets (deg at shaft)
float EncPitchZero = 100.0;
float EncRollZero  = 123.0;

// -------------------- MOTION INPUT ----------------
struct MotionInput {
    // raw incoming values (platform angles)
    float pitch, roll, yaw, airspeed;

    // previous values
    float pitch_prev, roll_prev, yaw_prev;

    // computed rates (deg/s)
    float pitchRate, rollRate, yawRate;

    // computed accelerations (deg/s²)
    float pitchAcc, rollAcc, yawAcc;

    // computed jerk (deg/s³)
    float pitchJerk, rollJerk, yawJerk;

    // timestamps
    uint32_t timestamp_us;
    uint32_t dt_us;
} motion;

// limits for derived quantities
const float MAX_RATE = 60.0f;     // deg/s
const float MAX_ACC  = 200.0f;    // deg/s²
const float MAX_JERK = 2000.0f;   // deg/s³

// desired platform angles (deg)
float pitch_desired = 0.0f;
float roll_desired  = 0.0f;

// -------------------- CONTROL LOOP ----------------
bool servoEnabled = false;
bool servoPendingHandover = false;
float handoverPitch = 0.0f;
float handoverRoll  = 0.0f;

elapsedMicros controlTimer;
const uint32_t CONTROL_PERIOD_US = 1000; // 1 kHz

// PD gains (tune as needed)
float Kp_pitch = 12.0f;
float Kd_pitch = 0.5f;
float Kp_roll  = 12.0f;
float Kd_roll  = 0.5f;

// for derivative term
float pitchErr_prev = 0.0f;
float rollErr_prev  = 0.0f;

// -------------------- WATCHDOG --------------------
unsigned long lastPacketTime = 0;
const unsigned long timeoutMs = 200;  // 200 ms
bool streamLost = false;

bool controlLoopEnabled = false;
uint32_t lastXTimestamp_us = 0;
uint32_t xInterval_us = 0;
// threshold for minimum acceptable rate (2 Hz)
const uint32_t MIN_RATE_INTERVAL_US = 500000;  // 0.5 seconds


// -------------------- POWER RELAY -----------------
int motionPowerToggle = 0;
const int RELAY_PIN = 8;

// -------------------- DEBUG -----------------------
float PrintMaxFreq = 0;


// --------------------------------------------------
// READ ENCODERS → PLATFORM ANGLES (deg)
// --------------------------------------------------
void readPitchRollDegrees(float& pitchDeg, float& rollDeg) {
    // raw shaft angles in deg
    float rawPitch = pitchSensor.getRawAngle() * 0.087890625 - EncPitchZero;
    float rawRoll  = rollSensor.getRawAngle()  * 0.087890625 - EncRollZero;

    // convert shaft → platform
    pitchDeg = rawPitch / LEVER_RATIO;
    rollDeg  = rawRoll  / LEVER_RATIO;
}

// --------------------------------------------------
// DRIVE AXIS (existing PWM-based step generator)
// rate = commanded platform rate (deg/s)
// --------------------------------------------------
void driveAxis(float rate, float resolution, int maxFreq, int stepPin, int dirPin) {
    int dir = (rate >= 0) ? HIGH : LOW;
    digitalWrite(dirPin, dir);

    float absRate = fabs(rate);
    int freq = absRate / resolution;  // resolution is deg/step at shaft

    freq = constrain(freq, 0, maxFreq);
    analogWriteFrequency(stepPin, freq);
    analogWrite(stepPin, freq > 0 ? 127 : 0);  // 50% duty cycle if active

    if (freq > PrintMaxFreq) {
        PrintMaxFreq = freq;  // DEBUG
    }
}

// --------------------------------------------------
// APPLY LIMITS TO DESIRED MOTION (PLATFORM ANGLES)
// --------------------------------------------------
void applyLimits() {
    // clamp desired angles to mechanical limits
    pitch_desired = constrain(pitch_desired, -LIMIT_PITCH, LIMIT_PITCH);
    roll_desired  = constrain(roll_desired,  -LIMIT_ROLL,  LIMIT_ROLL);
}

// --------------------------------------------------
// PARSE X; STREAM (STEP 1)
// Format: X; pitch; roll; yaw; airspeed;
// All angles are platform angles (deg)
// --------------------------------------------------
void parsePacket(char* line) {
    // check if values are in a certain rate
    // measure arrival interval
    uint32_t now_us = micros();
    xInterval_us = now_us - lastXTimestamp_us;
    lastXTimestamp_us = now_us;
    // enable/disable control loop based on rate
    if (xInterval_us < MIN_RATE_INTERVAL_US) {
        controlLoopEnabled = true;
    } else {
        controlLoopEnabled = false;
    }


    // 1) timestamp
    uint32_t now = micros();
    motion.dt_us = now - motion.timestamp_us;
    motion.timestamp_us = now;
    if (motion.dt_us == 0) motion.dt_us = 1;
    float dt = motion.dt_us / 1e6f;

    // 2) save previous values
    motion.pitch_prev = motion.pitch;
    motion.roll_prev  = motion.roll;
    motion.yaw_prev   = motion.yaw;

    // 3) parse tokens
    char* token = strtok(line, ";");
    int field = 0;

    while (token != nullptr) {
        switch (field) {
            case 1: motion.pitch    = atof(token); break;
            case 2: motion.roll     = atof(token); break;
            case 3: motion.yaw      = atof(token); break;
            case 4: motion.airspeed = atof(token); break;
        }
        field++;
        token = strtok(nullptr, ";");
    }

    // 4) compute rates (deg/s)
    motion.pitchRate = (motion.pitch - motion.pitch_prev) / dt;
    motion.rollRate  = (motion.roll  - motion.roll_prev)  / dt;
    motion.yawRate   = (motion.yaw   - motion.yaw_prev)   / dt;

    // 5) compute accelerations (deg/s²)
    static float pitchRate_prev = 0, rollRate_prev = 0, yawRate_prev = 0;
    motion.pitchAcc = (motion.pitchRate - pitchRate_prev) / dt;
    motion.rollAcc  = (motion.rollRate  - rollRate_prev)  / dt;
    motion.yawAcc   = (motion.yawRate   - yawRate_prev)   / dt;

    // 6) compute jerk (deg/s³)
    static float pitchAcc_prev = 0, rollAcc_prev = 0, yawAcc_prev = 0;
    motion.pitchJerk = (motion.pitchAcc - pitchAcc_prev) / dt;
    motion.rollJerk  = (motion.rollAcc  - rollAcc_prev)  / dt;
    motion.yawJerk   = (motion.yawAcc   - yawAcc_prev)   / dt;

    // 7) apply limits
    motion.pitchRate = constrain(motion.pitchRate, -MAX_RATE, MAX_RATE);
    motion.rollRate  = constrain(motion.rollRate,  -MAX_RATE, MAX_RATE);
    motion.yawRate   = constrain(motion.yawRate,   -MAX_RATE, MAX_RATE);

    motion.pitchAcc = constrain(motion.pitchAcc, -MAX_ACC, MAX_ACC);
    motion.rollAcc  = constrain(motion.rollAcc,  -MAX_ACC, MAX_ACC);
    motion.yawAcc   = constrain(motion.yawAcc,   -MAX_ACC, MAX_ACC);

    motion.pitchJerk = constrain(motion.pitchJerk, -MAX_JERK, MAX_JERK);
    motion.rollJerk  = constrain(motion.rollJerk,  -MAX_JERK, MAX_JERK);
    motion.yawJerk   = constrain(motion.yawJerk,   -MAX_JERK, MAX_JERK);

    // 8) save for next iteration
    pitchRate_prev = motion.pitchRate;
    rollRate_prev  = motion.rollRate;
    yawRate_prev   = motion.yawRate;

    pitchAcc_prev = motion.pitchAcc;
    rollAcc_prev  = motion.rollAcc;
    yawAcc_prev   = motion.yawAcc;

    // 9) desired platform angles from motion input
    pitch_desired = motion.pitch;
    roll_desired  = motion.roll;

    // 10) if switching on servo loop, approximate axis first
    if (servoPendingHandover) {
    handoverPitch = pitch_desired;
    handoverRoll  = roll_desired;

    servoPendingHandover = false;

    // Move to initial streaming position using position mode
    moveToTarget('P', handoverPitch, MAX_FREQ_PITCH_ROLL);
    moveToTarget('R', handoverRoll,  MAX_FREQ_PITCH_ROLL);

    servoEnabled = true;
    Serial.println("C;ON");
}

}

// --------------------------------------------------
// CLOSED-LOOP CONTROL (STEP 2) - PITCH & ROLL ONLY
// --------------------------------------------------
void updateControlLoop() {
    // C - command driven
    if (!servoEnabled) { return; }
    
    // frame rate driven 
    if (!controlLoopEnabled) {
        // Control loop disabled due to low update rate
        // Motors are left in their last safe state
        return;
    }

    // Run at 1 kHz
    if (controlTimer < CONTROL_PERIOD_US) return;
    controlTimer -= CONTROL_PERIOD_US;

    // 1) Read actual platform angles from encoders
    float pitch_meas, roll_meas;
    readPitchRollDegrees(pitch_meas, roll_meas);

    // 2) Enforce mechanical limits on measured angles
    pitch_meas = constrain(pitch_meas, -LIMIT_PITCH, LIMIT_PITCH);
    roll_meas  = constrain(roll_meas,  -LIMIT_ROLL,  LIMIT_ROLL);

    // 3) Enforce mechanical limits on desired angles
    pitch_desired = constrain(pitch_desired, -LIMIT_PITCH, LIMIT_PITCH);
    roll_desired  = constrain(roll_desired,  -LIMIT_ROLL,  LIMIT_ROLL);

    // 4) Compute control errors
    float pitchErr = pitch_desired - pitch_meas;
    float rollErr  = roll_desired  - roll_meas;

    // 5) PD control with derivative filtering
    float dt = CONTROL_PERIOD_US / 1e6f;

    float pitchDeriv = (pitchErr - pitchErr_prev) / dt;
    float rollDeriv  = (rollErr  - rollErr_prev)  / dt;

    pitchErr_prev = pitchErr;
    rollErr_prev  = rollErr;

    // --- Step 4: Derivative filtering ---
    static float pitchDerivFilt = 0.0f;
    static float rollDerivFilt  = 0.0f;

    const float D_FILTER = 0.1f;  // 0 = no filter, 1 = heavy filter

    pitchDerivFilt = pitchDerivFilt * (1.0f - D_FILTER) + pitchDeriv * D_FILTER;
    rollDerivFilt  = rollDerivFilt  * (1.0f - D_FILTER) + rollDeriv  * D_FILTER;

    // --- Step 4: Updated PD gains ---
    float pitch_cmd = Kp_pitch * pitchErr + Kd_pitch * pitchDerivFilt;
    float roll_cmd  = Kp_roll  * rollErr  + Kd_roll  * rollDerivFilt;

    // 6) Limit commanded rates (deg/s)
    const float MAX_CMD_RATE = 120.0f;  // increased for faster response
    pitch_cmd = constrain(pitch_cmd, -MAX_CMD_RATE, MAX_CMD_RATE);
    roll_cmd  = constrain(roll_cmd,  -MAX_CMD_RATE, MAX_CMD_RATE);

    // 7) Safety: stop pushing against mechanical limits
    if ((pitch_meas >= LIMIT_PITCH && pitchErr > 0) ||
        (pitch_meas <= -LIMIT_PITCH && pitchErr < 0)) {
        pitch_cmd = 0;
    }

    if ((roll_meas >= LIMIT_ROLL && rollErr > 0) ||
        (roll_meas <= -LIMIT_ROLL && rollErr < 0)) {
        roll_cmd = 0;
    }

    // 8) Drive motors (pitch & roll only)
    driveAxis(pitch_cmd, RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_PITCH, DIR_PITCH);
    driveAxis(roll_cmd,  RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_ROLL,  DIR_ROLL);

    // Yaw intentionally excluded from closed-loop control
}



void updateControlLoop_OLD() {
    if (!controlLoopEnabled) {
        // stop motors safely if update rate is to slow
        //driveAxis(0, RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_PITCH, DIR_PITCH);
        //driveAxis(0, RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_ROLL,  DIR_ROLL);
        return;
    }

    // run at 1 kHz
    if (controlTimer < CONTROL_PERIOD_US) return;
    controlTimer -= CONTROL_PERIOD_US;

    // 1) read actual platform angles from encoders
    float pitch_meas, roll_meas;
    readPitchRollDegrees(pitch_meas, roll_meas);

    // 2) enforce mechanical limits on measured angles
    pitch_meas = constrain(pitch_meas, -LIMIT_PITCH, LIMIT_PITCH);
    roll_meas  = constrain(roll_meas,  -LIMIT_ROLL,  LIMIT_ROLL);

    // 3) enforce mechanical limits on desired angles
    pitch_desired = constrain(pitch_desired, -LIMIT_PITCH, LIMIT_PITCH);
    roll_desired  = constrain(roll_desired,  -LIMIT_ROLL,  LIMIT_ROLL);

    // 4) compute control errors
    float pitchErr = pitch_desired - pitch_meas;
    float rollErr  = roll_desired  - roll_meas;

    // 5) PD control
    float dt = CONTROL_PERIOD_US / 1e6f;

    float pitchDeriv = (pitchErr - pitchErr_prev) / dt;
    float rollDeriv  = (rollErr  - rollErr_prev)  / dt;

    float pitch_cmd = Kp_pitch * pitchErr + Kd_pitch * pitchDeriv;
    float roll_cmd  = Kp_roll  * rollErr  + Kd_roll  * rollDeriv;

    pitchErr_prev = pitchErr;
    rollErr_prev  = rollErr;

    // 6) limit commanded rates (deg/s)
    const float MAX_CMD_RATE = 120.0f;
    pitch_cmd = constrain(pitch_cmd, -MAX_CMD_RATE, MAX_CMD_RATE);
    roll_cmd  = constrain(roll_cmd,  -MAX_CMD_RATE, MAX_CMD_RATE);

    // 7) safety: stop pushing against mechanical limits
    if ((pitch_meas >= LIMIT_PITCH && pitchErr > 0) ||
        (pitch_meas <= -LIMIT_PITCH && pitchErr < 0)) {
        pitch_cmd = 0;
    }

    if ((roll_meas >= LIMIT_ROLL && rollErr > 0) ||
        (roll_meas <= -LIMIT_ROLL && rollErr < 0)) {
        roll_cmd = 0;
    }

    // 8) drive motors (pitch & roll only)
    driveAxis(pitch_cmd, RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_PITCH, DIR_PITCH);
    driveAxis(roll_cmd,  RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_ROLL,  DIR_ROLL);

    // yaw intentionally excluded from closed-loop control
}

// --------------------------------------------------
// MOVE-TO-TARGET (P/R/Y) - unchanged logic for commands
// --------------------------------------------------
void moveToTarget(char axis, float targetDeg, int targetSpeed) {
  const float tolerance = 0.5;

  int stepPin, dirPin;
  AMS_5600* sensor = nullptr;
  float zeroOffset = 0.0;
  float resolution = 0.0;
  int maxFreq = 0;

  switch (axis) {
  case 'P':
      stepPin = STEP_PITCH; dirPin = DIR_PITCH;
      sensor = &pitchSensor; zeroOffset = EncPitchZero;
      resolution = RES_PITCH_ROLL; maxFreq = MAX_FREQ_PITCH_ROLL;

      // Convert platform → shaft
      targetDeg = targetDeg * LEVER_RATIO;

      // Apply encoder zero offset (shaft degrees)
      targetDeg += zeroOffset;
      break;

  case 'R':
      stepPin = STEP_ROLL; dirPin = DIR_ROLL;
      sensor = &rollSensor; zeroOffset = EncRollZero;
      resolution = RES_PITCH_ROLL; maxFreq = MAX_FREQ_PITCH_ROLL;

      // Convert platform → shaft
      targetDeg = targetDeg * LEVER_RATIO;

      // Apply encoder zero offset (shaft degrees)
      targetDeg += zeroOffset;
      break;


    case 'Y': {
        stepPin = STEP_YAW; dirPin = DIR_YAW;
        resolution = RES_YAW; maxFreq = MAX_FREQ_YAW / 2;

        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);

        float currentYaw = yawStepCount * resolution;
        float error = targetDeg - currentYaw;
        int dir = (error >= 0) ? HIGH : LOW;
        digitalWrite(dirPin, dir);

        long steps = abs(error / resolution);
        targetSpeed = constrain(targetSpeed, 0, maxFreq);

        int minSpeed   = 50;   // Hz
        int accelStep  = 5;    // Hz increment per step
        int currentSpeed = minSpeed;

        long accelSteps = (targetSpeed - minSpeed) / accelStep;
        bool triangle = (2 * accelSteps > steps);

        for (long i = 0; i < steps; i++) {
            if (triangle) {
                if (i < steps / 2) currentSpeed += accelStep;
                else               currentSpeed -= accelStep;
            } else {
                if (i < accelSteps) currentSpeed += accelStep;
                else if (i >= steps - accelSteps) currentSpeed -= accelStep;
                else currentSpeed = targetSpeed;
            }

            currentSpeed = constrain(currentSpeed, minSpeed, targetSpeed);
            int period_us = 1000000 / currentSpeed;

            digitalWrite(stepPin, HIGH);
            delayMicroseconds(period_us / 2);
            digitalWrite(stepPin, LOW);
            delayMicroseconds(period_us / 2);

            yawStepCount += (dir == HIGH) ? 1 : -1;
        }
        return;
    }
    default:
      Serial.println("Invalid axis for moveToTarget");
      return;
  }

  if (axis == 'P' || axis == 'R') {
      targetSpeed = constrain(targetSpeed, 0, maxFreq);
      float initialError = fabs(targetDeg - sensor->getRawAngle() * 0.087890625);
      float decelThreshold = initialError / 2.0;

      int currentSpeed = 0;
      int accelStep   = 100;
      int minSpeed    = 200;
      int rampDelay   = 20;

      analogWrite(stepPin, 0);
      analogWriteFrequency(stepPin, 1);
      delay(5);

      while (true) {
          if (Serial.available() > 0) {
              analogWrite(stepPin, 0);
              break;
          }

          float deg = sensor->getRawAngle() * 0.087890625;
          float error = targetDeg - deg;

          if (fabs(error) < tolerance) {
              analogWrite(stepPin, 0);
              Serial.print(axis);
              Serial.println(";DONE");
              break;
          }

          int dir = (error >= 0) ? HIGH : LOW;
          digitalWrite(dirPin, dir);

          if (currentSpeed < targetSpeed && fabs(error) > decelThreshold) {
              currentSpeed = min(currentSpeed + accelStep, targetSpeed);
          }

          if (fabs(error) < decelThreshold) {
              currentSpeed = max(currentSpeed - accelStep, minSpeed);
          }

          if (currentSpeed > 0) {
              analogWriteFrequency(stepPin, currentSpeed);
              analogWrite(stepPin, 127);
          } else {
              analogWrite(stepPin, 0);
          }

          delay(rampDelay);
      }
  }
}

// --------------------------------------------------
// SETUP
// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial);
  //Serial.println(myversion);

  Wire.begin();
  Wire1.begin();
  pitchSensor.setWire(&Wire);
  rollSensor.setWire(&Wire1);

  while (!pitchSensor.detectMagnet()) {
    Serial.println("Pitch sensor error!");
    delay(500);
  }
  while (!rollSensor.detectMagnet()) {
    Serial.println("Roll sensor error!");
    delay(500);
  }

  Serial.println("Sensors ok");

  pinMode(STEP_PITCH, OUTPUT);
  pinMode(DIR_PITCH, OUTPUT);
  pinMode(STEP_ROLL, OUTPUT);
  pinMode(DIR_ROLL, OUTPUT);
  pinMode(STEP_YAW, OUTPUT);
  pinMode(DIR_YAW, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  analogWriteResolution(8);  // 0–255
}

// --------------------------------------------------
// LOOP
// --------------------------------------------------
void loop() {
  static char buffer[256];
  static int index = 0;

  // watchdog: if stream lost, bring platform back to neutral
  if (millis() - lastPacketTime > timeoutMs) {
    pitch_desired = 0.0f;
    roll_desired  = 0.0f;

    if (!streamLost) {
      Serial.println("!;ERROR: stream interrupted");
      streamLost = true;
    }
  } else {
    streamLost = false;
  }

  // closed-loop control at 1 kHz
  updateControlLoop();

  // serial command handling
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0';
      if (index > 0 && buffer[index - 1] == '\r') {
        buffer[index - 1] = '\0';
      }

      if (strncmp(buffer, "X;", 2) == 0) {
        lastPacketTime = millis();
        parsePacket(buffer);
        applyLimits();
      } else if (strcmp(buffer, "H") == 0 || strcmp(buffer, "H;") == 0) {
        moveToTarget('P', 0.0, MAX_FREQ_PITCH_ROLL);
        moveToTarget('R', 0.0, MAX_FREQ_PITCH_ROLL);
        Serial.println("H;DONE");
      } else if (strncmp(buffer, "M;", 2) == 0) {
        motionPowerToggle = atoi(buffer + 2);
        if (motionPowerToggle == 1) {
          digitalWrite(RELAY_PIN, HIGH);
          delay(1000); // give time to power and initial
          Serial.println("M;ON");
        } else {
          digitalWrite(RELAY_PIN, LOW);
          Serial.println("M;OFF");
        }
      } else if (strncmp(buffer, "E;", 2) == 0) {
        float pitchDeg, rollDeg;
        readPitchRollDegrees(pitchDeg, rollDeg);
        float yawDeg = yawStepCount * DEG_PER_YAW_STEP;
        Serial.print("E;pitch=");
        Serial.print(pitchDeg, 2);
        Serial.print(";roll=");
        Serial.print(rollDeg, 2);
        Serial.print(";yaw=");
        Serial.print(yawDeg, 2);
        Serial.print(";motionPowerToggle=");
        Serial.println(motionPowerToggle);
        Serial.print(";controlLoop=");
        Serial.println(servoPendingHandover);
      } else if (strncmp(buffer, "P;", 2) == 0) {
        float target = constrain(atof(buffer + 2), -LIMIT_PITCH, LIMIT_PITCH);
        moveToTarget('P', target, MAX_FREQ_PITCH_ROLL);
        Serial.println("P;DONE");
      } else if (strncmp(buffer, "R;", 2) == 0) {
        float target = constrain(atof(buffer + 2), -LIMIT_ROLL, LIMIT_ROLL);
        moveToTarget('R', target, MAX_FREQ_PITCH_ROLL);
        Serial.println("R;DONE");
      } else if (strncmp(buffer, "Y;", 2) == 0) {
        float target = atof(buffer + 2);
        moveToTarget('Y', target, MAX_FREQ_YAW);
        Serial.println("Y;DONE");
      } else if (strncmp(buffer, "Z;", 2) == 0) {
        float yawDeg = atof(buffer + 2);
        yawStepCount = (long)(yawDeg / DEG_PER_YAW_STEP);
        Serial.print("Z;");
        Serial.println(yawDeg);
      } else if (strncmp(buffer, "C;", 2) == 0) {
            int val = atoi(buffer + 2);
            if (val == 0) {
                servoEnabled = false;
                servoPendingHandover = false;
                Serial.println("C;OFF");
            } else {
                servoPendingHandover = true;
                servoEnabled = false;
                Serial.println("C;ON");
            }
      } else if (strcmp(buffer, "?") == 0 || strcmp(buffer, "?;") == 0) {
        Serial.println(myversion);
        Serial.println("Available commands:");
        Serial.println("X;... -> live motion stream from simulator");
        Serial.println("H; -> homes pitch and roll to center");
        Serial.println("E; -> prints current pitch, roll, yaw and power status");
        Serial.println("V; -> firmware version");
        Serial.println("C;[bool] -> enable / disable control loop");
        Serial.println("M;[bool] -> toggle motion platform power");
        Serial.println("P;[angle] -> move pitch to [angle] (±17° limit)");
        Serial.println("R;[angle] -> move roll to [angle] (±15° limit)");
        Serial.println("Y;[angle] -> move yaw to [angle] (step count only)");
        Serial.println("Z;[angle] -> set yaw counter to [angle]");
        Serial.println("?; -> prints this help menu");
      } else if (strcmp(buffer, "V") == 0 || strcmp(buffer, "V;") == 0) {
        Serial.println(myversion);
      } else {
        Serial.print("Ignored unknown command: ");
        Serial.println(buffer);
      }
      index = 0;
    } else if (index < (int)sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}
