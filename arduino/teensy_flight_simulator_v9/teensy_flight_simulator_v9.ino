#include <Wire.h>
#include <AS5600.h>

// AS5600 sensor instances
AMS_5600 pitchSensor;
AMS_5600 rollSensor;

// Motor control pins
const int STEP_PITCH = 2;
const int DIR_PITCH  = 3;
const int STEP_ROLL  = 4;
const int DIR_ROLL   = 5;
const int STEP_YAW   = 6;
const int DIR_YAW    = 9;

// Motion resolution and limits
//const float RES_PITCH_ROLL = 0.00045;  // deg/step at 40000stp/rev
const float RES_PITCH_ROLL = 0.0045;  // deg/step at 4000stp/rev
const float RES_YAW        = 0.0045;   // deg/step

const float DEG_PER_YAW_STEP = RES_YAW;
long yawStepCount = (long)(180 / DEG_PER_YAW_STEP); ;                 // yaw position tracker

const int MAX_FREQ_PITCH_ROLL = 70000;
const int MAX_FREQ_YAW        = 5000;

const float LIMIT_PITCH = 72.0;
const float LIMIT_ROLL  = 45.0;
float EncPitchZero = 100.0;
float EncRollZero = 123.0;

// Incoming data from X-Plane
float pitch = 0, pitch_rate = 0;
float roll = 0, roll_rate = 0;
float yaw = 0, yaw_rate = 0;
float airspeed = 0;

float PrintMaxFreq = 0;

// watchdog
unsigned long lastPacketTime = 0;
const unsigned long timeoutMs = 200;  // 200ms = 10 missed packets at 50Hz
bool streamLost = false;

int motionPowerToggle = 0;
const int RELAY_PIN = 8;

const char* myversion = "teensy_flight_simulator_v91.ino";

// Initializes serial, sensors, and motor pins
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println(myversion);

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

// Main loop: parses serial commands and dispatches actions
void loop() {
  static char buffer[256];
  static int index = 0;

  // watchdog
  if (millis() - lastPacketTime > timeoutMs) {
    pitch_rate = 0;
    roll_rate = 0;
    yaw_rate = 0;

    if (!streamLost) {
      Serial.println("!;ERROR: stream interrupted");
      streamLost = true;
    }
  } else {
    // Stream is active again — reset the flag
    streamLost = false;
  }
  driveMotors();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer[index] = '\0';
      // Trim trailing carriage return
      if (index > 0 && buffer[index - 1] == '\r') {
        buffer[index - 1] = '\0';
      }
      // DEBUG;
      // Serial.print("Received: [");
      // Serial.print(buffer);
      // Serial.println("]");
      // Serial.print("pitch_rate = ");
      // Serial.println(pitch_rate);
      if (strncmp(buffer, "X;", 2) == 0) {
        // X-PLANE STREAM DECODING
        lastPacketTime = millis();  // watchdog
        parsePacket(buffer);
        applyLimits();
        driveMotors();
      } else if (strcmp(buffer, "H") == 0 || strcmp(buffer, "H;") == 0) {
        // HOMING ROUTINE
        moveToTarget('P', 0.0, MAX_FREQ_PITCH_ROLL);
        moveToTarget('R', 0.0, MAX_FREQ_PITCH_ROLL);
        Serial.println("H;DONE");
      } else if (strncmp(buffer, "M;", 2) == 0) {
          // TOGGLE MOTION PLATFORM POWER
          motionPowerToggle = atoi(buffer + 2);  // parse digits after "M;"
          if (motionPowerToggle == 1) {
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("M;ON");
          } else {
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("M;OFF");
          }
      } else if (strncmp(buffer, "E;", 2) == 0) {
        // RETURN CURRENT POSITIONS
        float pitchDeg, rollDeg;
        readPitchRollDegrees(pitchDeg, rollDeg);
        float yawDeg = yawStepCount * DEG_PER_YAW_STEP;
        Serial.print("Max Frequ=");
        Serial.println(PrintMaxFreq);
        PrintMaxFreq = 0;
        Serial.print("E;pitch=");
        Serial.print(pitchDeg, 2);
        Serial.print(";roll=");
        Serial.print(rollDeg, 2);
        Serial.print(";yaw=");
        Serial.print(yawDeg, 2);
        Serial.print(";motionPowerToggle=");
        Serial.println(motionPowerToggle);
      } else if (strncmp(buffer, "P;", 2) == 0) {
        // MOVE PITCH AXIS IN DEGREE
        float target = constrain(atof(buffer + 2), -LIMIT_PITCH, LIMIT_PITCH);
        moveToTarget('P', target, MAX_FREQ_PITCH_ROLL);
        Serial.println("P;DONE");
      } else if (strncmp(buffer, "R;", 2) == 0) {
        // MOVES ROLL AXIS IN DEGREE
        float target = constrain(atof(buffer + 2), -LIMIT_ROLL, LIMIT_ROLL);
        moveToTarget('R', target, MAX_FREQ_PITCH_ROLL);
        Serial.println("R;DONE");
      } else if (strncmp(buffer, "Y;", 2) == 0) {
        // MOVES YAW AXIS IN DEGREE
        float target = atof(buffer + 2);
        moveToTarget('Y', target, MAX_FREQ_YAW);
        Serial.println("Y;DONE");
      } else if (strncmp(buffer, "Z;", 2) == 0) {
        // SET YAW COUNTER TO VALUE
        // yawStepCount = atof(buffer + 2);
        // Serial.print("Z;");
        // Serial.println(yawStepCount);
        // Serial.print("Raw buffer: '");
        // Serial.print(buffer);
        // Serial.println("'");
        float yawDeg= atof(buffer + 2);  // convert string to long
        yawStepCount = (long)(yawDeg / DEG_PER_YAW_STEP);
        Serial.print("Z;");
        Serial.println(yawDeg);
      } else if (strcmp(buffer, "?") == 0 || strcmp(buffer, "?;") == 0) {
        // RETURN COMMAND LIST
        Serial.println(myversion);
        Serial.println("Available commands:");
        Serial.println("X;... -> live motion stream from X-Plane");
        Serial.println("H; -> homes pitch and roll to center");
        Serial.println("E; -> prints current pitch, roll, yaw and power status");
        Serial.println("V; -> firmware version");
        Serial.println("M;[bool] -> toggle motion platform power");
        Serial.println("P;[angle] -> move pitch to [angle] (±30° limit)");
        Serial.println("R;[angle] -> move roll to [angle] (±30° limit)");
        Serial.println("Y;[angle] -> move yaw to [angle] (step count only)");
        Serial.println("Z;[angle] -> set yaw counter to [angle]");
        Serial.println("?; -> prints this help menu");
      } else if (strcmp(buffer, "V") == 0 || strcmp(buffer, "V;") == 0) {
        // RETURN FIRMWARE VERSION
        Serial.println(myversion);
      }      
      else {
        Serial.print("Ignored unknown command: ");
        Serial.println(buffer);
      }
      index = 0;
    } else if (index < sizeof(buffer) - 1) {
      buffer[index++] = c;
    }
  }
}

// Moves specified axis to target angle with speed ramping
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
      targetDeg += zeroOffset;
      break;

    case 'R':
      stepPin = STEP_ROLL; dirPin = DIR_ROLL;
      sensor = &rollSensor; zeroOffset = EncRollZero;
      resolution = RES_PITCH_ROLL; maxFreq = MAX_FREQ_PITCH_ROLL;
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

        // Steps needed to reach targetSpeed
        long accelSteps = (targetSpeed - minSpeed) / accelStep;

        bool triangle = (2 * accelSteps > steps);

        for (long i = 0; i < steps; i++) {
            if (triangle) {
                // Triangle profile: accelerate until halfway, then decelerate
                if (i < steps / 2) {
                    currentSpeed += accelStep;
                } else {
                    currentSpeed -= accelStep;
                }
            } else {
                // Trapezoid profile
                if (i < accelSteps) {
                    currentSpeed += accelStep; // accelerate
                } else if (i >= steps - accelSteps) {
                    currentSpeed -= accelStep; // decelerate
                } else {
                    // cruise at targetSpeed
                    currentSpeed = targetSpeed;
                }
            }

            // Clamp speed
            currentSpeed = constrain(currentSpeed, minSpeed, targetSpeed);

            // Convert frequency to period
            int period_us = 1000000 / currentSpeed;

            // One step pulse
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


      int currentSpeed = 1000;     // start low
      int accelStep   = 1000;       // Hz increment per loop
      int minSpeed    = 1000;      // floor speed
      int rampDelay   = 20;         // ms between increments

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

            // Ramp up until targetSpeed
            if (currentSpeed < targetSpeed && fabs(error) > decelThreshold) {
                currentSpeed = min(currentSpeed + accelStep, targetSpeed);
            }

            // Ramp down when within decel zone
            if (fabs(error) < decelThreshold) {
                currentSpeed = max(currentSpeed - accelStep, minSpeed);
            }

            analogWriteFrequency(stepPin, currentSpeed);
            analogWrite(stepPin, 127);
            delay(rampDelay); // let PWM run at this speed
        }
  }


}




// Moves specified axis to target angle at given speed
// void moveToTarget(char axis, float targetDeg, int speed) {
//   const float tolerance = 0.5;

//   int stepPin, dirPin;
//   AMS_5600* sensor = nullptr;
//   float zeroOffset = 0.0;
//   float resolution = 0.0;
//   int maxFreq = 0;

//   switch (axis) {
//     case 'P':
//       stepPin = STEP_PITCH; dirPin = DIR_PITCH;
//       sensor = &pitchSensor; zeroOffset = EncPitchZero;
//       resolution = RES_PITCH_ROLL; maxFreq = MAX_FREQ_PITCH_ROLL;
//       targetDeg += zeroOffset;
//       break;

//     case 'R':
//       stepPin = STEP_ROLL; dirPin = DIR_ROLL;
//       sensor = &rollSensor; zeroOffset = EncRollZero;
//       resolution = RES_PITCH_ROLL; maxFreq = MAX_FREQ_PITCH_ROLL;
//       targetDeg += zeroOffset;
//       break;

//     case 'Y': {
//       stepPin = STEP_YAW; dirPin = DIR_YAW;
//       resolution = RES_YAW; maxFreq = MAX_FREQ_YAW;

//       float currentYaw = yawStepCount * resolution;
//       float error = targetDeg - currentYaw;
//       int dir = (error >= 0) ? HIGH : LOW;
//       digitalWrite(dirPin, dir);

//       long steps = abs(error / resolution);
//       speed = constrain(speed, 0, maxFreq);

//       for (long i = 0; i < steps; i++) {
//         analogWriteFrequency(stepPin, speed);
//         analogWrite(stepPin, 127);
//         delayMicroseconds(100);
//         analogWrite(stepPin, 0);
//         delayMicroseconds(100);
//         yawStepCount += (dir == HIGH) ? 1 : -1;
//       }
//       return;
//     }

//     default:
//       Serial.println("Invalid axis for moveToTarget");
//       return;
//   }

//   speed = constrain(speed, 0, maxFreq);

//   while (true) {
//     float deg = sensor->getRawAngle() * 0.087890625;
//     float error = targetDeg - deg;

//     if (fabs(error) < tolerance) {
//       analogWrite(stepPin, 0);
//       break;
//     }

//     int dir = (error >= 0) ? HIGH : LOW;
//     digitalWrite(dirPin, dir);
//     analogWriteFrequency(stepPin, speed);
//     analogWrite(stepPin, 127);
//     delay(10);
//   }
// }

// Parses incoming X-Plane packet and extracts motion data
void parsePacket(char* line) {
  char* token = strtok(line, ";");
  while (token != nullptr) {
    if (strncmp(token, "pitch=", 6) == 0) pitch = atof(token + 6);
    else if (strncmp(token, "pitch_rate=", 11) == 0) pitch_rate = atof(token + 11);
    else if (strncmp(token, "roll=", 5) == 0) roll = atof(token + 5);
    else if (strncmp(token, "roll_rate=", 10) == 0) roll_rate = atof(token + 10);
    else if (strncmp(token, "yaw=", 4) == 0) yaw = atof(token + 4);
    else if (strncmp(token, "yaw_rate=", 9) == 0) yaw_rate = atof(token + 9);
    else if (strncmp(token, "airspeed=", 9) == 0) airspeed = atof(token + 9);
    token = strtok(nullptr, ";");
  }
}

// Reads pitch and roll angles in degrees, offset-corrected
void readPitchRollDegrees(float& pitchDeg, float& rollDeg) {
  pitchDeg = pitchSensor.getRawAngle() * 0.087890625 - EncPitchZero;
  rollDeg  = rollSensor.getRawAngle() * 0.087890625 - EncRollZero;
}

// Drives all three axes using current rate values and PWM control
void driveMotors() {
  driveAxis(pitch_rate, RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_PITCH, DIR_PITCH);
  driveAxis(roll_rate,  RES_PITCH_ROLL, MAX_FREQ_PITCH_ROLL, STEP_ROLL,  DIR_ROLL);
  driveAxis(yaw_rate,   RES_YAW,        MAX_FREQ_YAW,        STEP_YAW,   DIR_YAW);
}

// Converts angular rate to PWM frequency and drives motor via step/direction pins
void driveAxis(float rate, float resolution, int maxFreq, int stepPin, int dirPin) {
  int dir = (rate >= 0) ? HIGH : LOW;
  digitalWrite(dirPin, dir);

  float absRate = fabs(rate);
  int freq = absRate / resolution;

  freq = constrain(freq, 0, maxFreq);
  analogWriteFrequency(stepPin, freq);
  analogWrite(stepPin, freq > 0 ? 127 : 0);  // 50% duty cycle if active
  if (freq > PrintMaxFreq){PrintMaxFreq = freq;}  // DEBUG
}

// Applies pitch and roll motion limits based on encoder readings
void applyLimits() {
  float pitchDeg, rollDeg;
  readPitchRollDegrees(pitchDeg, rollDeg);

  if (pitchDeg > LIMIT_PITCH && pitch_rate > 0) {
    pitch_rate = 0;
  }
  if (pitchDeg < -LIMIT_PITCH && pitch_rate < 0) {
    pitch_rate = 0;
  }

  if (rollDeg > LIMIT_ROLL && roll_rate > 0) {
    roll_rate = 0;
  }
  if (rollDeg < -LIMIT_ROLL && roll_rate < 0) {
    roll_rate = 0;
  }
}
