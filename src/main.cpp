#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>
//Pin Settings on Robot
// unint8_t = unassigned bit integer from 0 to 255 stored in one byte
const uint8_t PIN_MOTOR_PWMA = 5;  // Right motor speed
const uint8_t PIN_MOTOR_AIN1 = 7;  // Right motor direction
const uint8_t PIN_MOTOR_PWMB = 6;  // Left motor speed
const uint8_t PIN_MOTOR_BIN1 = 8;  // Left motor direction
const uint8_t PIN_MOTOR_STBY = 3;  // Standby pin
const uint8_t PIN_TRIG  = 13;
const uint8_t PIN_ECHO  = 12;
const uint8_t PIN_SERVO = 10;
const uint8_t MPU_ADDR  = 0x68;
// Distance Settings
const float STOP_DISTANCE_CM = 30.0f;
const float CLEAR_MARGIN_CM  = 20.0f;
const float DEAD_END_CM      = STOP_DISTANCE_CM + 10.0f; 
// speed settings
const uint8_t SPEED_DRIVE    = 100;  // speed
const uint8_t SPEED_TURN     = 100;  // speed of turn
const uint8_t SPEED_REVERSE  = 100;  // speed of reverse
const int RAMP_STEP = 8;        // PWM change per 10 ms
int curLeft = 0, curRight = 0;  // speeds actually being applied
// turn parameters settings
const float TURN_ANGLE_NORMAL   = 75.0f;
const float TURN_ANGLE_UTURN    = 170.0f;
const unsigned long TURN_TIMEOUT_NORMAL_MS = 2000;
const unsigned long TURN_TIMEOUT_UTURN_MS  = 4000;
const uint8_t OBSTACLE_HITS_REQUIRED = 2;  // debounce count
const float MIN_EXIT_ANGLE = 30.0f;        // sets it so there is no early exit before turning this far
const float GYRO_SCALE = 65.5f;            // LSB per deg/s at +/-500 deg/s range

Servo panServo;
// Gyro calibration offsets
float gz_offset = 0.0f;

// Mahony Filter
class MahonyFilter {
private:
    float twoKp = 1.5f; // 2 times proportional gain (kp = 0.75)
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// g = gyroscope: rotation rate around an axis, 
// a = means accelerometer, measures acceleration along axis.
// so gx ,mean gyro along x axis, az mean accelerometer along the z axis etc
public:
    void update(float gx, float gy, float gz, float ax, float ay, float az, float dt) { // dt means delta time: secounds since the last filter update
        float recipNorm; // 1 divided by the length of the vector
        float halfvx, halfvy, halfvz; //half of the vector of estimated gravity direction of x, y, z
        float halfex, halfey, halfez; // half of the error between measured and estimated gravity of x, y, z
        float qa, qb, qc; // temporary copies of qauternion values

        if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
            recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            halfvx = q1 * q3 - q0 * q2;
            halfvy = q0 * q1 + q2 * q3;
            halfvz = q0 * q0 - 0.5f + q3 * q3;

            halfex = (ay * halfvz - az * halfvy);
            halfey = (az * halfvx - ax * halfvz);
            halfez = (ax * halfvy - ay * halfvx);

            gx += twoKp * halfex;
            gy += twoKp * halfey;
            gz += twoKp * halfez;
        }

        gx *= (0.5f * dt);
        gy *= (0.5f * dt);
        gz *= (0.5f * dt);
        qa = q0; qb = q1; qc = q2;
        q0 += (-qb * gx - qc * gy - q3 * gz);
        q1 += (qa * gx + qc * gz - q3 * gy);
        q2 += (qa * gy - qb * gz + q3 * gx);
        q3 += (qa * gz + qb * gy - qc * gx);

        recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }

    float getYaw() { // robot heading: how it has rotated around the vertical axis
        return atan2(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3) * 57.29578f;
    }
};
MahonyFilter ahrs; // attitude and heading reference system (ahrs)

// wrapped angle difference (fixes the +/-180 wraparound bug)
float angleDiff(float a, float b) {
    float d = a - b;
    while (d > 180.0f)  d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}
// explicit-order 16-bit read (avoids unspecified evaluation order)
int16_t read16() {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
}
// Low Level sensor stuff
void initMPU() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);   // Wake up
    Wire.write(0);
    Wire.endTransmission(true);
    delay(100); // let the MPU settle after wake-up

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);   // DLPF config (~44 Hz) to reduce motor vibration noise
    Wire.write(3);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);   // gyro range, 0x08 = +/-500 deg/s (default was +/-250 and clipped)
    Wire.write(0x08);
    Wire.endTransmission(true);
}

// returns bool so a failed read never reaches the filter
bool readMPU(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint8_t)true);

    if (Wire.available() < 14) return false;

    int16_t rawAx = read16();
    int16_t rawAy = read16();
    int16_t rawAz = read16();
    read16();  // Skip temperature
    int16_t rawGx = read16();
    int16_t rawGy = read16();
    int16_t rawGz = read16();

    ax = (float)rawAx / 16384.0f;
    ay = (float)rawAy / 16384.0f;
    az = (float)rawAz / 16384.0f;
    gx = ((float)rawGx / GYRO_SCALE) * 0.0174533f;
    gy = ((float)rawGy / GYRO_SCALE) * 0.0174533f;
    gz = (((float)rawGz / GYRO_SCALE) * 0.0174533f) - gz_offset;
    return true;
}

// Calibrate stationary resting offset for Z-axis gyro
void calibrateGyro() {
    float sumGz = 0.0f;
    int good = 0;
    const int SAMPLES = 200;
    for (int i = 0; i < SAMPLES; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x47); // GYRO_ZOUT_H
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, (uint8_t)2, (uint8_t)true);
        if (Wire.available() >= 2) {
            int16_t rawGz = read16();
            sumGz += ((float)rawGz / GYRO_SCALE) * 0.0174533f;
            good++;
        }
        delay(5);
    }
    if (good > 0) gz_offset = sumGz / (float)good;  // average only successful reads
}

float readDistanceCM() {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    long duration = pulseIn(PIN_ECHO, HIGH, 25000);
    if (duration == 0) return 400.0f;
    return (duration * 0.0343f) / 2.0f;
}
void setMotors(int leftSpeed, int rightSpeed) {
    static unsigned long lastRamp = 0;
    if (millis() - lastRamp >= 10) {
        lastRamp = millis();
        curLeft  += constrain(leftSpeed  - curLeft,  -RAMP_STEP, RAMP_STEP);
        curRight += constrain(rightSpeed - curRight, -RAMP_STEP, RAMP_STEP);
    }

    digitalWrite(PIN_MOTOR_STBY, HIGH);

    digitalWrite(PIN_MOTOR_BIN1, curLeft >= 0 ? HIGH : LOW);
    analogWrite(PIN_MOTOR_PWMB, abs(curLeft));

    digitalWrite(PIN_MOTOR_AIN1, curRight >= 0 ? HIGH : LOW);
    analogWrite(PIN_MOTOR_PWMA, abs(curRight));
}

void stopMotors() {
    analogWrite(PIN_MOTOR_PWMA, 0);
    analogWrite(PIN_MOTOR_PWMB, 0);
    digitalWrite(PIN_MOTOR_AIN1, LOW);  // tidy direction pins
    digitalWrite(PIN_MOTOR_BIN1, LOW);
    digitalWrite(PIN_MOTOR_STBY, LOW);
    curLeft = 0; curRight = 0;
}

// Scanning Decisions
// it Returns: 1 for Right, -1 for Left, 0 for Dead End (Reverse needed)
int evaluateScanPaths() {
    stopMotors();
    panServo.write(45); // Check Right
    delay(280);
    float rightDist = readDistanceCM();
    panServo.write(135); // Check Left
    delay(280);
    float leftDist = readDistanceCM();
    panServo.write(90); // Reset Center
    delay(150);
    // dead end uses DEAD_END_CM (bigger than STOP_DISTANCE_CM)
    if (rightDist < DEAD_END_CM && leftDist < DEAD_END_CM) {
        return 0;
    }
    return (rightDist >= leftDist) ? 1 : -1;
}
// Navigation
enum RobotState {
    STATE_FORWARD,
    STATE_DECIDE_TURN,
    STATE_TURNING,
    STATE_REVERSE_BACKOUT
};
RobotState currentState = STATE_FORWARD;

unsigned long lastFilterTime = 0;
unsigned long lastSensorTime = 0;
unsigned long reverseStartTime = 0;
unsigned long turnStartTime = 0;  
unsigned long turnTimeoutMs = TURN_TIMEOUT_NORMAL_MS;
float currentDistance = 100.0f;
float startYaw = 0.0f;
float targetAngle = TURN_ANGLE_NORMAL; 
bool allowEarlyExit = true; // false during the U-turn
uint8_t obstacleHits = 0; 
uint8_t noEchoHits = 0; // debounce counter
int turnDirection = 1;

void beginTurn(int dir, float angle, unsigned long timeoutMs, bool earlyExit) {
    turnDirection = dir;
    targetAngle = angle;
    turnTimeoutMs = timeoutMs;
    allowEarlyExit = earlyExit;
    startYaw = ahrs.getYaw();
    turnStartTime = millis();
    currentState = STATE_TURNING;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    initMPU();
    pinMode(PIN_MOTOR_PWMA, OUTPUT);
    pinMode(PIN_MOTOR_PWMB, OUTPUT);
    pinMode(PIN_MOTOR_AIN1, OUTPUT);
    pinMode(PIN_MOTOR_BIN1, OUTPUT);
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    panServo.attach(PIN_SERVO);
    panServo.write(90);
    stopMotors();
    Serial.println("Calibrating IMU - keep robot stationary...");
    calibrateGyro();
    Serial.print("Gyro Z Bias: "); Serial.println(gz_offset, 5);
    lastFilterTime = micros();
    Serial.println("Navigation Online.");
    Serial.println(F("BOOT"));
}
void loop() {
    unsigned long nowMicros = micros();
    unsigned long nowMillis = millis();

    // Fast Sensor Fusion Loop
    float dt = (nowMicros - lastFilterTime) / 1000000.0f;
    lastFilterTime = nowMicros;

    float ax, ay, az, gx, gy, gz;
    // only updates the filter when the read is good
    if (readMPU(ax, ay, az, gx, gy, gz)) {
        ahrs.update(gx, gy, gz, ax, ay, az, dt);
    }

    // Range Polling Loop 20 Hz
    if (nowMillis - lastSensorTime >= 50) {
        lastSensorTime = nowMillis;
        currentDistance = readDistanceCM();
        if (currentDistance >= 400.0f) { if (noEchoHits < 255) noEchoHits++; }
        else noEchoHits = 0;
        if (currentDistance < STOP_DISTANCE_CM) { // a debounce, does consective readings
            if (obstacleHits < 255) obstacleHits++;
        } else {
            obstacleHits = 0;
        }
    }
    switch (currentState) {  // finite state machine
        case STATE_FORWARD:
            if (obstacleHits >= OBSTACLE_HITS_REQUIRED || noEchoHits >= 3) {  
                stopMotors();
                obstacleHits = 0;
                noEchoHits = 0;
                currentState = STATE_DECIDE_TURN;
            } else {
                setMotors(SPEED_DRIVE, SPEED_DRIVE);
            }
            break;

        case STATE_DECIDE_TURN:
            turnDirection = evaluateScanPaths();
            lastFilterTime = micros();  // might seem unnessary but scan blocked a 700 ms, so I didnt get feed a huge dt
            if (turnDirection == 0) {
                reverseStartTime = millis();
                currentState = STATE_REVERSE_BACKOUT;
            } else {
                beginTurn(turnDirection, TURN_ANGLE_NORMAL, TURN_TIMEOUT_NORMAL_MS, true);
            }
            break;
        case STATE_REVERSE_BACKOUT: // Reverse straight back for 600 ms to clear obstacle
            setMotors(-SPEED_REVERSE, -SPEED_REVERSE);
            if (millis() - reverseStartTime >= 600) {
                stopMotors();
                beginTurn(1, TURN_ANGLE_UTURN, TURN_TIMEOUT_UTURN_MS, false); // 180 degree turn, no early exit
            }
            break;
        case STATE_TURNING: {  // braces are local variables so they are legal
            if (turnDirection == 1) {
                setMotors(SPEED_TURN, -SPEED_TURN);
            } else {
                setMotors(-SPEED_TURN, SPEED_TURN);
            }
            // wrapped difference
            float angleTurned = fabs(angleDiff(ahrs.getYaw(), startYaw));
            // uses currentDistance instead of a blocking read every pass
            //  early exit only after a minimum turn angle
            bool pathClear = allowEarlyExit &&
                             angleTurned >= MIN_EXIT_ANGLE &&
                             currentDistance > (STOP_DISTANCE_CM + CLEAR_MARGIN_CM);
            bool timedOut = (millis() - turnStartTime) >= turnTimeoutMs;

            if (angleTurned >= targetAngle || pathClear || timedOut) {
                stopMotors();
                obstacleHits = 0;
                noEchoHits = 0;
                currentState = STATE_FORWARD;
            }
            break;
        }
    }
    static unsigned long lastDbg = 0;
    if (millis() - lastDbg >= 200) {
        lastDbg = millis();
        Serial.print(F("state=")); Serial.print(currentState);
        Serial.print(F(" dist="));  Serial.print(currentDistance);
        Serial.print(F(" yaw="));   Serial.println(ahrs.getYaw());
    }
}
