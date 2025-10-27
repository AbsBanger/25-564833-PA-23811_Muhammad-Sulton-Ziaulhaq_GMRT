// ESP32 + MPU6050 + 5 Servos + PIR
// Default pins (ubah bila perlu)
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;

// Pins (default)
const int PIN_SDA = 21;
const int PIN_SCL = 22;
const int PIN_PIR = 4;

const int PIN_SERVO1 = 13;
const int PIN_SERVO2 = 12;
const int PIN_SERVO3 = 14;
const int PIN_SERVO4 = 27;
const int PIN_SERVO5 = 26;

// Servo objects
Servo servo1, servo2, servo3, servo4, servo5;

const int CENTER_ANGLE = 90;    // posisi awal / tengah
const int PIR_MOVE_ANGLE = 45;  // saat PIR trigger -> semua bergerak ke 45°
const int PIR_MOVE_DELAY = 1000; // ms, tahan di posisi PIR sebelum kembali

// smoothing factor (0..1) untuk lerp; kecil = lebih halus
const float SMOOTH_FACTOR = 0.2f; 

// deadzone (derajat) sekitar 0 agar tidak jitter
const float DEADZONE = 5.0f;

// last smoothed angles
float lastA1 = CENTER_ANGLE, lastA2 = CENTER_ANGLE, lastA3 = CENTER_ANGLE, lastA4 = CENTER_ANGLE, lastA5 = CENTER_ANGLE;

void setup() {
  Serial.begin(115200);
  delay(100);

  // I2C init (SDA/SCL pins default ESP32)
  Wire.begin(PIN_SDA, PIN_SCL);

  // init MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip. Check wiring!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 found!");

  // configure MPU (default accelerometer/gyro ranges ok,
  // bisa diubah jika perlu: mpu.setAccelerometerRange(...), etc.)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // attach servos
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo4.attach(PIN_SERVO4);
  servo5.attach(PIN_SERVO5);

  // start all at center
  writeAllImmediate(CENTER_ANGLE);

  // PIR pin
  pinMode(PIN_PIR, INPUT);

  Serial.println("Setup done.");
}

void loop() {
  // Check PIR first (gives priority)
  if (digitalRead(PIN_PIR) == HIGH) {
    Serial.println("PIR detected! Moving all servos to PIR_MOVE_ANGLE.");
    moveAllToAngle(PIR_MOVE_ANGLE, true);
    delay(PIR_MOVE_DELAY);
    moveAllToAngle(CENTER_ANGLE, true);
    // reset last smoothed values to center to avoid jump
    lastA1 = lastA2 = lastA3 = lastA4 = lastA5 = CENTER_ANGLE;
    // small debounce
    delay(250);
    return;
  }

  // Read MPU event
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Compute roll (in degrees) from accelerometer
  // roll = atan2(accY, accZ) * RAD_TO_DEG
  float accX = a.acceleration.x;
  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;
  float roll = atan2(accY, accZ) * 180.0 / M_PI;

  // normalize (optional) - keep within -180..180
  if (roll > 180) roll -= 360;
  if (roll < -180) roll += 360;

  // debug
  // Serial.print("accY="); Serial.print(accY); Serial.print(" accZ="); Serial.print(accZ);
  Serial.print("Roll: ");
  Serial.println(roll);

  // Decide servo targets
  float target1 = CENTER_ANGLE, target2 = CENTER_ANGLE, target3 = CENTER_ANGLE, target4 = CENTER_ANGLE, target5 = CENTER_ANGLE;

  if (roll < -DEADZONE) {
    // Negative roll -> left side active: servo1 & servo2 follow negative roll
    // Map roll range -90..0 -> 0..CENTER_ANGLE (so -90 is extreme left (0 deg), 0 is center)
    float mapped = mapFloat(roll, -90.0f, 0.0f, 0.0f, (float)CENTER_ANGLE);
    mapped = constrain(mapped, 0.0f, 180.0f);
    target1 = mapped;
    target2 = mapped;
    // others stay center
  } else if (roll > DEADZONE) {
    // Positive roll -> right side active: servo3 & servo4 follow positive roll
    // Map roll range 0..90 -> CENTER..180 (so 90 is extreme right 180deg)
    float mapped = mapFloat(roll, 0.0f, 90.0f, (float)CENTER_ANGLE, 180.0f);
    mapped = constrain(mapped, 0.0f, 180.0f);
    target3 = mapped;
    target4 = mapped;
    // servo5 stays center (per asumsi)
  } else {
    // within deadzone -> center all
    // target already set to center
  }

  // Smooth transitions with linear interpolation
  lastA1 = lerp(lastA1, target1, SMOOTH_FACTOR);
  lastA2 = lerp(lastA2, target2, SMOOTH_FACTOR);
  lastA3 = lerp(lastA3, target3, SMOOTH_FACTOR);
  lastA4 = lerp(lastA4, target4, SMOOTH_FACTOR);
  lastA5 = lerp(lastA5, target5, SMOOTH_FACTOR);

  // write to servos
  servo1.write((int)round(lastA1));
  servo2.write((int)round(lastA2));
  servo3.write((int)round(lastA3));
  servo4.write((int)round(lastA4));
  servo5.write((int)round(lastA5));

  delay(50); // loop rate ~20 Hz
}

// Utility: linear interpolation
float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}

// Utility: map float (like map() but for float)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Move all servos to an angle (option immediate: bypass smoothing)
void moveAllToAngle(int angle, bool immediate=false) {
  if (immediate) {
    servo1.write(angle);
    servo2.write(angle);
    servo3.write(angle);
    servo4.write(angle);
    servo5.write(angle);
    lastA1 = lastA2 = lastA3 = lastA4 = lastA5 = angle;
  } else {
    // smooth transition (simple)
    for (int step = 0; step <= 10; step++) {
      float t = step / 10.0f;
      servo1.write((int)round(lerp(lastA1, angle, t)));
      servo2.write((int)round(lerp(lastA2, angle, t)));
      servo3.write((int)round(lerp(lastA3, angle, t)));
      servo4.write((int)round(lerp(lastA4, angle, t)));
      servo5.write((int)round(lerp(lastA5, angle, t)));
      delay(25);
    }
    lastA1 = lastA2 = lastA3 = lastA4 = lastA5 = angle;
  }
}

// immediate write all (no smoothing) used in setup
void writeAllImmediate(int angle) {
  servo1.write(angle);
  servo2.write(angle);
  servo3.write(angle);
  servo4.write(angle);
  servo5.write(angle);
}
