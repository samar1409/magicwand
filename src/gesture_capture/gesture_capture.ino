
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// === CONFIGURATION ===
#define MPU_SDA 5
#define MPU_SCL 4

#define SWITCH_PIN 3  // SPDT switch (C = GND, NO = D3)
#define RED_PIN   6
#define GREEN_PIN 7
#define BLUE_PIN  8

const int SAMPLE_RATE_HZ = 100; // 100 samples/sec
const int DURATION_MS = 1000;   // Collect for 1 second

Adafruit_MPU6050 mpu;

// === HELPER: LED Control ===
void setColor(bool r, bool g, bool b) {
  digitalWrite(RED_PIN, r ? HIGH : LOW);
  digitalWrite(GREEN_PIN, g ? HIGH : LOW);
  digitalWrite(BLUE_PIN, b ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Setup LED
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  setColor(1, 1, 1); // White flash on boot
  delay(300);
  setColor(0, 0, 0);

  // Setup switch
  pinMode(SWITCH_PIN, INPUT_PULLUP);  // Trigger on LOW

  // Setup I2C
  Wire.begin(MPU_SDA, MPU_SCL);
  if (!mpu.begin()) {
    setColor(1, 0, 0); // Red = failure
    while (1) delay(10);
  }

  // Configure MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  setColor(0, 1, 0); // Green = ready
}

void loop() {
  if (digitalRead(SWITCH_PIN) == LOW) {
    setColor(0, 0, 1); // Blue = recording

    Serial.println("timestamp,x,y,z");
    unsigned long startTime = millis();

    while (millis() - startTime < DURATION_MS) {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      unsigned long ts = millis() - startTime;
      Serial.print(ts);
      Serial.print(",");
      Serial.print(a.acceleration.x);
      Serial.print(",");
      Serial.print(a.acceleration.y);
      Serial.print(",");
      Serial.println(a.acceleration.z);

      delay(1000 / SAMPLE_RATE_HZ);
    }

    setColor(0, 1, 0); // Back to green (ready)
    delay(1000); // debounce window
  }
}
