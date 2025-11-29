#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

Adafruit_MPU6050 mpu;
TinyGPSPlus gps;

// --- GPS pins (RX, TX)
const uint8_t GPS_RX_PIN = 3; // Arduino reads GPS TX
const uint8_t GPS_TX_PIN = 4; // Arduino TX -> GPS RX (optional)
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

const unsigned long INTERVAL_MS = 500; // slower output rate for readability
unsigned long lastPrint = 0;

float pitch_deg = 0.0;
const float alpha = 0.98;
unsigned long lastImuMicros = 0;

double lastSpeed_mps = 0.0;
unsigned long lastSpeedMillis = 0;
double accel_mps2 = 0.0;

const float STATIONARY_MAX_MPS = 0.14; // 0.5 km/h
const float WALKING_MAX_MPS = 1.94;    // 7 km/h
float BIKE_PITCH_DEG = 8.0;

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found. Check wiring!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  gpsSerial.begin(9600);
  lastImuMicros = micros();

  Serial.println("System ready. Waiting for GPS fix...");
}

void loop() {
  // --- Read IMU ---
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  float ax = accelEvent.acceleration.x;
  float ay = accelEvent.acceleration.y;
  float az = accelEvent.acceleration.z;

  float gy_dps = gyroEvent.gyro.y * 180.0 / PI;

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastImuMicros) / 1e6;
  if (dt <= 0 || dt > 0.1) dt = 0.01;
  lastImuMicros = nowMicros;

  float pitch_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float pitch_from_gyro = pitch_deg + gy_dps * dt;
  pitch_deg = alpha * pitch_from_gyro + (1.0f - alpha) * pitch_acc;

  // --- Read GPS stream ---
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  bool gpsValid = gps.location.isValid() && gps.altitude.isValid();

  double lat = gpsValid ? gps.location.lat() : 0.0;
  double lon = gpsValid ? gps.location.lng() : 0.0;
  double speed_mps = gps.speed.isValid() ? gps.speed.mps() : 0.0;
  bool speedValid = gps.speed.isValid();

  // --- Compute acceleration ---
  unsigned long nowMillis = millis();
  if (lastSpeedMillis != 0) {
    double dt_s = (nowMillis - lastSpeedMillis) / 1000.0;
    if (dt_s < 0.001) dt_s = 0.001;
    double raw_accel = (speed_mps - lastSpeed_mps) / dt_s;
    accel_mps2 = 0.6 * accel_mps2 + 0.4 * raw_accel; // smooth
  }
  lastSpeedMillis = nowMillis;
  lastSpeed_mps = speed_mps;

  // --- Motion classification ---
  String primary_label = "unknown";
  if (speed_mps <= STATIONARY_MAX_MPS) primary_label = "stationary";
  else if (speed_mps <= WALKING_MAX_MPS) primary_label = "walking";
  else primary_label = "riding";

  String riding_type = "null";
  if (primary_label == "riding") {
    riding_type = (fabs(pitch_deg) >= BIKE_PITCH_DEG) ? "bike" : "scooter";
  }

  // --- Output JSON ---
  if (nowMillis - lastPrint >= INTERVAL_MS) {
    lastPrint = nowMillis;

    Serial.print("{\"t_ms\":");
    Serial.print(nowMillis);
    Serial.print(",\"lat\":");
    Serial.print(gpsValid ? String(lat, 6) : "null");
    Serial.print(",\"lon\":");
    Serial.print(gpsValid ? String(lon, 6) : "null");
    Serial.print(",\"speed_mps\":");
    Serial.print(speed_mps, 3);
    Serial.print(",\"speed_kmph\":");
    Serial.print(speed_mps * 3.6, 2);
    Serial.print(",\"speed_valid\":");
    Serial.print(speedValid ? "true" : "false");
    Serial.print(",\"accel_mps2\":");
    Serial.print(accel_mps2, 3);
    Serial.print(",\"pitch_deg\":");
    Serial.print(pitch_deg, 2);
    Serial.print(",\"primary\":\"");
    Serial.print(primary_label);
    Serial.print("\",\"riding_type\":\"");
    Serial.print(riding_type);
    Serial.println("\"}");
  }
}
