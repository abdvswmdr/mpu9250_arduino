#include "Wire.h"
#include "I2C.h"

#define MPU9250_IMU_ADDRESS 0x68
#define MPU9250_MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2G  0x00
#define ACC_FULL_SCALE_4G  0x08
#define ACC_FULL_SCALE_8G  0x10
#define ACC_FULL_SCALE_16G 0x18

#define TEMPERATURE_OFFSET 21 // As defined in documentation

#define INTERVAL_MS_PRINT 1000

#define G 9.80665

struct gyroscope_raw {
  int16_t x, y, z;
} gyroscope;

struct accelerometer_raw {
  int16_t x, y, z;
} accelerometer;

struct magnetometer_raw {
  int16_t x, y, z;

  struct {
    int8_t x, y, z;
  } adjustment;
} magnetometer;

struct temperature_raw {
  int16_t value;
} temperature;

struct {
  struct {
    float x, y, z;
  } accelerometer, gyroscope, magnetometer;

  float temperature;
} normalized;

unsigned long lastPrintMillis = 0;

// Function to check if IMU data is ready
bool isImuReady() {
  uint8_t status;
  I2Cread(MPU9250_IMU_ADDRESS, 0x3A, 1, &status); // Read INT_STATUS register
  return (status & 0x01); // Check if data ready bit is set
}

// Function to check if magnetometer data is ready
bool isMagnetometerReady() {
  uint8_t status;
  I2Cread(MPU9250_MAG_ADDRESS, 0x02, 1, &status); // Read ST1 register
  return (status & 0x01); // Check if data ready bit is set
}

// Function to read raw IMU data (accelerometer, gyroscope, temperature)
void readRawImu() {
  uint8_t data[14];
  I2Cread(MPU9250_IMU_ADDRESS, 0x3B, 14, data); // Read from ACCEL_XOUT_H to GYRO_ZOUT_L
  
  // Accelerometer data
  accelerometer.x = (data[0] << 8) | data[1];
  accelerometer.y = (data[2] << 8) | data[3];
  accelerometer.z = (data[4] << 8) | data[5];
  
  // Temperature data
  temperature.value = (data[6] << 8) | data[7];
  
  // Gyroscope data
  gyroscope.x = (data[8] << 8) | data[9];
  gyroscope.y = (data[10] << 8) | data[11];
  gyroscope.z = (data[12] << 8) | data[13];
}

// Function to read raw magnetometer data
void readRawMagnetometer() {
  uint8_t data[7];
  I2Cread(MPU9250_MAG_ADDRESS, 0x03, 7, data); // Read from HXL to ST2
  
  // Check if data is valid (ST2 bit 3 should be 0)
  if (!(data[6] & 0x08)) {
    magnetometer.x = (data[1] << 8) | data[0]; // Note: LSB first for magnetometer
    magnetometer.y = (data[3] << 8) | data[2];
    magnetometer.z = (data[5] << 8) | data[4];
  }
}

// Function to set magnetometer adjustment values
void setMagnetometerAdjustmentValues() {
  // Set magnetometer to fuse ROM access mode
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x0F);
  delay(10);
  
  // Read adjustment values
  uint8_t data[3];
  I2Cread(MPU9250_MAG_ADDRESS, 0x10, 3, data); // Read ASAX, ASAY, ASAZ
  
  magnetometer.adjustment.x = data[0];
  magnetometer.adjustment.y = data[1];
  magnetometer.adjustment.z = data[2];
  
  // Set magnetometer back to power down mode
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x00);
  delay(10);
}

// Normalize gyroscope data
void normalize(gyroscope_raw gyro) {
  // Convert to degrees per second (1000 DPS range, 16-bit)
  normalized.gyroscope.x = gyro.x * 1000.0 / 32768.0;
  normalized.gyroscope.y = gyro.y * 1000.0 / 32768.0;
  normalized.gyroscope.z = gyro.z * 1000.0 / 32768.0;
}

// Normalize accelerometer data
void normalize(accelerometer_raw accel) {
  // Convert to m/s² (2G range, 16-bit)
  normalized.accelerometer.x = accel.x * 2.0 * G / 32768.0;
  normalized.accelerometer.y = accel.y * 2.0 * G / 32768.0;
  normalized.accelerometer.z = accel.z * 2.0 * G / 32768.0;
}

// Normalize temperature data
void normalize(temperature_raw temp) {
  // Convert to Celsius
  normalized.temperature = (temp.value - TEMPERATURE_OFFSET) / 333.87 + 21.0;
}

// Normalize magnetometer data
void normalize(magnetometer_raw mag) {
  // Apply sensitivity adjustment and convert to microTesla
  float asax = (mag.adjustment.x - 128) / 256.0 + 1.0;
  float asay = (mag.adjustment.y - 128) / 256.0 + 1.0;
  float asaz = (mag.adjustment.z - 128) / 256.0 + 1.0;
  
  // Convert to microTesla (16-bit, ±4800 µT range)
  normalized.magnetometer.x = mag.x * 4800.0 / 32768.0 * asax;
  normalized.magnetometer.y = mag.y * 4800.0 / 32768.0 * asay;
  normalized.magnetometer.z = mag.z * 4800.0 / 32768.0 * asaz;
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);

  I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS); // Configure gyroscope range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);        // Configure accelerometer range

  I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02); // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x01); // Enable interrupt pin for raw data

  setMagnetometerAdjustmentValues();

  //Start magnetometer
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x12); // Request continuous magnetometer measurements in 16 bits (mode 1)
}

void loop()
{
  unsigned long currentMillis = millis();

  if (isImuReady()) {
    readRawImu();

    normalize(gyroscope);
    normalize(accelerometer);
    normalize(temperature);
  }

  if (isMagnetometerReady()) {
    readRawMagnetometer();

    normalize(magnetometer);
  }

  if (currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) {
    Serial.print("TEMP:\t");
    Serial.print(normalized.temperature, 2);
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.print("C");
    Serial.println();

    Serial.print("GYR (");
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.print("/s):\t");
    Serial.print(normalized.gyroscope.x, 3);
    Serial.print("\t\t");
    Serial.print(normalized.gyroscope.y, 3);
    Serial.print("\t\t");
    Serial.print(normalized.gyroscope.z, 3);
    Serial.println();

    Serial.print("ACC (m/s^2):\t");
    Serial.print(normalized.accelerometer.x, 3);
    Serial.print("\t\t");
    Serial.print(normalized.accelerometer.y, 3);
    Serial.print("\t\t");
    Serial.print(normalized.accelerometer.z, 3);
    Serial.println();

    Serial.print("MAG (");
    Serial.print("\xce\xbc"); //Print micro symbol
    Serial.print("T):\t");
    Serial.print(normalized.magnetometer.x, 3);
    Serial.print("\t\t");
    Serial.print(normalized.magnetometer.y, 3);
    Serial.print("\t\t");
    Serial.print(normalized.magnetometer.z, 3);
    Serial.println();

    Serial.println();

    lastPrintMillis = currentMillis;
  }
}
