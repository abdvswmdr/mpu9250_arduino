# MPU9250 9-Axis Motion Sensor Arduino Library

A comprehensive Arduino library for interfacing with the InvenSense MPU9250 9-axis motion tracking device, featuring 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer (AK8963).

## Overview

The MPU9250 is a multi-chip module (MCM) that combines:
- **3-axis accelerometer** (measures linear acceleration)
- **3-axis gyroscope** (measures angular velocity) 
- **3-axis magnetometer** (AK8963 - measures magnetic field)
- **Temperature sensor**
- **Digital Motion Processor (DMP)**

## Hardware Specifications

### MPU9250 Sensor Specifications

| Parameter | Specification |
|-----------|---------------|
| **Accelerometer** |
| Full Scale Range | ±2g, ±4g, ±8g, ±16g |
| Sensitivity | 16,384 LSB/g (±2g mode) |
| Resolution | 16-bit |
| **Gyroscope** |
| Full Scale Range | ±250°/s, ±500°/s, ±1000°/s, ±2000°/s |
| Sensitivity | 131 LSB/(°/s) (±250°/s mode) |
| Resolution | 16-bit |
| **Magnetometer (AK8963)** |
| Full Scale Range | ±4800 µT |
| Resolution | 14-bit or 16-bit |
| **Temperature** |
| Range | -40°C to +85°C |
| Sensitivity | 333.87 LSB/°C |
| **Communication** |
| Interface | I²C (400kHz) / SPI (1MHz/20MHz) |
| I²C Address | 0x68 (MPU9250), 0x0C (AK8963) |

### Pin Configuration
MPU9250 Pinout:
┌─────────────────┐
│ 1 │ 2 │ 3 │ 4 │
│ NC │VDD│ GND│FSYNC│
├────┼───┼───┼────┤
│ 12 │11 │10 │ 9 │
│INT │AD0│SCL│SDA │
├────┼───┼───┼────┤
│ 13 │14 │15 │16 │
│ NC │NC │NC │ NC │
├────┼───┼───┼────┤
│ 20 │19 │18 │17 │
│CPOUT│NC │NC │ NC │
└─────────────────┘



**Essential Connections:**
- VDD: 3.3V power supply
- GND: Ground
- SDA: I²C data line
- SCL: I²C clock line
- AD0: Address select (LOW = 0x68, HIGH = 0x69)

## Software Architecture

### Data Flow Diagram

**Essential Connections:**
- VDD: 3.3V power supply
- GND: Ground
- SDA: I²C data line
- SCL: I²C clock line
- AD0: Address select (LOW = 0x68, HIGH = 0x69)

## Software Architecture

### Data Flow Diagram
┌─────────────┐ I²C ┌──────────────┐ ┌─────────────┐
│ Arduino │◄──────────►│ MPU9250 │ │ AK8963 │
│ │ │ (Accel+Gyro) │◄──►│ (Magnetom.) │
└─────────────┘ └──────────────┘ └─────────────┘
│ │ │
▼ ▼ ▼
┌─────────────┐ ┌──────────────┐ ┌─────────────┐
│ Normalized │ │ Raw Data │ │ Raw Data │
│ Data Output │ │ Registers │ │ Registers │
└─────────────┘ └──────────────┘ └─────────────┘




### Register Map (Key Registers)

#### MPU9250 Main Registers
| Address | Register Name | Description |
|---------|---------------|-------------|
| 0x1B | GYRO_CONFIG | Gyroscope configuration |
| 0x1C | ACCEL_CONFIG | Accelerometer configuration |
| 0x37 | INT_PIN_CFG | Interrupt/bypass configuration |
| 0x38 | INT_ENABLE | Interrupt enable |
| 0x3A | INT_STATUS | Interrupt status |
| 0x3B-0x48 | Sensor Data | Accel, Temp, Gyro data |
| 0x6B | PWR_MGMT_1 | Power management |

Where:
- `Full_Scale_Range` = 2, 4, 8, or 16 (in g)
- `g` = 9.80665 m/s² (standard gravity)
- `Raw_Value` = 16-bit signed integer (-32768 to +32767)

**Example (±2g range):**
```cpp
float accel_x = raw_accel_x * 2.0 * 9.80665 / 32768.0;
```

#### 2. Gyroscope Conversion
Angular_Velocity (°/s) = Raw_Value × Full_Scale_Range / 32768

Where:
- `Full_Scale_Range` = 250, 500, 1000, or 2000 (in °/s)

**Example (±1000°/s range):**
```cpp
float gyro_x = raw_gyro_x * 1000.0 / 32768.0;
```

#### 3. Temperature Conversion

Temperature (°C) = (Raw_Value - Room_Temp_Offset) / Sensitivity + 21.0

Where:
- `Room_Temp_Offset` = 21 (from datasheet)
- `Sensitivity` = 333.87 LSB/°C

**Implementation:**
```cpp
float temperature = (raw_temp - 21) / 333.87 + 21.0;
```

#### 4. Magnetometer Conversion
Magnetic_Field (µT) = Raw_Value × 4800 / 32768 × Sensitivity_Adjustment


**Sensitivity Adjustment Formula:**
ASA_adjusted = ((ASA - 128) × 0.5 / 128) + 1

Where `ASA` is the factory adjustment value (0x10-0x12 registers).

**Implementation:**
```cpp
float asax = (adjustment_x - 128) / 256.0 + 1.0;
float mag_x = raw_mag_x * 4800.0 / 32768.0 * asax;
```

## Code Structure

### Data Structures
```cpp
// Raw sensor data structures
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

// Normalized data structure
struct {
    struct {
        float x, y, z;
    } accelerometer, gyroscope, magnetometer;
    float temperature;
} normalized;
```

### Key Functions

#### Sensor Ready Check
```cpp
bool isImuReady() {
    uint8_t status;
    I2Cread(MPU9250_IMU_ADDRESS, 0x3A, 1, &status);
    return (status & 0x01);
}
```

#### Data Reading
```cpp
void readRawImu() {
    uint8_t data[14];
    I2Cread(MPU9250_IMU_ADDRESS, 0x3B, 14, data);
    
    // Parse accelerometer data (registers 0x3B-0x40)
    accelerometer.x = (data[0] << 8) | data[1];
    accelerometer.y = (data[2] << 8) | data[3];
    accelerometer.z = (data[4] << 8) | data[5];
    
    // Parse temperature data (registers 0x41-0x42)
    temperature.value = (data[6] << 8) | data[7];
    
    // Parse gyroscope data (registers 0x43-0x48)
    gyroscope.x = (data[8] << 8) | data[9];
    gyroscope.y = (data[10] << 8) | data[11];
    gyroscope.z = (data[12] << 8) | data[13];
}
```

## Setup and Configuration

### 1. Hardware Setup
```cpp
void setup() {
    Wire.begin();                    // Initialize I²C
    Serial.begin(115200);           // Initialize serial communication
    
    // Configure sensor ranges
    I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS);
    I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);
    
    // Enable magnetometer bypass mode
    I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02);
    I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x01);
    
    // Initialize magnetometer
    setMagnetometerAdjustmentValues();
    I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x12); // Continuous mode, 16-bit
}
```

### 2. Magnetometer Calibration
```cpp
void setMagnetometerAdjustmentValues() {
    // Enter Fuse ROM access mode
    I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x0F);
    delay(10);
    
    // Read factory adjustment values
    uint8_t data[3];
    I2Cread(MPU9250_MAG_ADDRESS, 0x10, 3, data);
    
    magnetometer.adjustment.x = data[0];
    magnetometer.adjustment.y = data[1];
    magnetometer.adjustment.z = data[2];
    
    // Return to power-down mode
    I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x00);
    delay(10);
}
```

## Usage Example

### Basic Reading Loop
```cpp
void loop() {
    // Check if new IMU data is available
    if (isImuReady()) {
        readRawImu();
        normalize(gyroscope);
        normalize(accelerometer);
        normalize(temperature);
    }
    
    // Check if new magnetometer data is available
    if (isMagnetometerReady()) {
        readRawMagnetometer();
        normalize(magnetometer);
    }
    
    // Print data every second
    if (millis() - lastPrintMillis > 1000) {
        printSensorData();
        lastPrintMillis = millis();
    }
}
```

### Sample Output
TEMP: 23.45°C
GYR (°/s): 0.123 -0.456 0.789
ACC (m/s^2): 0.234 0.567 9.801
MAG (µT): 12.345 -23.456 45.678