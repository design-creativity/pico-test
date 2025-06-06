# ICM20948 IMU Sensor Real-Time Data Visualization

A comprehensive system for collecting, processing, and visualizing real-time motion data from the ICM20948 9-axis IMU sensor using Raspberry Pi Pico and Windows PC.

## What You'll Get
- **5-panel real-time plots** with live sensor data
- **Terminal-style data monitor** (same as Thonny output)
- **Movement detection** with color coding
- **Finger gesture recognition** (ROTATION, BEND, SWIPE, TAP)
- **Professional data visualization** for presentations/demos
- **Automatic COM port detection** for easy setup

## Understanding the Sensor Data

### Sample Data Output
```
Reading 323 - 644453ms:
   Accel (raw): X=-14760, Y= -4120, Z=  6468
   Gyro  (raw): X=   -26, Y=   -23, Z=    26
   Accel (g):   X= -0.90, Y= -0.25, Z=  0.39
   Gyro (°/s):  X=  -0.2, Y=  -0.2, Z=   0.2
   FINGER_BEND DETECTED
```

### Data Explanation

#### **Accelerometer Data**
- **Raw Values**: Direct digital readings from the sensor (-32768 to +32767 range)
- **G-Force Values**: Acceleration relative to Earth's gravity (1g = 9.8 m/s²)
  - X-axis: Forward/backward movement
  - Y-axis: Left/right movement  
  - Z-axis: Up/down movement

#### **Gyroscope Data**
- **Raw Values**: Direct digital readings for rotational movement
- **Angular Velocity (°/s)**: Rotation speed in degrees per second
  - X-axis: Roll rotation
  - Y-axis: Pitch rotation
  - Z-axis: Yaw rotation

#### **Additional Information**
- **Reading Number**: Sequential data sample count
- **Timestamp**: Time in milliseconds since system start
- **Gesture Detection**: Real-time recognition of specific movements

## Hardware Requirements

### Required Components:
- **Raspberry Pi Pico** (or Pico W)
- **ICM20948 9-DOF IMU breakout board**
- **4 jumper wires** (male-to-female or male-to-male)
- **USB cable** (micro-USB or USB-C for Pico)
- **Windows computer** (Windows 10/11)

### Hardware Connections:
```
Pico Pin → IMU Pin
--------------------------------
Pin 4 (GP2)  → SDA (Data)
Pin 5 (GP3)  → SCL (Clock)  
Pin 36 (3V3) → VCC (Power)
Pin 38 (GND) → GND (Ground)
```

## Software Requirements

### Raspberry Pi Pico
- **MicroPython** firmware
- **ICM20948 library** for MicroPython
- **Serial communication** setup

### Windows PC
- **Python 3.8+**
- Required libraries:
  ```bash
  pip install pyserial matplotlib numpy pandas
  ```

## Applications

### Gesture Recognition
- **Finger bend detection**
- **Hand movement tracking**
- **Sign language recognition**
- **Gaming controllers**

### Health & Fitness
- **Rehabilitation monitoring**
- **Exercise form analysis**
- **Activity tracking**
- **Physical therapy assistance**

### IoT & Smart Devices
- **Wearable technology**
- **Smart home controls**
- **Motion-activated systems**
- **Security applications**

## Configuration Options

### Sensor Settings
- **Sample Rate**: Adjustable from 1Hz to 1000Hz
- **Sensitivity**: Configurable accelerometer and gyroscope ranges
- **Filtering**: Built-in digital filters for noise reduction

### Data Processing
- **Calibration**: Automatic sensor calibration routines
- **Threshold Setting**: Customizable gesture detection sensitivity
- **Data Logging**: CSV export for further analysis

## Data Format Specifications

### Serial Output Format
```
Reading [COUNT] - [TIMESTAMP]ms:
   Accel (raw): X=[RAW_X], Y=[RAW_Y], Z=[RAW_Z]
   Gyro  (raw): X=[RAW_X], Y=[RAW_Y], Z=[RAW_Z]
   Accel (g):   X=[G_X], Y=[G_Y], Z=[G_Z]
   Gyro (°/s):  X=[DEG_X], Y=[DEG_Y], Z=[DEG_Z]
   [GESTURE_STATUS]
```

### Value Ranges
- **Accelerometer Raw**: -32768 to +32767
- **Gyroscope Raw**: -32768 to +32767
- **Accelerometer (g)**: -16g to +16g (configurable)
- **Gyroscope (°/s)**: -2000°/s to +2000°/s (configurable)

## Calibration Process

### Accelerometer Calibration
1. Place sensor on flat, level surface
2. Record baseline readings for all axes
3. Apply offset corrections to raw data
4. Verify gravity reads approximately 1g on Z-axis

### Gyroscope Calibration
1. Keep sensor completely still
2. Record drift values for all axes
3. Apply bias correction to eliminate drift
4. Verify zero readings when stationary

## Troubleshooting

### Common Issues
- **No data received**: Check USB connection and COM port
- **Erratic readings**: Verify wiring and power supply stability
- **Gesture not detected**: Adjust detection thresholds
- **High noise levels**: Enable digital filtering options

### Performance Optimization
- **Reduce sample rate** for battery-powered applications
- **Enable sleep modes** between readings
- **Use interrupt-driven** data collection for efficiency

## Technical Specifications

### ICM20948 Sensor Features
- **9-axis motion tracking**: 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer
- **16-bit ADC resolution**
- **I2C and SPI interfaces**
- **Programmable interrupts**
- **Built-in temperature sensor**
- **Low power consumption**

### Communication Protocol
- **Interface**: I2C (400kHz max)
- **Address**: 0x68 (default) or 0x69
- **Data Rate**: Up to 1000Hz
- **Latency**: <1ms typical

## Further Reading

### Documentation Links
- [ICM20948](https://www.kiwi-electronics.com/en/icm20948-9dof-motion-sensor-breakout-10893)
- [Raspberry Pi Pico Documentation](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#pico2w-technical-specification)
- [MicroPython Guide](https://micropython.org/)

### Example Projects
- Smart glove for sign language translation
- VR/AR motion controllers
- Fitness tracking wearables
- Drone stabilization systems

---

*For technical support or questions, please refer to the documentation or create an issue in the project repository.*
