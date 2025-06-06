# Complete Linux Guide: ICM20948 Real-time Plotting System

## Overview
This guide will help you set up a complete real-time data visualization system for the ICM20948 IMU sensor using Raspberry Pi Pico and a Linux computer.

## What You'll Get
- **5-panel real-time plots** with live sensor data
- **Terminal-style data monitor** (same as Thonny output)
- **Movement detection** with color coding
- **Finger gesture recognition** (ROTATION, BEND, SWIPE, TAP)
- **Professional data visualization** for presentations/demos

---

## Hardware Requirements

### Required Components:
- **Raspberry Pi Pico** (or Pico W)
- **ICM20948 9-DOF IMU breakout board**
- **4 jumper wires** (male-to-female or male-to-male)
- **USB cable** (micro-USB or USB-C for Pico)
- **Linux computer** (Ubuntu, Debian, Fedora, etc.)

### Hardware Connections:
```
Pico Pin → IMU Pin
--------------------------------
Pin 4 (GP2)  → SDA (Data)
Pin 5 (GP3)  → SCL (Clock)  
Pin 36 (3V3) → VCC (Power)
Pin 38 (GND) → GND (Ground)
```

---

## Software Installation

### Step 1: Update System
```bash
# Update package lists
sudo apt update && sudo apt upgrade -y
```

### Step 2: Install Python Packages
```bash
# Method 1: Using pip (recommended)
pip3 install matplotlib pyserial numpy

# Method 2: Using apt (alternative)
sudo apt install python3-matplotlib python3-serial python3-numpy python3-tk

# Method 3: All-in-one
pip3 install matplotlib pyserial numpy && sudo apt install python3-tk
```

### Step 3: Setup Serial Permissions
```bash
# Add user to dialout group (permanent fix)
sudo usermod -a -G dialout $USER

# Temporary fix for current session
sudo chmod 666 /dev/ttyACM0

# Log out and back in for permanent fix to take effect
```

### Step 4: Verify Installation
```bash
# Test if packages work
python3 -c "
import matplotlib.pyplot as plt
import serial
import numpy as np
print('All packages installed successfully!')
print('matplotlib version:', plt.matplotlib.__version__)
print('pyserial version:', serial.__version__)
print('numpy version:', np.__version__)
"
```

---

## Pico Setup (main.py)

### Step 1: Install Thonny
```bash
# Install Thonny IDE
sudo apt install thonny

# Or download from: https://thonny.org/
```

### Step 2: Create main.py on Pico

**Copy this code and save as `main.py` on your Pico:**

```python
# Final main.py for Raspberry Pi Pico - ICM20948 IMU Sensor
# This version runs standalone and outputs data for computer plotting

from machine import I2C, Pin
import time
import struct

print("=== ICM20948 IMU - Final Standalone Version ===")
print("Real-time Accelerometer & Gyroscope Data")
print("=" * 50)

# Initialize I2C1 with working configuration
# GP2 (Pin 4) = SDA, GP3 (Pin 5) = SCL, 10kHz for stability
i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=10000)
ICM20948_ADDR = 0x68

def initialize_icm20948():
    """Initialize ICM20948 with proper wake-up sequence"""
    print("Initializing ICM20948...")
    
    try:
        # Verify device
        who_am_i = i2c.readfrom_mem(ICM20948_ADDR, 0x00, 1)[0]
        if who_am_i == 0xEA:
            print(f"ICM20948 detected (WHO_AM_I: 0x{who_am_i:02X})")
        else:
            print(f"Unexpected device ID: 0x{who_am_i:02X} (expected 0xEA)")
        
        # Reset device
        i2c.writeto_mem(ICM20948_ADDR, 0x06, bytes([0x80]))  # PWR_MGMT_1 = reset
        time.sleep(0.1)
        
        # Wake up with auto clock
        i2c.writeto_mem(ICM20948_ADDR, 0x06, bytes([0x01]))  # PWR_MGMT_1 = auto clock
        time.sleep(0.05)
        
        # Enable all sensors
        i2c.writeto_mem(ICM20948_ADDR, 0x07, bytes([0x00]))  # PWR_MGMT_2 = enable all
        time.sleep(0.05)
        
        print("ICM20948 initialization complete!")
        return True
        
    except Exception as e:
        print(f"IMU initialization error: {e}")
        print("Continuing with basic operation...")
        return False

def read_imu_data():
    """Read accelerometer and gyroscope data"""
    try:
        # Read accelerometer (6 bytes: X, Y, Z)
        accel_data = i2c.readfrom_mem(ICM20948_ADDR, 0x2D, 6)
        ax = struct.unpack('>h', accel_data[0:2])[0]
        ay = struct.unpack('>h', accel_data[2:4])[0]
        az = struct.unpack('>h', accel_data[4:6])[0]
        
        # Read gyroscope (6 bytes: X, Y, Z)
        gyro_data = i2c.readfrom_mem(ICM20948_ADDR, 0x33, 6)
        gx = struct.unpack('>h', gyro_data[0:2])[0]
        gy = struct.unpack('>h', gyro_data[2:4])[0]
        gz = struct.unpack('>h', gyro_data[4:6])[0]
        
        # Convert to physical units
        # Accelerometer: ±2g range, 16384 LSB/g
        ax_g = ax / 16384.0
        ay_g = ay / 16384.0
        az_g = az / 16384.0
        
        # Gyroscope: ±250°/s range, 131 LSB/(°/s)
        gx_dps = gx / 131.0
        gy_dps = gy / 131.0
        gz_dps = gz / 131.0
        
        return {
            'accel_raw': (ax, ay, az),
            'gyro_raw': (gx, gy, gz),
            'accel_g': (ax_g, ay_g, az_g),
            'gyro_dps': (gx_dps, gy_dps, gz_dps)
        }
        
    except Exception as e:
        print(f"Sensor read error: {e}")
        return None

def detect_movement(accel_g, gyro_dps):
    """Detect different types of movement"""
    ax, ay, az = accel_g
    gx, gy, gz = gyro_dps
    
    # Finger rotation detection (sensitive to small rotations)
    if abs(gx) > 20 or abs(gy) > 20 or abs(gz) > 20:
        return "FINGER_ROTATION"
    
    # Finger bending detection
    elif abs(ax) > 0.3:
        return "FINGER_BEND"
    
    # Finger lateral movement (swipe)
    elif abs(ay) > 0.5:
        return "FINGER_SWIPE"
    
    # Finger tapping/vertical movement
    elif abs(az) > 0.4:
        return "FINGER_TAP"
    
    # Subtle movement detection
    elif abs(gx) > 5 or abs(gy) > 5 or abs(gz) > 5:
        return "FINGER_MICRO_MOVEMENT"
    
    # General movement detection
    elif abs(gx) > 50 or abs(gy) > 50 or abs(gz) > 50:
        return "ROTATION"
    elif abs(ax) > 0.8 or abs(ay) > 0.8:
        return "TILT"
    
    return "STABLE"

# Initialize the sensor
print("Starting ICM20948 IMU system...")
imu_initialized = initialize_icm20948()

print("\nBeginning continuous data stream...")
print("Move the IMU to see real-time motion detection!")
print("Connect computer plotter to visualize data in real-time")
print("-" * 60)

# Main monitoring loop
count = 0
start_time = time.ticks_ms()

while True:
    try:
        # Read sensor data
        imu_data = read_imu_data()
        
        if imu_data:
            count += 1
            current_time = time.ticks_ms()
            
            # Extract data
            accel_raw = imu_data['accel_raw']
            gyro_raw = imu_data['gyro_raw']
            accel_g = imu_data['accel_g']
            gyro_dps = imu_data['gyro_dps']
            
            # Detect movement
            movement = detect_movement(accel_g, gyro_dps)
            
            # Output data in computer plotter friendly format
            print(f"\nReading {count:3d} - {current_time}ms:")
            print(f"   Accel (raw): X={accel_raw[0]:6d}, Y={accel_raw[1]:6d}, Z={accel_raw[2]:6d}")
            print(f"   Gyro  (raw): X={gyro_raw[0]:6d}, Y={gyro_raw[1]:6d}, Z={gyro_raw[2]:6d}")
            print(f"   Accel (g):   X={accel_g[0]:6.2f}, Y={accel_g[1]:6.2f}, Z={accel_g[2]:6.2f}")
            print(f"   Gyro (°/s):  X={gyro_dps[0]:6.1f}, Y={gyro_dps[1]:6.1f}, Z={gyro_dps[2]:6.1f}")
            
            # Movement indication
            if movement != "STABLE":
                print(f"   {movement} DETECTED!")
            else:
                print(f"   {movement}")
        
        else:
            # Fallback if IMU reading fails
            count += 1
            print(f"Reading {count}: IMU data unavailable")
        
        time.sleep(1)  # 1Hz update rate for optimal plotting
        
    except KeyboardInterrupt:
        print(f"\nData stream stopped by user after {count} readings")
        print("ICM20948 IMU system shutdown complete")
        break
        
    except Exception as e:
        count += 1
        print(f"Error in reading {count}: {e}")
        print("Continuing...")
        time.sleep(1)

print("\nSession Summary:")
print(f"Total readings: {count}")
print(f"Runtime: {(time.ticks_ms() - start_time) // 1000} seconds")
print("IMU data logging complete!")
```

### Step 3: Deploy to Pico
1. **Open Thonny**
2. **Connect Pico via USB**
3. **Copy the main.py code above**
4. **File → Save As → Choose "Raspberry Pi Pico" → Save as `main.py`**
5. **Press Ctrl+D to soft reset**
6. **Verify it works in Thonny (you should see sensor readings)**
7. **Close Thonny completely**

---

## Linux Computer Setup

### Step 1: Find Your Pico's Port
```bash
# Method 1: List serial devices
ls /dev/ttyACM* /dev/ttyUSB*

# Method 2: Check USB devices
lsusb | grep -i pico

# Method 3: Monitor connection
dmesg | tail -10

# Most common result: /dev/ttyACM0
```

### Step 2: Test Serial Connection
```bash
# Test if data is flowing (Pico main.py should be running)
timeout 10s cat /dev/ttyACM0

# You should see sensor readings like:
# Reading   1 - 12345ms:
#    Accel (raw): X=  -1234, Y=   5678, Z=  -9012
#    ...
```

### Step 3: Create Linux Plotter Script

**Save this as `computer_plotter.py` on your Linux machine:**

```python
# computer_plotter.py - Linux Real-time Plotter for ICM20948
# Reads data from Raspberry Pi Pico and creates live plots

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import time
import numpy as np

# Linux Configuration
SERIAL_PORT = '/dev/ttyACM0'  # Change if your port is different
BAUD_RATE = 115200
WINDOW_SIZE = 50  # Number of data points to show in plots

class ICM20948PlotterLinux:
    def __init__(self):
        print("ICM20948 Real-time Plotter for Linux")
        print("=" * 45)
        
        # Data storage with maximum size
        self.time_data = deque(maxlen=WINDOW_SIZE)
        self.accel_x = deque(maxlen=WINDOW_SIZE)
        self.accel_y = deque(maxlen=WINDOW_SIZE)
        self.accel_z = deque(maxlen=WINDOW_SIZE)
        self.gyro_x = deque(maxlen=WINDOW_SIZE)
        self.gyro_y = deque(maxlen=WINDOW_SIZE)
        self.gyro_z = deque(maxlen=WINDOW_SIZE)
        
        # Store recent readings for live display
        self.recent_readings = deque(maxlen=10)  # Show last 10 readings
        
        # Setup matplotlib with custom subplot layout
        plt.style.use('default')
        self.fig = plt.figure(figsize=(16, 12))
        
        # Create custom subplot layout using subplot2grid
        # Top row: 3 equal plots
        self.ax1 = plt.subplot2grid((2, 3), (0, 0), colspan=1)  # Accelerometer
        self.ax2 = plt.subplot2grid((2, 3), (0, 1), colspan=1)  # Gyroscope  
        self.ax3 = plt.subplot2grid((2, 3), (0, 2), colspan=1)  # Magnitude
        
        # Bottom row: Movement detection (1 cell) + Live data monitor (2 cells)
        self.ax4 = plt.subplot2grid((2, 3), (1, 0), colspan=1)  # Movement detection
        self.ax5 = plt.subplot2grid((2, 3), (1, 1), colspan=2)  # Live data display
        
        self.fig.suptitle('ICM20948 Real-time Data Visualization & Monitor', fontsize=16, fontweight='bold')
        
        # Configure accelerometer plot
        self.ax1.set_title('Accelerometer Data', fontsize=12, fontweight='bold')
        self.ax1.set_ylabel('Acceleration (g)')
        self.ax1.set_xlabel('Time (seconds)')
        self.ax1.grid(True, alpha=0.3)
        self.line_ax, = self.ax1.plot([], [], 'r-', label='X-axis', linewidth=2)
        self.line_ay, = self.ax1.plot([], [], 'g-', label='Y-axis', linewidth=2) 
        self.line_az, = self.ax1.plot([], [], 'b-', label='Z-axis', linewidth=2)
        self.ax1.legend(loc='upper right')
        self.ax1.set_ylim(-2.5, 2.5)
        
        # Configure gyroscope plot
        self.ax2.set_title('Gyroscope Data', fontsize=12, fontweight='bold')
        self.ax2.set_ylabel('Angular Velocity (°/s)')
        self.ax2.set_xlabel('Time (seconds)')
        self.ax2.grid(True, alpha=0.3)
        self.line_gx, = self.ax2.plot([], [], 'r-', label='X-axis', linewidth=2)
        self.line_gy, = self.ax2.plot([], [], 'g-', label='Y-axis', linewidth=2)
        self.line_gz, = self.ax2.plot([], [], 'b-', label='Z-axis', linewidth=2)
        self.ax2.legend(loc='upper right')
        self.ax2.set_ylim(-150, 150)
        
        # Configure acceleration magnitude plot
        self.ax3.set_title('Total Acceleration Magnitude', fontsize=12, fontweight='bold')
        self.ax3.set_ylabel('Magnitude (g)')
        self.ax3.set_xlabel('Time (seconds)')
        self.ax3.grid(True, alpha=0.3)
        self.line_mag, = self.ax3.plot([], [], 'purple', linewidth=3, label='Magnitude')
        self.ax3.legend()
        self.ax3.set_ylim(0, 3)
        
        # Configure movement detection display
        self.ax4.set_title('Movement Detection', fontsize=12, fontweight='bold')
        self.ax4.set_xlim(0, 10)
        self.ax4.set_ylim(0, 6)
        
        # Movement status display
        self.movement_text = self.ax4.text(5, 4, 'INITIALIZING...', ha='center', va='center', 
                                         fontsize=16, fontweight='bold',
                                         bbox=dict(boxstyle="round,pad=0.5", facecolor='lightblue'))
        
        # Statistics display
        self.stats_text = self.ax4.text(5, 2, 'Readings: 0\nConnection: Starting...', 
                                       ha='center', va='center', fontsize=12,
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow'))
        
        self.ax4.set_xticks([])
        self.ax4.set_yticks([])
        
        # Configure live data display panel with black background
        self.ax5.set_title('Live Data Monitor', fontsize=12, fontweight='bold', color='white')
        self.ax5.set_xlim(0, 1)
        self.ax5.set_ylim(0, 1)
        self.ax5.set_xticks([])
        self.ax5.set_yticks([])
        self.ax5.set_facecolor('black')  # Make panel background black
        
        # Live data text display
        self.data_display_text = self.ax5.text(0.03, 0.92, 'Waiting for data...', 
                                              transform=self.ax5.transAxes,
                                              fontsize=9, fontfamily='monospace',
                                              verticalalignment='top',
                                              bbox=dict(boxstyle="round,pad=0.3", facecolor='black', alpha=0.9, edgecolor='gray'),
                                              color='lightgreen',
                                              linespacing=1.3,
                                              wrap=True)
        
        # Initialize serial connection
        self.setup_serial()
        
        # Tracking variables
        self.start_time = time.time()
        self.reading_count = 0
        self.last_movement = "STABLE"
        self.current_reading_data = {}

    def setup_serial(self):
        """Setup serial connection with Linux-specific error handling"""
        print(f"Connecting to Pico at {SERIAL_PORT}...")
        
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Successfully connected to {SERIAL_PORT}")
            
            # Test connection by reading a few lines
            print("Testing connection...")
            for i in range(3):
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   Received: {line[:50]}..." if len(line) > 50 else f"   Received: {line}")
                    break
                time.sleep(0.5)
                
        except PermissionError:
            print(f"Permission denied for {SERIAL_PORT}")
            print("Fix with: sudo chmod 666 /dev/ttyACM0")
            print("Or add user to dialout group: sudo usermod -a -G dialout $USER")
            exit(1)
            
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            print("Available ports:")
            self.list_available_ports()
            print(f"Update SERIAL_PORT in script if needed")
            exit(1)
            
        except Exception as e:
            print(f"Connection error: {e}")
            print("Make sure:")
            print("   1. Pico is connected via USB")
            print("   2. Thonny is closed")
            print("   3. main.py is running on Pico")
            exit(1)
    
    def list_available_ports(self):
        """List available serial ports on Linux"""
        import glob
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if ports:
            for port in ports:
                print(f"   {port}")
        else:
            print("   No serial ports found")

    def parse_sensor_data(self, line):
        """Parse IMU data from Pico output and store detailed information"""
        try:
            # Look for reading header: "Reading 575 - 17643216ms:"
            reading_match = re.search(r'Reading\s+(\d+)\s+-\s+(\d+)ms:', line)
            if reading_match:
                reading_num, timestamp = reading_match.groups()
                self.current_reading_data = {
                    'reading_num': int(reading_num),
                    'timestamp': int(timestamp),
                    'raw_line': line
                }
                return False
            
            # Look for raw accelerometer data
            accel_raw_match = re.search(r'Accel \(raw\):\s+X=\s*([-\d]+),\s+Y=\s*([-\d]+),\s+Z=\s*([-\d]+)', line)
            if accel_raw_match:
                ax_raw, ay_raw, az_raw = map(int, accel_raw_match.groups())
                self.current_reading_data.update({
                    'accel_raw': (ax_raw, ay_raw, az_raw),
                    'accel_raw_line': line
                })
                return False
                
            # Look for raw gyroscope data
            gyro_raw_match = re.search(r'Gyro\s+\(raw\):\s+X=\s*([-\d]+),\s+Y=\s*([-\d]+),\s+Z=\s*([-\d]+)', line)
            if gyro_raw_match:
                gx_raw, gy_raw, gz_raw = map(int, gyro_raw_match.groups())
                self.current_reading_data.update({
                    'gyro_raw': (gx_raw, gy_raw, gz_raw),
                    'gyro_raw_line': line
                })
                return False
            
            # Look for accelerometer data in g's
            accel_match = re.search(r'Accel \(g\):\s+X=\s*([-\d\.]+),\s+Y=\s*([-\d\.]+),\s+Z=\s*([-\d\.]+)', line)
            if accel_match:
                ax, ay, az = map(float, accel_match.groups())
                current_time = time.time() - self.start_time
                
                self.time_data.append(current_time)
                self.accel_x.append(ax)
                self.accel_y.append(ay)
                self.accel_z.append(az)
                
                self.current_reading_data.update({
                    'accel_g': (ax, ay, az),
                    'accel_g_line': line
                })
                
                self.reading_count += 1
                return True
                
            # Look for gyroscope data in °/s
            gyro_match = re.search(r'Gyro \(°/s\):\s+X=\s*([-\d\.]+),\s+Y=\s*([-\d\.]+),\s+Z=\s*([-\d\.]+)', line)
            if gyro_match:
                gx, gy, gz = map(float, gyro_match.groups())
                # Sync with accelerometer data
                if len(self.gyro_x) < len(self.accel_x):
                    self.gyro_x.append(gx)
                    self.gyro_y.append(gy)
                    self.gyro_z.append(gz)
                    
                self.current_reading_data.update({
                    'gyro_dps': (gx, gy, gz),
                    'gyro_dps_line': line
                })
                return False
                    
            # Look for movement detection
            movement_match = re.search(r'(.+?)\s+DETECTED!|(\w+)', line.strip())
            if movement_match and 'DETECTED' in line:
                movement = movement_match.group(1) or movement_match.group(2)
                self.last_movement = movement
                self.current_reading_data.update({
                    'movement': movement,
                    'movement_line': line
                })
                
                # Complete reading - add to display
                self.add_reading_to_display()
                self.update_movement_display(movement)
                return False
                
        except Exception as e:
            print(f"Parse error: {e}")
            
        return False
    
    def add_reading_to_display(self):
        """Add complete reading data to the live display"""
        if not self.current_reading_data:
            return
            
        try:
            # Format the reading data similar to Thonny output
            reading_text = []
            
            if 'reading_num' in self.current_reading_data:
                reading_text.append(f"Reading {self.current_reading_data['reading_num']} - {self.current_reading_data['timestamp']}ms:")
                
            if 'accel_raw' in self.current_reading_data:
                ax, ay, az = self.current_reading_data['accel_raw']
                reading_text.append(f"   Accel (raw): X={ax:6d}, Y={ay:6d}, Z={az:6d}")
                
            if 'gyro_raw' in self.current_reading_data:
                gx, gy, gz = self.current_reading_data['gyro_raw']
                reading_text.append(f"   Gyro  (raw): X={gx:6d}, Y={gy:6d}, Z={gz:6d}")
                
            if 'accel_g' in self.current_reading_data:
                ax, ay, az = self.current_reading_data['accel_g']
                reading_text.append(f"   Accel (g):   X={ax:6.2f}, Y={ay:6.2f}, Z={az:6.2f}")
                
            if 'gyro_dps' in self.current_reading_data:
                gx, gy, gz = self.current_reading_data['gyro_dps']
                reading_text.append(f"   Gyro (°/s):  X={gx:6.1f}, Y={gy:6.1f}, Z={gz:6.1f}")
                
            if 'movement' in self.current_reading_data:
                movement = self.current_reading_data['movement']
                if movement != 'STABLE':
                    reading_text.append(f"   {movement} DETECTED!")
                else:
                    reading_text.append(f"   {movement}")
            
            # Add to recent readings
            if reading_text:
                self.recent_readings.append('\n'.join(reading_text))
                self.update_data_display()
                
            # Reset current reading data
            self.current_reading_data = {}
            
        except Exception as e:
            print(f"Display update error: {e}")
    
    def update_data_display(self):
        """Update the live data display panel"""
        try:
            # Create scrolling text from recent readings
            display_text = '\n\n'.join(list(self.recent_readings))
            
            # Limit text length to prevent overflow
            if len(display_text) > 1200:
                display_text = display_text[-1200:]
            
            # Truncate very long lines to prevent horizontal overflow
            lines = display_text.split('\n')
            truncated_lines = []
            for line in lines:
                if len(line) > 80:  # Limit line length
                    truncated_lines.append(line[:77] + '...')
                else:
                    truncated_lines.append(line)
            
            display_text = '\n'.join(truncated_lines)
            self.data_display_text.set_text(display_text)
            
        except Exception as e:
            print(f"Data display error: {e}")
    
    def update_movement_display(self, movement):
        """Update movement detection display with colors"""
        # Color mapping for different movements
        movement_colors = {
            'STABLE': 'lightgreen',
            'FINGER_ROTATION': 'red',
            'FINGER_BEND': 'orange', 
            'FINGER_SWIPE': 'blue',
            'FINGER_TAP': 'purple',
            'FINGER_MICRO_MOVEMENT': 'yellow',
            'ROTATION': 'darkred',
            'TILT': 'darkorange'
        }
        
        color = movement_colors.get(movement, 'gray')
        self.movement_text.set_text(movement)
        self.movement_text.set_bbox(dict(boxstyle="round,pad=0.5", facecolor=color))
    
    def update_plots(self, frame):
        """Update all plots with new data"""
        try:
            # Read available serial data
            if hasattr(self, 'serial_conn') and self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_sensor_data(line)
            
            # Update plots if we have data
            if len(self.time_data) > 1:
                times = list(self.time_data)
                
                # Update accelerometer plot
                self.line_ax.set_data(times, list(self.accel_x))
                self.line_ay.set_data(times, list(self.accel_y))
                self.line_az.set_data(times, list(self.accel_z))
                
                # Auto-scale X axis
                time_window = 20  # Show last 20 seconds
                self.ax1.set_xlim(max(0, times[-1] - time_window), times[-1] + 1)
                
                # Update gyroscope plot (sync with accelerometer)
                if len(self.gyro_x) >= len(times):
                    gyro_times = times[-len(self.gyro_x):]
                    self.line_gx.set_data(gyro_times, list(self.gyro_x)[-len(gyro_times):])
                    self.line_gy.set_data(gyro_times, list(self.gyro_y)[-len(gyro_times):])
                    self.line_gz.set_data(gyro_times, list(self.gyro_z)[-len(gyro_times):])
                    self.ax2.set_xlim(max(0, times[-1] - time_window), times[-1] + 1)
                
                # Update magnitude plot
                magnitudes = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in 
                             zip(list(self.accel_x), list(self.accel_y), list(self.accel_z))]
                self.line_mag.set_data(times, magnitudes)
                self.ax3.set_xlim(max(0, times[-1] - time_window), times[-1] + 1)
                
                # Auto-scale magnitude Y axis
                if magnitudes:
                    max_mag = max(magnitudes)
                    self.ax3.set_ylim(0, max(3, max_mag * 1.1))
                
                # Update statistics
                runtime = times[-1] if times else 0
                stats_text = f"Readings: {self.reading_count}\nRuntime: {runtime:.1f}s\nLast: {self.last_movement}"
                self.stats_text.set_text(stats_text)
                    
        except Exception as e:
            print(f"Plot update error: {e}")
        
        return [self.line_ax, self.line_ay, self.line_az, 
                self.line_gx, self.line_gy, self.line_gz, self.line_mag]

    def start_plotting(self):
        """Start the real-time plotting animation"""
        print("Starting real-time plotting...")
        print("5-Panel Display:")
        print("   Accelerometer plots")
        print("   Gyroscope plots") 
        print("   Magnitude plot")
        print("   Movement detection")
        print("   Live data monitor (Thonny-style)")
        print("Move your IMU to see live data visualization!")
        print("Close plot window or press Ctrl+C to stop")
        
        # Create animation with 50ms interval (20 FPS)
        self.animation = animation.FuncAnimation(
            self.fig, self.update_plots, interval=50, blit=False, cache_frame_data=False
        )
        
        plt.tight_layout(pad=2.0)
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nPlotting stopped by user")
        finally:
            if hasattr(self, 'serial_conn'):
                self.serial_conn.close()
                print("Serial connection closed")

def main():
    """Main function to run the Linux plotter"""
    print("ICM20948 Real-time Plotter for Linux")
    print("=" * 45)
    print("Prerequisites:")
    print("   1. Pico connected via USB")
    print("   2. main.py running on Pico")
    print("   3. Thonny closed")
    print("   4. Python packages: pip3 install matplotlib pyserial numpy")
    print("Features:")
    print("   • 5-panel real-time visualization")
    print("   • Live data monitor (same as Thonny output)")
    print("   • Movement detection with colors")
    print("   • Accelerometer, gyroscope, and magnitude plots")
    print()
    
    try:
        plotter = ICM20948PlotterLinux()
        plotter.start_plotting()
    except KeyboardInterrupt:
        print("\nGoodbye!")
    except Exception as e:
        print(f"Fatal error: {e}")

if __name__ == "__main__":
    main()
```

---

## Running the System

### Step 1: Prepare the Pico
```bash
# 1. Make sure main.py is running on Pico
# 2. Close Thonny completely
# 3. Test data flow:
timeout 10s cat /dev/ttyACM0

# You should see IMU readings
```

### Step 2: Set Permissions
```bash
# Give serial port access
sudo chmod 666 /dev/ttyACM0

# Or permanently add user to dialout group
sudo usermod -a -G dialout $USER
# (requires logout/login to take effect)
```

### Step 3: Run the Plotter
```bash
# Navigate to your script directory
cd ~/Desktop  # or wherever you saved computer_plotter.py

# Run the real-time plotter
python3 computer_plotter.py
```

### Step 4: Enjoy Real-time Visualization!
- **Move, tilt, rotate your IMU**
- **Watch live graphs update**
- **See movement detection in action**
- **Monitor detailed data like Thonny**

---

## Troubleshooting

### Common Issues and Solutions:

#### **1. "Permission denied" for /dev/ttyACM0**
```bash
sudo chmod 666 /dev/ttyACM0
# OR
sudo usermod -a -G dialout $USER && newgrp dialout
```

#### **2. "No module named 'serial'"**
```bash
pip3 install pyserial
```

#### **3. "No such file or directory" /dev/ttyACM0**
```bash
# Find your actual port:
ls /dev/ttyACM* /dev/ttyUSB*
dmesg | tail

# Update SERIAL_PORT in computer_plotter.py
```

#### **4. Matplotlib doesn't show plots**
```bash
# Install GUI backend
sudo apt install python3-tk

# Set backend
export MPLBACKEND=TkAgg
```

#### **5. No data flowing from Pico**
```bash
# Check if main.py is running:
# 1. Open Thonny
# 2. Connect to Pico  
# 3. Run main.py
# 4. Close Thonny
# 5. Try: timeout 5s cat /dev/ttyACM0
```

#### **6. IMU not detected**
- Check wiring connections
- Verify power (3.3V between VCC and GND)
- Try power cycling the Pico
- Check for loose jumper wires

---

## Features Explained

### **5-Panel Display:**
1. **Accelerometer Plot** - Real-time X, Y, Z acceleration in g's
2. **Gyroscope Plot** - Real-time X, Y, Z angular velocity in °/s
3. **Magnitude Plot** - Total acceleration magnitude
4. **Movement Detection** - Color-coded movement status
5. **Live Data Monitor** - Terminal-style scrolling data (same as Thonny)

### **Movement Detection:**
- **FINGER_ROTATION** - Small rotational movements
- **FINGER_BEND** - Bending gestures  
- **FINGER_SWIPE** - Lateral movements
- **FINGER_TAP** - Vertical tapping
- **FINGER_MICRO_MOVEMENT** - Subtle movements
- **ROTATION/TILT** - General movements

### **Data Formats:**
- **Raw values** - Direct sensor readings
- **Physical units** - Converted to g's and °/s
- **Timestamps** - Millisecond precision
- **Color coding** - Movement type indication

---

## Applications

### **Use Cases:**
- **Gesture recognition research**
- **Motion analysis projects**
- **Educational demonstrations**
- **Finger movement studies**
- **Real-time sensor visualization**
- **IMU calibration and testing**

### **Workshop/Demo Setup:**
1. **Attach IMU to finger** with tape/velcro
2. **Run the plotting system**
3. **Demonstrate finger gestures**
4. **Show real-time plots** to audience
5. **Explain movement detection** algorithms

---

## Customization

### **Adjust Movement Sensitivity:**
Edit the thresholds in main.py `detect_movement()` function:
```python
# More sensitive (smaller thresholds)
if abs(gx) > 10 or abs(gy) > 10 or abs(gz) > 10:
    return "FINGER_ROTATION"

# Less sensitive (larger thresholds)  
if abs(gx) > 50 or abs(gy) > 50 or abs(gz) > 50:
    return "FINGER_ROTATION"
```

### **Change Update Rate:**
In main.py, adjust the sleep time:
```python
time.sleep(0.5)  # 2Hz update rate
time.sleep(1)    # 1Hz update rate (default)
time.sleep(2)    # 0.5Hz update rate
```

### **Modify Plot Colors:**
In computer_plotter.py, change line colors:
```python
self.line_ax, = self.ax1.plot([], [], 'purple', label='X-axis')  # Change 'r-' to 'purple'
```

---

## Additional Resources

### **Documentation:**
- [ICM20948 Datasheet](https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf)
- [Raspberry Pi Pico Documentation](https://datasheets.raspberrypi.org/pico/pico-datasheet.pdf)
- [MicroPython Documentation](https://docs.micropython.org/)
- [Matplotlib Documentation](https://matplotlib.org/stable/contents.html)

### **Code Repository:**
- Save both `main.py` and `computer_plotter.py` for future use
- Consider version control with git for modifications
- Document any custom thresholds or modifications

---

## Success Checklist

**Hardware Setup:**
- [ ] ICM20948 connected to Pico (4 wires)
- [ ] Pico connected to Linux computer via USB
- [ ] Power LED on IMU board lit (if available)

**Software Setup:**  
- [ ] Python packages installed (matplotlib, pyserial, numpy)
- [ ] Serial permissions configured
- [ ] main.py saved on Pico and running independently
- [ ] computer_plotter.py saved on Linux computer
- [ ] Thonny closed when running plotter

**Testing:**
- [ ] `timeout 10s cat /dev/ttyACM0` shows IMU data
- [ ] WHO_AM_I register reads 0xEA (ICM20948 detected)
- [ ] Accelerometer and gyroscope values change when moving IMU
- [ ] Movement detection works (FINGER_ROTATION, FINGER_BEND, etc.)

**Real-time Plotting:**
- [ ] computer_plotter.py connects to Pico successfully
- [ ] All 5 panels display correctly
- [ ] Live data scrolls in terminal-style monitor
- [ ] Graphs update smoothly when moving IMU
- [ ] Movement detection colors work properly

---

## Advanced Tips

### **Performance Optimization:**
```bash
# For better plotting performance:
export MPLBACKEND=Qt5Agg  # Try different backends
# OR
export MPLBACKEND=TkAgg

# Reduce CPU usage:
# In computer_plotter.py, change animation interval:
# interval=100  # Lower FPS, less CPU usage
# interval=50   # Higher FPS, more CPU usage
```

### **Data Logging:**
Add this to computer_plotter.py for data recording:
```python
# Add to __init__ method:
self.log_file = open('imu_data.csv', 'w')
self.log_file.write('timestamp,ax,ay,az,gx,gy,gz,movement\n')

# Add to parse_sensor_data method:
if self.current_reading_data:
    timestamp = time.time()
    ax, ay, az = self.current_reading_data.get('accel_g', (0,0,0))
    gx, gy, gz = self.current_reading_data.get('gyro_dps', (0,0,0))
    movement = self.current_reading_data.get('movement', 'UNKNOWN')
    self.log_file.write(f'{timestamp},{ax},{ay},{az},{gx},{gy},{gz},{movement}\n')
    self.log_file.flush()
```

### **Multiple IMU Support:**
For multiple sensors, modify I2C addresses:
```python
# In main.py, if using multiple ICM20948s:
ICM20948_ADDR_1 = 0x68  # Default address
ICM20948_ADDR_2 = 0x69  # Alternative address (connect AD0 to VCC)
```

### **Wireless Setup (Pico W):**
For wireless data transmission:
```python
# Add to main.py for Pico W:
import network
import socket

# Setup WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect('YourWiFi', 'YourPassword')

# Send data via UDP/TCP instead of serial
```

### **Calibration:**
Add calibration routine to main.py:
```python
def calibrate_imu():
    """Calibrate IMU by measuring bias when stationary"""
    print("Calibrating IMU - keep sensor still for 5 seconds...")
    
    accel_bias = [0, 0, 0]
    gyro_bias = [0, 0, 0]
    samples = 50
    
    for i in range(samples):
        data = read_imu_data()
        if data:
            for j in range(3):
                accel_bias[j] += data['accel_g'][j]
                gyro_bias[j] += data['gyro_dps'][j]
        time.sleep(0.1)
    
    # Calculate average bias
    accel_bias = [b/samples for b in accel_bias]
    gyro_bias = [b/samples for b in gyro_bias]
    
    print(f"Calibration complete:")
    print(f"   Accel bias: {accel_bias}")
    print(f"   Gyro bias: {gyro_bias}")
    
    return accel_bias, gyro_bias
```

---

## Maintenance & Updates

### **Regular Maintenance:**
```bash
# Update Python packages
pip3 install --upgrade matplotlib pyserial numpy

# Update system packages
sudo apt update && sudo apt upgrade

# Check for MicroPython updates on Pico
# Download latest from: https://micropython.org/download/
```

### **Backup Your Setup:**
```bash
# Create backup directory
mkdir ~/imu_project_backup

# Copy files
cp computer_plotter.py ~/imu_project_backup/
cp main.py ~/imu_project_backup/  # if saved locally

# Document your hardware setup
echo "Hardware Setup Notes:" > ~/imu_project_backup/setup_notes.txt
echo "Pico Pin 4 (GP2) → IMU SDA" >> ~/imu_project_backup/setup_notes.txt
echo "Pico Pin 5 (GP3) → IMU SCL" >> ~/imu_project_backup/setup_notes.txt
echo "Working serial port: /dev/ttyACM0" >> ~/imu_project_backup/setup_notes.txt
```

---

## Project Extensions

### **1. Machine Learning Integration:**
```python
# Add to computer_plotter.py for gesture classification:
from sklearn.ensemble import RandomForestClassifier
import pandas as pd

# Collect labeled gesture data
# Train classifier on movement patterns
# Real-time gesture prediction
```

### **2. Web Dashboard:**
```python
# Create web interface using Flask:
from flask import Flask, render_template
import json

app = Flask(__name__)

@app.route('/')
def dashboard():
    return render_template('imu_dashboard.html')

@app.route('/api/data')
def get_data():
    return json.dumps(current_imu_data)
```

### **3. Mobile App Integration:**
```python
# Send data to mobile app via Bluetooth or WiFi
# Real-time finger tracking on smartphone
# Gesture-controlled games or applications
```

### **4. 3D Visualization:**
```python
# Add 3D orientation visualization:
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Calculate orientation from accelerometer/gyroscope
# Display 3D cube showing IMU orientation
```

---

## Support & Community

### **Getting Help:**
- **GitHub Issues:** Create detailed bug reports with error logs
- **Forums:** Raspberry Pi Foundation Forum, MicroPython Forum
- **Documentation:** Always check official docs first
- **Stack Overflow:** Tag questions with `raspberry-pi-pico`, `micropython`, `matplotlib`

### **Contributing:**
- **Share improvements** to the code
- **Document new features** or optimizations  
- **Report bugs** with detailed reproduction steps
- **Create tutorials** for specific use cases

### **Sharing Your Project:**
- **Social media:** Share videos of real-time plotting
- **Educational use:** Adapt for classroom demonstrations
- **Research applications:** Cite in academic papers
- **Open source:** Contribute to IMU/sensor communities

---

## Conclusion

You now have a complete, professional-grade real-time IMU visualization system! This setup provides:

### **What You've Achieved:**
- **Hardware integration** between ICM20948 and Raspberry Pi Pico
- **Real-time data streaming** from embedded device to computer
- **Professional visualization** with 5-panel matplotlib interface
- **Movement detection algorithms** for finger gesture recognition
- **Cross-platform compatibility** (Linux-focused with Windows version available)
- **Terminal-style monitoring** for detailed sensor analysis
- **Extensible codebase** for future enhancements

### **Next Steps:**
- **Experiment with different mounting** methods for the IMU
- **Adjust sensitivity thresholds** for your specific application
- **Explore advanced signal processing** techniques
- **Integrate with other sensors** (GPS, environmental sensors)
- **Develop specific applications** (gesture control, motion analysis, etc.)

### **Learning Outcomes:**
- **Embedded programming** with MicroPython
- **Real-time data visualization** with matplotlib
- **Serial communication** between devices
- **Signal processing** and movement detection
- **System integration** and troubleshooting

**Congratulations on building a sophisticated real-time sensor monitoring system!**

This guide serves as both a complete setup tutorial and a reference for future development. Save it for troubleshooting, modifications, and sharing with others interested in IMU-based projects.

---

## Quick Reference Commands

```bash
# Essential Linux commands for this project:

# Setup
sudo apt update && pip3 install matplotlib pyserial numpy
sudo usermod -a -G dialout $USER

# Troubleshooting  
ls /dev/ttyACM* /dev/ttyUSB*
timeout 10s cat /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0

# Running
python3 computer_plotter.py

# Verification
python3 -c "import matplotlib, serial, numpy; print('Ready!')"
```

**Happy plotting!**
