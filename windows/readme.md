# Complete Windows Guide: ICM20948 Real-time Plotting System

## Overview
This comprehensive guide will help you set up a complete real-time data visualization system for the ICM20948 IMU sensor using Raspberry Pi Pico and a Windows computer.

## What You'll Get
- **5-panel real-time plots** with live sensor data
- **Terminal-style data monitor** (same as Thonny output)
- **Movement detection** with color coding
- **Finger gesture recognition** (ROTATION, BEND, SWIPE, TAP)
- **Professional data visualization** for presentations/demos
- **Automatic COM port detection** for easy setup

---

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

---

## Windows Software Installation

### Step 1: Install Python
```powershell
# Option 1: Download from Microsoft Store (Recommended)
# Search "Python" in Microsoft Store and install Python 3.11+

# Option 2: Download from python.org
# Visit: https://www.python.org/downloads/windows/
# Download and install latest Python 3.x

# Verify installation
python --version
pip --version
```

### Step 2: Install Required Python Packages
```powershell
# Open PowerShell or Command Prompt as Administrator (recommended)
# Install required packages
pip install matplotlib pyserial numpy

# Alternative if above fails:
pip install --user matplotlib pyserial numpy

# Verify installation
python -c "import matplotlib, serial, numpy; print('All packages installed!')"
```

### Step 3: Install Thonny IDE
```powershell
# Download Thonny from: https://thonny.org/
# Or use winget (Windows Package Manager):
winget install ThonnyIDE.Thonny

# Alternative: Download installer directly
# https://github.com/thonny/thonny/releases/latest
```

### Step 4: Check Device Manager Setup
1. **Open Device Manager**: `Win + X` → Device Manager
2. **Connect Pico via USB**
3. **Look for COM port** under "Ports (COM & LPT)"
4. **Note the COM port number** (e.g., COM3, COM5)

---

## Pico Setup (main.py)

### Step 1: Configure Thonny for Pico

1. **Open Thonny**
2. **Go to**: Tools → Options → Interpreter
3. **Select**: "MicroPython (Raspberry Pi Pico)"
4. **Choose Port**: Select your COM port (auto-detected)
5. **Click OK**

### Step 2: Create main.py on Pico

**Copy this code and save as `main.py` on your Pico:**

```python
# Final main.py for Raspberry Pi Pico - ICM20948 IMU Sensor
# Windows-compatible version with enhanced output formatting

from machine import I2C, Pin
import time
import struct

print("=== ICM20948 IMU - Windows Compatible Version ===")
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
print("Connect Windows plotter to visualize data in real-time")
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
            
            # Output data in Windows plotter friendly format
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
6. **Verify it works in Thonny** (you should see sensor readings)
7. **Close Thonny completely**

---

## Windows Computer Setup

### Step 1: Find Your Pico's COM Port

#### Method 1: Device Manager (Visual)
1. **Open Device Manager**: `Win + X` → Device Manager
2. **Expand "Ports (COM & LPT)"**
3. **Look for your Pico** (might show as "USB Serial Device" or "Raspberry Pi Pico")
4. **Note the COM port** (e.g., COM3, COM5)

#### Method 2: PowerShell Command
```powershell
# List all COM ports
Get-WmiObject -Class Win32_SerialPort | Select-Object Name, DeviceID

# Alternative method
mode
```

#### Method 3: Python Script
```powershell
# Quick port detection
python -c "
import serial.tools.list_ports
ports = serial.tools.list_ports.comports()
for port in ports:
    print(f'{port.device}: {port.description}')
"
```

### Step 2: Test Serial Connection
```powershell
# Test if data is flowing from Pico
# (Make sure main.py is running and Thonny is closed)

# Option 1: Using PowerShell (basic test)
# You cannot directly read COM port in PowerShell easily

# Option 2: Using Python (recommended)
python -c "
import serial
import time
try:
    ser = serial.Serial('COM3', 115200, timeout=1)  # Change COM3 to your port
    print('Connected to Pico, reading data...')
    for i in range(5):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f'Received: {line}')
        time.sleep(1)
    ser.close()
except Exception as e:
    print(f'Error: {e}')
"
```

### Step 3: Create Windows Plotter Script

**Save this as `computer_plotter_windows.py` on your Windows computer:**

```python
# computer_plotter_windows.py - Windows Real-time Plotter for ICM20948
# Automatic COM port detection and Windows-optimized plotting

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import time
import numpy as np

# Windows Configuration - Auto-detect COM port
BAUD_RATE = 115200
WINDOW_SIZE = 50  # Number of data points to show in plots

class ICM20948PlotterWindows:
    def __init__(self):
        print("ICM20948 Real-time Plotter for Windows")
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
        
        # Setup matplotlib with Windows-optimized settings
        plt.style.use('default')
        
        # Set matplotlib backend for Windows
        try:
            import matplotlib
            matplotlib.use('TkAgg')  # Best for Windows
        except:
            pass  # Use default backend
            
        self.fig = plt.figure(figsize=(16, 12))
        
        # Create custom subplot layout using subplot2grid
        # Top row: 3 equal plots
        self.ax1 = plt.subplot2grid((2, 3), (0, 0), colspan=1)  # Accelerometer
        self.ax2 = plt.subplot2grid((2, 3), (0, 1), colspan=1)  # Gyroscope  
        self.ax3 = plt.subplot2grid((2, 3), (0, 2), colspan=1)  # Magnitude
        
        # Bottom row: Movement detection (1 cell) + Live data monitor (2 cells)
        self.ax4 = plt.subplot2grid((2, 3), (1, 0), colspan=1)  # Movement detection
        self.ax5 = plt.subplot2grid((2, 3), (1, 1), colspan=2)  # Live data display
        
        self.fig.suptitle('ICM20948 Real-time Data Visualization & Monitor (Windows)', 
                         fontsize=16, fontweight='bold')
        
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

    def find_pico_port(self):
        """Find Raspberry Pi Pico COM port on Windows"""
        print("Scanning for Raspberry Pi Pico...")
        
        ports = serial.tools.list_ports.comports()
        pico_ports = []
        
        print("Available COM ports:")
        for port in ports:
            print(f"   {port.device}: {port.description}")
            
            # Look for Raspberry Pi Pico identifiers
            description_lower = port.description.lower()
            if any(keyword in description_lower for keyword in 
                   ['pico', 'raspberry pi', 'rp2040', 'micropython', 'usb serial']):
                pico_ports.append(port.device)
                print(f"   Potential Pico found: {port.device}")
        
        if pico_ports:
            return pico_ports[0]  # Return first Pico found
        
        # If no Pico-specific port found, try common COM ports
        for port in ports:
            if 'COM' in port.device:
                print(f"   Trying generic port: {port.device}")
                return port.device
                
        return None

    def setup_serial(self):
        """Setup serial connection with Windows-specific handling"""
        
        # Try to find Pico automatically
        serial_port = self.find_pico_port()
        
        if not serial_port:
            print("No COM ports found!")
            print("Manual steps:")
            print("   1. Open Device Manager (Win + X → Device Manager)")
            print("   2. Look under 'Ports (COM & LPT)' for your Pico")
            print("   3. Note the COM port (e.g., COM3, COM5)")
            print("   4. Update SERIAL_PORT manually in script")
            print("   5. Make sure Pico is connected and main.py is running")
            input("Press Enter after checking Device Manager...")
            return
        
        print(f"Attempting connection to {serial_port}...")
        
        try:
            self.serial_conn = serial.Serial(serial_port, BAUD_RATE, timeout=1)
            print(f"Successfully connected to {serial_port}")
            
            # Test connection by reading a few lines
            print("Testing connection...")
            for i in range(3):
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    print(f"   Received: {line[:50]}..." if len(line) > 50 else f"   Received: {line}")
                    break
                time.sleep(0.5)
            else:
                print("No data received. Make sure:")
                print("   1. main.py is running on Pico")
                print("   2. Thonny is closed")
                print("   3. Pico is properly connected")
                
        except serial.SerialException as e:
            print(f"Serial connection failed: {e}")
            print("Troubleshooting:")
            print("   1. Close Thonny completely")
            print("   2. Try a different COM port")
            print("   3. Check Windows Device Manager")
            print("   4. Reconnect Pico USB cable")
            print("   5. Restart computer if needed")
            input("Press Enter after troubleshooting...")
            return
            
        except Exception as e:
            print(f"Connection error: {e}")
            input("Press Enter to continue...")
            return

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
        
        if not hasattr(self, 'serial_conn'):
            print("No serial connection available")
            return
        
        # Create animation with 100ms interval for Windows stability
        self.animation = animation.FuncAnimation(
            self.fig, self.update_plots, interval=100, blit=False, cache_frame_data=False
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
    """Main function to run the Windows plotter"""
    print("ICM20948 Real-time Plotter for Windows")
    print("=" * 50)
    print("Prerequisites:")
    print("   1. Pico connected via USB")
    print("   2. main.py running on Pico")
    print("   3. Thonny closed")
    print("   4. Python packages: pip install matplotlib pyserial numpy")
    print("Features:")
    print("   • 5-panel real-time visualization")
    print("   • Live data monitor (same as Thonny output)")
    print("   • Movement detection with colors")
    print("   • Accelerometer, gyroscope, and magnitude plots")
    print("   • Automatic COM port detection")
    print("   • Windows-optimized performance")
    print()
    
    try:
        plotter = ICM20948PlotterWindows()
        plotter.start_plotting()
    except KeyboardInterrupt:
        print("\nGoodbye!")
    except Exception as e:
        print(f"Fatal error: {e}")
        input("Press Enter to exit...")

if __name__ == "__main__":
    main()
```

---

## Running the System

### Step 1: Prepare the Pico
```powershell
# 1. Make sure main.py is running on Pico
# 2. Close Thonny completely
# 3. Test data flow with Python:

python -c "
import serial
import time
try:
    ser = serial.Serial('COM3', 115200, timeout=2)  # Change COM3 to your port
    print('Testing Pico connection...')
    for i in range(3):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print(f'Data received: {line[:50]}...')
            break
        time.sleep(1)
    else:
        print('No data received')
    ser.close()
except Exception as e:
    print(f'Error: {e}')
"
```

### Step 2: Run the Plotter
```powershell
# Navigate to your script directory
cd C:\Users\YourName\Desktop  # or wherever you saved the script

# Run the real-time plotter
python computer_plotter_windows.py
```

### Step 3: Enjoy Real-time Visualization!
- **Move, tilt, rotate your IMU**
- **Watch live graphs update**
- **See movement detection in action**
- **Monitor detailed data like Thonny**

---

## Windows-Specific Troubleshooting

### Common Issues and Solutions:

#### **1. "No module named 'serial'"**
```powershell
# Install pyserial
pip install pyserial

# If pip fails, try:
python -m pip install pyserial

# Or install with --user flag:
pip install --user pyserial
```

#### **2. COM Port Not Found**
```powershell
# Check Device Manager:
# Win + X → Device Manager → Ports (COM & LPT)

# List ports with Python:
python -c "
import serial.tools.list_ports
for port in serial.tools.list_ports.comports():
    print(f'{port.device}: {port.description}')
"

# Manual port testing:
python -c "
import serial
try:
    ser = serial.Serial('COM5', 115200, timeout=1)  # Try different COM numbers
    print('COM5 works')
    ser.close()
except:
    print('COM5 failed')
"
```

#### **3. "Access Denied" to COM Port**
```powershell
# Close all programs using the COM port:
# 1. Close Thonny completely
# 2. Close any other serial terminal programs
# 3. Restart the Pico (unplug/plug USB)

# Check if port is in use:
netstat -a | findstr :COM3  # Replace COM3 with your port
```

#### **4. Python Installation Issues**
```powershell
# Verify Python installation:
python --version
pip --version

# If not found, add Python to PATH:
# 1. Search "Environment Variables" in Start Menu
# 2. Click "Environment Variables"
# 3. Under "System Variables", find "Path"
# 4. Add Python installation directory

# Alternative: Reinstall Python from Microsoft Store
```

#### **5. Matplotlib Display Issues**
```powershell
# Install tkinter (GUI backend):
# Usually comes with Python, but if missing:
pip install tk

# Try different matplotlib backend:
python -c "
import matplotlib
print('Available backends:', matplotlib.backend_bases.Backend.list())
matplotlib.use('TkAgg')  # Force TkAgg backend
import matplotlib.pyplot as plt
plt.plot([1,2,3])
plt.show()
"
```

#### **6. Firewall/Antivirus Blocking**
- **Windows Defender** might block Python scripts
- **Add exception** for Python.exe and your script directory
- **Temporarily disable** real-time protection for testing

#### **7. No Data from Pico**
```powershell
# Verify Pico is running main.py:
# 1. Open Thonny
# 2. Connect to Pico
# 3. Check if main.py is in Pico files
# 4. Run main.py manually
# 5. Close Thonny and test serial connection

# Reset Pico:
# Press reset button (if available) or unplug/plug USB
```

---

## Windows Performance Optimization

### **Improve Performance:**
```python
# In computer_plotter_windows.py, modify for better performance:

# Reduce animation interval for slower computers:
interval=200  # 5 FPS (from 100ms = 10 FPS)

# Reduce data window size:
WINDOW_SIZE = 30  # From 50

# Use fewer recent readings:
self.recent_readings = deque(maxlen=5)  # From 10
```

### **Windows-Specific Settings:**
```python
# Add to __init__ method for Windows optimization:
import matplotlib
matplotlib.use('TkAgg')  # Best Windows backend
plt.ioff()  # Turn off interactive mode for better performance

# Reduce plot update frequency:
plt.rcParams['animation.html'] = 'none'
```

### **Memory Management:**
```python
# Add memory cleanup to update_plots method:
import gc
if self.reading_count % 100 == 0:  # Every 100 readings
    gc.collect()  # Force garbage collection
```

---

## Windows Applications

### **Educational Setup:**
- **Classroom demonstrations** with projector
- **Student projects** for STEM education
- **Science fair displays**
- **Engineering workshops**

### **Professional Use:**
- **Product demonstrations** for clients
- **Research data collection**
- **Motion analysis studies**
- **Sensor calibration and testing**

### **Development Environment:**
- **Rapid prototyping** of motion-based applications
- **Algorithm testing** for gesture recognition
- **Real-time debugging** of sensor data
- **Performance optimization** studies

---

## Customization for Windows

### **Adjust for Your Hardware:**
```python
# In main.py, modify update rate for your needs:
time.sleep(0.5)  # 2Hz - Less CPU usage
time.sleep(1)    # 1Hz - Default
time.sleep(2)    # 0.5Hz - Very low CPU usage

# In computer_plotter_windows.py, adjust sensitivity:
# More sensitive detection:
if abs(gx) > 10:  # Lower threshold
    return "FINGER_ROTATION"

# Less sensitive detection:
if abs(gx) > 30:  # Higher threshold
    return "FINGER_ROTATION"
```

### **Change Visual Appearance:**
```python
# Modify colors in computer_plotter_windows.py:
self.line_ax, = self.ax1.plot([], [], 'purple', label='X-axis')  # Custom colors
self.ax5.set_facecolor('darkblue')  # Different background color

# Change window size:
self.fig = plt.figure(figsize=(14, 10))  # Smaller window

# Modify fonts:
plt.rcParams['font.size'] = 12  # Larger font
plt.rcParams['font.family'] = 'Arial'  # Different font
```

### **Add Data Logging:**
```python
# Add to __init__ method:
import csv
import datetime

self.log_file = open(f'imu_data_{datetime.datetime.now().strftime("%Y%m%d_%H%M%S")}.csv', 'w', newline='')
self.csv_writer = csv.writer(self.log_file)
self.csv_writer.writerow(['timestamp', 'ax_g', 'ay_g', 'az_g', 'gx_dps', 'gy_dps', 'gz_dps', 'movement'])

# Add to parse_sensor_data method:
if self.current_reading_data and 'accel_g' in self.current_reading_data:
    timestamp = time.time()
    ax, ay, az = self.current_reading_data['accel_g']
    gx, gy, gz = self.current_reading_data.get('gyro_dps', (0, 0, 0))
    movement = self.current_reading_data.get('movement', 'UNKNOWN')
    self.csv_writer.writerow([timestamp, ax, ay, az, gx, gy, gz, movement])
    self.log_file.flush()
```

---

## Windows-Specific Resources

### **Useful Tools:**
- **Device Manager**: Monitor COM ports and hardware
- **Task Manager**: Monitor CPU/memory usage during plotting
- **PowerShell**: Advanced scripting and automation
- **Windows Terminal**: Modern command-line interface
- **Visual Studio Code**: Code editing with Python extensions

### **Python Environments:**
```powershell
# Create virtual environment (optional):
python -m venv imu_project
cd imu_project
Scripts\activate
pip install matplotlib pyserial numpy

# Deactivate when done:
deactivate
```

### **Batch Script for Easy Running:**
Create `run_plotter.bat`:
```batch
@echo off
echo Starting ICM20948 Real-time Plotter...
cd /d "%~dp0"
python computer_plotter_windows.py
pause
```

### **Scheduled Tasks:**
```powershell
# Create scheduled task to run plotter automatically:
schtasks /create /tn "ICM20948_Plotter" /tr "python C:\path\to\computer_plotter_windows.py" /sc onlogon
```

---

## Windows Success Checklist

**Hardware Setup:**
- [ ] ICM20948 connected to Pico (4 wires)
- [ ] Pico connected to Windows PC via USB
- [ ] Power LED on IMU board lit (if available)
- [ ] Device Manager shows COM port for Pico

**Software Setup:**  
- [ ] Python 3.7+ installed (check: `python --version`)
- [ ] Required packages installed: `pip install matplotlib pyserial numpy`
- [ ] Thonny IDE installed and configured
- [ ] main.py saved on Pico and running independently
- [ ] computer_plotter_windows.py saved on Windows PC

**Testing:**
- [ ] COM port identified in Device Manager
- [ ] Serial connection test shows IMU data
- [ ] WHO_AM_I register reads 0xEA (ICM20948 detected)
- [ ] Accelerometer and gyroscope values change when moving IMU
- [ ] Movement detection works (FINGER_ROTATION, FINGER_BEND, etc.)

**Real-time Plotting:**
- [ ] computer_plotter_windows.py connects to Pico successfully
- [ ] All 5 panels display correctly
- [ ] Live data scrolls in terminal-style monitor
- [ ] Graphs update smoothly when moving IMU
- [ ] Movement detection colors work properly
- [ ] No performance issues or lag

---

## Advanced Windows Features

### **1. Integration with Windows APIs:**
```python
# Add Windows notifications:
import win10toast
toaster = win10toast.ToastNotifier()

# In movement detection:
if movement == "FINGER_TAP":
    toaster.show_toast("IMU Alert", "Finger tap detected!", duration=2)
```

### **2. System Tray Integration:**
```python
# Add system tray icon:
import pystray
from PIL import Image

# Create system tray icon for background monitoring
```

### **3. Windows Service:**
```python
# Convert to Windows service for background operation:
import win32serviceutil
import win32service

class IMUService(win32serviceutil.ServiceFramework):
    _svc_name_ = "ICM20948Monitor"
    _svc_display_name_ = "ICM20948 Real-time Monitor"
    
    def SvcDoRun(self):
        # Run plotter in background
        pass
```

### **4. Integration with Excel:**
```python
# Export data to Excel:
import openpyxl
from openpyxl.chart import LineChart

# Create Excel workbook with charts
wb = openpyxl.Workbook()
ws = wb.active
# Add data and charts
```

---

## Windows Conclusion

You now have a complete, professional-grade real-time IMU visualization system optimized for Windows! This setup provides:

### **What You've Achieved:**
- **Seamless Windows integration** with automatic COM port detection
- **Professional 5-panel visualization** with real-time updates
- **Windows-optimized performance** with stable matplotlib backend
- **Comprehensive error handling** for Windows-specific issues
- **Easy deployment** with step-by-step setup instructions
- **Extensible codebase** for Windows-specific features

### **Windows-Specific Benefits:**
- **Device Manager integration** for hardware monitoring
- **PowerShell compatibility** for advanced scripting
- **Windows notification support** for alerts
- **Excel integration capabilities** for data analysis
- **Task scheduler support** for automation
- **System tray integration** options

### **Perfect for Windows Users:**
- **Educational institutions** using Windows computers
- **Corporate environments** with Windows-based development
- **Home users** familiar with Windows ecosystem
- **Researchers** using Windows for data analysis
- **Students** learning on Windows platforms

**Congratulations on building a sophisticated Windows-compatible real-time sensor monitoring system!**

This Windows guide serves as both a complete setup tutorial and a reference for future Windows-specific development. The system is now ready for:
- **Classroom demonstrations**
- **Research projects**
- **Commercial applications**
- **Educational workshops**
- **Personal experimentation**

**Your Windows-based ICM20948 real-time plotting system is now ready for professional use!**
