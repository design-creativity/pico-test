import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import time
import numpy as np

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200
WINDOW_SIZE = 50

class ICM20948Plotter:
    def __init__(self):
        print("ICM20948 Real-time Plotter for Ubuntu")
        print("=" * 45)
        
        self.time_data = deque(maxlen=WINDOW_SIZE)
        self.accel_x = deque(maxlen=WINDOW_SIZE)
        self.accel_y = deque(maxlen=WINDOW_SIZE)
        self.accel_z = deque(maxlen=WINDOW_SIZE)
        self.gyro_x = deque(maxlen=WINDOW_SIZE)
        self.gyro_y = deque(maxlen=WINDOW_SIZE)
        self.gyro_z = deque(maxlen=WINDOW_SIZE)
        
        self.recent_readings = deque(maxlen=10)
        
        plt.style.use('default')
        self.fig = plt.figure(figsize=(16, 12))
        
        self.ax1 = plt.subplot2grid((2, 3), (0, 0), colspan=1)
        self.ax2 = plt.subplot2grid((2, 3), (0, 1), colspan=1)
        self.ax3 = plt.subplot2grid((2, 3), (0, 2), colspan=1)
        self.ax4 = plt.subplot2grid((2, 3), (1, 0), colspan=1)
        self.ax5 = plt.subplot2grid((2, 3), (1, 1), colspan=2)
        
        self.fig.suptitle('ICM20948 Real-time Data Visualization & Monitor', fontsize=16, fontweight='bold')
        
        self.ax1.set_title('Accelerometer Data', fontsize=12, fontweight='bold')
        self.ax1.set_ylabel('Acceleration (g)')
        self.ax1.set_xlabel('Time (seconds)')
        self.ax1.grid(True, alpha=0.3)
        self.line_ax, = self.ax1.plot([], [], 'r-', label='X-axis', linewidth=2)
        self.line_ay, = self.ax1.plot([], [], 'g-', label='Y-axis', linewidth=2) 
        self.line_az, = self.ax1.plot([], [], 'b-', label='Z-axis', linewidth=2)
        self.ax1.legend(loc='upper right')
        self.ax1.set_ylim(-2.5, 2.5)
        
        self.ax2.set_title('Gyroscope Data', fontsize=12, fontweight='bold')
        self.ax2.set_ylabel('Angular Velocity (°/s)')
        self.ax2.set_xlabel('Time (seconds)')
        self.ax2.grid(True, alpha=0.3)
        self.line_gx, = self.ax2.plot([], [], 'r-', label='X-axis', linewidth=2)
        self.line_gy, = self.ax2.plot([], [], 'g-', label='Y-axis', linewidth=2)
        self.line_gz, = self.ax2.plot([], [], 'b-', label='Z-axis', linewidth=2)
        self.ax2.legend(loc='upper right')
        self.ax2.set_ylim(-150, 150)
        
        self.ax3.set_title('Total Acceleration Magnitude', fontsize=12, fontweight='bold')
        self.ax3.set_ylabel('Magnitude (g)')
        self.ax3.set_xlabel('Time (seconds)')
        self.ax3.grid(True, alpha=0.3)
        self.line_mag, = self.ax3.plot([], [], 'purple', linewidth=3, label='Magnitude')
        self.ax3.legend()
        self.ax3.set_ylim(0, 3)
        
        self.ax4.set_title('Movement Detection', fontsize=12, fontweight='bold')
        self.ax4.set_xlim(0, 10)
        self.ax4.set_ylim(0, 6)
        
        self.movement_text = self.ax4.text(5, 4, 'INITIALIZING...', ha='center', va='center', 
                                         fontsize=16, fontweight='bold',
                                         bbox=dict(boxstyle="round,pad=0.5", facecolor='lightblue'))
        
        self.stats_text = self.ax4.text(5, 2, 'Readings: 0\nConnection: Starting...', 
                                       ha='center', va='center', fontsize=12,
                                       bbox=dict(boxstyle="round,pad=0.3", facecolor='lightyellow'))
        
        self.ax4.set_xticks([])
        self.ax4.set_yticks([])
        
        self.ax5.set_title('Live Data Monitor', fontsize=12, fontweight='bold', color='white')
        self.ax5.set_xlim(0, 1)
        self.ax5.set_ylim(0, 1)
        self.ax5.set_xticks([])
        self.ax5.set_yticks([])
        self.ax5.set_facecolor('black')
        
        self.data_display_text = self.ax5.text(0.03, 0.92, 'Waiting for data...', 
                                              transform=self.ax5.transAxes,
                                              fontsize=9, fontfamily='monospace',
                                              verticalalignment='top',
                                              bbox=dict(boxstyle="round,pad=0.3", facecolor='black', alpha=0.9, edgecolor='gray'),
                                              color='lightgreen',
                                              linespacing=1.3,
                                              wrap=True)
        
        self.setup_serial()
        
        self.start_time = time.time()
        self.reading_count = 0
        self.last_movement = "STABLE"
        self.current_reading_data = {}
        
    def setup_serial(self):
        print(f"Connecting to Pico at {SERIAL_PORT}...")
        
        try:
            self.serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print(f"Successfully connected to {SERIAL_PORT}")
            
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
        import glob
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        if ports:
            for port in ports:
                print(f"   {port}")
        else:
            print("   No serial ports found")
    
    def parse_sensor_data(self, line):
        try:
            reading_match = re.search(r'Reading\s+(\d+)\s+-\s+(\d+)ms:', line)
            if reading_match:
                reading_num, timestamp = reading_match.groups()
                self.current_reading_data = {
                    'reading_num': int(reading_num),
                    'timestamp': int(timestamp),
                    'raw_line': line
                }
                return False
            
            accel_raw_match = re.search(r'Accel \(raw\):\s+X=\s*([-\d]+),\s+Y=\s*([-\d]+),\s+Z=\s*([-\d]+)', line)
            if accel_raw_match:
                ax_raw, ay_raw, az_raw = map(int, accel_raw_match.groups())
                self.current_reading_data.update({
                    'accel_raw': (ax_raw, ay_raw, az_raw),
                    'accel_raw_line': line
                })
                return False
                
            gyro_raw_match = re.search(r'Gyro\s+\(raw\):\s+X=\s*([-\d]+),\s+Y=\s*([-\d]+),\s+Z=\s*([-\d]+)', line)
            if gyro_raw_match:
                gx_raw, gy_raw, gz_raw = map(int, gyro_raw_match.groups())
                self.current_reading_data.update({
                    'gyro_raw': (gx_raw, gy_raw, gz_raw),
                    'gyro_raw_line': line
                })
                return False
            
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
                
            gyro_match = re.search(r'Gyro \(°/s\):\s+X=\s*([-\d\.]+),\s+Y=\s*([-\d\.]+),\s+Z=\s*([-\d\.]+)', line)
            if gyro_match:
                gx, gy, gz = map(float, gyro_match.groups())
                if len(self.gyro_x) < len(self.accel_x):
                    self.gyro_x.append(gx)
                    self.gyro_y.append(gy)
                    self.gyro_z.append(gz)
                    
                self.current_reading_data.update({
                    'gyro_dps': (gx, gy, gz),
                    'gyro_dps_line': line
                })
                return False
                    
            movement_match = re.search(r'(.+?)\s+DETECTED!|(\w+)', line)
            if movement_match:
                movement = movement_match.group(1) or movement_match.group(2)
                self.last_movement = movement
                self.current_reading_data.update({
                    'movement': movement,
                    'movement_line': line
                })
                
                self.add_reading_to_display()
                self.update_movement_display(movement)
                return False
                
        except Exception as e:
            print(f"Parse error: {e}")
            
        return False
    
    def add_reading_to_display(self):
        if not self.current_reading_data:
            return
            
        try:
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
            
            if reading_text:
                self.recent_readings.append('\n'.join(reading_text))
                self.update_data_display()
                
            self.current_reading_data = {}
            
        except Exception as e:
            print(f"Display update error: {e}")
    
    def update_data_display(self):
        try:
            display_text = '\n\n'.join(list(self.recent_readings))
            
            if len(display_text) > 1200:
                display_text = display_text[-1200:]
            
            lines = display_text.split('\n')
            truncated_lines = []
            for line in lines:
                if len(line) > 80:
                    truncated_lines.append(line[:77] + '...')
                else:
                    truncated_lines.append(line)
            
            display_text = '\n'.join(truncated_lines)
            self.data_display_text.set_text(display_text)
            
        except Exception as e:
            print(f"Data display error: {e}")
    
    def update_movement_display(self, movement):
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
        try:
            while self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_sensor_data(line)
            
            if len(self.time_data) > 1:
                times = list(self.time_data)
                
                self.line_ax.set_data(times, list(self.accel_x))
                self.line_ay.set_data(times, list(self.accel_y))
                self.line_az.set_data(times, list(self.accel_z))
                
                time_window = 20
                self.ax1.set_xlim(max(0, times[-1] - time_window), times[-1] + 1)
                
                if len(self.gyro_x) >= len(times):
                    gyro_times = times[-len(self.gyro_x):]
                    self.line_gx.set_data(gyro_times, list(self.gyro_x)[-len(gyro_times):])
                    self.line_gy.set_data(gyro_times, list(self.gyro_y)[-len(gyro_times):])
                    self.line_gz.set_data(gyro_times, list(self.gyro_z)[-len(gyro_times):])
                    self.ax2.set_xlim(max(0, times[-1] - time_window), times[-1] + 1)
                
                magnitudes = [np.sqrt(x**2 + y**2 + z**2) for x, y, z in 
                             zip(list(self.accel_x), list(self.accel_y), list(self.accel_z))]
                self.line_mag.set_data(times, magnitudes)
                self.ax3.set_xlim(max(0, times[-1] - time_window), times[-1] + 1)
                
                if magnitudes:
                    max_mag = max(magnitudes)
                    self.ax3.set_ylim(0, max(3, max_mag * 1.1))
                
                runtime = times[-1] if times else 0
                stats_text = f"Readings: {self.reading_count}\nRuntime: {runtime:.1f}s\nLast: {self.last_movement}"
                self.stats_text.set_text(stats_text)
                    
        except Exception as e:
            print(f"Plot update error: {e}")
        
        return [self.line_ax, self.line_ay, self.line_az, 
                self.line_gx, self.line_gy, self.line_gz, self.line_mag]

    def start_plotting(self):
        print("Starting real-time plotting...")
        print("5-Panel Display:")
        print("   Accelerometer plots")
        print("   Gyroscope plots") 
        print("   Magnitude plot")
        print("   Movement detection")
        print("   Live data monitor")
        print("Move your IMU to see live data visualization!")
        print("Close plot window or press Ctrl+C to stop")
        
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
    print("ICM20948 Real-time Plotter for Ubuntu")
    print("=" * 45)
    print("Prerequisites:")
    print("   1. Pico connected via USB")
    print("   2. main.py running on Pico")
    print("   3. Thonny closed")
    print("   4. Python packages: pip3 install matplotlib pyserial")
    print("Features:")
    print("   5-panel real-time visualization")
    print("   Live data monitor")
    print("   Movement detection with colors")
    print("   Accelerometer, gyroscope, and magnitude plots")
    print()
    
    try:
        plotter = ICM20948Plotter()
        plotter.start_plotting()
    except KeyboardInterrupt:
        print("\nGoodbye!")
    except Exception as e:
        print(f"Fatal error: {e}")

if __name__ == "__main__":
    main()