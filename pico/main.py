from machine import I2C, Pin
import time
import struct

print("=== ICM20948 IMU - Final Standalone Version ===")
print("Real-time Accelerometer & Gyroscope Data")
print("=" * 50)

i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=10000)
ICM20948_ADDR = 0x68

def initialize_icm20948():
    print("Initializing ICM20948...")
    
    try:
        who_am_i = i2c.readfrom_mem(ICM20948_ADDR, 0x00, 1)[0]
        if who_am_i == 0xEA:
            print(f"ICM20948 detected (WHO_AM_I: 0x{who_am_i:02X})")
        else:
            print(f"Unexpected device ID: 0x{who_am_i:02X} (expected 0xEA)")
        
        i2c.writeto_mem(ICM20948_ADDR, 0x06, bytes([0x80]))
        time.sleep(0.1)
        
        i2c.writeto_mem(ICM20948_ADDR, 0x06, bytes([0x01]))
        time.sleep(0.05)
        
        i2c.writeto_mem(ICM20948_ADDR, 0x07, bytes([0x00]))
        time.sleep(0.05)
        
        print("ICM20948 initialization complete!")
        return True
        
    except Exception as e:
        print(f"IMU initialization error: {e}")
        print("Continuing with basic operation...")
        return False

def read_imu_data():
    try:
        accel_data = i2c.readfrom_mem(ICM20948_ADDR, 0x2D, 6)
        ax = struct.unpack('>h', accel_data[0:2])[0]
        ay = struct.unpack('>h', accel_data[2:4])[0]
        az = struct.unpack('>h', accel_data[4:6])[0]
        
        gyro_data = i2c.readfrom_mem(ICM20948_ADDR, 0x33, 6)
        gx = struct.unpack('>h', gyro_data[0:2])[0]
        gy = struct.unpack('>h', gyro_data[2:4])[0]
        gz = struct.unpack('>h', gyro_data[4:6])[0]
        
        ax_g = ax / 16384.0
        ay_g = ay / 16384.0
        az_g = az / 16384.0
        
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
    ax, ay, az = accel_g
    gx, gy, gz = gyro_dps
    
    if abs(gx) > 20 or abs(gy) > 20 or abs(gz) > 20:
        return "FINGER_ROTATION"
    
    elif abs(ax) > 0.3:
        return "FINGER_BEND"
    
    elif abs(ay) > 0.5:
        return "FINGER_SWIPE"
    
    elif abs(az) > 0.4:
        return "FINGER_TAP"
    
    elif abs(gx) > 5 or abs(gy) > 5 or abs(gz) > 5:
        return "FINGER_MICRO_MOVEMENT"
    
    elif abs(gx) > 50 or abs(gy) > 50 or abs(gz) > 50:
        return "ROTATION"
    elif abs(ax) > 0.8 or abs(ay) > 0.8:
        return "TILT"
    
    return "STABLE"

print("Starting ICM20948 IMU system...")
imu_initialized = initialize_icm20948()

print("\nBeginning continuous data stream...")
print("Move the IMU to see real-time motion detection!")
print("Connect computer plotter to visualize data in real-time")
print("-" * 60)

count = 0
start_time = time.ticks_ms()

while True:
    try:
        imu_data = read_imu_data()
        
        if imu_data:
            count += 1
            current_time = time.ticks_ms()
            
            accel_raw = imu_data['accel_raw']
            gyro_raw = imu_data['gyro_raw']
            accel_g = imu_data['accel_g']
            gyro_dps = imu_data['gyro_dps']
            
            movement = detect_movement(accel_g, gyro_dps)
            
            print(f"\nReading {count:3d} - {current_time}ms:")
            print(f"   Accel (raw): X={accel_raw[0]:6d}, Y={accel_raw[1]:6d}, Z={accel_raw[2]:6d}")
            print(f"   Gyro  (raw): X={gyro_raw[0]:6d}, Y={gyro_raw[1]:6d}, Z={gyro_raw[2]:6d}")
            print(f"   Accel (g):   X={accel_g[0]:6.2f}, Y={accel_g[1]:6.2f}, Z={accel_g[2]:6.2f}")
            print(f"   Gyro (°/s):  X={gyro_dps[0]:6.1f}, Y={gyro_dps[1]:6.1f}, Z={gyro_dps[2]:6.1f}")
            
            if movement != "STABLE":
                print(f"   {movement} DETECTED!")
            else:
                print(f"   {movement}")
        
        else:
            count += 1
            print(f"Reading {count}: IMU data unavailable")
        
        time.sleep(1)
        
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