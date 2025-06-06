import time
import struct
from machine import I2C

# ICM20948 I2C address
ICM20948_ADDRESS = 0x68

# Register addresses
ICM20948_WHO_AM_I = 0x00
ICM20948_PWR_MGMT_1 = 0x06
ICM20948_PWR_MGMT_2 = 0x07
ICM20948_ACCEL_XOUT_H = 0x2D
ICM20948_GYRO_XOUT_H = 0x33
ICM20948_TEMP_OUT_H = 0x39

# Expected device ID
ICM20948_CHIP_ID = 0xEA

class ICM20948:
    """
    Working MicroPython driver for ICM20948 9-DOF IMU
    
    Hardware connections (tested configuration):
    - Pico GP2 (Pin 4) → IMU SDA
    - Pico GP3 (Pin 5) → IMU SCL  
    - Pico 3V3 (Pin 36) → IMU VCC
    - Pico GND → IMU GND
    
    Usage:
        i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=10000)
        imu = ICM20948(i2c)
        ax, ay, az = imu.read_accel()
        gx, gy, gz = imu.read_gyro()
    """
    
    def __init__(self, i2c, addr=ICM20948_ADDRESS):
        """
        Initialize ICM20948
        
        Args:
            i2c: I2C object (use freq=10000 for stability)
            addr: I2C address (default 0x68)
        """
        self.i2c = i2c
        self.addr = addr
        
        # Verify device
        self._verify_device()
        
        # Initialize with working sequence
        self._initialize()
        
        print("ICM20948 driver initialized successfully")
    
    def _verify_device(self):
        """Verify this is an ICM20948"""
        try:
            who_am_i = self.i2c.readfrom_mem(self.addr, ICM20948_WHO_AM_I, 1)[0]
            if who_am_i != ICM20948_CHIP_ID:
                raise RuntimeError(f"Wrong device ID: 0x{who_am_i:02X}, expected 0x{ICM20948_CHIP_ID:02X}")
            print(f"ICM20948 detected (WHO_AM_I: 0x{who_am_i:02X})")
        except Exception as e:
            raise RuntimeError(f"Failed to communicate with ICM20948: {e}")
    
    def _initialize(self):
        """Initialize ICM20948 with proven working sequence"""
        try:
            # Reset device
            self.i2c.writeto_mem(self.addr, ICM20948_PWR_MGMT_1, bytes([0x80]))
            time.sleep(0.1)
            
            # Wake up with auto clock
            self.i2c.writeto_mem(self.addr, ICM20948_PWR_MGMT_1, bytes([0x01]))
            time.sleep(0.05)
            
            # Enable all sensors
            self.i2c.writeto_mem(self.addr, ICM20948_PWR_MGMT_2, bytes([0x00]))
            time.sleep(0.05)
            
            print("ICM20948 initialization sequence complete")
            
        except Exception as e:
            raise RuntimeError(f"ICM20948 initialization failed: {e}")
    
    def read_accel(self):
        """
        Read accelerometer data
        
        Returns:
            tuple: (ax, ay, az) in raw units
            
        Note: 
            Raw values are approximately ±2g range with 16384 LSB/g
            To convert to g: value / 16384.0
        """
        try:
            data = self.i2c.readfrom_mem(self.addr, ICM20948_ACCEL_XOUT_H, 6)
            ax = struct.unpack('>h', data[0:2])[0]
            ay = struct.unpack('>h', data[2:4])[0]
            az = struct.unpack('>h', data[4:6])[0]
            return (ax, ay, az)
        except Exception as e:
            raise RuntimeError(f"Failed to read accelerometer: {e}")
    
    def read_gyro(self):
        """
        Read gyroscope data
        
        Returns:
            tuple: (gx, gy, gz) in raw units
            
        Note:
            Raw values are approximately ±250°/s range with 131 LSB/(°/s)  
            To convert to °/s: value / 131.0
        """
        try:
            data = self.i2c.readfrom_mem(self.addr, ICM20948_GYRO_XOUT_H, 6)
            gx = struct.unpack('>h', data[0:2])[0]
            gy = struct.unpack('>h', data[2:4])[0]
            gz = struct.unpack('>h', data[4:6])[0]
            return (gx, gy, gz)
        except Exception as e:
            raise RuntimeError(f"Failed to read gyroscope: {e}")
    
    def read_accel_g(self):
        """
        Read accelerometer data in g units
        
        Returns:
            tuple: (ax, ay, az) in g's
        """
        ax, ay, az = self.read_accel()
        return (ax / 16384.0, ay / 16384.0, az / 16384.0)
    
    def read_gyro_dps(self):
        """
        Read gyroscope data in degrees/second
        
        Returns:
            tuple: (gx, gy, gz) in °/s
        """
        gx, gy, gz = self.read_gyro()
        return (gx / 131.0, gy / 131.0, gz / 131.0)
    
    def read_temperature(self):
        """
        Read internal temperature sensor
        
        Returns:
            float: Temperature in Celsius (approximate)
        """
        try:
            data = self.i2c.readfrom_mem(self.addr, ICM20948_TEMP_OUT_H, 2)
            temp_raw = struct.unpack('>h', data)[0]
            # Convert using datasheet formula (approximate)
            temp_c = (temp_raw / 333.87) + 21.0
            return temp_c
        except Exception as e:
            raise RuntimeError(f"Failed to read temperature: {e}")
    
    def read_all(self):
        """
        Read all sensor data at once
        
        Returns:
            dict: {'accel': (ax,ay,az), 'gyro': (gx,gy,gz), 'temp': temp_c}
        """
        return {
            'accel': self.read_accel(),
            'gyro': self.read_gyro(),
            'temp': self.read_temperature()
        }
    
    def whoami(self):
        """
        Read WHO_AM_I register
        
        Returns:
            int: Device ID (should be 0xEA for ICM20948)
        """
        return self.i2c.readfrom_mem(self.addr, ICM20948_WHO_AM_I, 1)[0]

# Example usage
if __name__ == "__main__":
    from machine import Pin
    
    print("ICM20948 Driver Test")
    
    # Initialize I2C with working configuration
    i2c = I2C(1, scl=Pin(3), sda=Pin(2), freq=10000)
    
    # Create IMU object
    imu = ICM20948(i2c)
    
    # Test readings
    print("\nTesting sensor readings...")
    for i in range(5):
        ax, ay, az = imu.read_accel()
        gx, gy, gz = imu.read_gyro()
        temp = imu.read_temperature()
        
        print(f"Reading {i+1}:")
        print(f"  Accel: {ax:6d}, {ay:6d}, {az:6d}")
        print(f"  Gyro:  {gx:6d}, {gy:6d}, {gz:6d}")
        print(f"  Temp:  {temp:.1f}°C")
        
        time.sleep(1)
    
    print("Driver test complete!")