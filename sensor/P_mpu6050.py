# sensors/mpu6050_kalman.py
from .base import Sensor
import time
import math
from profensensors.utils.kalman import KalmanAngle

class MPU6050(Sensor):
    # Register addresses
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    ACCEL_CONFIG = 0x1C
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47

    def __init__(self, bus, mux=None, channel=None, address=0x68, name="MPU6050", restrict_pitch=True):
        super().__init__(name, channel, uses_i2c=True)
        self.bus = bus
        self.mux = mux
        self.address = address
        self.restrict_pitch = restrict_pitch
        self.rad_to_deg = 57.2957786

        # Kalman filters
        self.kalmanX = KalmanAngle()
        self.kalmanY = KalmanAngle()
        self.kalAngleX = 0
        self.kalAngleY = 0
        self.timer = time.time()

        # Initialize the device
        try:
            self._init_mpu()
            self._calibrate_initial_angles()
            print(f"MPU6050 {name} initialized successfully")
        except Exception as e:
            print(f"Error initializing MPU6050 {name}: {e}")
            raise  # Re-raise to indicate initialization failure

    def _select_channel(self):
        """Select I2C multiplexer channel with settling time"""
        if self.mux and self.channel is not None:
            self.mux.select_channel(self.channel)
            time.sleep(0.002)  # 2ms settling time for MPU6050

    def _init_mpu(self):
        """Initialize MPU6050 registers"""
        self._select_channel()
        
        # Wake up the MPU6050 (it starts in sleep mode)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        
        # Set sample rate divider (125Hz with 1kHz internal sampling)
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 7)
        # Configure accelerometer (+/-2g)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        # Configure gyroscope (+/-250deg/s)  
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        # Set digital low pass filter (5Hz bandwidth)
        self.bus.write_byte_data(self.address, self.CONFIG, 0x06)
        # Enable data ready interrupt
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 0x01)

    def _calibrate_initial_angles(self):
        """Calibrate initial angles for Kalman filter"""
        time.sleep(0.1)  # Let sensor stabilize
        
        try:
            accX, accY, accZ = self._read_accel_internal(select_channel=True)
            
            # Check for valid readings
            if abs(accX) + abs(accY) + abs(accZ) < 1000:
                raise ValueError("Invalid accelerometer readings - check connections")
            
            # Calculate initial angles with zero-division protection
            if self.restrict_pitch:
                roll = math.atan2(accY, accZ) * self.rad_to_deg if accZ != 0 else 0
                denominator = math.sqrt((accY**2) + (accZ**2))
                pitch = math.atan(-accX / denominator) * self.rad_to_deg if denominator != 0 else 0
            else:
                denominator1 = math.sqrt((accX**2) + (accZ**2))
                roll = math.atan(accY / denominator1) * self.rad_to_deg if denominator1 != 0 else 0
                pitch = math.atan2(-accX, accZ) * self.rad_to_deg

            self.kalmanX.setAngle(roll)
            self.kalmanY.setAngle(pitch)
            self.kalAngleX = roll
            self.kalAngleY = pitch
            
        except Exception as e:
            print(f"Calibration warning for {self.name}: {e}")
            self.kalAngleX = 0
            self.kalAngleY = 0

    def _read_raw_data(self, addr, select_channel=True):
        """Read raw 16-bit data from specified register"""
        if select_channel:
            self._select_channel()
        
        data = self.bus.read_i2c_block_data(self.address, addr, 2)
        value = (data[0] << 8) | data[1]
        return value - 65536 if value > 32767 else value

    def _read_all_sensors_raw(self, select_channel=True):
        """Read all sensor data in one optimized burst"""
        if select_channel:
            self._select_channel()
        
        try:
            # Read all 14 bytes at once (accel + temp + gyro)
            data = self.bus.read_i2c_block_data(self.address, self.ACCEL_XOUT_H, 14)
            
            # Parse accelerometer data
            accX = (data[0] << 8) | data[1]
            if accX > 32767: accX -= 65536
            accY = (data[2] << 8) | data[3]
            if accY > 32767: accY -= 65536
            accZ = (data[4] << 8) | data[5]
            if accZ > 32767: accZ -= 65536
            
            # Skip temperature (bytes 6-7)
            
            # Parse gyroscope data
            gyroX = (data[8] << 8) | data[9]
            if gyroX > 32767: gyroX -= 65536
            gyroY = (data[10] << 8) | data[11]
            if gyroY > 32767: gyroY -= 65536
            gyroZ = (data[12] << 8) | data[13]
            if gyroZ > 32767: gyroZ -= 65536
            
            return accX, accY, accZ, gyroX, gyroY, gyroZ
            
        except Exception as e:
            # Fallback to individual reads
            return self._read_sensors_individual(select_channel=False)

    def _read_sensors_individual(self, select_channel=True):
        """Fallback method: read sensors individually"""
        accX, accY, accZ = self._read_accel_internal(select_channel)
        gyroX, gyroY, gyroZ = self._read_gyro_internal(select_channel=False)  # Channel already selected
        return accX, accY, accZ, gyroX, gyroY, gyroZ

    def _read_accel_internal(self, select_channel=True):
        """Internal accelerometer reading with channel control"""
        if select_channel:
            self._select_channel()
            
        accX = self._read_raw_data(self.ACCEL_XOUT_H, select_channel=False)
        accY = self._read_raw_data(self.ACCEL_YOUT_H, select_channel=False)
        accZ = self._read_raw_data(self.ACCEL_ZOUT_H, select_channel=False)
        return accX, accY, accZ

    def _read_gyro_internal(self, select_channel=True):
        """Internal gyroscope reading with channel control"""
        if select_channel:
            self._select_channel()
            
        gyroX = self._read_raw_data(self.GYRO_XOUT_H, select_channel=False)
        gyroY = self._read_raw_data(self.GYRO_YOUT_H, select_channel=False)
        gyroZ = self._read_raw_data(self.GYRO_ZOUT_H, select_channel=False)
        return gyroX, gyroY, gyroZ

    def read(self):
        """Standard read method with channel selection"""
        return self._read_internal(select_channel=True)
    
    def read_without_channel_select(self):
        """Read without changing channel (assumes channel already set)"""
        return self._read_internal(select_channel=False)

    def _read_internal(self, select_channel=True):
        """Internal read method with channel selection control"""
        try:
            # Read all sensor data with minimal I2C transactions
            accX, accY, accZ, gyroX, gyroY, gyroZ = self._read_all_sensors_raw(select_channel)

            # Calculate time delta for Kalman filter
            current_time = time.time()
            dt = current_time - self.timer
            self.timer = current_time

            # Compute roll & pitch from accelerometer with zero-division protection
            if self.restrict_pitch:
                roll = math.atan2(accY, accZ) * self.rad_to_deg if accZ != 0 else 0
                denominator = math.sqrt((accY**2) + (accZ**2))
                pitch = math.atan(-accX / denominator) * self.rad_to_deg if denominator != 0 else 0
            else:
                denominator1 = math.sqrt((accX**2) + (accZ**2))
                roll = math.atan(accY / denominator1) * self.rad_to_deg if denominator1 != 0 else 0
                pitch = math.atan2(-accX, accZ) * self.rad_to_deg

            # Convert gyro readings to degrees per second
            gyroXRate = gyroX / 131.0
            gyroYRate = gyroY / 131.0

            # Apply Kalman filtering
            if self.restrict_pitch:
                # Handle angle wrapping for roll
                if ((roll < -90 and self.kalAngleX > 90) or (roll > 90 and self.kalAngleX < -90)):
                    self.kalmanX.setAngle(roll)
                    self.kalAngleX = roll
                else:
                    self.kalAngleX = self.kalmanX.getAngle(roll, gyroXRate, dt)

                # Adjust gyro rate based on roll angle
                if abs(self.kalAngleX) > 90:
                    gyroYRate = -gyroYRate
                self.kalAngleY = self.kalmanY.getAngle(pitch, gyroYRate, dt)
            else:
                # Handle angle wrapping for pitch
                if ((pitch < -90 and self.kalAngleY > 90) or (pitch > 90 and self.kalAngleY < -90)):
                    self.kalmanY.setAngle(pitch)
                    self.kalAngleY = pitch
                else:
                    self.kalAngleY = self.kalmanY.getAngle(pitch, gyroYRate, dt)

                # Adjust gyro rate based on pitch angle
                if abs(self.kalAngleY) > 90:
                    gyroXRate = -gyroXRate
                self.kalAngleX = self.kalmanX.getAngle(roll, gyroXRate, dt)

            return {
                "ok": True,
                "ax": accX, "ay": accY, "az": accZ,
                "gx": gyroX, "gy": gyroY, "gz": gyroZ,
                "roll": round(self.kalAngleX, 2),
                "pitch": round(self.kalAngleY, 2),
                "dt": round(dt * 1000, 1),  # Delta time in milliseconds
                "channel_selected": select_channel
            }
            
        except Exception as e:
            return {
                "ok": False,
                "error": str(e),
                "ax": None, "ay": None, "az": None,  # None instead of 0
                "gx": None, "gy": None, "gz": None,
                "roll": None, "pitch": None,
                "dt": None
            }

    # Legacy methods for backward compatibility
    def _read_accel(self):
        """Legacy method - use _read_accel_internal instead"""
        return self._read_accel_internal(select_channel=True)

    def _read_gyro(self):
        """Legacy method - use _read_gyro_internal instead"""  
        return self._read_gyro_internal(select_channel=True)