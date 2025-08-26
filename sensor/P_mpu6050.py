# sensors/mpu6050_kalman.py
from .base import Sensor
import time
import math
from utils.kalman import KalmanAngle

class MPU6050(Sensor):
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
        super().__init__(name, channel)
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

    def _select_channel(self):
        if self.mux and self.channel is not None:
            self.mux.select_channel(self.channel)

    def _init_mpu(self):
        self._select_channel()
        # Wake up the MPU6050 (it starts in sleep mode)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)
        time.sleep(0.05)
        
        # Set sample rate divider
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 7)
        # Configure accelerometer (+/-2g)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        # Configure gyroscope (+/-250deg/s)  
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        # Set digital low pass filter
        self.bus.write_byte_data(self.address, self.CONFIG, 0x06)
        # Enable data ready interrupt
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 0x01)

    def _calibrate_initial_angles(self):
        """Calibrate initial angles for Kalman filter"""
        time.sleep(0.1)  # Let sensor stabilize
        
        try:
            accX, accY, accZ = self._read_accel()
            
            # Check for valid readings
            if abs(accX) + abs(accY) + abs(accZ) < 1000:  # Very low readings indicate no connection
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
            print(f"Calibration warning: {e}")
            self.kalAngleX = 0
            self.kalAngleY = 0

    def _read_raw_data(self, addr):
        self._select_channel()
        data = self.bus.read_i2c_block_data(self.address, addr, 2)
        value = (data[0] << 8) | data[1]
        return value - 65536 if value > 32767 else value

    def _read_accel(self):
        return (
            self._read_raw_data(self.ACCEL_XOUT_H),
            self._read_raw_data(self.ACCEL_YOUT_H),
            self._read_raw_data(self.ACCEL_ZOUT_H),
        )

    def _read_gyro(self):
        return (
            self._read_raw_data(self.GYRO_XOUT_H),
            self._read_raw_data(self.GYRO_YOUT_H),
            self._read_raw_data(self.GYRO_ZOUT_H),
        )

    def read(self):
        try:
            accX, accY, accZ = self._read_accel()
            gyroX, gyroY, gyroZ = self._read_gyro()

            dt = time.time() - self.timer
            self.timer = time.time()

            # Compute roll & pitch from accel with zero-division protection
            if self.restrict_pitch:
                roll = math.atan2(accY, accZ) * self.rad_to_deg if accZ != 0 else 0
                denominator = math.sqrt((accY**2) + (accZ**2))
                pitch = math.atan(-accX / denominator) * self.rad_to_deg if denominator != 0 else 0
            else:
                denominator1 = math.sqrt((accX**2) + (accZ**2))
                roll = math.atan(accY / denominator1) * self.rad_to_deg if denominator1 != 0 else 0
                pitch = math.atan2(-accX, accZ) * self.rad_to_deg

            gyroXRate = gyroX / 131.0
            gyroYRate = gyroY / 131.0

            # Kalman filtering
            if self.restrict_pitch:
                if ((roll < -90 and self.kalAngleX > 90) or (roll > 90 and self.kalAngleX < -90)):
                    self.kalmanX.setAngle(roll)
                    self.kalAngleX = roll
                else:
                    self.kalAngleX = self.kalmanX.getAngle(roll, gyroXRate, dt)

                if abs(self.kalAngleX) > 90:
                    gyroYRate = -gyroYRate
                self.kalAngleY = self.kalmanY.getAngle(pitch, gyroYRate, dt)
            else:
                if ((pitch < -90 and self.kalAngleY > 90) or (pitch > 90 and self.kalAngleY < -90)):
                    self.kalmanY.setAngle(pitch)
                    self.kalAngleY = pitch
                else:
                    self.kalAngleY = self.kalmanY.getAngle(pitch, gyroYRate, dt)

                if abs(self.kalAngleY) > 90:
                    gyroXRate = -gyroXRate
                self.kalAngleX = self.kalmanX.getAngle(roll, gyroXRate, dt)

            return {
                "ok": True,
                "ax": accX, "ay": accY, "az": accZ,
                "gx": gyroX, "gy": gyroY, "gz": gyroZ,
                "roll": round(self.kalAngleX, 2),
                "pitch": round(self.kalAngleY, 2)
            }
        except Exception as e:
            return {
                "ok": False,
                "error": str(e),
                "ax": 0, "ay": 0, "az": 0,
                "gx": 0, "gy": 0, "gz": 0,
                "roll": 0, "pitch": 0
            }
