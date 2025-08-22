# sensors/mpu6050_kalman.py
from .base import Sensor
import smbus
import time
import math
from utils.kalman import KalmanAngle

class MPU6050(Sensor):
    PWR_MGMT_1   = 0x6B
    SMPLRT_DIV   = 0x19
    CONFIG       = 0x1A
    GYRO_CONFIG  = 0x1B
    INT_ENABLE   = 0x38
    ACCEL_XOUT_H = 0x3B
    ACCEL_YOUT_H = 0x3D
    ACCEL_ZOUT_H = 0x3F
    GYRO_XOUT_H  = 0x43
    GYRO_YOUT_H  = 0x45
    GYRO_ZOUT_H  = 0x47

    def __init__(self, bus, address=0x68, name="MPU6050", restrict_pitch=True):
        super().__init__(name, bus)
        self.bus = bus
        self.address = address
        self.restrict_pitch = restrict_pitch

        self.kalmanX = KalmanAngle()
        self.kalmanY = KalmanAngle()
        self.rad_to_deg = 57.2957786

        self._init_mpu()

        # Initial orientation
        accX, accY, accZ = self._read_accel()
        if self.restrict_pitch:
            roll  = math.atan2(accY, accZ) * self.rad_to_deg
            pitch = math.atan(-accX / math.sqrt((accY**2) + (accZ**2))) * self.rad_to_deg
        else:
            roll  = math.atan(accY / math.sqrt((accX**2) + (accZ**2))) * self.rad_to_deg
            pitch = math.atan2(-accX, accZ) * self.rad_to_deg

        self.kalmanX.setAngle(roll)
        self.kalmanY.setAngle(pitch)
        self.kalAngleX = roll
        self.kalAngleY = pitch

        self.timer = time.time()

    def _init_mpu(self):
        # Initialize MPU registers
        self.bus.write_byte_data(self.address, self.SMPLRT_DIV, 7)
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 1)
        self.bus.write_byte_data(self.address, self.CONFIG, int('0000110',2)) # DLPF
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 24)
        self.bus.write_byte_data(self.address, self.INT_ENABLE, 1)

    def _read_raw_data(self, addr):
        high = self.bus.read_byte_data(self.address, addr)
        low = self.bus.read_byte_data(self.address, addr+1)
        value = (high << 8) | low
        if value > 32768:  # convert to signed
            value -= 65536
        return value

    def _read_accel(self):
        accX = self._read_raw_data(self.ACCEL_XOUT_H)
        accY = self._read_raw_data(self.ACCEL_YOUT_H)
        accZ = self._read_raw_data(self.ACCEL_ZOUT_H)
        return accX, accY, accZ

    def _read_gyro(self):
        gyroX = self._read_raw_data(self.GYRO_XOUT_H)
        gyroY = self._read_raw_data(self.GYRO_YOUT_H)
        gyroZ = self._read_raw_data(self.GYRO_ZOUT_H)
        return gyroX, gyroY, gyroZ

    def read(self):
        accX, accY, accZ = self._read_accel()
        gyroX, gyroY, _ = self._read_gyro()

        dt = time.time() - self.timer
        self.timer = time.time()

        # Compute roll, pitch
        if self.restrict_pitch:
            roll  = math.atan2(accY, accZ) * self.rad_to_deg
            pitch = math.atan(-accX / math.sqrt((accY**2) + (accZ**2))) * self.rad_to_deg
        else:
            roll  = math.atan(accY / math.sqrt((accX**2) + (accZ**2))) * self.rad_to_deg
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
            "roll": self.kalAngleX,
            "pitch": self.kalAngleY
        }
