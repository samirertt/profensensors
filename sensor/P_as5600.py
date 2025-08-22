# sensors/as5600.py
from .base import Sensor

class AS5600(Sensor):
    def __init__(self, bus, address=0x36, name="AS5600", channel=None):
        super().__init__(name, channel)
        self.bus = bus  # pass tca[0] or any I2C bus object
        self.address = address

    def read_raw_angle(self):
        read_bytes = self.bus.read_i2c_block_data(self.address, 0x0C, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def read_magnitude(self):
        read_bytes = self.bus.read_i2c_block_data(self.address, 0x1B, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def read(self):
        angle = self.read_raw_angle()
        magnitude = self.read_magnitude()
        return {
            "angle": angle,
            "magnitude": magnitude,
            "deg": angle * 360 / 4096
        }
