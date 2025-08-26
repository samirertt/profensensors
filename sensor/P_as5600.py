from .base import Sensor

class AS5600(Sensor):
    def __init__(self, bus, mux=None, channel=None, address=0x36, name="AS5600"):
        super().__init__(name, channel)
        self.bus = bus
        self.mux = mux
        self.address = address

    def _select_channel(self):
        if self.mux and self.channel is not None:
            self.mux.select_channel(self.channel)

    def read_raw_angle(self):
        self._select_channel()
        read_bytes = self.bus.read_i2c_block_data(self.address, 0x0C, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def read_magnitude(self):
        self._select_channel()
        read_bytes = self.bus.read_i2c_block_data(self.address, 0x1B, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def read(self):
        try:
            angle = self.read_raw_angle()
            magnitude = self.read_magnitude()
            return {
                "ok": True,
                "angle": angle,
                "magnitude": magnitude,
                "deg": angle * 360 / 4096
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}