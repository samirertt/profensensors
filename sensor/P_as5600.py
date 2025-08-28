from .base import Sensor
import time

class AS5600(Sensor):
    def __init__(self, bus, mux=None, channel=None, address=0x36, name="AS5600"):
        super().__init__(name, channel, uses_i2c=True)
        self.bus = bus
        self.mux = mux
        self.address = address

    def _select_channel(self):
        if self.mux and self.channel is not None:
            self.mux.select_channel(self.channel)
            time.sleep(0.001)  # 1ms settling time

    def read_raw_angle(self, select_channel=True):
        if select_channel:
            self._select_channel()
        read_bytes = self.bus.read_i2c_block_data(self.address, 0x0C, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def read_magnitude(self, select_channel=True):
        if select_channel:
            self._select_channel()
        read_bytes = self.bus.read_i2c_block_data(self.address, 0x1B, 2)
        return (read_bytes[0] << 8) | read_bytes[1]

    def read(self, retries=2):
        """Standard read method with channel selection"""
        return self._read_internal(retries, select_channel=True)
    
    def read_without_channel_select(self, retries=2):
        """Read without changing channel (assumes channel already set)"""
        return self._read_internal(retries, select_channel=False)
    
    def _read_internal(self, retries, select_channel=True):
        """Internal read method with channel selection control"""
        for attempt in range(retries):
            try:
                # Select channel only if requested
                if select_channel:
                    self._select_channel()
                
                # Read angle data
                angle_bytes = self.bus.read_i2c_block_data(self.address, 0x0C, 2)
                angle = (angle_bytes[0] << 8) | angle_bytes[1]
                
                # For now, hardcode magnitude to 1 (as in original)
                magnitude = 1
                
                return {
                    "ok": True,
                    "angle": angle,
                    "magnitude": magnitude,
                    "deg": angle * 360 / 4096,
                    "channel_selected": select_channel
                }
            except Exception as e:
                if attempt < retries - 1:
                    time.sleep(0.001)
                else:
                    return {"ok": False, "error": str(e)}
        
        return {"ok": False, "error": "Max retries exceeded"}
        
    def reset(self):
        """Zero the sensor at current position"""
        angle = self.read_raw_angle()
        self.offset_deg = angle * 360 / 4096