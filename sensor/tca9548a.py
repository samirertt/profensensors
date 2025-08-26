# sensors/tca9548a.py
import smbus2

class TCA9548A:
    def __init__(self, bus, address=0x70):
        self.bus = bus
        self.address = address
        self.current_channel = None
    
    def select_channel(self, channel):
        """Select a channel (0-7) on the TCA9548A multiplexer"""
        if channel < 0 or channel > 7:
            raise ValueError("Channel must be between 0 and 7")
        
        if self.current_channel != channel:
            self.bus.write_byte(self.address, 1 << channel)
            self.current_channel = channel
