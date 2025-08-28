# sensors/base.py
from abc import ABC, abstractmethod

class Sensor(ABC):
    def __init__(self, name, channel=None, uses_i2c=False):
        self.name = name
        self.channel = channel
        self.uses_i2c = uses_i2c
        
    @abstractmethod
    def read(self):
        """Return sensor data as a dictionary"""
        pass
