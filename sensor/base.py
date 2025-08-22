# sensors/base.py
from abc import ABC, abstractmethod

class Sensor(ABC):
    def __init__(self, name, channel=None):
        self.name = name
        self.channel = channel

    @abstractmethod
    def read(self):
        """Return sensor data as a dictionary"""
        pass
