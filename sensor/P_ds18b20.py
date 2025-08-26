# sensors/ds18b20.py
from .base import Sensor
from adafruit_ds18x20 import DS18X20

class DS18B20(Sensor):
    def __init__(self, device, name="DS18B20", channel=None):
        super().__init__(name, channel)
        self.sensor = DS18X20(device)

    def read(self):
        try:
            temp_c = self.sensor.temperature
            temp_f = temp_c * 9.0 / 5.0 + 32.0
            return {"temperature_C": temp_c, "temperature_F": temp_f}
        except:
            return {"temperature_C": None, "temperature_F": None}