# sensors/ds18b20_raw.py
from .base import Sensor
import os
import glob
import time

class DS18B20(Sensor):
    def __init__(self, name="DS18B20", device_index=0, channel=None):
        super().__init__(name, channel)
        os.system('modprobe w1-gpio')
        os.system('modprobe w1-therm')
        base_dir = '/sys/bus/w1/devices/'
        device_folders = glob.glob(base_dir + '28*')
        if len(device_folders) == 0:
            raise RuntimeError("No DS18B20 sensors found!")
        self.device_file = device_folders[device_index] + '/w1_slave'

    def read_temp_raw(self):
        with open(self.device_file, 'r') as f:
            lines = f.readlines()
        return lines

    def read(self):
        lines = self.read_temp_raw()
        # Wait until the sensor output is valid
        while lines[0].strip()[-3:] != 'YES':
            time.sleep(0.2)
            lines = self.read_temp_raw()
        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_c = float(lines[1][equals_pos+2:]) / 1000.0
            temp_f = temp_c * 9.0 / 5.0 + 32.0
            return {"temperature_C": temp_c, "temperature_F": temp_f}
        else:
            return {"temperature_C": None, "temperature_F": None}
