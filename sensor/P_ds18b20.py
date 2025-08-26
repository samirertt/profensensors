import glob
from .base import Sensor

class DS18B20(Sensor):
    def __init__(self, device_index=0, name="DS18B20"):
        super().__init__(name)
        base_dir = '/sys/bus/w1/devices/'
        device_folders = glob.glob(base_dir + '28-*')
        if device_index >= len(device_folders):
            raise IndexError(f"No DS18B20 found at index {device_index}")
        self.device_file = device_folders[device_index] + '/w1_slave'

    def read_temp(self):
        with open(self.device_file, 'r') as f:
            lines = f.readlines()

        if lines[0].strip()[-3:] != 'YES':
            return None

        equals_pos = lines[1].find('t=')
        if equals_pos != -1:
            temp_string = lines[1][equals_pos+2:]
            temp_c = float(temp_string) / 1000.0
            return temp_c
        return None

    def read(self):
        temp = self.read_temp()
        return {"temperature_C": temp}
