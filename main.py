# main.py
import time
import board, busio
from adafruit_tca9548a import TCA9548A

# Sensor classes
from sensor.P_as5600 import AS5600
from sensor.P_mpu6050 import MPU6050
from sensor.P_ds18b20 import DS18B20
from manager import SensorManager
import adafruit_onewire.bus
import glob

# --- I2C setup ---
i2c = busio.I2C(board.SCL, board.SDA)
mux = TCA9548A(i2c)

# --- OneWire setup for DS18B20 ---
ow_bus = adafruit_onewire.bus.OneWireBus(board.D4)  # connect DS18B20
devices = ow_bus.scan()  # list of DS18B20 addresses

# --- Sensor Manager ---
manager = SensorManager(mux)

# --- Instantiate sensors ---
# AS5600s
as1 = AS5600(mux[0], channel=0, name="AS5600_1")
as2 = AS5600(mux[1], channel=1, name="AS5600_2")
manager.register(as1)
manager.register(as2)

# Two MPU6050 sensors on different mux channels
mpu1 = MPU6050(mux[0], name="MPU6050_1")
mpu2 = MPU6050(mux[1], name="MPU6050_2")

manager.register(mpu1)
manager.register(mpu2)

# DS18B20 sensors (supports multiple by index)
ds1 = DS18B20(name="DS18B20_1", device_index=0)
ds2 = DS18B20(name="DS18B20_2", device_index=1)
manager.register(ds1)
manager.register(ds2)

while True:
    all_data = manager.update_all()
    for name, data in all_data.items():
        print(f"{name}: {data}")
    time.sleep(1)