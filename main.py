import time
import board, busio
from adafruit_tca9548a import TCA9548A

# Sensor classes
from sensor.P_as5600 import AS5600
from sensor.P_mpu6050 import MPU6050
from sensor.P_ds18b20 import DS18B20
from manager import SensorManager

# --- I2C setup ---
i2c = busio.I2C(board.SCL, board.SDA)
mux = TCA9548A(i2c)

# --- Sensor Manager ---
manager = SensorManager(mux)

# --- Instantiate sensors ---
# AS5600s (channels 0,1)
as1 = AS5600(mux[0], name="AS5600_1")
as2 = AS5600(mux[1], name="AS5600_2")
manager.register(as1)
manager.register(as2)

# MPU6050s (channels 2,3)
mpu1 = MPU6050(mux[2], name="MPU6050_1")
mpu2 = MPU6050(mux[3], name="MPU6050_2")
manager.register(mpu1)
manager.register(mpu2)

# DS18B20s (from /sys/bus/w1/devices/)
ds1 = DS18B20(name="DS18B20_1", device_index=0)
ds2 = DS18B20(name="DS18B20_2", device_index=1)
manager.register(ds1)
manager.register(ds2)

# --- Main loop ---
while True:
    all_data = manager.update_all()
    for name, data in all_data.items():
        print(f"{name}: {data}")
    time.sleep(1)
