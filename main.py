import time
import smbus2
from sensor.tca9548a import TCA9548A
from sensor.P_as5600 import AS5600
from sensor.P_mpu6050 import MPU6050
from sensor.P_ds18b20 import DS18B20
from manager import SensorManager

def main():
    print("Initializing sensor system...")
    
    # I2C setup
    try:
        bus = smbus2.SMBus(1)
        mux = TCA9548A(bus)
        print("I2C initialized")
    except Exception as e:
        print(f"I2C error: {e}")
        return

    # Sensor Manager
    manager = SensorManager(mux)

    # Initialize sensors
    sensors_to_init = [
        ("AS5600_1", lambda: AS5600(bus, mux=mux, channel=0, name="AS5600_1")),
        ("AS5600_2", lambda: AS5600(bus, mux=mux, channel=1, name="AS5600_2")),
        ("MPU6050_1", lambda: MPU6050(bus, mux=mux, channel=2, name="MPU6050_1")),
        ("MPU6050_2", lambda: MPU6050(bus, mux=mux, channel=3, name="MPU6050_2")),
        ("DS18B20_1", lambda: DS18B20(device_index=0, name="DS18B20_1")),
        ("DS18B20_2", lambda: DS18B20(device_index=1, name="DS18B20_2")),
    ]

    for sensor_name, sensor_factory in sensors_to_init:
        try:
            sensor = sensor_factory()
            manager.register(sensor)
        except Exception as e:
            print(f"Failed to init {sensor_name}: {e}")

    print(f"Initialized {len(manager.sensors)} sensors")
    
    # Main reading loop
    try:
        while True:
            # Read different sensor types asynchronously
            as5600_data = manager.update_as5600()
            mpu6050_data = manager.update_mpu6050() 
            ds18b20_data = manager.update_ds18b20()
            
            # Print results
            print(f"\n--- {time.strftime('%H:%M:%S')} ---")
            
            for name, data in as5600_data.items():
                if data.get('ok'):
                    print(f"{name}: {data.get('deg', 0):.1f}°")
                else:
                    print(f"{name}: ERROR")
            
            for name, data in mpu6050_data.items():
                if data.get('ok'):
                    print(f"{name}: Roll={data.get('roll', 0)}° Pitch={data.get('pitch', 0)}°")
                else:
                    print(f"{name}: ERROR")
            
            for name, data in ds18b20_data.items():
                if data.get('ok'):
                    print(f"{name}: {data.get('temperature_C', 0):.1f}°C")
                else:
                    print(f"{name}: ERROR")
            
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        bus.close()

if __name__ == "__main__":
    main()