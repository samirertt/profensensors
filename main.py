import time
import threading
import smbus2
from sensor.tca9548a import TCA9548A
from sensor.P_as5600 import AS5600
from sensor.P_mpu6050 import MPU6050
from sensor.P_ds18b20 import DS18B20
from manager import SensorManager

class AsyncSensorReader:
    def __init__(self, manager):
        self.manager = manager
        self.running = False
        self.print_lock = threading.Lock()  # To prevent mixed output
        
    def start(self):
        """Start reading all sensor types in separate threads"""
        self.running = True
        
        # Start each sensor type in its own thread with different frequencies
        threading.Thread(target=self._read_as5600_loop, daemon=True).start()
        threading.Thread(target=self._read_mpu6050_loop, daemon=True).start() 
        threading.Thread(target=self._read_ds18b20_loop, daemon=True).start()
        
    def stop(self):
        self.running = False
        
    def _print_sensor_data(self, sensor_type, data):
        """Thread-safe printing of sensor data"""
        with self.print_lock:
            timestamp = time.strftime('%H:%M:%S.%f')[:-3]  # Include milliseconds
            print(f"[{timestamp}] {sensor_type}:")
            
            for name, sensor_data in data.items():
                if sensor_data.get('ok'):
                    if 'deg' in sensor_data:  # AS5600
                        print(f"  {name}: {sensor_data.get('deg', 0):.1f}°")
                    elif 'roll' in sensor_data:  # MPU6050
                        print(f"  {name}: Roll={sensor_data.get('roll', 0)}° Pitch={sensor_data.get('pitch', 0)}°")
                    elif 'temperature_C' in sensor_data:  # DS18B20
                        temp = sensor_data.get('temperature_C', 0)
                        print(f"  {name}: {temp:.1f}°C")
                else:
                    print(f"  {name}: ERROR")
            print()  # Empty line for separation
    
    def _read_as5600_loop(self):
        """Read AS5600 sensors at high frequency"""
        while self.running:
            try:
                data = self.manager.update_as5600()
                if data:
                    self._print_sensor_data("AS5600", data)
                time.sleep(0.1)  # 10Hz - fast for encoders
            except Exception as e:
                with self.print_lock:
                    print(f"AS5600 error: {e}")
                time.sleep(0.2)
                
    def _read_mpu6050_loop(self):
        """Read MPU6050 sensors at medium frequency"""  
        while self.running:
            try:
                data = self.manager.update_mpu6050()
                if data:
                    self._print_sensor_data("MPU6050", data)
                time.sleep(0.05)  # 20Hz - fast for IMU
            except Exception as e:
                with self.print_lock:
                    print(f"MPU6050 error: {e}")
                time.sleep(0.1)
                
    def _read_ds18b20_loop(self):
        """Read DS18B20 sensors at low frequency"""
        while self.running:
            try:
                data = self.manager.update_ds18b20()
                if data:
                    self._print_sensor_data("DS18B20", data)
                time.sleep(2.0)  # 0.5Hz - slow for temperature
            except Exception as e:
                with self.print_lock:
                    print(f"DS18B20 error: {e}")
                time.sleep(3.0)

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
    print("Starting async sensor reading...")
    print("AS5600 (encoders): 10Hz, MPU6050 (IMU): 20Hz, DS18B20 (temp): 0.5Hz")
    print("Each sensor type prints as soon as ready!\n")
    
    # Start async reading
    reader = AsyncSensorReader(manager)
    reader.start()
    
    try:
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        reader.stop()
    finally:
        bus.close()

if __name__ == "__main__":
    main()