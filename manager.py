import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

class SensorManager:
    def __init__(self, mux=None):
        self.mux = mux
        self.sensors = []
        self.lock = threading.Lock()
        
    def register(self, sensor):
        """Register a sensor"""
        self.sensors.append(sensor)
        print(f"Registered: {sensor.name}")

    def _read_sensor_safe(self, sensor):
        """Safe sensor reading with error handling"""
        try:
            data = sensor.read()
            return sensor.name, data
        except Exception as e:
            return sensor.name, {"ok": False, "error": str(e)}

    def _get_sensors_by_type(self, sensor_type):
        """Get sensors by type"""
        return [s for s in self.sensors if s.name.lower().startswith(sensor_type.lower())]

    def _read_sensors_parallel(self, sensors, timeout=2.0):
        """Read multiple sensors in parallel"""
        if not sensors:
            return {}
            
        results = {}
        
        with ThreadPoolExecutor(max_workers=len(sensors)) as executor:
            future_to_sensor = {
                executor.submit(self._read_sensor_safe, sensor): sensor 
                for sensor in sensors
            }
            
            try:
                for future in as_completed(future_to_sensor, timeout=timeout):
                    sensor_name, data = future.result()
                    results[sensor_name] = data
            except TimeoutError:
                # Get completed results and mark others as failed
                for future, sensor in future_to_sensor.items():
                    if future.done():
                        try:
                            sensor_name, data = future.result(timeout=0.01)
                            results[sensor_name] = data
                        except:
                            results[sensor.name] = {"ok": False, "error": "Read error"}
                    else:
                        results[sensor.name] = {"ok": False, "error": "Timeout"}
                        
        return results

    def update_as5600(self):
        """Read AS5600 sensors"""
        sensors = self._get_sensors_by_type('AS5600')
        return self._read_sensors_parallel(sensors, timeout=1.0)

    def update_mpu6050(self):
        """Read MPU6050 sensors"""
        sensors = self._get_sensors_by_type('MPU6050')
        return self._read_sensors_parallel(sensors, timeout=1.5)

    def update_ds18b20(self):
        """Read DS18B20 sensors"""
        sensors = self._get_sensors_by_type('DS18B20')
        return self._read_sensors_parallel(sensors, timeout=3.0)

    def update_all(self):
        """Read all sensors in parallel"""
        return self._read_sensors_parallel(self.sensors, timeout=3.0)