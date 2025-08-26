import threading
import time

class SensorManager:
    def __init__(self, mux=None):
        self.mux = mux
        self.sensors = []
        self.lock = threading.Lock()
        self.sensor_data = {}

    def register(self, sensor):
        self.sensors.append(sensor)
        print(f"Registered sensor: {sensor.name}")

    def _read_sensor(self, sensor):
        """Thread function to read a single sensor"""
        try:
            data = sensor.read()
            with self.lock:
                self.sensor_data[sensor.name] = data
        except Exception as e:
            with self.lock:
                self.sensor_data[sensor.name] = {"ok": False, "error": str(e)}

    def update_all(self):
        """Read all sensors in parallel"""
        self.sensor_data = {}
        threads = []
        
        # Start a thread for each sensor
        for sensor in self.sensors:
            thread = threading.Thread(target=self._read_sensor, args=(sensor,))
            thread.start()
            threads.append(thread)
        
        # Wait for all threads to complete
        for thread in threads:
            thread.join(timeout=2.0)  # Timeout after 2 seconds per sensor
        
        return self.sensor_data.copy()