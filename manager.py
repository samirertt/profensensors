import threading
import time
from typing import Dict, List, Optional, Callable
from concurrent.futures import ThreadPoolExecutor, as_completed

class SensorManager:
    def __init__(self, mux=None, max_workers=6):
        self.mux = mux
        self.sensors = []
        self.lock = threading.Lock()
        self.sensor_data = {}
        self.max_workers = max_workers
        
        # Cached data for different sensor types
        self._cached_data = {
            'as5600': {},
            'mpu6050': {},
            'ds18b20': {},
            'all': {}
        }
        
        # Last update timestamps
        self._last_update = {
            'as5600': 0,
            'mpu6050': 0,
            'ds18b20': 0,
            'all': 0
        }
        
    def register(self, sensor):
        """Register a sensor with the manager"""
        self.sensors.append(sensor)
        print(f"Registered sensor: {sensor.name}")

    def _read_sensor_safe(self, sensor) -> tuple:
        """Thread-safe sensor reading with error handling"""
        try:
            data = sensor.read()
            return sensor.name, data
        except Exception as e:
            return sensor.name, {"ok": False, "error": str(e)}

    def _get_sensors_by_type(self, sensor_type: str) -> List:
        """Get sensors by type (as5600, mpu6050, ds18b20)"""
        return [sensor for sensor in self.sensors 
                if sensor.name.lower().startswith(sensor_type.lower())]

    def _update_sensors_parallel(self, sensors: List, timeout: float = 2.0) -> Dict:
        """Update multiple sensors in parallel with timeout"""
        if not sensors:
            return {}
            
        results = {}
        
        with ThreadPoolExecutor(max_workers=min(len(sensors), self.max_workers)) as executor:
            # Submit all sensor reading tasks
            future_to_sensor = {
                executor.submit(self._read_sensor_safe, sensor): sensor 
                for sensor in sensors
            }
            
            # Collect results with timeout
            for future in as_completed(future_to_sensor, timeout=timeout):
                try:
                    sensor_name, data = future.result(timeout=0.1)
                    results[sensor_name] = data
                except Exception as e:
                    sensor = future_to_sensor[future]
                    results[sensor.name] = {"ok": False, "error": f"Timeout or error: {str(e)}"}
                    
        return results

    def update_all(self, force_update: bool = False, cache_duration: float = 0.1) -> Dict:
        """Read all sensors in parallel with optional caching"""
        current_time = time.time()
        
        # Check cache if not forcing update
        if not force_update and (current_time - self._last_update['all']) < cache_duration:
            return self._cached_data['all'].copy()
        
        # Update all sensors
        results = self._update_sensors_parallel(self.sensors)
        
        # Update cache
        with self.lock:
            self._cached_data['all'] = results
            self._last_update['all'] = current_time
            
        return results.copy()

    def update_as5600(self, force_update: bool = False, cache_duration: float = 0.05) -> Dict:
        """Read only AS5600 sensors asynchronously"""
        current_time = time.time()
        
        # Check cache if not forcing update
        if not force_update and (current_time - self._last_update['as5600']) < cache_duration:
            return self._cached_data['as5600'].copy()
        
        as5600_sensors = self._get_sensors_by_type('AS5600')
        if not as5600_sensors:
            return {}
            
        results = self._update_sensors_parallel(as5600_sensors, timeout=1.0)
        
        # Update cache
        with self.lock:
            self._cached_data['as5600'] = results
            self._last_update['as5600'] = current_time
            
        return results.copy()

    def update_mpu6050(self, force_update: bool = False, cache_duration: float = 0.02) -> Dict:
        """Read only MPU6050 sensors asynchronously"""
        current_time = time.time()
        
        # Check cache if not forcing update
        if not force_update and (current_time - self._last_update['mpu6050']) < cache_duration:
            return self._cached_data['mpu6050'].copy()
            
        mpu6050_sensors = self._get_sensors_by_type('MPU6050')
        if not mpu6050_sensors:
            return {}
            
        results = self._update_sensors_parallel(mpu6050_sensors, timeout=1.5)
        
        # Update cache
        with self.lock:
            self._cached_data['mpu6050'] = results
            self._last_update['mpu6050'] = current_time
            
        return results.copy()

    def update_ds18b20(self, force_update: bool = False, cache_duration: float = 1.0) -> Dict:
        """Read only DS18B20 sensors asynchronously (temperature sensors are slower)"""
        current_time = time.time()
        
        # Check cache if not forcing update (longer cache for temp sensors)
        if not force_update and (current_time - self._last_update['ds18b20']) < cache_duration:
            return self._cached_data['ds18b20'].copy()
            
        ds18b20_sensors = self._get_sensors_by_type('DS18B20')
        if not ds18b20_sensors:
            return {}
            
        results = self._update_sensors_parallel(ds18b20_sensors, timeout=3.0)
        
        # Update cache
        with self.lock:
            self._cached_data['ds18b20'] = results
            self._last_update['ds18b20'] = current_time
            
        return results.copy()

    def get_sensor_by_name(self, name: str) -> Optional[object]:
        """Get a specific sensor by name"""
        for sensor in self.sensors:
            if sensor.name == name:
                return sensor
        return None

    def read_sensor_by_name(self, name: str) -> Dict:
        """Read a specific sensor by name"""
        sensor = self.get_sensor_by_name(name)
        if not sensor:
            return {"ok": False, "error": f"Sensor '{name}' not found"}
        
        try:
            return sensor.read()
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def get_all_sensor_names(self) -> List[str]:
        """Get names of all registered sensors"""
        return [sensor.name for sensor in self.sensors]

    def get_sensors_by_type(self, sensor_type: str) -> List[str]:
        """Get sensor names by type"""
        return [sensor.name for sensor in self._get_sensors_by_type(sensor_type)]

    def clear_cache(self, sensor_type: Optional[str] = None):
        """Clear cached data for specific sensor type or all"""
        with self.lock:
            if sensor_type and sensor_type in self._cached_data:
                self._cached_data[sensor_type] = {}
                self._last_update[sensor_type] = 0
            elif sensor_type is None:
                for key in self._cached_data:
                    self._cached_data[key] = {}
                    self._last_update[key] = 0

    def get_cache_info(self) -> Dict:
        """Get information about cached data and last update times"""
        current_time = time.time()
        return {
            sensor_type: {
                'last_update_ago': round(current_time - self._last_update[sensor_type], 3),
                'cached_sensors': list(self._cached_data[sensor_type].keys()),
                'count': len(self._cached_data[sensor_type])
            }
            for sensor_type in self._cached_data
        }

    def health_check(self) -> Dict:
        """Perform a health check on all sensors"""
        results = self.update_all(force_update=True)
        
        health = {
            'total_sensors': len(self.sensors),
            'healthy_sensors': 0,
            'failed_sensors': 0,
            'sensor_status': {}
        }
        
        for name, data in results.items():
            is_healthy = data.get('ok', True)
            health['sensor_status'][name] = {
                'healthy': is_healthy,
                'error': data.get('error') if not is_healthy else None
            }
            
            if is_healthy:
                health['healthy_sensors'] += 1
            else:
                health['failed_sensors'] += 1
                
        return health