# Optimized manager.py
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from collections import defaultdict
import logging

class SensorManager:
    def __init__(self, mux=None):
        self.mux = mux
        self.sensors = []
        self.sensors_by_type = defaultdict(list)
        self.sensors_by_channel = defaultdict(list)
        self.lock = threading.Lock()
        self.logger = logging.getLogger(__name__)
        
    def register(self, sensor):
        """Register a sensor with improved categorization"""
        with self.lock:
            self.sensors.append(sensor)
            
            # Categorize by type
            sensor_type = sensor.__class__.__name__.lower()
            self.sensors_by_type[sensor_type].append(sensor)
            
            # Categorize by I2C channel for optimization
            if hasattr(sensor, 'channel') and sensor.uses_i2c:
                self.sensors_by_channel[sensor.channel].append(sensor)
            
            self.logger.info(f"Registered: {sensor.name} (type: {sensor_type})")

    def _read_sensor_safe(self, sensor):
        """Safe sensor reading with improved error handling"""
        try:
            start_time = time.time()
            data = sensor.read()
            read_time = time.time() - start_time
            
            # Add timing info for debugging
            if hasattr(data, 'update') and isinstance(data, dict):
                data['read_time_ms'] = round(read_time * 1000, 2)
            
            return sensor.name, data
        except Exception as e:
            self.logger.error(f"Error reading {sensor.name}: {e}")
            return sensor.name, {
                "ok": False, 
                "error": str(e),
                "error_type": type(e).__name__
            }

    def _read_i2c_sensors_optimized(self, sensors, timeout=2.0):
        """Optimized I2C sensor reading with proper channel management"""
        results = {}
        
        # Group sensors by channel to minimize mux switching
        for channel, channel_sensors in self.sensors_by_channel.items():
            # Filter to only requested sensors
            requested_sensors = [s for s in channel_sensors if s in sensors]
            if not requested_sensors:
                continue
                
            try:
                # Select channel once for all sensors on this channel
                if (channel is not None and 
                    hasattr(requested_sensors[0], 'mux') and 
                    requested_sensors[0].mux):
                    
                    # Use the sensor's _select_channel method for thread safety
                    requested_sensors[0]._select_channel()
                
                # Read all sensors on this channel sequentially
                for sensor in requested_sensors:
                    sensor_name, data = self._read_sensor_safe(sensor)
                    results[sensor_name] = data
                    
            except Exception as e:
                self.logger.error(f"Error with I2C channel {channel}: {e}")
                # Mark all sensors on this channel as failed
                for sensor in requested_sensors:
                    results[sensor.name] = {
                        "ok": False, 
                        "error": f"Channel {channel} error: {str(e)}"
                    }
        
        return results

    def _read_non_i2c_sensors_parallel(self, sensors, timeout=2.0):
        """Read non-I2C sensors in parallel"""
        results = {}
        
        if not sensors:
            return results
            
        with ThreadPoolExecutor(max_workers=min(len(sensors), 4)) as executor:
            future_to_sensor = {
                executor.submit(self._read_sensor_safe, sensor): sensor 
                for sensor in sensors
            }
            
            try:
                for future in as_completed(future_to_sensor, timeout=timeout):
                    sensor_name, data = future.result()
                    results[sensor_name] = data
                    
            except TimeoutError:
                self.logger.warning("Timeout reading non-I2C sensors")
                # Handle partial results
                for future, sensor in future_to_sensor.items():
                    if future.done():
                        try:
                            sensor_name, data = future.result(timeout=0.01)
                            results[sensor_name] = data
                        except Exception as e:
                            results[sensor.name] = {
                                "ok": False, 
                                "error": f"Future result error: {str(e)}"
                            }
                    else:
                        future.cancel()
                        results[sensor.name] = {"ok": False, "error": "Timeout"}
                        
        return results

    def read_sensors_by_type(self, sensor_type, timeout=2.0):
        """Read sensors by type with optimization"""
        sensors = self.sensors_by_type.get(sensor_type.lower(), [])
        if not sensors:
            return {}
            
        # Separate I2C and non-I2C sensors
        i2c_sensors = [s for s in sensors if s.uses_i2c]
        non_i2c_sensors = [s for s in sensors if not s.uses_i2c]
        
        results = {}
        
        # Read I2C sensors with channel optimization
        if i2c_sensors:
            results.update(self._read_i2c_sensors_optimized(i2c_sensors, timeout))
        
        # Read non-I2C sensors in parallel
        if non_i2c_sensors:
            results.update(self._read_non_i2c_sensors_parallel(non_i2c_sensors, timeout))
            
        return results

    def update_as5600(self):
        """Read AS5600 sensors with shorter timeout"""
        return self.read_sensors_by_type('as5600', timeout=1.0)

    def update_mpu6050(self):
        """Read MPU6050 sensors"""
        return self.read_sensors_by_type('mpu6050', timeout=1.5)

    def update_ds18b20(self):
        """Read DS18B20 sensors with longer timeout"""
        return self.read_sensors_by_type('ds18b20', timeout=3.0)

    def update_all(self):
        """Read all sensors with type-specific optimizations"""
        all_results = {}
        
        # Read each sensor type with appropriate timeouts
        for sensor_type, timeout in [('as5600', 1.0), ('mpu6050', 1.5), ('ds18b20', 3.0)]:
            results = self.read_sensors_by_type(sensor_type, timeout)
            all_results.update(results)
            
        return all_results

    def get_sensor_stats(self):
        """Get statistics about registered sensors"""
        return {
            "total_sensors": len(self.sensors),
            "by_type": {k: len(v) for k, v in self.sensors_by_type.items()},
            "i2c_channels": {k: len(v) for k, v in self.sensors_by_channel.items()},
            "i2c_sensors": sum(1 for s in self.sensors if s.uses_i2c),
            "non_i2c_sensors": sum(1 for s in self.sensors if not s.uses_i2c)
        }