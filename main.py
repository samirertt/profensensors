import time
import smbus2
import threading
from sensor.tca9548a import TCA9548A
from sensor.P_as5600 import AS5600
from sensor.P_mpu6050 import MPU6050
from sensor.P_ds18b20 import DS18B20
from manager import SensorManager

class AsyncSensorReader:
    def __init__(self, manager):
        self.manager = manager
        self.running = False
        self.threads = {}
        
    def start_continuous_reading(self):
        """Start continuous reading of different sensor types at different rates"""
        self.running = True
        
        # Different update frequencies for different sensor types
        self.threads['as5600'] = threading.Thread(target=self._read_as5600_loop, daemon=True)
        self.threads['mpu6050'] = threading.Thread(target=self._read_mpu6050_loop, daemon=True)
        self.threads['ds18b20'] = threading.Thread(target=self._read_ds18b20_loop, daemon=True)
        
        for thread in self.threads.values():
            thread.start()
            
    def stop_continuous_reading(self):
        """Stop continuous reading"""
        self.running = False
        
    def _read_as5600_loop(self):
        """Read AS5600 sensors at high frequency (20Hz)"""
        while self.running:
            try:
                data = self.manager.update_as5600()
                if data:
                    self._process_as5600_data(data)
                time.sleep(0.05)  # 20Hz
            except Exception as e:
                print(f"AS5600 loop error: {e}")
                time.sleep(0.1)
                
    def _read_mpu6050_loop(self):
        """Read MPU6050 sensors at very high frequency (50Hz)"""
        while self.running:
            try:
                data = self.manager.update_mpu6050()
                if data:
                    self._process_mpu6050_data(data)
                time.sleep(0.02)  # 50Hz
            except Exception as e:
                print(f"MPU6050 loop error: {e}")
                time.sleep(0.05)
                
    def _read_ds18b20_loop(self):
        """Read DS18B20 sensors at low frequency (1Hz)"""
        while self.running:
            try:
                data = self.manager.update_ds18b20()
                if data:
                    self._process_ds18b20_data(data)
                time.sleep(1.0)  # 1Hz
            except Exception as e:
                print(f"DS18B20 loop error: {e}")
                time.sleep(2.0)
    
    def _process_as5600_data(self, data):
        """Process AS5600 data (angle sensors)"""
        for name, sensor_data in data.items():
            if sensor_data.get('ok'):
                angle_deg = sensor_data.get('deg', 0)
                # You can add your processing logic here
                # For example, store to database, send to network, etc.
                
    def _process_mpu6050_data(self, data):
        """Process MPU6050 data (IMU sensors)"""
        for name, sensor_data in data.items():
            if sensor_data.get('ok'):
                roll = sensor_data.get('roll', 0)
                pitch = sensor_data.get('pitch', 0)
                # You can add your processing logic here
                
    def _process_ds18b20_data(self, data):
        """Process DS18B20 data (temperature sensors)"""
        for name, sensor_data in data.items():
            if sensor_data.get('ok'):
                temp = sensor_data.get('temperature_C', 0)
                # You can add your processing logic here

def print_sensor_data_formatted(data_dict, title="Sensor Data"):
    """Pretty print sensor data"""
    if not data_dict:
        return
        
    print(f"\n{'='*20} {title} {'='*20}")
    for name, data in data_dict.items():
        if data.get("ok", True):
            if 'deg' in data:  # AS5600
                print(f"{name}: Angle={data.get('deg', 0):.1f}° Raw={data.get('angle', 0)} Mag={data.get('magnitude', 0)}")
            elif 'roll' in data:  # MPU6050
                print(f"{name}: Roll={data.get('roll', 0)}° Pitch={data.get('pitch', 0)}° ")
            elif 'temperature_C' in data:  # DS18B20
                temp = data.get('temperature_C', 0)
                print(f"{name}: Temperature={temp:.2f}°C" if temp else f"{name}: Temperature=N/A")
            else:
                print(f"{name}: {data}")
        else:
            print(f"{name}: ERROR - {data.get('error', 'Unknown error')}")

def demonstrate_async_reading(manager):
    """Demonstrate different ways to read sensors asynchronously"""
    print("\n" + "="*60)
    print("ASYNC SENSOR READING DEMONSTRATION")
    print("="*60)
    
    # Method 1: Read specific sensor types individually
    print("\n1. Reading AS5600 sensors only:")
    as5600_data = manager.update_as5600()
    print_sensor_data_formatted(as5600_data, "AS5600 Data")
    
    print("\n2. Reading MPU6050 sensors only:")
    mpu6050_data = manager.update_mpu6050()
    print_sensor_data_formatted(mpu6050_data, "MPU6050 Data")
    
    print("\n3. Reading DS18B20 sensors only:")
    ds18b20_data = manager.update_ds18b20()
    print_sensor_data_formatted(ds18b20_data, "DS18B20 Data")
    
    # Method 2: Read specific sensor by name
    print("\n4. Reading specific sensors by name:")
    if manager.get_sensor_by_name("AS5600_1"):
        sensor_data = manager.read_sensor_by_name("AS5600_1")
        print(f"AS5600_1: {sensor_data}")
    
    # Method 3: Get cache information
    print("\n5. Cache information:")
    cache_info = manager.get_cache_info()
    for sensor_type, info in cache_info.items():
        print(f"{sensor_type}: {info['count']} sensors, last update {info['last_update_ago']}s ago")

def main():
    print("Initializing optimized sensor system...")
    
    # I2C setup with smbus2
    try:
        bus = smbus2.SMBus(1)  # Use I2C bus 1
        mux = TCA9548A(bus)
        print("I2C bus and multiplexer initialized")
    except Exception as e:
        print(f"Error initializing I2C: {e}")
        return

    # Sensor Manager with optimized settings
    manager = SensorManager(mux, max_workers=4)

    # Initialize sensors with error handling
    sensors_to_init = [
        ("AS5600_1", lambda: AS5600(bus, mux=mux, channel=0, name="AS5600_1")),
        ("AS5600_2", lambda: AS5600(bus, mux=mux, channel=1, name="AS5600_2")),
        ("MPU6050_1", lambda: MPU6050(bus, mux=mux, channel=2, name="MPU6050_1")),
        ("MPU6050_2", lambda: MPU6050(bus, mux=mux, channel=3, name="MPU6050_2")),
        ("DS18B20_1", lambda: DS18B20(device_index=0, name="DS18B20_1")),
        ("DS18B20_2", lambda: DS18B20(device_index=1, name="DS18B20_2")),
    ]

    initialized_count = 0
    for sensor_name, sensor_factory in sensors_to_init:
        try:
            sensor = sensor_factory()
            manager.register(sensor)
            initialized_count += 1
        except Exception as e:
            print(f"Failed to initialize {sensor_name}: {e}")

    print(f"Initialized {initialized_count}/{len(sensors_to_init)} sensors")
    
    if initialized_count == 0:
        print("No sensors initialized. Exiting...")
        return
    
    # Perform health check
    print("\nPerforming health check...")
    health = manager.health_check()
    print(f"Healthy: {health['healthy_sensors']}/{health['total_sensors']} sensors")
    
    # Show available sensors
    print(f"\nAvailable sensor types:")
    print(f"AS5600 sensors: {manager.get_sensors_by_type('AS5600')}")
    print(f"MPU6050 sensors: {manager.get_sensors_by_type('MPU6050')}")
    print(f"DS18B20 sensors: {manager.get_sensors_by_type('DS18B20')}")
    
    # Demonstrate async reading
    demonstrate_async_reading(manager)
    
    print("\nStarting continuous async data collection...")
    print("Choose mode:")
    print("1. High-frequency individual sensor type reading")
    print("2. Mixed reading (some async, some together)")
    print("3. Traditional all-sensors reading")
    
    try:
        # Mode 1: High-frequency async reading
        async_reader = AsyncSensorReader(manager)
        async_reader.start_continuous_reading()
        
        print("\nRunning async sensor reading (Ctrl+C to change mode)...")
        print("AS5600: 20Hz, MPU6050: 50Hz, DS18B20: 1Hz")
        
        # Display loop for monitoring
        last_display = 0
        while True:
            current_time = time.time()
            if current_time - last_display >= 2.0:  # Display every 2 seconds
                print(f"\n--- Time: {time.strftime('%H:%M:%S')} ---")
                
                # Get latest data from cache
                as5600_data = manager.update_as5600()
                mpu6050_data = manager.update_mpu6050() 
                ds18b20_data = manager.update_ds18b20()
                
                if as5600_data:
                    print_sensor_data_formatted(as5600_data, "AS5600 (20Hz)")
                if mpu6050_data:
                    print_sensor_data_formatted(mpu6050_data, "MPU6050 (50Hz)")
                if ds18b20_data:
                    print_sensor_data_formatted(ds18b20_data, "DS18B20 (1Hz)")
                    
                # Show cache info
                cache_info = manager.get_cache_info()
                print(f"Cache age - AS5600: {cache_info['as5600']['last_update_ago']:.3f}s, "
                      f"MPU6050: {cache_info['mpu6050']['last_update_ago']:.3f}s, "
                      f"DS18B20: {cache_info['ds18b20']['last_update_ago']:.3f}s")
                
                last_display = current_time
            
            time.sleep(0.1)  # Small delay to prevent excessive CPU usage
            
    except KeyboardInterrupt:
        print("\nSwitching to mixed mode...")
        async_reader.stop_continuous_reading()
        
        # Mode 2: Mixed reading demonstration
        try:
            while True:
                print(f"\n--- Mixed Reading Mode - Time: {time.strftime('%H:%M:%S')} ---")
                
                # Read fast sensors more frequently
                as5600_data = manager.update_as5600(force_update=True)
                mpu6050_data = manager.update_mpu6050(force_update=True)
                
                # Read temp sensors less frequently (use cache)
                ds18b20_data = manager.update_ds18b20()
                
                print_sensor_data_formatted(as5600_data, "AS5600 (Fresh)")
                print_sensor_data_formatted(mpu6050_data, "MPU6050 (Fresh)") 
                print_sensor_data_formatted(ds18b20_data, "DS18B20 (Cached)")
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\nSwitching to traditional mode...")
            
            # Mode 3: Traditional all-sensors reading
            try:
                while True:
                    all_data = manager.update_all(force_update=True)
                    print(f"\n--- All Sensors Mode - Time: {time.strftime('%H:%M:%S')} ---")
                    print_sensor_data_formatted(all_data, "All Sensors")
                    time.sleep(1)
                    
            except KeyboardInterrupt:
                print("\nShutting down...")
            
    finally:
        if 'async_reader' in locals():
            async_reader.stop_continuous_reading()
        bus.close()
        print("System shutdown complete.")

if __name__ == "__main__":
    main()