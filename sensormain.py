# Improved sensormain.py
import time
import threading
import smbus2
import json
import logging
from profensensors.sensor.tca9548a import TCA9548A
from profensensors.sensor.P_as5600 import AS5600
from profensensors.sensor.P_mpu6050 import MPU6050
from profensensors.sensor.P_ds18b20 import DS18B20
from profensensors.manager import SensorManager

# Configuration constants
SENSOR_CONFIG = {
    'AS5600': {'frequency': 50, 'timeout': 1.0},  # 50Hz
    'MPU6050': {'frequency': 20, 'timeout': 1.5}, # 20Hz  
    'DS18B20': {'frequency': 0.5, 'timeout': 3.0} # 0.5Hz
}

class SensorTCPSystem:
    def __init__(self, tcp_server):
        self.server = tcp_server
        
        # Sensor state variables with proper naming
        self.body_roll = 0.0
        self.body_pitch = 0.0
        self.x_encoder = 0.0
        self.y_encoder = 0.0
        
        # System components
        self.manager = None
        self.bus = None
        self.mux = None
        self.running = False
        self.sensor_readers = []
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'errors_count': 0,
            'last_error': None,
            'start_time': None
        }
        
        # Setup logging
        self.logger = logging.getLogger(__name__)
        
    def initialize_sensors(self):
        """Initialize all sensors with better error handling"""
        self.logger.info("Initializing sensor system...")
        
        # I2C setup with retry logic
        max_retries = 3
        for attempt in range(max_retries):
            try:
                self.bus = smbus2.SMBus(1)
                self.mux = TCA9548A(self.bus)
                self.logger.info("I2C bus initialized successfully")
                break
            except Exception as e:
                self.logger.error(f"I2C initialization attempt {attempt + 1} failed: {e}")
                if attempt == max_retries - 1:
                    return False
                time.sleep(0.5)

        # Initialize sensor manager
        self.manager = SensorManager(self.mux)

        # Sensor initialization configuration
        sensors_config = [
            ("AS5600_1", lambda: AS5600(self.bus, mux=self.mux, channel=6, name="AS5600_1")),
            ("AS5600_2", lambda: AS5600(self.bus, mux=self.mux, channel=7, name="AS5600_2")),
            ("MPU6050_1", lambda: MPU6050(self.bus, mux=self.mux, channel=2, name="MPU6050_1")),
            ("MPU6050_2", lambda: MPU6050(self.bus, mux=self.mux, channel=3, name="MPU6050_2")),
            ("DS18B20_1", lambda: DS18B20(device_index=0, name="DS18B20_Motor1")),
            ("DS18B20_2", lambda: DS18B20(device_index=1, name="DS18B20_Motor2")),
        ]

        initialized_count = 0
        for sensor_name, sensor_factory in sensors_config:
            try:
                sensor = sensor_factory()
                self.manager.register(sensor)
                initialized_count += 1
                self.logger.info(f"✓ {sensor_name} initialized")
            except Exception as e:
                self.logger.error(f"✗ Failed to initialize {sensor_name}: {e}")

        self.logger.info(f"Sensor initialization complete: {initialized_count}/{len(sensors_config)} sensors")
        
        # Log sensor statistics
        stats = self.manager.get_sensor_stats()
        self.logger.info(f"Sensor stats: {stats}")
        
        return initialized_count > 0

    def start_sensor_readers(self):
        """Start optimized sensor reader threads"""
        self.running = True
        self.stats['start_time'] = time.time()
        
        self.sensor_readers = [
            threading.Thread(target=self._read_and_send_as5600, daemon=True, name="AS5600_Reader"),
            threading.Thread(target=self._read_and_send_mpu6050, daemon=True, name="MPU6050_Reader"),
            threading.Thread(target=self._read_and_send_ds18b20, daemon=True, name="DS18B20_Reader")
        ]
        
        for reader in self.sensor_readers:
            reader.start()
            
        self.logger.info("All sensor readers started")
    
    def stop_sensor_readers(self):
        """Stop all sensor reader threads gracefully"""
        self.running = False
        
        for reader in self.sensor_readers:
            if reader.is_alive():
                reader.join(timeout=2.0)
                if reader.is_alive():
                    self.logger.warning(f"Thread {reader.name} did not stop gracefully")
                    
        self.logger.info("All sensor readers stopped")
    
    def _send_sensor_data(self, sensor_type, data):
        """Enhanced data sending with error handling"""
        if not (hasattr(self.server, 'client_socket') and self.server.client_socket):
            return False
            
        try:
            message = {
                "type": "sensor_data",
                "sensor_type": sensor_type,
                "data": data,
                "timestamp": time.time(),
                "sequence": self.stats['messages_sent']
            }
            
            json_message = json.dumps(message) + '\n'
            self.server.client_socket.sendall(json_message.encode('utf-8'))
            self.stats['messages_sent'] += 1
            return True
            
        except Exception as e:
            self.stats['errors_count'] += 1
            self.stats['last_error'] = str(e)
            self.logger.error(f"Failed to send {sensor_type} data: {e}")
            return False
    
    def _read_and_send_as5600(self):
        """Optimized AS5600 reading with proper frequency control"""
        sleep_time = 1.0 / SENSOR_CONFIG['AS5600']['frequency']
        
        while self.running:
            try:
                start_time = time.time()
                data = self.manager.update_as5600()
                
                if data:
                    # Filter successful readings only
                    filtered_data = {k: v for k, v in data.items() if v.get("ok", False)}
                    
                    if filtered_data:
                        self._send_sensor_data("AS5600", filtered_data)
                        
                        # Update encoder values with proper mapping
                        if "AS5600_1" in filtered_data:
                            self.x_encoder = filtered_data["AS5600_1"].get("deg", self.x_encoder)
                        if "AS5600_2" in filtered_data:
                            self.y_encoder = filtered_data["AS5600_2"].get("deg", self.y_encoder)
                
                # Maintain consistent frequency
                elapsed = time.time() - start_time
                sleep_duration = max(0, sleep_time - elapsed)
                time.sleep(sleep_duration)
                
            except Exception as e:
                self.logger.error(f"AS5600 reader error: {e}")
                time.sleep(0.1)
    
    def _read_and_send_mpu6050(self):
        """Optimized MPU6050 reading"""
        sleep_time = 1.0 / SENSOR_CONFIG['MPU6050']['frequency']
        
        while self.running:
            try:
                start_time = time.time()
                data = self.manager.update_mpu6050()
                
                if data:
                    self._send_sensor_data("MPU6050", data)
                    
                    # Update body orientation from primary MPU6050
                    if "MPU6050_Body" in data and data["MPU6050_Body"].get("ok", False):
                        body_data = data["MPU6050_Body"]
                        self.body_roll = body_data.get("roll", self.body_roll)
                        self.body_pitch = body_data.get("pitch", self.body_pitch)
                
                # Maintain frequency
                elapsed = time.time() - start_time
                sleep_duration = max(0, sleep_time - elapsed)
                time.sleep(sleep_duration)
                
            except Exception as e:
                self.logger.error(f"MPU6050 reader error: {e}")
                time.sleep(0.1)
    
    def _read_and_send_ds18b20(self):
        """DS18B20 reader with longer intervals"""
        sleep_time = 1.0 / SENSOR_CONFIG['DS18B20']['frequency']
        
        while self.running:
            try:
                data = self.manager.update_ds18b20()
                if data:
                    self._send_sensor_data("DS18B20", data)
                    
                time.sleep(sleep_time)
                
            except Exception as e:
                self.logger.error(f"DS18B20 reader error: {e}")
                time.sleep(sleep_time)
    
    def get_system_status(self):
        """Get comprehensive system status"""
        uptime = time.time() - self.stats['start_time'] if self.stats['start_time'] else 0
        
        return {
            "running": self.running,
            "uptime_seconds": round(uptime, 1),
            "sensors_registered": len(self.manager.sensors) if self.manager else 0,
            "messages_sent": self.stats['messages_sent'],
            "errors_count": self.stats['errors_count'],
            "last_error": self.stats['last_error'],
            "current_values": {
                "x_encoder": round(self.x_encoder, 2),
                "y_encoder": round(self.y_encoder, 2), 
                "body_roll": round(self.body_roll, 2),
                "body_pitch": round(self.body_pitch, 2)
            },
            "client_connected": hasattr(self.server, 'client_socket') and bool(self.server.client_socket)
        }
    
    def start(self):
        """Start the sensor system with enhanced monitoring"""
        # Initialize sensors
        if not self.initialize_sensors():
            self.logger.error("Failed to initialize sensors - aborting")
            return
        
        self.logger.info("Sensor system initialized. Waiting for client connection...")
        
        # Connection monitoring loop
        try:
            while self.server.running:
                # Wait for client connection
                if not (hasattr(self.server, 'client_socket') and self.server.client_socket):
                    if self.running:  # Stop readers if client disconnected
                        self.logger.info("Client disconnected. Stopping sensor readers.")
                        self.stop_sensor_readers()
                    
                    # Wait for new connection
                    while not (hasattr(self.server, 'client_socket') and self.server.client_socket):
                        if not self.server.running:
                            break
                        time.sleep(0.5)
                    
                    if hasattr(self.server, 'client_socket') and self.server.client_socket:
                        self.logger.info(f"New client connected: {self.server.client_address}")
                        self.start_sensor_readers()
                
                time.sleep(1)
                
                # Log periodic status
                if self.stats['messages_sent'] > 0 and self.stats['messages_sent'] % 1000 == 0:
                    status = self.get_system_status()
                    self.logger.info(f"System status: {status}")
                        
        except KeyboardInterrupt:
            self.logger.info("Shutdown requested by user")
        except Exception as e:
            self.logger.error(f"Unexpected error in main loop: {e}")
        finally:
            self.logger.info("Shutting down sensor system...")
            self.stop_sensor_readers()
            if hasattr(self, 'bus') and self.bus:
                try:
                    self.bus.close()
                    self.logger.info("I2C bus closed")
                except:
                    pass