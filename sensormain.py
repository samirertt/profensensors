# sensorMain.py
import time
import threading
import smbus2
import json
from profensensors.sensor.tca9548a import TCA9548A
from profensensors.sensor.P_as5600 import AS5600
from profensensors.sensor.P_mpu6050 import MPU6050
from profensensors.sensor.P_ds18b20 import DS18B20
from profensensors.manager import SensorManager

class SensorTCPSystem:
    def __init__(self, tcp_server):
        # Reference to the existing TCP server
        self.server = tcp_server
        
        # Initialize sensor system
        self.manager = None
        self.bus = None
        self.mux = None
        self.running = False
        self.sensor_readers = []
        
    def initialize_sensors(self):
        """Initialize all sensors"""
        print("Initializing sensor system...")
        
        try:
            # I2C setup
            self.bus = smbus2.SMBus(1)
            self.mux = TCA9548A(self.bus)
            print("I2C initialized")
        except Exception as e:
            print(f"I2C error: {e}")
            return False

        # Sensor Manager
        self.manager = SensorManager(self.mux)

        # Initialize sensors
        sensors_to_init = [
            ("AS5600_1", lambda: AS5600(self.bus, mux=self.mux, channel=0, name="AS5600_1")),
            ("AS5600_2", lambda: AS5600(self.bus, mux=self.mux, channel=1, name="AS5600_2")),
            ("MPU6050_1", lambda: MPU6050(self.bus, mux=self.mux, channel=2, name="MPU6050_1")),
            ("MPU6050_2", lambda: MPU6050(self.bus, mux=self.mux, channel=3, name="MPU6050_2")),
            ("DS18B20_1", lambda: DS18B20(device_index=0, name="DS18B20_1")),
            ("DS18B20_2", lambda: DS18B20(device_index=1, name="DS18B20_2")),
        ]

        for sensor_name, sensor_factory in sensors_to_init:
            try:
                sensor = sensor_factory()
                self.manager.register(sensor)
            except Exception as e:
                print(f"Failed to init {sensor_name}: {e}")

        print(f"Initialized {len(self.manager.sensors)} sensors")
        return True

    def start_sensor_readers(self):
        """Start threads to read sensors and send data to client"""
        self.running = True
        
        # Start each sensor type in its own thread with different frequencies
        self.sensor_readers = [
            threading.Thread(target=self._read_and_send_as5600, daemon=True),
            threading.Thread(target=self._read_and_send_mpu6050, daemon=True),
            threading.Thread(target=self._read_and_send_ds18b20, daemon=True)
        ]
        
        for reader in self.sensor_readers:
            reader.start()
            
        print("Sensor readers started")
    
    def stop_sensor_readers(self):
        """Stop all sensor reader threads"""
        self.running = False
        for reader in self.sensor_readers:
            if reader.is_alive():
                reader.join(timeout=1.0)
    
    def _send_sensor_data(self, sensor_type, data):
        """Send sensor data to connected client using the existing TCP server"""
        if hasattr(self.server, 'client_socket') and self.server.client_socket:
            try:
                message = {
                    "type": "sensor_data",
                    "sensor_type": sensor_type,
                    "data": data,
                    "timestamp": time.time()
                }
                json_message = json.dumps(message) + '\n'
                self.server.client_socket.sendall(json_message.encode('utf-8'))
            except Exception as e:
                print(f"Failed to send sensor data: {e}")
    
    def _read_and_send_as5600(self):
        """Read AS5600 sensors and send data"""
        while self.running:
            try:
                data = self.manager.update_as5600()
                if data:
                    self._send_sensor_data("AS5600", data)
                time.sleep(0.1)  # 10Hz
            except Exception as e:
                print(f"AS5600 error: {e}")
                time.sleep(0.2)
    
    def _read_and_send_mpu6050(self):
        """Read MPU6050 sensors and send data"""
        while self.running:
            try:
                data = self.manager.update_mpu6050()
                if data:
                    self._send_sensor_data("MPU6050", data)
                time.sleep(0.1)  # 10Hz
            except Exception as e:
                print(f"MPU6050 error: {e}")
                time.sleep(0.1)
    
    def _read_and_send_ds18b20(self):
        """Read DS18B20 sensors and send data"""
        while self.running:
            try:
                data = self.manager.update_ds18b20()
                if data:
                    self._send_sensor_data("DS18B20", data)
                time.sleep(2.0)  # 0.5Hz
            except Exception as e:
                print(f"DS18B20 error: {e}")
                time.sleep(3.0)
    
    def start(self):
        """Start the sensor system"""
        # Initialize sensors
        if not self.initialize_sensors():
            print("Failed to initialize sensors")
            return
        
        print("Sensor system initialized. Waiting for client connection...")
        
        # Wait for client connection before starting sensor readers
        while not hasattr(self.server, 'client_socket') or not self.server.client_socket:
            if not self.server.running:
                return
            time.sleep(0.5)
        
        print(f"Client connected: {self.server.client_address}")
        self.start_sensor_readers()
        
        try:
            # Monitor connection status
            while self.server.running:
                time.sleep(1)
                # Check if client is still connected
                if not hasattr(self.server, 'client_socket') or not self.server.client_socket:
                    print("Client disconnected. Stopping sensor readers.")
                    self.stop_sensor_readers()
                    # Wait for new connection
                    while not hasattr(self.server, 'client_socket') or not self.server.client_socket:
                        if not self.server.running:
                            break
                        time.sleep(0.5)
                    if hasattr(self.server, 'client_socket') and self.server.client_socket:
                        print(f"New client connected: {self.server.client_address}")
                        self.start_sensor_readers()
                        
        except KeyboardInterrupt:
            print("\nShutting down sensor system...")
        finally:
            self.stop_sensor_readers()
            if hasattr(self, 'bus') and self.bus:
                self.bus.close()


