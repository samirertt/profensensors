# core/manager.py
class SensorManager:
    def __init__(self, mux=None):
        self.mux = mux
        self.sensors = []

    def register(self, sensor):
        self.sensors.append(sensor)

    def update_all(self):
        data = {}
        for sensor in self.sensors:
            # select channel if using multiplexer
            if sensor.channel is not None and self.mux:
                self.mux[sensor.channel]  # Adafruit TCA9548A selects channel automatically
            data[sensor.name] = sensor.read()
        return data
