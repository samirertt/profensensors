class SensorManager:
    def __init__(self, mux=None):
        self.mux = mux
        self.sensors = []

    def register(self, sensor):
        self.sensors.append(sensor)
        print(f"Registered sensor: {sensor.name}")

    def update_all(self):
        data = {}
        for sensor in self.sensors:
            try:
                data[sensor.name] = sensor.read()
            except Exception as e:
                data[sensor.name] = {"ok": False, "error": str(e)}
        return data