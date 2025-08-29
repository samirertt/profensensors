import time
import csv
from ds18b20 import DS18B20  # assumes your class is saved in ds18b20.py

# Initialize two sensors: motor X (index 0), motor Y (index 1)
sensor_x = DS18B20(device_index=0, name="Motor_X")
sensor_y = DS18B20(device_index=1, name="Motor_Y")

# Open CSV file for writing
with open("motor_temps.csv", mode="w", newline="") as file:
    writer = csv.writer(file)
    # Write header
    writer.writerow(["Time (s)", "Motor X Temp (°C)", "Motor Y Temp (°C)"])

    start_time = time.time()

    while True:
        try:
            # Read temperatures
            temp_x = sensor_x.read()["temperature_C"]
            temp_y = sensor_y.read()["temperature_C"]
            t = time.time() - start_time

            # Write to CSV
            writer.writerow([f"{t:.2f}", 
                             f"{temp_x:.2f}" if temp_x is not None else "NaN",
                             f"{temp_y:.2f}" if temp_y is not None else "NaN"])

            # Optional: print to console
            print(f"[{t:.1f}s] Motor X: {temp_x:.2f} °C | Motor Y: {temp_y:.2f} °C")

            time.sleep(0.5)  # read every 0.5 seconds

        except KeyboardInterrupt:
            print("Stopped by user.")
            break
