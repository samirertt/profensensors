import smbus2, time, math, csv
from datetime import datetime

# I2C addresses
DEVICE_TCA9548A = 0x70   # TCA9548A multiplexer default address
DEVICE_AS5600   = 0x36   # AS5600 encoder default address

bus = smbus2.SMBus(1)

# Function to select a TCA channel
def tca_select(channel):
    if channel < 0 or channel > 7:
        raise ValueError("TCA channel must be 0–7")
    bus.write_byte(DEVICE_TCA9548A, 1 << channel)
    time.sleep(0.01)  # small delay for switching

# Function to read raw angle
def ReadRawAngle():
    read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x0C, 2)
    return (read_bytes[0]<<8) | read_bytes[1]

# Function to read magnet strength
def ReadMagnitude():
    read_bytes = bus.read_i2c_block_data(DEVICE_AS5600, 0x1B, 2)
    return (read_bytes[0]<<8) | read_bytes[1]

# ============================================================
# Setup CSV file
# ============================================================
csv_filename = "as5600_log.csv"
with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "RawAngle", "Magnitude", "AngleDeg"])  # header

tca_select(0)  # enable channel 0

# ============================================================
# Read loop and log to CSV
# ============================================================
while True:
    raw_angle = ReadRawAngle()
    magnitude = ReadMagnitude()
    angle_deg = (raw_angle * 360.0 / 4096) - 90
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    # Print to console
    print(f"Time: {timestamp} | Raw: {raw_angle:4d}   Mag: {magnitude:4d}   Angle: {angle_deg:6.2f}°")

    # Append to CSV
    with open(csv_filename, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([timestamp, raw_angle, magnitude, angle_deg])

    time.sleep(0.1)
