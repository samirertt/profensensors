import smbus2, time, math

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
# Example: read from AS5600 on channel 0 of the TCA9548A
# ============================================================

tca_select(0)  # enable channel 0

while True:
    raw_angle = ReadRawAngle()
    magnitude = ReadMagnitude()
    angle_deg = raw_angle * 360.0 / 4096

    print(f"Raw: {raw_angle:4d}   Mag: {magnitude:4d}   Angle: {angle_deg:6.2f}°")
    time.sleep(0.1)
