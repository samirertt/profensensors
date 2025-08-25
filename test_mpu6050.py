from utils.kalman import KalmanAngle
import smbus2
import time, math

# ---------------- Kalman setup ----------------
RestrictPitch = True
radToDeg = 57.2957786

# Each MPU has its own filter instances
kalmanX1, kalmanY1 = KalmanAngle(), KalmanAngle()
kalmanX2, kalmanY2 = KalmanAngle(), KalmanAngle()

# ---------------- MPU6050 registers ----------------
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# ---------------- I2C addresses ----------------
DEVICE_TCA9548A = 0x70    # TCA multiplexer
MPU_ADDR        = 0x68    # MPU6050 default address

bus = smbus2.SMBus(1)

# ---------------- Multiplexer control ----------------
def tca_select(channel: int):
    if not 0 <= channel <= 7:
        raise ValueError("TCA channel must be 0–7")
    bus.write_byte(DEVICE_TCA9548A, 1 << channel)
    time.sleep(0.01)

# ---------------- MPU6050 routines ----------------
def MPU_Init():
    bus.write_byte_data(MPU_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x06)     # DLPF
    bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 24)  # ±2000 °/s
    bus.write_byte_data(MPU_ADDR, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low  = bus.read_byte_data(MPU_ADDR, addr+1)
    value = (high << 8) | low
    if value > 32767:   # signed 16-bit
        value -= 65536
    return value

# ---------------- Initialization ----------------
def init_mpu(channel, kalmanX, kalmanY):
    tca_select(channel)
    MPU_Init()
    time.sleep(0.5)

    accX = read_raw_data(ACCEL_XOUT_H)
    accY = read_raw_data(ACCEL_YOUT_H)
    accZ = read_raw_data(ACCEL_ZOUT_H)

    if RestrictPitch:
        roll  = math.atan2(accY, accZ) * radToDeg
        pitch = math.atan(-accX / math.sqrt((accY**2)+(accZ**2))) * radToDeg
    else:
        roll  = math.atan(accY / math.sqrt((accX**2)+(accZ**2))) * radToDeg
        pitch = math.atan2(-accX, accZ) * radToDeg

    kalmanX.setAngle(roll)
    kalmanY.setAngle(pitch)

    return roll, pitch

# Init both MPUs
roll1, pitch1 = init_mpu(3, kalmanX1, kalmanY1)
roll2, pitch2 = init_mpu(4, kalmanX2, kalmanY2)

print("MPU1 init roll:", roll1)
print("MPU2 init roll:", roll2)

timer = time.time()

# ---------------- Main loop ----------------
while True:
    dt = time.time() - timer
    timer = time.time()

    # --- MPU1 (channel 3) ---
    tca_select(3)
    accX = read_raw_data(ACCEL_XOUT_H)
    accY = read_raw_data(ACCEL_YOUT_H)
    accZ = read_raw_data(ACCEL_ZOUT_H)
    gyroX = read_raw_data(GYRO_XOUT_H)
    gyroY = read_raw_data(GYRO_YOUT_H)

    if RestrictPitch:
        roll1  = math.atan2(accY, accZ) * radToDeg
        pitch1 = math.atan(-accX / math.sqrt((accY**2)+(accZ**2))) * radToDeg
    else:
        roll1  = math.atan(accY / math.sqrt((accX**2)+(accZ**2))) * radToDeg
        pitch1 = math.atan2(-accX, accZ) * radToDeg

    kalAngleX1 = kalmanX1.getAngle(roll1, gyroX/131.0, dt)
    kalAngleY1 = kalmanY1.getAngle(pitch1, gyroY/131.0, dt)

    # --- MPU2 (channel 4) ---
    tca_select(4)
    accX = read_raw_data(ACCEL_XOUT_H)
    accY = read_raw_data(ACCEL_YOUT_H)
    accZ = read_raw_data(ACCEL_ZOUT_H)
    gyroX = read_raw_data(GYRO_XOUT_H)
    gyroY = read_raw_data(GYRO_YOUT_H)

    if RestrictPitch:
        roll2  = math.atan2(accY, accZ) * radToDeg
        pitch2 = math.atan(-accX / math.sqrt((accY**2)+(accZ**2))) * radToDeg
    else:
        roll2  = math.atan(accY / math.sqrt((accX**2)+(accZ**2))) * radToDeg
        pitch2 = math.atan2(-accX, accZ) * radToDeg

    kalAngleX2 = kalmanX2.getAngle(roll2, gyroX/131.0, dt)
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroY/131.0, dt)

    # Print both
    print(f"MPU1 [Ch3] -> X:{kalAngleX1:.2f}°  Y:{kalAngleY1:.2f}°   |   MPU2 [Ch4] -> X:{kalAngleX2:.2f}°  Y:{kalAngleY2:.2f}°")
    time.sleep(0.01)
