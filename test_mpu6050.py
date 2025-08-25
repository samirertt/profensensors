from utils.kalman import KalmanAngle
import smbus2
import time, math

# ---------------- config ----------------
RestrictPitch = True
radToDeg = 57.2957786

# sensor channels (from your scan)
CH_MPU1 = 2   # channel where first MPU6050 is connected
CH_MPU2 = 3   # channel where second MPU6050 is connected

# Each MPU has its own Kalman filter instances
kalmanX1, kalmanY1 = KalmanAngle(), KalmanAngle()
kalmanX2, kalmanY2 = KalmanAngle(), KalmanAngle()

# ---------------- MPU6050 registers / addresses ----------------
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

DEVICE_TCA9548A = 0x70
MPU_ADDR = 0x68

bus = smbus2.SMBus(1)

# ---------------- multiplexer ----------------
def tca_select(channel: int):
    if not 0 <= channel <= 7:
        raise ValueError("TCA channel must be 0–7")
    bus.write_byte(DEVICE_TCA9548A, 1 << channel)
    time.sleep(0.02)   # settle

# ---------------- MPU routines ----------------
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
    if value > 32767:
        value -= 65536
    return value

# ---------------- higher-level helpers ----------------
def init_mpu_on_channel(channel, kalmanX, kalmanY):
    """Select channel, init MPU, read initial roll/pitch and seed Kalman filter."""
    tca_select(channel)
    MPU_Init()
    time.sleep(0.1)
    ax = read_raw_data(ACCEL_XOUT_H)
    ay = read_raw_data(ACCEL_YOUT_H)
    az = read_raw_data(ACCEL_ZOUT_H)

    if RestrictPitch:
        roll  = math.atan2(ay, az) * radToDeg
        pitch = math.atan(-ax / math.sqrt((ay**2)+(az**2))) * radToDeg
    else:
        roll  = math.atan(ay / math.sqrt((ax**2)+(az**2))) * radToDeg
        pitch = math.atan2(-ax, az) * radToDeg

    kalmanX.setAngle(roll)
    kalmanY.setAngle(pitch)
    return roll, pitch

def read_mpu(channel, kalmanX, kalmanY, dt):
    """Select channel, read raw accel/gyro, compute roll/pitch and update Kalman. Returns (kalX, kalY)."""
    try:
        tca_select(channel)
        ax = read_raw_data(ACCEL_XOUT_H)
        ay = read_raw_data(ACCEL_YOUT_H)
        az = read_raw_data(ACCEL_ZOUT_H)
        gx = read_raw_data(GYRO_XOUT_H)
        gy = read_raw_data(GYRO_YOUT_H)

        if RestrictPitch:
            roll  = math.atan2(ay, az) * radToDeg
            pitch = math.atan(-ax / math.sqrt((ay**2)+(az**2))) * radToDeg
        else:
            roll  = math.atan(ay / math.sqrt((ax**2)+(az**2))) * radToDeg
            pitch = math.atan2(-ax, az) * radToDeg

        kalX = kalmanX.getAngle(roll, gx / 131.0, dt)
        kalY = kalmanY.getAngle(pitch, gy / 131.0, dt)
        return kalX, kalY

    except Exception as exc:
        # return None on error so caller can handle it
        return None

# ---------------- startup ----------------
roll1, pitch1 = init_mpu_on_channel(CH_MPU1, kalmanX1, kalmanY1)
roll2, pitch2 = init_mpu_on_channel(CH_MPU2, kalmanX2, kalmanY2)

print(f"MPU1 (ch {CH_MPU1}) init roll: {roll1:.2f}")
print(f"MPU2 (ch {CH_MPU2}) init roll: {roll2:.2f}")

timer = time.time()

# ---------------- main loop ----------------
while True:
    dt = time.time() - timer
    timer = time.time()

    res1 = read_mpu(CH_MPU1, kalmanX1, kalmanY1, dt)
    res2 = read_mpu(CH_MPU2, kalmanX2, kalmanY2, dt)

    s1 = "N/A"
    s2 = "N/A"
    if res1 is not None:
        kalX1, kalY1 = res1
        s1 = f"X:{kalX1:.2f}° Y:{kalY1:.2f}°"
    if res2 is not None:
        kalX2, kalY2 = res2
        s2 = f"X:{kalX2:.2f}° Y:{kalY2:.2f}°"

    print(f"MPU1 [ch {CH_MPU1}] -> {s1}   |   MPU2 [ch {CH_MPU2}] -> {s2}")
    time.sleep(0.01)
