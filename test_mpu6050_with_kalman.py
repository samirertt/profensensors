# test_mpu6050_log.py
from utils.kalman import KalmanAngle
import smbus2
import time, math, csv, os
from datetime import datetime, timezone

# ---------------- config ----------------
RestrictPitch = True
radToDeg = 57.2957786

CH_MPU1 = 2   # channel where first MPU6050 is connected (from your scan)
CH_MPU2 = 3   # channel where second MPU6050 is connected

LOG_PATH = "mpu_log_with_kalman.csv"   # path to log file
SAMPLE_DELAY = 0.01        # loop delay (seconds)

# ---------------- Kalman filters ----------------
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

# ---------------- helpers ----------------
def tca_select(channel: int):
    if not 0 <= channel <= 7:
        raise ValueError("TCA channel must be 0–7")
    bus.write_byte(DEVICE_TCA9548A, 1 << channel)
    time.sleep(0.02)   # settle

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

def init_mpu_on_channel(channel, kalmanX, kalmanY):
    """Init MPU on the given TCA channel and seed Kalman with initial roll/pitch.
       Returns (ok, roll, pitch) where ok is boolean."""
    try:
        tca_select(channel)
        MPU_Init()
        time.sleep(0.05)
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
        return True, roll, pitch
    except Exception:
        return False, None, None

def read_mpu(channel, kalmanX, kalmanY, dt):
    """Read raw + kalman for MPU on channel. Returns dict or None on error."""
    try:
        tca_select(channel)
        ax = read_raw_data(ACCEL_XOUT_H)
        ay = read_raw_data(ACCEL_YOUT_H)
        az = read_raw_data(ACCEL_ZOUT_H)
        gx = read_raw_data(GYRO_XOUT_H)
        gy = read_raw_data(GYRO_YOUT_H)
        gz = read_raw_data(GYRO_ZOUT_H)

        if RestrictPitch:
            roll  = math.atan2(ay, az) * radToDeg
            pitch = math.atan(-ax / math.sqrt((ay**2)+(az**2))) * radToDeg
        else:
            roll  = math.atan(ay / math.sqrt((ax**2)+(az**2))) * radToDeg
            pitch = math.atan2(-ax, az) * radToDeg

        kalX = kalmanX.getAngle(roll, gx / 131.0, dt)
        kalY = kalmanY.getAngle(pitch, gy / 131.0, dt)

        return {
            "ok": True,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "kalX": kalX, "kalY": kalY
        }
    except Exception:
        return {"ok": False}

# ---------------- log file setup ----------------
write_header = not os.path.exists(LOG_PATH)
log_file = open(LOG_PATH, "a", newline="")
csv_writer = csv.writer(log_file)

if write_header:
    csv_writer.writerow([
        "timestamp_iso", "ts_epoch", "dt_s",
        # MPU1 raw + kal
        "mpu1_ok", "mpu1_ax", "mpu1_ay", "mpu1_az", "mpu1_gx", "mpu1_gy", "mpu1_gz", "mpu1_kalX", "mpu1_kalY",
        # MPU2 raw + kal
        "mpu2_ok", "mpu2_ax", "mpu2_ay", "mpu2_az", "mpu2_gx", "mpu2_gy", "mpu2_gz", "mpu2_kalX", "mpu2_kalY"
    ])
    log_file.flush()

# ---------------- startup init ----------------
ok1, r1, p1 = init_mpu_on_channel(CH_MPU1, kalmanX1, kalmanY1)
ok2, r2, p2 = init_mpu_on_channel(CH_MPU2, kalmanX2, kalmanY2)

print(f"MPU1 init ok={ok1} roll={r1}")
print(f"MPU2 init ok={ok2} roll={r2}")

timer = time.time()

# ---------------- main loop (logging) ----------------
try:
    while True:
        now = time.time()
        dt = now - timer
        timer = now
        ts_iso = datetime.now(timezone.utc).astimezone().isoformat()

        res1 = read_mpu(CH_MPU1, kalmanX1, kalmanY1, dt)
        res2 = read_mpu(CH_MPU2, kalmanX2, kalmanY2, dt)

        row = [
            ts_iso, f"{now:.6f}", f"{dt:.6f}",
            # MPU1
            res1.get("ok", False),
            res1.get("ax",""), res1.get("ay",""), res1.get("az",""),
            res1.get("gx",""), res1.get("gy",""), res1.get("gz",""),
            res1.get("kalX",""), res1.get("kalY",""),
            # MPU2
            res2.get("ok", False),
            res2.get("ax",""), res2.get("ay",""), res2.get("az",""),
            res2.get("gx",""), res2.get("gy",""), res2.get("gz",""),
            res2.get("kalX",""), res2.get("kalY","")
        ]

        csv_writer.writerow(row)
        log_file.flush()   # ensure data is written to disk immediately

        # also print a compact status line to console
        s1 = f"{res1.get('kalX','N/A'):.2f},{res1.get('kalY','N/A'):.2f}" if res1.get("ok") else "N/A"
        s2 = f"{res2.get('kalX','N/A'):.2f},{res2.get('kalY','N/A'):.2f}" if res2.get("ok") else "N/A"
        print(f"{ts_iso} dt={dt:.4f}s | MPU1(ch{CH_MPU1})={s1} | MPU2(ch{CH_MPU2})={s2}")

        time.sleep(SAMPLE_DELAY)

except KeyboardInterrupt:
    print("Stopping and closing log.")
finally:
    log_file.close()
