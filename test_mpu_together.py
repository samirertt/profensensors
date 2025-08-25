# test_mpu_both_logs.py
from utils.kalman import KalmanAngle
import smbus2
import time, math, csv, os
from datetime import datetime, timezone

# ---------------- config ----------------
CH_MPU1 = 2   # channel where first MPU6050 is connected (from your scan)
CH_MPU2 = 3   # channel where second MPU6050 is connected
SAMPLE_DELAY = 0.01   # seconds between samples

RAW_LOG = "mpu_raw_log_together.csv"
KALMAN_LOG = "mpu_kalman_log_together.csv"

# ---------------- IMU / Kalman ----------------
RestrictPitch = True
radToDeg = 57.2957786

# Kalman filter instances (per sensor)
kalmanX1, kalmanY1 = KalmanAngle(), KalmanAngle()
kalmanX2, kalmanY2 = KalmanAngle(), KalmanAngle()

# ---------------- I2C / registers ----------------
TCA_ADDR = 0x70
MPU_ADDR = 0x68

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

# Sensitivities (based on init settings below)
ACCEL_SENS = 16384.0    # LSB/g for ±2g (default)
GYRO_SENS  = 16.4       # LSB/(°/s) for ±2000 °/s (FS_SEL=3)

bus = smbus2.SMBus(1)

# ---------------- helpers ----------------
def tca_select(ch: int):
    if not 0 <= ch <= 7:
        raise ValueError("TCA channel must be 0-7")
    bus.write_byte(TCA_ADDR, 1 << ch)
    time.sleep(0.02)

def MPU_Init():
    # Keep the same configuration you used previously
    bus.write_byte_data(MPU_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x06)     # DLPF
    bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 24)  # FS_SEL=3 -> ±2000 °/s
    bus.write_byte_data(MPU_ADDR, INT_ENABLE, 1)

def read_raw16(addr):
    hi = bus.read_byte_data(MPU_ADDR, addr)
    lo = bus.read_byte_data(MPU_ADDR, addr+1)
    val = (hi << 8) | lo
    if val & 0x8000:
        val -= 65536
    return val

def read_sensor_once(channel):
    """Select channel and read raw accel+gyro. Returns dict with ok flag and both raw & converted vals."""
    try:
        tca_select(channel)
        ax = read_raw16(ACCEL_XOUT_H)
        ay = read_raw16(ACCEL_YOUT_H)
        az = read_raw16(ACCEL_ZOUT_H)
        gx = read_raw16(GYRO_XOUT_H)
        gy = read_raw16(GYRO_YOUT_H)
        gz = read_raw16(GYRO_ZOUT_H)

        # convert to physical units
        ax_g = ax / ACCEL_SENS
        ay_g = ay / ACCEL_SENS
        az_g = az / ACCEL_SENS
        gx_dps = gx / GYRO_SENS
        gy_dps = gy / GYRO_SENS
        gz_dps = gz / GYRO_SENS

        # accel-based roll/pitch (no filtering)
        denom = math.sqrt(ay*ay + az*az)
        roll_acc = math.atan2(ay, az) * radToDeg if denom != 0 else None
        pitch_acc = math.atan(-ax / math.sqrt((ay**2)+(az**2))) * radToDeg if (ay!=0 or az!=0) else None

        return {
            "ok": True,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "ax_g": ax_g, "ay_g": ay_g, "az_g": az_g,
            "gx_dps": gx_dps, "gy_dps": gy_dps, "gz_dps": gz_dps,
            "roll_acc": roll_acc, "pitch_acc": pitch_acc
        }
    except Exception:
        return {"ok": False}

def init_and_seed(channel, kx, ky):
    ok = False
    roll = pitch = None
    try:
        tca_select(channel)
        MPU_Init()
        time.sleep(0.05)
        s = read_sensor_once(channel)
        if s.get("ok"):
            roll = s["roll_acc"]
            pitch = s["pitch_acc"]
            kx.setAngle(roll)
            ky.setAngle(pitch)
            ok = True
    except Exception:
        ok = False
    return ok, roll, pitch

# ---------------- prepare logs ----------------
raw_header = [
    "timestamp_iso", "ts_epoch", "dt_s",
    "mpu_num", "ok",
    "ax", "ay", "az", "ax_g", "ay_g", "az_g",
    "gx", "gy", "gz", "gx_dps", "gy_dps", "gz_dps",
    "roll_acc", "pitch_acc"
]

kalman_header = [
    "timestamp_iso", "ts_epoch", "dt_s",
    "mpu_num", "ok",
    # include raw for reference too
    "ax", "ay", "az", "ax_g", "ay_g", "az_g",
    "gx", "gy", "gz", "gx_dps", "gy_dps", "gz_dps",
    # filtered outputs
    "kalX_deg", "kalY_deg"
]

raw_file_exists = os.path.exists(RAW_LOG)
kal_file_exists = os.path.exists(KALMAN_LOG)

raw_f = open(RAW_LOG, "a", newline="")
kal_f = open(KALMAN_LOG, "a", newline="")
raw_writer = csv.writer(raw_f)
kal_writer = csv.writer(kal_f)

if not raw_file_exists:
    raw_writer.writerow(raw_header)
    raw_f.flush()
if not kal_file_exists:
    kal_writer.writerow(kalman_header)
    kal_f.flush()

# ---------------- init sensors / seed kalman ----------------
ok1, r1, p1 = init_and_seed(CH_MPU1, kalmanX1, kalmanY1)
ok2, r2, p2 = init_and_seed(CH_MPU2, kalmanX2, kalmanY2)

print(f"MPU1 init ok={ok1} roll={r1}")
print(f"MPU2 init ok={ok2} roll={r2}")

timer = time.time()

# ---------------- main loop ----------------
try:
    while True:
        now = time.time()
        dt = now - timer
        timer = now
        ts_iso = datetime.now(timezone.utc).astimezone().isoformat()

        # read MPUs once each loop (raw)
        s1 = read_sensor_once(CH_MPU1)
        s2 = read_sensor_once(CH_MPU2)

        # --- write raw log (one row per MPU for raw file) ---
        for idx, s in enumerate((s1, s2), start=1):
            row = [
                ts_iso, f"{now:.6f}", f"{dt:.6f}",
                idx, s.get("ok", False),
                s.get("ax",""), s.get("ay",""), s.get("az",""),
                s.get("ax_g",""), s.get("ay_g",""), s.get("az_g",""),
                s.get("gx",""), s.get("gy",""), s.get("gz",""),
                s.get("gx_dps",""), s.get("gy_dps",""), s.get("gz_dps",""),
                s.get("roll_acc",""), s.get("pitch_acc","")
            ]
            raw_writer.writerow(row)
        raw_f.flush()

        # --- compute kalman updates (use gyro dps and dt) and write kalman log ---
        # MPU1
        if s1.get("ok"):
            kalX1 = kalmanX1.getAngle(s1["roll_acc"], s1["gx_dps"], dt)
            kalY1 = kalmanY1.getAngle(s1["pitch_acc"], s1["gy_dps"], dt)
            ok_flag1 = True
        else:
            kalX1 = kalY1 = None
            ok_flag1 = False

        # MPU2
        if s2.get("ok"):
            kalX2 = kalmanX2.getAngle(s2["roll_acc"], s2["gx_dps"], dt)
            kalY2 = kalmanY2.getAngle(s2["pitch_acc"], s2["gy_dps"], dt)
            ok_flag2 = True
        else:
            kalX2 = kalY2 = None
            ok_flag2 = False

        # Write kalman rows (one row per MPU)
        kal_row1 = [
            ts_iso, f"{now:.6f}", f"{dt:.6f}",
            1, ok_flag1,
            s1.get("ax",""), s1.get("ay",""), s1.get("az",""),
            s1.get("ax_g",""), s1.get("ay_g",""), s1.get("az_g",""),
            s1.get("gx",""), s1.get("gy",""), s1.get("gz",""),
            s1.get("gx_dps",""), s1.get("gy_dps",""), s1.get("gz_dps",""),
            kalX1, kalY1
        ]
        kal_row2 = [
            ts_iso, f"{now:.6f}", f"{dt:.6f}",
            2, ok_flag2,
            s2.get("ax",""), s2.get("ay",""), s2.get("az",""),
            s2.get("ax_g",""), s2.get("ay_g",""), s2.get("az_g",""),
            s2.get("gx",""), s2.get("gy",""), s2.get("gz",""),
            s2.get("gx_dps",""), s2.get("gy_dps",""), s2.get("gz_dps",""),
            kalX2, kalY2
        ]

        kal_writer.writerow(kal_row1)
        kal_writer.writerow(kal_row2)
        kal_f.flush()

        # console status
        s1_txt = f"{kalX1:.2f},{kalY1:.2f}" if ok_flag1 and kalX1 is not None else "N/A"
        s2_txt = f"{kalX2:.2f},{kalY2:.2f}" if ok_flag2 and kalX2 is not None else "N/A"
        print(f"{ts_iso} dt={dt:.4f}s | MPU1(ch{CH_MPU1})={s1_txt} | MPU2(ch{CH_MPU2})={s2_txt}")

        time.sleep(SAMPLE_DELAY)

except KeyboardInterrupt:
    print("stopping, closing files.")
finally:
    raw_f.close()
    kal_f.close()
