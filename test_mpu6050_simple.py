# test_mpu6050_raw_log.py
import smbus2
import time
import math
import csv
import os
from datetime import datetime, timezone

# ---------------- config ----------------
CH_MPU1 = 2   # channel of first MPU (from your scan)
CH_MPU2 = 3   # channel of second MPU (from your scan)

LOG_PATH = "mpu_raw_log.csv"
SAMPLE_DELAY = 0.01   # seconds

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

radToDeg = 57.2957786

bus = smbus2.SMBus(1)

# ---------------- helpers ----------------
def tca_select(ch: int):
    if not 0 <= ch <= 7:
        raise ValueError("TCA channel must be 0-7")
    bus.write_byte(TCA_ADDR, 1 << ch)
    time.sleep(0.02)

def mpu_wake():
    # Wake MPU (simple init). Assumes correct channel already selected.
    bus.write_byte_data(MPU_ADDR, SMPLRT_DIV, 7)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 1)
    bus.write_byte_data(MPU_ADDR, CONFIG, 0x06)
    bus.write_byte_data(MPU_ADDR, GYRO_CONFIG, 24)
    bus.write_byte_data(MPU_ADDR, INT_ENABLE, 1)

def read_raw16(addr):
    hi = bus.read_byte_data(MPU_ADDR, addr)
    lo = bus.read_byte_data(MPU_ADDR, addr + 1)
    val = (hi << 8) | lo
    if val & 0x8000:
        val = -((~val & 0xFFFF) + 1)  # signed 16-bit
    return val

def read_mpu_raw(channel):
    """Select channel and read raw sensor registers. Returns dict with ok flag."""
    try:
        tca_select(channel)
        # read raw accel
        ax = read_raw16(ACCEL_XOUT_H)
        ay = read_raw16(ACCEL_YOUT_H)
        az = read_raw16(ACCEL_ZOUT_H)
        # read raw gyro
        gx = read_raw16(GYRO_XOUT_H)
        gy = read_raw16(GYRO_YOUT_H)
        gz = read_raw16(GYRO_ZOUT_H)

        # compute simple accel-based roll/pitch (no filtering)
        # avoid division by zero
        denom = math.sqrt(ay*ay + az*az)
        if denom == 0:
            roll = None
        else:
            roll = math.atan2(ay, az) * radToDeg

        denom2 = math.sqrt(ax*ax + az*az)
        if denom2 == 0:
            pitch = None
        else:
            pitch = math.atan2(-ax, denom) * radToDeg  # using same denom as common formula

        return {
            "ok": True,
            "ax": ax, "ay": ay, "az": az,
            "gx": gx, "gy": gy, "gz": gz,
            "roll_acc": roll, "pitch_acc": pitch
        }
    except Exception:
        return {"ok": False}

# ---------------- logging setup ----------------
write_header = not os.path.exists(LOG_PATH)
f = open(LOG_PATH, "a", newline="")
writer = csv.writer(f)
if write_header:
    writer.writerow([
        "timestamp_iso", "ts_epoch", "dt_s",
        # MPU1
        "mpu1_ok", "mpu1_ax", "mpu1_ay", "mpu1_az", "mpu1_gx", "mpu1_gy", "mpu1_gz", "mpu1_roll_acc", "mpu1_pitch_acc",
        # MPU2
        "mpu2_ok", "mpu2_ax", "mpu2_ay", "mpu2_az", "mpu2_gx", "mpu2_gy", "mpu2_gz", "mpu2_roll_acc", "mpu2_pitch_acc"
    ])
    f.flush()

# ---------------- init sensors ----------------
# Try to init both MPUs (select proper channel before waking)
try:
    tca_select(CH_MPU1)
    mpu_wake()
except Exception as e:
    print(f"Warning: could not init MPU1 on channel {CH_MPU1}: {e}")

try:
    tca_select(CH_MPU2)
    mpu_wake()
except Exception as e:
    print(f"Warning: could not init MPU2 on channel {CH_MPU2}: {e}")

time.sleep(0.1)

# ---------------- main logging loop ----------------
timer = time.time()
try:
    while True:
        now = time.time()
        dt = now - timer
        timer = now
        ts_iso = datetime.now(timezone.utc).astimezone().isoformat()

        r1 = read_mpu_raw(CH_MPU1)
        r2 = read_mpu_raw(CH_MPU2)

        row = [
            ts_iso, f"{now:.6f}", f"{dt:.6f}",
            # MPU1
            r1.get("ok", False),
            r1.get("ax", ""), r1.get("ay", ""), r1.get("az", ""),
            r1.get("gx", ""), r1.get("gy", ""), r1.get("gz", ""),
            r1.get("roll_acc", ""), r1.get("pitch_acc", ""),
            # MPU2
            r2.get("ok", False),
            r2.get("ax", ""), r2.get("ay", ""), r2.get("az", ""),
            r2.get("gx", ""), r2.get("gy", ""), r2.get("gz", ""),
            r2.get("roll_acc", ""), r2.get("pitch_acc", "")
        ]

        writer.writerow(row)
        f.flush()

        # compact console status
        s1 = f"{r1.get('roll_acc','N/A')},{r1.get('pitch_acc','N/A')}" if r1.get("ok") else "N/A"
        s2 = f"{r2.get('roll_acc','N/A')},{r2.get('pitch_acc','N/A')}" if r2.get("ok") else "N/A"
        print(f"{ts_iso} dt={dt:.4f}s | MPU1(ch{CH_MPU1})={s1} | MPU2(ch{CH_MPU2})={s2}")

        time.sleep(SAMPLE_DELAY)
except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    f.close()
