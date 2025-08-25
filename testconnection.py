import smbus2, time

bus = smbus2.SMBus(1)
TCA_ADDR = 0x70

def tca_select(ch):
    bus.write_byte(TCA_ADDR, 1 << ch)
    time.sleep(0.05)

print("Scanning TCA channels 0–7...")
for ch in range(8):
    tca_select(ch)
    print(f"\nChannel {ch}:")
    for addr in range(0x03, 0x77):
        try:
            bus.write_byte(addr, 0)
            print(f"  Found device at 0x{addr:02X}")
        except:  
            pass