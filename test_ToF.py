from machine import I2C, Pin
import time

tests = [
    ("I2C0 GP4/GP5", 0, 4, 5),
    ("I2C1 GP2/GP3", 1, 2, 3),
]

for label, pid, sda, scl in tests:
    print("\n=== {} ===".format(label))
    try:
        i2c = I2C(pid, sda=Pin(sda), scl=Pin(scl), freq=100000)
        found = i2c.scan()
        print("scan:", found)

        if 0x29 in found:
            print("VL53L0X found at 0x29, attempting init + range...")
            from modules.vl53l0x import VL53L0X
            tof = VL53L0X(i2c)
            print("init OK")
            for i in range(5):
                print("  reading {}: {} mm".format(i + 1, tof.range))
                time.sleep_ms(200)
        else:
            print("no VL53L0X on this bus")

    except Exception as e:
        print("ERROR: {} - {}".format(type(e).__name__, e))