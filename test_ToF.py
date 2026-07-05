from machine import I2C, Pin
import time
from modules.vl53l0x import VL53L0X

# front_ToF = no controllable XSHUT (always powered) -> 0x2A
# left      = XSHUT on GP14                          -> 0x2B
# right     = XSHUT on GP15                           -> keeps default 0x29

XSHUT_LEFT = Pin(14, Pin.OUT)
XSHUT_RIGHT = Pin(15, Pin.OUT)
DEFAULT_ADDR = 0x29
ADDR_FRONT = 0x2A
ADDR_LEFT = 0x2B

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)


def safe_call(label, addr, fn):
    try:
        return fn()
    except OSError as e:
        print("FAILED: %-28s addr=0x%02X  (%s)" % (label, addr, e))
        return None


# 1) Hold left and right in standby so only front answers at default
XSHUT_LEFT.value(0)
XSHUT_RIGHT.value(0)
time.sleep_ms(50)

print("Only front powered:", safe_call("scan (front only)", DEFAULT_ADDR, i2c.scan))
safe_call("reassign front -> 0x2A", DEFAULT_ADDR,
          lambda: i2c.writeto_mem(DEFAULT_ADDR, 0x8A, bytes([ADDR_FRONT])))
time.sleep_ms(10)
print("After moving front:", safe_call("scan (post-front)", ADDR_FRONT, i2c.scan))

# 2) Bring left up alone, reassign it
XSHUT_LEFT.value(1)
time.sleep_ms(50)
print("front moved + left at default:", safe_call("scan (front+left)", DEFAULT_ADDR, i2c.scan))
safe_call("reassign left -> 0x2B", DEFAULT_ADDR,
          lambda: i2c.writeto_mem(DEFAULT_ADDR, 0x8A, bytes([ADDR_LEFT])))
time.sleep_ms(10)
print("After moving left:", safe_call("scan (post-left)", ADDR_LEFT, i2c.scan))

# 3) Bring right up last -- keeps default address
XSHUT_RIGHT.value(1)
time.sleep_ms(50)
print("Final bus state:", safe_call("scan (final)", DEFAULT_ADDR, i2c.scan))

# 4) Init each sensor independently -- one failing doesn't block the others
sensors = {}
for name, addr in (("front", ADDR_FRONT), ("left", ADDR_LEFT), ("right", DEFAULT_ADDR)):
    tof = safe_call("init sensor %s" % name, addr,
                     lambda addr=addr: VL53L0X(i2c, address=addr, io_timeout_ms=1000))
    if tof is not None:
        sensors[name] = tof
        print("Sensor %s init OK (addr=0x%02X)" % (name, addr))
    else:
        print("Sensor %s NOT available -- skipping" % name)

if not sensors:
    print("No sensors initialized -- check wiring before continuing.")


def measure():
    """Type measure() in the REPL to get one reading from every available sensor."""
    readings = []
    for name in ("front", "left", "right"):
        if name not in sensors:
            readings.append("%s: n/a" % name)
            continue
        try:
            readings.append("%s: %d mm" % (name, sensors[name].range))
        except OSError as e:
            readings.append("%s: read error (%s)" % (name, e))
    print("  |  ".join(readings))


print("\nSetup done. Type measure() in the REPL to take a reading.")