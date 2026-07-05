from machine import I2C, Pin, ADC
import time
import math
from modules.vl53l0x import VL53L0X

# =====================================================================
# HARDWARE SETUP
# =====================================================================

# ---- VL53L0X ToF sensors (I2C0, GP4/GP5) ----
XSHUT_LEFT  = Pin(14, Pin.OUT)
XSHUT_RIGHT = Pin(15, Pin.OUT)
DEFAULT_ADDR = 0x29
ADDR_FRONT   = 0x2A
ADDR_LEFT    = 0x2B

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# ---- TCS3200 color sensor ----
s2  = Pin(19, Pin.OUT)
s3  = Pin(17, Pin.OUT)
out = Pin(16, Pin.IN)

FILTERS = {
    "red":   (0, 0),
    "blue":  (0, 1),
    "clear": (1, 0),
    "green": (1, 1),
}

# Irradiance responsivity at 20% scaling (S0=H, S1=L)
# TCS3200 datasheet p.7, clear channel, Hz/(µW/cm²)
RESPONSIVITY = 150

# ---- NTC thermistor on ADC0 (GP26) ----
# SSR-1016: 10K @ 25°C, B25/50 = 3950 (datasheet)
# Divider: 3.3V -> 10K fixed -> ADC0 -> NTC -> GND
ntc_adc = ADC(Pin(26))
R_FIXED = 10000
R0      = 10000
B       = 3950
T0      = 298.15


# =====================================================================
# SENSOR HELPERS
# =====================================================================

def safe_call(label, addr, fn):
    try:
        return fn()
    except OSError as e:
        print("FAILED: %-28s addr=0x%02X  (%s)" % (label, addr, e))
        return None

def read_temp_c():
    raw = ntc_adc.read_u16()
    v = raw / 65535 * 3.3
    if v >= 3.29 or v <= 0.01:
        return None
    r = R_FIXED * v / (3.3 - v)
    return 1.0 / (1.0 / T0 + (1.0 / B) * math.log(r / R0)) - 273.15

def read_freq(samples=10):
    total = 0
    for _ in range(samples):
        while out.value() == 1: pass
        while out.value() == 0: pass
        t0 = time.ticks_us()
        while out.value() == 1: pass
        while out.value() == 0: pass
        t1 = time.ticks_us()
        total += time.ticks_diff(t1, t0)
    period = total / samples
    return 1_000_000 / period if period > 0 else 0

def read_color():
    result = {}
    for name, (s2v, s3v) in FILTERS.items():
        s2.value(s2v)
        s3.value(s3v)
        time.sleep_ms(20)
        result[name] = read_freq(10)
    return result

def read_tof():
    """Read all available ToF sensors. Returns dict name -> mm (or None)."""
    readings = {}
    for name in ("front", "left", "right"):
        if name not in sensors:
            readings[name] = None
            continue
        try:
            val = sensors[name].range
            readings[name] = val if val < 8190 else None  # 8190 = no target
        except OSError:
            readings[name] = None
    return readings


# =====================================================================
# VL53L0X ADDRESS DANCE (runs once at boot)
# =====================================================================

XSHUT_LEFT.value(0)
XSHUT_RIGHT.value(0)
time.sleep_ms(50)

print("=== VL53L0X init ===")
print("scan (front only):", safe_call("scan", DEFAULT_ADDR, i2c.scan))
safe_call("front -> 0x2A", DEFAULT_ADDR,
          lambda: i2c.writeto_mem(DEFAULT_ADDR, 0x8A, bytes([ADDR_FRONT])))
time.sleep_ms(10)

XSHUT_LEFT.value(1)
time.sleep_ms(50)
safe_call("left -> 0x2B", DEFAULT_ADDR,
          lambda: i2c.writeto_mem(DEFAULT_ADDR, 0x8A, bytes([ADDR_LEFT])))
time.sleep_ms(10)

XSHUT_RIGHT.value(1)
time.sleep_ms(50)
print("final bus:", safe_call("scan", DEFAULT_ADDR, i2c.scan))

sensors = {}
for name, addr in (("front", ADDR_FRONT), ("left", ADDR_LEFT), ("right", DEFAULT_ADDR)):
    tof = safe_call("init %s" % name, addr,
                     lambda a=addr: VL53L0X(i2c, address=a, io_timeout_ms=1000))
    if tof is not None:
        sensors[name] = tof
        print("  %s OK (0x%02X)" % (name, addr))
    else:
        print("  %s FAILED" % name)


# =====================================================================
# MEASURE COMMAND
# =====================================================================

def measure(n=1, interval_ms=500):
    """
    Read all sensors. Call measure() for one shot, measure(10) for 10
    readings at 500ms intervals, or measure(10, 1000) for 1s intervals.
    """
    for i in range(n):
        # ToF distances
        tof = read_tof()
        tof_parts = []
        for name in ("front", "left", "right"):
            if tof[name] is not None:
                tof_parts.append("%s:%4dmm" % (name[0].upper(), tof[name]))
            else:
                tof_parts.append("%s:  n/a" % name[0].upper())

        # Color + irradiance
        ch = read_color()
        clr = ch["clear"]
        irr = clr / RESPONSIVITY
        if clr > 0:
            rr = ch["red"]   / clr
            gr = ch["green"] / clr
            br = ch["blue"]  / clr
        else:
            rr = gr = br = 0

        # Temperature
        temp = read_temp_c()

        # Output
        if n == 1:
            # detailed single-shot
            print("--- ToF ---")
            print("  " + "  |  ".join(tof_parts))
            print("--- Color ---")
            print("  irr: %.1f uW/cm2  R:%.3f G:%.3f B:%.3f" % (irr, rr, gr, br))
            print("  raw: clr=%.0f R=%.0f G=%.0f B=%.0f" % (clr, ch["red"], ch["green"], ch["blue"]))
            print("--- Temp ---")
            print("  %.1f C" % temp if temp else "  ERR")
        else:
            # compact multi-shot
            t_str = "%.1fC" % temp if temp else "ERR"
            print("[%3d] %s  | %.1fuW R:%.2f G:%.2f B:%.2f | %s" % (
                i + 1,
                " ".join(tof_parts),
                irr, rr, gr, br,
                t_str))
            if i < n - 1:
                time.sleep_ms(interval_ms)


print("\n=== Ready ===")
print("  measure()          single reading, all sensors")
print("  measure(20)        20 readings @ 500ms")
print("  measure(10, 1000)  10 readings @ 1s")