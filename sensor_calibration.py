from machine import I2C, Pin, ADC
import time
import math
from modules.vl53l0x import VL53L0X

# =====================================================================
# HARDWARE SETUP
# =====================================================================

# ---- VL53L0X ToF sensors (I2C0, GP4/GP5) ----
# NOTE: GP14 enables the RIGHT sensor, GP15 the LEFT.
XSHUT_RIGHT = Pin(14, Pin.OUT)
XSHUT_LEFT  = Pin(15, Pin.OUT)
DEFAULT_ADDR = 0x29
ADDR_FRONT   = 0x2A
ADDR_RIGHT   = 0x2B

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# ToF measurement profile — KEEP CONSISTENT between calibration and
# operation (accuracy depends on timing budget!):
TOF_BUDGET_US   = 20000  # 20ms high-speed mode (~+-5%); default would be 33ms
TOF_SAMPLES     = 5      # samples per sensor per burst cycle
TOF_MIN_MM      = 50     # discard implausible near readings
TOF_INVALID_MM  = 8190   # sensor sentinel for "no target"

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
# TCS3200 datasheet p.7, clear channel, Hz/(uW/cm2)
RESPONSIVITY = 150

# Color measurement profile — KEEP CONSISTENT between calibration
# and operation:
COLOR_PASSES     = 3     # full RGBC passes averaged per reported value
COLOR_PERIODS    = 10    # OUT periods counted per filter per pass
FILTER_SETTLE_MS = 20    # settle time after switching S2/S3
FREQ_TIMEOUT_US  = 60000 # per-edge timeout (dark surface / dead line)

# ---- NTC thermistor on ADC0 (GP26) ----
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
    except (OSError, RuntimeError) as e:
        print("FAILED: %-28s addr=0x%02X  (%s)" % (label, addr, e))
        return None

# Empirical correction — measured vs. hand-verified reference temps
# (raw: 24/28/31C -> actual: 23/26/27.5C). Only 3 data points; refine
# with more readings across the expected range if precision matters.
TEMP_CORR_SLOPE  = 0.649
TEMP_CORR_OFFSET = 7.55

def read_temp_c():
    raw = ntc_adc.read_u16()
    v = raw / 65535 * 3.3
    if v >= 3.29 or v <= 0.01:
        return None
    r = R_FIXED * v / (3.3 - v)
    t_raw = 1.0 / (1.0 / T0 + (1.0 / B) * math.log(r / R0)) - 273.15
    return TEMP_CORR_SLOPE * t_raw + TEMP_CORR_OFFSET

def _wait_edge(target, deadline):
    while out.value() != target:
        if time.ticks_diff(deadline, time.ticks_us()) <= 0:
            return False
    return True

def read_freq(periods=COLOR_PERIODS):
    """Measure OUT frequency over n periods; None on timeout."""
    deadline = time.ticks_add(time.ticks_us(), FREQ_TIMEOUT_US)
    if not (_wait_edge(0, deadline) and _wait_edge(1, deadline)):
        return None
    t0 = time.ticks_us()
    for _ in range(periods):
        if not (_wait_edge(0, deadline) and _wait_edge(1, deadline)):
            return None
    t1 = time.ticks_us()
    period = time.ticks_diff(t1, t0) / periods
    return 1_000_000 / period if period > 0 else None

def read_color_once():
    result = {}
    for name, (s2v, s3v) in FILTERS.items():
        s2.value(s2v)
        s3.value(s3v)
        time.sleep_ms(FILTER_SETTLE_MS)
        result[name] = read_freq()
    return result

def read_color(passes=COLOR_PASSES):
    """Mean of several RGBC passes — the pattern used in operation."""
    acc = {name: [] for name in FILTERS}
    for _ in range(passes):
        single = read_color_once()
        for name, val in single.items():
            if val is not None:
                acc[name].append(val)
    return {name: (sum(v) / len(v) if v else 0) for name, v in acc.items()}

# ---------------------------------------------------------------------
# ToF: burst reading with median filter — the operational pattern.
# Phase 1: left/right alternating (TOF_SAMPLES each), phase 2: front.
# ---------------------------------------------------------------------

def _tof_sample(name):
    """One raw sample from one sensor; None if invalid/implausible."""
    if name not in sensors:
        return None
    try:
        val = sensors[name].range
    except OSError:
        return None
    if val is None or val >= TOF_INVALID_MM or val < TOF_MIN_MM:
        return None
    return val

def _median(vals):
    if not vals:
        return None
    s = sorted(vals)
    m = len(s) // 2
    if len(s) % 2:
        return s[m]
    return (s[m - 1] + s[m]) // 2

def read_tof():
    """Burst-read all ToF sensors in the operational phase pattern.
    Returns dict name -> median mm (or None if no valid samples)."""
    raw = {"front": [], "left": [], "right": []}

    # Phase 1: sides, alternating L/R
    for _ in range(TOF_SAMPLES):
        for name in ("left", "right"):
            v = _tof_sample(name)
            if v is not None:
                raw[name].append(v)

    # Phase 2: front
    for _ in range(TOF_SAMPLES):
        v = _tof_sample("front")
        if v is not None:
            raw["front"].append(v)

    return {name: _median(vals) for name, vals in raw.items()}


# =====================================================================
# VL53L0X ADDRESS DANCE (runs once at boot)
# =====================================================================

XSHUT_RIGHT.value(0)
XSHUT_LEFT.value(0)
time.sleep_ms(50)

print("=== VL53L0X init ===")
print("scan (front only):", safe_call("scan", DEFAULT_ADDR, i2c.scan))
safe_call("front -> 0x2A", DEFAULT_ADDR,
          lambda: i2c.writeto_mem(DEFAULT_ADDR, 0x8A, bytes([ADDR_FRONT])))
time.sleep_ms(10)

XSHUT_RIGHT.value(1)
time.sleep_ms(50)
safe_call("right -> 0x2B", DEFAULT_ADDR,
          lambda: i2c.writeto_mem(DEFAULT_ADDR, 0x8A, bytes([ADDR_RIGHT])))
time.sleep_ms(10)

XSHUT_LEFT.value(1)
time.sleep_ms(50)
print("final bus:", safe_call("scan", DEFAULT_ADDR, i2c.scan))

sensors = {}
for name, addr in (("front", ADDR_FRONT), ("right", ADDR_RIGHT), ("left", DEFAULT_ADDR)):
    tof = safe_call("init %s" % name, addr,
                     lambda a=addr: VL53L0X(i2c, address=a, io_timeout_ms=1000))
    if tof is not None:
        sensors[name] = tof
        print("  %s OK (0x%02X)" % (name, addr))
    else:
        print("  %s FAILED" % name)

# Set the timing budget (high-speed mode). Depends on driver support —
# watch this output! If unsupported, samples run at default ~33ms.
for name, tof in sensors.items():
    fn = getattr(tof, "set_measurement_timing_budget", None)
    if callable(fn):
        try:
            fn(TOF_BUDGET_US)
            print("  %s: timing budget %dus OK" % (name, TOF_BUDGET_US))
        except Exception as e:
            print("  %s: budget set FAILED (%s) -> default ~33ms" % (name, e))
    else:
        print("  %s: driver has no set_measurement_timing_budget -> default ~33ms" % name)


# =====================================================================
# MEASURE COMMAND
# =====================================================================

def measure(n=1, interval_ms=500):
    """
    Read all sensors. measure() = one shot, measure(10) = 10 readings
    @500ms, measure(10, 1000) = @1s. ToF values are the median of
    TOF_SAMPLES burst samples; color is the mean of COLOR_PASSES passes.
    """
    for i in range(n):
        tof = read_tof()
        tof_parts = []
        for name in ("front", "left", "right"):
            if tof[name] is not None:
                tof_parts.append("%s:%4dmm" % (name[0].upper(), tof[name]))
            else:
                tof_parts.append("%s:  n/a" % name[0].upper())

        ch = read_color()
        clr = ch["clear"]
        irr = clr / RESPONSIVITY
        if clr > 0:
            rr = ch["red"]   / clr
            gr = ch["green"] / clr
            br = ch["blue"]  / clr
        else:
            rr = gr = br = 0

        temp = read_temp_c()

        if n == 1:
            print("--- ToF (median of %d @ %dms budget) ---" % (TOF_SAMPLES, TOF_BUDGET_US // 1000))
            print("  " + "  |  ".join(tof_parts))
            print("--- Color (mean of %d passes) ---" % COLOR_PASSES)
            print("  irr: %.1f uW/cm2  R:%.3f G:%.3f B:%.3f" % (irr, rr, gr, br))
            print("  raw: clr=%.0f R=%.0f G=%.0f B=%.0f" % (clr, ch["red"], ch["green"], ch["blue"]))
            print("--- Temp ---")
            print("  %.1f C" % temp if temp else "  ERR")
        else:
            t_str = "%.1fC" % temp if temp else "ERR"
            print("[%3d] %s  | %.1fuW/cm2 R:%.2f G:%.2f B:%.2f | %s" % (
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