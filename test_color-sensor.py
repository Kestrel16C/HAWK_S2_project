from machine import Pin, ADC
import time
import math

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
# from TCS3200 datasheet p.7, clear channel, Hz/(µW/cm²)
RESPONSIVITY = 150

# ---- NTC thermistor on ADC0 (GP26) ----
# SSR-1016: 10K @ 25°C, B25/50 = 3950
# Voltage divider: 3.3V -> 10K fixed -> ADC0 -> NTC -> GND
# So V_adc = 3.3 * R_ntc / (R_ntc + R_fixed)
#    R_ntc = R_fixed * V_adc / (3.3 - V_adc)
ntc_adc = ADC(Pin(26))
R_FIXED = 10000       # 10K fixed resistor (Ohm)
R0      = 10000       # NTC resistance at 25°C (Ohm), datasheet
B       = 3950        # B25/50 constant, datasheet
T0      = 298.15      # 25°C in Kelvin

def read_temp_c():
    raw = ntc_adc.read_u16()          # 0-65535
    v_adc = raw / 65535 * 3.3         # voltage at divider junction
    if v_adc >= 3.29 or v_adc <= 0.01:
        return None                   # out of range
    r_ntc = R_FIXED * v_adc / (3.3 - v_adc)
    # B-parameter equation (from datasheet B25/50 = 3950)
    t_kelvin = 1.0 / (1.0 / T0 + (1.0 / B) * math.log(r_ntc / R0))
    return t_kelvin - 273.15

# ---- TCS3200 helpers ----
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

def read_all():
    result = {}
    for name, (s2v, s3v) in FILTERS.items():
        s2.value(s2v)
        s3.value(s3v)
        time.sleep_ms(20)
        result[name] = read_freq(10)
    return result

# ---- main loop ----
print("Sensor test | TCS3200 + NTC on ADC0")
print()

while True:
    # Color + irradiance
    ch = read_all()
    brightness = ch["clear"]
    irradiance = brightness / RESPONSIVITY

    if brightness > 0:
        r = ch["red"]   / brightness
        g = ch["green"] / brightness
        b = ch["blue"]  / brightness
    else:
        r = g = b = 0

    # Temperature
    temp = read_temp_c()
    temp_str = "{:.1f}°C".format(temp) if temp is not None else "ERR"

    print("{:5.1f} uW/cm2  R:{:.2f} G:{:.2f} B:{:.2f}  {}".format(
        irradiance, r, g, b, temp_str))
    time.sleep(1)