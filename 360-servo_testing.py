# servo_test.py
# Standalone, on-demand servo test (independent of hipe.py).
# move("jaw"/"lock", pct, duration_ms) drives that servo, waits, then stops.
# pct: -100..+100, mapped onto the full 700-2300us pulse range around the
# 1500us stop point. Positive = CCW (us>1500), negative = CW (us<1500),
# per FS90R datasheet. Magnitude = "signal strength" (further from neutral).

from machine import Pin, PWM
import time

FREQ_HZ   = 50
PERIOD_US = 1_000_000 // FREQ_HZ   # 20000 us
STOP_US   = 1500
US_MIN    = 700
US_MAX    = 2300
DEADBAND  = 45                      # +/- us around STOP_US: no motion inside this

def us_to_duty(us):
    return int(us * 65535 // PERIOD_US)

class ContServo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(FREQ_HZ)
        self.stop()

    def stop(self):
        self.pwm.duty_u16(us_to_duty(STOP_US))

    def set_pct(self, pct):
        pct = max(-100, min(100, pct))
        if pct >= 0:
            us = STOP_US + (pct / 100) * (US_MAX - STOP_US)
        else:
            us = STOP_US + (pct / 100) * (STOP_US - US_MIN)
        self.pwm.duty_u16(us_to_duty(int(us)))

    def run(self, pct, duration_ms):
        self.set_pct(pct)
        time.sleep_ms(duration_ms)
        self.stop()

servos = {
    "jaw":  ContServo(7),   # GP7
    "lock": ContServo(8),   # GP8
}

def move(target, pct, duration_ms):
    """REPL command, e.g.: move("jaw", 30, 200)"""
    if target not in servos:
        print("Unknown target:", target, "- use 'jaw' or 'lock'")
        return
    direction = "CCW" if pct > 0 else ("CW" if pct < 0 else "STOP")
    print("[%s] pct=%+d (%s) for %dms" % (target, pct, direction, duration_ms))
    servos[target].run(pct, duration_ms)
    print("[%s] stopped" % target)