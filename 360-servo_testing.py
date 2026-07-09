# servo_test.py
# Standalone, on-demand servo test (independent of hipe.py).
# jaw: locked-in open/close function (calibrated values below).
# lock: still under test via move("lock", pct, duration_ms).

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

# #############################################################################
# ##  LOCKED-IN JAW CALIBRATION — confirmed working with/without ball load,  ##
# ##  no stalling/buzzing, clean "clonk" on close.                           ##
# #############################################################################
JAW_OPEN_PCT    = +40
JAW_OPEN_MS     = 1000
JAW_CLOSE_PCT   = -35
JAW_CLOSE_MS    = 1200
# #############################################################################

def jaw_open():
    print("[jaw] OPEN  pct=%+d for %dms" % (JAW_OPEN_PCT, JAW_OPEN_MS))
    servos["jaw"].run(JAW_OPEN_PCT, JAW_OPEN_MS)
    print("[jaw] stopped")

def jaw_close():
    print("[jaw] CLOSE pct=%+d for %dms" % (JAW_CLOSE_PCT, JAW_CLOSE_MS))
    servos["jaw"].run(JAW_CLOSE_PCT, JAW_CLOSE_MS)
    print("[jaw] stopped")

def move(target, pct, duration_ms):
    """Free-form REPL command for the still-untested lock servo,
    e.g.: move("lock", 40, 300)"""
    if target not in servos:
        print("Unknown target:", target, "- use 'jaw' or 'lock'")
        return
    direction = "CCW" if pct > 0 else ("CW" if pct < 0 else "STOP")
    print("[%s] pct=%+d (%s) for %dms" % (target, pct, direction, duration_ms))
    servos[target].run(pct, duration_ms)
    print("[%s] stopped" % target)