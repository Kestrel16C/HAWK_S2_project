# modules/senior.py
from machine import Pin, PWM
import time

# #############################################################################
# ##  KALIBRIERTE SERVO-WERTE (aus servo_test.py übernommen)                 ##
# ##  Jaw:  bestätigt mit/ohne Ball-Last, sauberer Anschlag ("Clonk").       ##
# ##  Lock: bestätigte Richtungen und Laufzeiten.                            ##
# #############################################################################
JAW_PIN          = 7
JAW_OPEN_PCT     = +40
JAW_OPEN_MS      = 1000
JAW_CLOSE_PCT    = -35
JAW_CLOSE_MS     = 1200

LOCK_PIN         = 8
LOCK_ENGAGE_PCT  = -45
LOCK_ENGAGE_MS   = 350
LOCK_RELEASE_PCT = +45
LOCK_RELEASE_MS  = 325
# #############################################################################

# FS90R-Pulsparameter (Datenblatt)
_FREQ_HZ   = 50
_PERIOD_US = 1_000_000 // _FREQ_HZ
_STOP_US   = 1500
_US_MIN    = 700
_US_MAX    = 2300


def _us_to_duty(us):
    return int(us * 65535 // _PERIOD_US)


class _ContServo:
    """Minimaler Treiber für FS90R-Dauerlaufservos (aus servo_test.py)."""

    def __init__(self, pin):
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(_FREQ_HZ)
        self.stop()

    def stop(self):
        self.pwm.duty_u16(_us_to_duty(_STOP_US))

    def run(self, pct, duration_ms):
        pct = max(-100, min(100, pct))
        if pct >= 0:
            us = _STOP_US + (pct / 100) * (_US_MAX - _STOP_US)
        else:
            us = _STOP_US + (pct / 100) * (_STOP_US - _US_MIN)
        self.pwm.duty_u16(_us_to_duty(int(us)))
        time.sleep_ms(duration_ms)   # kurzes Blockieren: nur im Aux-Pfad OK
        self.stop()


class SeniorProject:
    def __init__(self):
        print("🎓 Projekt (Senior) wurde gestartet.")

        # --- HARDWARE: Greifer (Jaw) + Verriegelung (Lock) -------------------
        self.jaw = None
        self.lock = None
        try:
            self.jaw = _ContServo(JAW_PIN)
            self.lock = _ContServo(LOCK_PIN)
        except Exception as e:
            print("Warnung: Servo-Init fehlgeschlagen:", e)

        # Angenommener Startzustand: Jaw geschlossen, Lock verriegelt.
        # (Servos werden bewusst NICHT beim Boot bewegt.)
        self.jaw_open = False
        self.lock_engaged = True

        # --- Zustandsspeicher für Autopilot ----------------------------------
        self.state = "IDLE"
        self.timer_start = 0

    # -------------------------------------------------------------------------
    # TEIL A: MANUELLE STEUERUNG (Buttons aus dem Web-UI via /aux)
    # -------------------------------------------------------------------------
    def handle_aux(self, command, data):
        if command == "jaw" and self.jaw:
            if data == "open" and not self.jaw_open:
                print("[jaw] OPEN  pct=%+d for %dms" % (JAW_OPEN_PCT, JAW_OPEN_MS))
                self.jaw.run(JAW_OPEN_PCT, JAW_OPEN_MS)
                self.jaw_open = True
            elif data == "close" and self.jaw_open:
                print("[jaw] CLOSE pct=%+d for %dms" % (JAW_CLOSE_PCT, JAW_CLOSE_MS))
                self.jaw.run(JAW_CLOSE_PCT, JAW_CLOSE_MS)
                self.jaw_open = False

        elif command == "lock" and self.lock:
            if data == "release" and self.lock_engaged:
                print("[lock] RELEASE pct=%+d for %dms" % (LOCK_RELEASE_PCT, LOCK_RELEASE_MS))
                self.lock.run(LOCK_RELEASE_PCT, LOCK_RELEASE_MS)
                self.lock_engaged = False
            elif data == "engage" and not self.lock_engaged:
                print("[lock] ENGAGE pct=%+d for %dms" % (LOCK_ENGAGE_PCT, LOCK_ENGAGE_MS))
                self.lock.run(LOCK_ENGAGE_PCT, LOCK_ENGAGE_MS)
                self.lock_engaged = True

    # -------------------------------------------------------------------------
    # TEIL B: AUTONOMES FAHREN (Platzhalter, kommt später)
    # -------------------------------------------------------------------------
    def run_autopilot(self, current_rpm):
        """Muss speed (-100..100) und steer (-100..100) liefern. Nicht blockieren!"""
        return 0, 0