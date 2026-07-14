# modules/senior.py
from machine import Pin, PWM
import time

# #############################################################################
# ##  KALIBRIERTE SERVO-WERTE                                                ##
# #############################################################################
JAW_PIN          = 7
JAW_OPEN_PCT     = +40
JAW_OPEN_MS      = 1000
JAW_CLOSE_PCT    = -35
JAW_CLOSE_MS     = 1200
LOCK_PIN         = 8
LOCK_ENGAGE_PCT  = -45
LOCK_ENGAGE_MS   = 425
LOCK_RELEASE_PCT = +45
LOCK_RELEASE_MS  = 400
# #############################################################################

# #############################################################################
# ##  AUTOPILOT — Wandfolge-Parameter (alle Werte hier tunen!)               ##
# #############################################################################
NAV_DRIVE_PCT    = 35     # Antriebsleistung pro Fahrzyklus (%)
NAV_DRIVE_MS     = 1000   # Dauer eines Fahrzyklus
NAV_SETTLE_MS    = 500    # Stillstand vor Messung (ToF-Mediane auffrischen)
NAV_BAND_MM      = 250    # Ziel-Wandabstand
NAV_BAND_TOL     = 20     # +- Totband
NAV_CLOSE_MM     = 180    # näher als das -> starke Korrektur
NAV_LOST_MM      = 450    # Wand verloren -> Kurve einleiten
NAV_REACQ_MM     = 400    # Wand wiedergefunden -> Kurve beenden
NAV_STEER_SMALL  = 11     # % (~10 Grad)
NAV_STEER_BIG    = 33     # % (~30 Grad)
NAV_BLUE_CONFIRM = 2      # aufeinanderfolgende Blau-Messungen = angekommen
# #############################################################################

_FREQ_HZ, _PERIOD_US, _STOP_US, _US_MIN, _US_MAX = 50, 20000, 1500, 700, 2300


def _us_to_duty(us):
    return int(us * 65535 // _PERIOD_US)


class _ContServo:
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
        time.sleep_ms(duration_ms)
        self.stop()


class SeniorProject:
    def __init__(self):
        print("🎓 Projekt (Senior) wurde gestartet.")
        self.hipe = None            # wird von hipe.py gesetzt
        self.jaw = None
        self.lock = None
        try:
            self.jaw = _ContServo(JAW_PIN)
        except Exception as e:
            print("Warnung: Jaw-Init fehlgeschlagen:", e)
        try:
            self.lock = _ContServo(LOCK_PIN)
        except Exception as e:
            print("Warnung: Lock-Init fehlgeschlagen:", e)

        self.jaw_open = False
        self.lock_engaged = True
        self.ball_retry_pending = 0

        # --- Autopilot-Zustand ---
        self.nav_side = "right"     # "right" = Hinweg, "left" = Rückweg
        self._nav_state = "MEASURE" # MEASURE -> DRIVE -> MEASURE ...
        self._nav_t0 = 0
        self._nav_steer = 0.0
        self._nav_turning = False
        self._blue_count = 0

    # -------------------------------------------------------------------------
    def handle_aux(self, command, data):
        if command == "jaw" and self.jaw:
            if data == "open" and not self.jaw_open:
                print("[jaw] OPEN")
                self.jaw.run(JAW_OPEN_PCT, JAW_OPEN_MS)
                self.jaw_open = True
            elif data == "close" and self.jaw_open:
                print("[jaw] CLOSE")
                self.jaw.run(JAW_CLOSE_PCT, JAW_CLOSE_MS)
                self.jaw_open = False
        elif command == "lock" and self.lock:
            if data == "release" and self.lock_engaged:
                print("[lock] RELEASE")
                self.lock.run(LOCK_RELEASE_PCT, LOCK_RELEASE_MS)
                self.lock_engaged = False
            elif data == "engage" and not self.lock_engaged:
                print("[lock] ENGAGE")
                self.lock.run(LOCK_ENGAGE_PCT, LOCK_ENGAGE_MS)
                self.lock_engaged = True
        elif command == "nav_side":
            self.nav_side = "left" if data == "left" else "right"
            self._nav_state = "MEASURE"
            self._nav_turning = False
            self._blue_count = 0
            print("[NAV] Wandseite:", self.nav_side)

    def jaw_reclose(self):
        """Jaw-Nachschluss OHNE Statuswechsel (Ball-Retry)."""
        if self.jaw:
            print("[jaw] RECLOSE")
            self.jaw.run(JAW_CLOSE_PCT, JAW_CLOSE_MS)

    # -------------------------------------------------------------------------
    # AUTOPILOT: Wandfolge in Fahr-/Messzyklen. Wird pro Loop-Tick gerufen,
    # muss (speed, steer) liefern und darf NUR in der Messphase blockieren
    # (Farbmessung ~100ms — Fahrzeug steht dann).
    # -------------------------------------------------------------------------
    def run_autopilot(self, current_rpm):
        if self.hipe is None:
            return 0, 0
        now = time.ticks_ms()
        side = self.nav_side
        d = self.hipe._tof_vals.get(side)

        if self._nav_state == "DRIVE":
            if time.ticks_diff(now, self._nav_t0) < NAV_DRIVE_MS:
                return NAV_DRIVE_PCT, self._nav_steer
            self._nav_state = "MEASURE"
            self._nav_t0 = now
            return 0, self._nav_steer

        # --- MEASURE ---
        if time.ticks_diff(now, self._nav_t0) < NAV_SETTLE_MS:
            return 0, self._nav_steer   # ausrollen, Mediane auffrischen

        color = self.hipe.read_color_name()   # blockiert ~100ms im Stand

        # Ziel erreicht?
        if color == "blue":
            self._blue_count += 1
            if self._blue_count >= NAV_BLUE_CONFIRM:
                print("[NAV] BLAU bestätigt -> Ziel erreicht, AUTO aus.")
                self.hipe.mode = "MANUAL"
                self._nav_state = "MEASURE"
                return 0, 0
        else:
            self._blue_count = 0

        # Lenkentscheidung (positiv = rechts; bei Linksfolge gespiegelt)
        sgn = 1 if side == "right" else -1
        steer = 0.0
        if d is None or d > NAV_LOST_MM:
            # Wand verloren -> Kurve ZUR Wandseite
            self._nav_turning = True
            steer = sgn * NAV_STEER_BIG
            print("[NAV] Wand verloren (%s: %s) -> Kurve" % (side, d))
        elif self._nav_turning and d <= NAV_REACQ_MM:
            self._nav_turning = False
            steer = 0.0
            print("[NAV] Wand wiedergefunden (%s: %d)" % (side, d))
        elif self._nav_turning:
            steer = sgn * NAV_STEER_BIG
        elif color in ("yellow", "green"):
            # Farb-Eskalation: zu nah an einer Wandmarkierung -> hart weg
            steer = -sgn * NAV_STEER_BIG
            print("[NAV] Farbwarnung %s -> weg von der Wand" % color)
        elif d < NAV_CLOSE_MM:
            steer = -sgn * NAV_STEER_BIG
        elif d < NAV_BAND_MM - NAV_BAND_TOL:
            steer = -sgn * NAV_STEER_SMALL
        elif d > NAV_BAND_MM + NAV_BAND_TOL:
            steer = sgn * NAV_STEER_SMALL
        # sonst: im Band -> geradeaus (steer 0)

        self._nav_steer = steer
        self._nav_state = "DRIVE"
        self._nav_t0 = now
        print("[NAV] %s=%s F=%s Farbe=%s -> steer %.0f%%"
              % (side, d, self.hipe._tof_vals.get("front"), color, steer))
        return NAV_DRIVE_PCT, steer