# modules/senior.py
from machine import Pin, PWM
import time

SENIOR_REV = "2026-07-14a"
print("senior.py Revision:", SENIOR_REV)

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
# ##  AUTOPILOT — Bounce-Navigation (alle drei ToF, Fahr-/Messzyklen)        ##
# ##  Prinzip: n/a = freie Richtung. Nur VON Hindernissen weg lenken.        ##
# #############################################################################
NAV_DRIVE_PCT   = 33     # Leistung Fahrzyklus (%)
NAV_DRIVE_MS    = 1000   # Dauer Fahrzyklus
NAV_TURN_MS     = 800    # Dauer Kurvenzyklus
NAV_SETTLE_MS   = 400    # Ausrollen vor Messung
NAV_FRONT_TURN  = 320    # Front darunter -> Kurve zur offeneren Seite
NAV_FRONT_BACK  = 200    # Front darunter -> zurücksetzen
NAV_SIDE_MIN    = 200    # Seite darunter -> weglenken
NAV_SIDE_BACK   = 130    # Seite darunter -> zurücksetzen
NAV_BACK_MS     = 800    # Dauer Rückwärtszyklus
NAV_BACK_PCT    = 35
NAV_STEER_TURN  = 30     # Lenk-% Kurve
NAV_STEER_NUDGE = 5     # Lenk-% Seitenkorrektur
NAV_DRIFT_MM    = 30     # Annäherung/Zyklus an nächster Wand -> vorhalten
NAV_FAR         = 9999   # Ersatzwert für n/a (= sicher/frei)
# #############################################################################

_FREQ_HZ   = 50
_PERIOD_US = 1_000_000 // _FREQ_HZ
_STOP_US   = 1500
_US_MIN    = 700
_US_MAX    = 2300


def _us_to_duty(us):
    return int(us * 65535 // _PERIOD_US)


class _ContServo:
    """Minimaler Treiber für FS90R-Dauerlaufservos."""

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
        self.hipe = None            # wird von hipe.py nach Konstruktion gesetzt

        self.jaw = None
        self.lock = None
        try:
            self.jaw = _ContServo(JAW_PIN)
        except Exception as e:
            print("Warnung: Jaw-Init fehlgeschlagen (GP%d):" % JAW_PIN, e)
        try:
            self.lock = _ContServo(LOCK_PIN)
        except Exception as e:
            print("Warnung: Lock-Init fehlgeschlagen (GP%d):" % LOCK_PIN, e)

        # Startzustand: Jaw geschlossen, Lock verriegelt (nicht bewegen!)
        self.jaw_open = False
        self.lock_engaged = True
        self.ball_retry_pending = 0

        # --- Autopilot-Zustand ---
        self._nav_state = "MEASURE"
        self._nav_t0 = 0
        self._nav_steer = 0.0
        self._nav_prev = {}

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

    def jaw_reclose(self):
        """Jaw-Nachschluss OHNE Statuswechsel (Ball-Retry)."""
        if self.jaw:
            print("[jaw] RECLOSE pct=%+d for %dms" % (JAW_CLOSE_PCT, JAW_CLOSE_MS))
            self.jaw.run(JAW_CLOSE_PCT, JAW_CLOSE_MS)

    # -------------------------------------------------------------------------
    # TEIL B: AUTOPILOT — Bounce. MEASURE (steht, misst) -> DRIVE/TURN/BACK
    # (zeitbegrenzt) -> MEASURE ... Richtungssymmetrisch, keine Farben.
    # -------------------------------------------------------------------------
    def run_autopilot(self, current_rpm):
        if self.hipe is None:
            return 0, 0
        now = time.ticks_ms()

        # --- laufende Aktionsphase zu Ende fahren ---
        if self._nav_state in ("DRIVE", "TURN", "BACK"):
            dur = {"DRIVE": NAV_DRIVE_MS, "TURN": NAV_TURN_MS,
                   "BACK": NAV_BACK_MS}[self._nav_state]
            if time.ticks_diff(now, self._nav_t0) < dur:
                spd = -NAV_BACK_PCT if self._nav_state == "BACK" else NAV_DRIVE_PCT
                return spd, self._nav_steer
            self._nav_state = "MEASURE"
            self._nav_t0 = now
            return 0, self._nav_steer

        # --- MEASURE: ausrollen, dann entscheiden ---
        if time.ticks_diff(now, self._nav_t0) < NAV_SETTLE_MS:
            return 0, self._nav_steer

        v = self.hipe._tof_vals
        F = v["front"] if v["front"] is not None else NAV_FAR
        L = v["left"]  if v["left"]  is not None else NAV_FAR
        R = v["right"] if v["right"] is not None else NAV_FAR
        open_sgn = 1 if R > L else -1   # +1 = rechts ist offener

        # 1) Kritisch nah -> gerade zurücksetzen
        if F < NAV_FRONT_BACK or L < NAV_SIDE_BACK or R < NAV_SIDE_BACK:
            action, steer = "BACK", 0.0
        # 2) Front zu -> Kurve zur offeneren Seite
        elif F < NAV_FRONT_TURN:
            action, steer = "TURN", open_sgn * NAV_STEER_TURN
        # 3) Seite zu nah -> weglenken
        elif L < NAV_SIDE_MIN:
            action, steer = "DRIVE", NAV_STEER_NUDGE
        elif R < NAV_SIDE_MIN:
            action, steer = "DRIVE", -NAV_STEER_NUDGE
        else:
            # 4) frei: geradeaus; Parallelitäts-Check — driftet die nähere
            #    Wand heran, früh gegenlenken
            steer = 0.0
            near_d, near_sgn = (L, 1) if L < R else (R, -1)
            prev = self._nav_prev.get("L" if near_sgn > 0 else "R", NAV_FAR)
            if near_d < 2 * NAV_SIDE_MIN and prev - near_d > NAV_DRIFT_MM:
                steer = near_sgn * NAV_STEER_NUDGE
            action = "DRIVE"

        self._nav_prev = {"L": L, "R": R}
        self._nav_steer = float(steer)
        self._nav_state = action
        self._nav_t0 = now
        print("[NAV] F=%s L=%s R=%s -> %s steer=%.0f" % (F, L, R, action, steer))
        return (-NAV_BACK_PCT if action == "BACK" else NAV_DRIVE_PCT,
                self._nav_steer)