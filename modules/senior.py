# modules/senior.py
from machine import Pin, PWM
import time

SENIOR_REV = "2026-07-14b"
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
# ##  AUTOPILOT — Korridor-Navigation (nur Seiten-ToF; Front = Notbremse)   ##
# ##  SEEK: geradeaus bis beide Seiten < Korridor. CRUISE: parallel fahren  ##
# ##  über Delta-Vergleich alter/neuer Seitenwerte. Sprung einer Seite      ##
# ##  nach oben = Abzweig -> zeitbegrenzte 90°-Kurve, dann wieder SEEK.     ##
# ##  Lenkung wird in JEDER Messphase im Stand zentriert -> keine Kreise.   ##
# #############################################################################
NAV_DRIVE_MS     = 1000   # Dauer Fahrzyklus
NAV_SETTLE_MS    = 800    # Stillstand: Mediane auffrischen + Servo zentrieren
NAV_CORRIDOR_MM  = 400    # beide Seiten darunter = im Korridor
NAV_DRIFT_MM     = 25     # Annäherung/Zyklus -> Gegenlenken (ein Zyklus)
NAV_ALIGN_STEER  = 12     # Lenk-% Parallel-Korrektur (klein!)
NAV_SIDE_MIN     = 180    # absolute Nähe-Grenze -> Gegenlenken
NAV_JUMP_MM      = 250    # Seiten-Sprung nach oben = Abzweig erkannt
NAV_TURN_STEER   = 60     # Lenk-% während der Kurve
NAV_TURN_MS      = 1500   # Kurvendauer (Lenkung wird DANACH zentriert)
NAV_TURN_PCT     = 30     # Antrieb während der Kurve
NAV_FAR          = 9999
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
        self._nav_prev = None        # (L, R) der letzten gültigen Messung
        self.nav_drive_pct = 30      # UI-einstellbar via nav_power

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
                
        elif command == "nav_power":
            try:
                self.nav_drive_pct = max(10, min(50, int(float(data))))
                print("[NAV] Fahrleistung:", self.nav_drive_pct)
            except (ValueError, TypeError):
                pass

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

        # --- Aktionsphase läuft ---
        if self._nav_state in ("DRIVE", "TURN"):
            dur = NAV_TURN_MS if self._nav_state == "TURN" else NAV_DRIVE_MS
            pct = NAV_TURN_PCT if self._nav_state == "TURN" else self.nav_drive_pct
            if time.ticks_diff(now, self._nav_t0) < dur:
                return pct, self._nav_steer
            if self._nav_state == "TURN":
                self._nav_prev = None      # alte Seitenwerte nach Kurve ungültig
            self._nav_state = "MEASURE"
            self._nav_t0 = now
            return 0, 0.0                  # Stillstand: Servo zentriert JETZT

        # --- MEASURE: stehen, zentrieren, Mediane auffrischen ---
        if time.ticks_diff(now, self._nav_t0) < NAV_SETTLE_MS:
            return 0, 0.0

        v = self.hipe._tof_vals
        L = v["left"]  if v["left"]  is not None else NAV_FAR
        R = v["right"] if v["right"] is not None else NAV_FAR

        steer = 0.0
        action = "DRIVE"
        in_corridor = (L < NAV_CORRIDOR_MM and R < NAV_CORRIDOR_MM)

        if self._nav_prev is not None:
            pL, pR = self._nav_prev
            was_corridor = (pL < NAV_CORRIDOR_MM and pR < NAV_CORRIDOR_MM)

            # Abzweig: eine Seite springt aus dem Korridor nach oben
            if was_corridor and (L - pL) > NAV_JUMP_MM and L >= NAV_CORRIDOR_MM:
                action, steer = "TURN", -NAV_TURN_STEER     # links öffnet -> links
            elif was_corridor and (R - pR) > NAV_JUMP_MM and R >= NAV_CORRIDOR_MM:
                action, steer = "TURN", NAV_TURN_STEER      # rechts öffnet -> rechts
            elif in_corridor:
                # Parallel-Korrektur: Annäherungs-Delta ODER absolute Nähe.
                # Korrektur gilt EINEN Fahrzyklus, danach wieder zentriert.
                if L < NAV_SIDE_MIN or (pL - L) > NAV_DRIFT_MM:
                    steer = NAV_ALIGN_STEER                 # weg von links
                elif R < NAV_SIDE_MIN or (pR - R) > NAV_DRIFT_MM:
                    steer = -NAV_ALIGN_STEER                # weg von rechts
            # sonst: SEEK -> geradeaus (steer 0)

        self._nav_prev = (L, R)
        self._nav_steer = float(steer)
        self._nav_state = action
        self._nav_t0 = now
        print("[NAV] L=%s R=%s korridor=%s -> %s steer=%.0f"
              % (L, R, in_corridor, action, steer))
        return (NAV_TURN_PCT if action == "TURN" else self.nav_drive_pct,
                self._nav_steer)