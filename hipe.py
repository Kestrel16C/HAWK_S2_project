# hipe.py
# MIT License
# Copyright (c) 2025
# Tobias Bürmann, HAWK – Hochschule für angewandte Wissenschaft und Kunst
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# -----------------------------------------------------------------------------
# HAUPT-ORCHESTRATOR
# -----------------------------------------------------------------------------
# Start aus der REPL:
#   >>> from hipe import hipe
#   >>> h = hipe("Hier WLAN-Passwort angeben")
#   >>> h.run()
# Beenden: Strg+C in der REPL.
# -----------------------------------------------------------------------------

import time

HIPE_REV = "2026-07-12a"   # Bei JEDER Änderung hochzählen!
print("hipe.py Revision:", HIPE_REV)

# --- Import der Teilmodule ----------------------------------------------------
from modules.led import LedBlinker
from modules.net import NetworkManager
from modules.webserver import WebServer

from secure.drive import DriveController          # Motorsteuerung inkl. Encoder
from modules.steering import Steering             # Servo-Lenkung (offen)
from secure.current_monitor import CurrentMonitor # Zweikanal-Strommessung
from secure.safety import SafetyManager           # Sicherheitslogik
from secure.crash_counter import CrashCounter     # „Deathmatch"-Lebenszähler

# --- ToF-Sensoren (Kollisionsvermeidung) --------------------------------------
try:
    from machine import I2C, Pin as _Pin
    from modules.vl53l0x import VL53L0X as _VL53L0X
    _TOF_HW = True
except ImportError:
    print("VL53L0X-Treiber nicht gefunden – Kollisionsschutz deaktiviert.")
    _TOF_HW = False

# ProjektSenior
try:
    from modules.senior import SeniorProject
    SENIOR_AVAILABLE = True
except ImportError:
    print("Kein Senior-Projekt gefunden (modules/senior.py fehlt).")
    SENIOR_AVAILABLE = False


class hipe:
    """Zentrale, nicht-blockierende Mainloop für Fahrzeug, Telemetrie und Web-UI."""

    # -------------------------------------------------------------------------
    # INITIALISIERUNG
    # -------------------------------------------------------------------------
    def __init__(self, wifi_password: str) -> None:

        # --- Zeit/Loop-Parameter ---------------------------------------------
        self.loop_hz = 100
        self.dt_ms = max(1, int(1000 // max(1, int(self.loop_hz))))
        self._last_adc = 0
        self._cur_cache = None
        self.HEARTBEAT_TIMEOUT_MS = 800
        self._last_heartbeat = time.ticks_ms()

        # --- Zielwerte (aus der Web-UI) --------------------------------------
        self._target_speed = 0.0
        self._target_steer = 0.0
        self._safe_speed   = 0

        # --- LED, Netzwerk, Webserver-Basis ----------------------------------
        self.led = LedBlinker()
        self.web_root = "/www"
        self.port = 80
        self.net = NetworkManager(country="DE")
        self.wifi_password = wifi_password

        # --- Zustandsautomat ---------------------------------------------------
        # MANUAL: Web-UI steuert direkt. AUTO: Autopilot übernimmt.
        self.mode = "MANUAL"

        # ---------------------------------------------------------------------
        # HARDWARE: ANTRIEB (MOTOR + ENCODER)
        # ---------------------------------------------------------------------
        self.kin = {
            "pulses_per_rev": 16,    # Flanken A-rising pro Motorumdrehung
            "gear_ratio": 6.3,       # Getriebe Motorwelle:Ausgangswelle
            "wheel_diameter": 0.02,  # Raddurchmesser in m
            "invert_dir": False,
        }
        self.drive = DriveController(**self.kin)

        # --- Odometrie (RPM-Integration) --------------------------------------
        self._dist_m = 0.0
        self._dist_last_ms = time.ticks_ms()
        self._odo_dir = 1          # zuletzt kommandierte Richtung (+1/-1)
        self._last_rpm = 0.0

        # #####################################################################
        # ##  FAHRPROFIL drive_dist — Launch-Boost bricht die Haftreibung,   ##
        # ##  danach Cruise-Tempo, das der ToF-Schutz sicher stoppen kann.   ##
        # #####################################################################
        self.MAN_BOOST_PCT      = 80     # Anfahr-Boost (%)
        self.MAN_BOOST_MS       = 100    # Boost-Dauer ab Manöverstart (ms)
        self.MAN_SPEED_PCT      = 40     # Cruise-Geschwindigkeit (%)
        self.MAN_SPEED_SLOW_PCT = 30     # Kriechgang kurz vor dem Ziel (%)
        self.MAN_SLOW_ZONE_M    = 0.10   # Kriechgang-Zone vor dem Ziel (m)
        # #####################################################################
        self.MAN_TIMEOUT_MS     = 15000  # Abbruch, falls Ziel nicht erreicht
        self.MAN_COAST_MAX_MS   = 2000   # max. Wartezeit auf Stillstand

        # #####################################################################
        # ##  INCH — kurzer Vollgas-Impuls zum Heranrücken an den Ball       ##
        # #####################################################################
        self.INCH_PCT = 100    # Impuls-Leistung (%)
        self.INCH_MS  = 80     # Impuls-Dauer (ms)  <-- ggf. eigenen Wert eintragen
        # #####################################################################
        self._inch_until = 0

        # --- Manöver-Zustandsvariablen (Zustandsmaschine drive_dist) ---------
        self._man_active   = False
        self._man_state    = "RUN"       # "RUN" | "COAST"
        self._man_ref_m    = 0.0         # Odometer-Stand bei Manöverstart
        self._man_target_m = 0.0         # Soll-Distanz (Betrag, m)
        self._man_dir      = 1           # +1 vorwärts, -1 rückwärts
        self._man_start_ms = 0
        self._man_coast_ms = 0

        # ---------------------------------------------------------------------
        # HARDWARE: LENKUNG (SERVO)
        # ---------------------------------------------------------------------
        self.steering = Steering(
            pin=6,
            pwm_freq_hz=50,
            min_us=560, max_us=2440,    # ~85° pro Seite (kalibriert)
            center_us=1500, deadband_us=10,
            angle_min=-90, angle_max=90,
            trim_deg=0,
            invert=False,
        )
        self.steering.center()  # Nach Start in Mittelstellung

        # ---------------------------------------------------------------------
        # HARDWARE: STROMMESSUNG (2 Kanäle)
        # ---------------------------------------------------------------------
        self.current = CurrentMonitor()
        self.current.start()

        # ---------------------------------------------------------------------
        # SICHERHEIT
        # ---------------------------------------------------------------------
        self.safety = SafetyManager()

        # ---------------------------------------------------------------------
        # „DEATHMATCH"-MODUS
        # ---------------------------------------------------------------------
        self.deathmatch_enabled = False
        self.crash = CrashCounter()

        # ---------------------------------------------------------------------
        # NETZ & WEB
        # ---------------------------------------------------------------------
        try:
            ap_ip = self.net.start_ap(password=self.wifi_password, channel=None)
            print("SSID =", getattr(self.net, "ap_ssid", "<unknown>"))
            print("AP aktiv: IP =", ap_ip)
            self.led.set_pattern("fast")
        except (OSError, RuntimeError, ValueError) as e:
            print("AP start fehlgeschlagen:", e)
            self.led.set_pattern("off")

        try:
            self.web = WebServer(
                port=self.port,
                web_root=self.web_root,
                on_control=self.on_control,
                on_aux=self.on_aux_command,
                get_telemetry=self.get_telemetry,
                on_heartbeat=self.on_heartbeat,
                safety=self.safety,
                steering=self.steering,
                current=self.current,
            )
            self.web.setup_server()
            print("HTTP bereit auf:", self.net.ip() or "0.0.0.0", "Port", self.port)
        except (OSError, RuntimeError, ValueError) as e:
            print("HTTP-Setup fehlgeschlagen:", e)
            raise

        # #####################################################################
        # ##  KOLLISIONSSCHUTZ — ToF-Abstandsschwellen (mm, KORRIGIERT)      ##
        # ##  Sofortiger Stopp der VORWÄRTSfahrt bei Unterschreitung.        ##
        # ##  Rückwärtsfahrt und Lenkung bleiben IMMER erlaubt.              ##
        # ##  Front-Schwelle ist bei geöffnetem Greifer deaktiviert.         ##
        # #####################################################################
        self.TOF_STOP_FRONT_MM   = 150   # 15 cm — Frontsensor
        self.TOF_STOP_SIDE_MM    = 100   # 15 cm — Seitensensoren (L/R)
        self.TOF_RELEASE_HYST_MM = 30    # Freigabe erst ab Schwelle + 30 mm
        # #####################################################################

        # Korrekturfaktoren aus der Kalibrierfahrt: real = (raw - offset) / slope
        self._tof_corr = {
            "left":  {"offset": 48.5, "slope": 1.00},
            "right": {"offset": 20.0, "slope": 1.05},
            "front": {"offset": 19.5, "slope": 1.05},
        }

        # Rollende Median-Puffer (5 Werte) + aktueller Median je Sensor
        self._tof_bufs = {"front": [], "left": [], "right": []}
        self._tof_vals = {"front": None, "left": None, "right": None}
        self._tof_blocked = False

        # Hardware-Init: Sensoren in CONTINUOUS-Modus (messen selbstständig,
        # Loop pollt nur Data-Ready -> keine blockierenden Waits im Takt).
        self._tof_sensors = {}
        self._tof_i2c = None
        self._tof_addr = {}
        if _TOF_HW:
            self._tof_init()

        # Für ruhigeres Logging nur bei Änderungen ausgeben
        self._last_out = {"safe": None, "safety": None}

        # ---------------------------------------------------------------------
        # PROJEKT-MODUL (SENIOR)
        # ---------------------------------------------------------------------
        self.senior = None
        if SENIOR_AVAILABLE:
            try:
                self.senior = SeniorProject()
                print("Senior-Projekt erfolgreich geladen.")
            except Exception as e:
                print("Fehler im Senior-Projekt Init:", e)

    # -------------------------------------------------------------------------
    # CALLBACKS AUS DEM WEBSERVER
    # -------------------------------------------------------------------------

    def on_control(self, spd, st) -> None:
        """Web-Callback: neue Zielwerte setzen (Speed/Steer)."""
        if self._man_active:
            self._maneuver_cancel("manuelle Eingabe")

        if spd > 100:
            spd = 100
        if spd < -100:
            spd = -100
        if st > 100:
            st = 100
        if st < -100:
            st = -100

        self._target_speed = float(spd)
        self._target_steer = float(st)
        self._last_heartbeat = time.ticks_ms()

    # -------------------------------------------------------------------------
    # Handler für Zusatz-Befehle (Setup & Erweiterungen)
    # -------------------------------------------------------------------------
    def on_aux_command(self, type, data) -> None:
        """Verarbeitet Zusatzbefehle vom Webserver (/aux?type=...&data=...)."""
        print(f"[AUX] Type: {type} | Data: {data}")

        # --- A: FAHRWERK SETUP (Lenkung) ---
        if type == "steer_config":
            try:
                parts = data.split(",")
                if len(parts) == 3:
                    if self.steering:
                        self.steering.angle_min = int(parts[0])
                        self.steering.angle_max = int(parts[1])
                        self.steering.trim_deg = int(parts[2])
                        print("-> Lenkungskonfiguration aktualisiert.")
            except Exception as e:
                print(f"-> Fehler bei steer_config: {e}")

        # --- B: MODUS WECHSEL (Autopilot) ---
        elif type == "mode":
            if data == "auto":
                self._maneuver_cancel("Moduswechsel AUTO")
                self.mode = "AUTO"
                print("-> Modus: AUTONOM")
            else:
                self.mode = "MANUAL"
                self._target_speed = 0
                print("-> Modus: MANUELL")

        # --- H: LENKWINKEL DISKRET SETZEN ---
        elif type == "steer_angle":
            try:
                deg = float(data)
                self._target_steer = max(-100.0, min(100.0, (deg / 90.0) * 100.0))
                self._last_heartbeat = time.ticks_ms()
                print("-> Lenkwinkel: %.0f°" % deg)
            except (ValueError, TypeError):
                print("-> steer_angle: ungültiger Wert:", data)

        # --- D: DISTANZ-MANÖVER ---
        elif type == "drive_dist":
            try:
                dist = float(data)
            except (ValueError, TypeError):
                print("-> drive_dist: ungültige Distanz:", data)
                return
            if dist == 0:
                print("-> drive_dist: Distanz 0 ignoriert.")
                return
            if self.mode != "MANUAL":
                print("-> drive_dist: nur im MANUAL-Modus möglich.")
                return
            self._maneuver_start(abs(dist), 1 if dist > 0 else -1)

        # --- E: INCH (kurzer Vorwärts-Impuls, Ballannäherung) ---
        elif type == "inch":
            if self._man_active:
                print("-> inch: Manöver läuft, ignoriert.")
                return
            self._inch_until = time.ticks_add(time.ticks_ms(), self.INCH_MS)
            print("-> Inch: %d%% für %d ms." % (self.INCH_PCT, self.INCH_MS))

        # --- F: KILL-SWITCH ---
        elif type == "kill":
            self._maneuver_cancel("Kill-Switch")
            self.mode = "MANUAL"
            self._target_speed = 0.0
            self._inch_until = 0
            print("-> KILL: Antrieb gestoppt, Modus MANUELL.")

        # --- G: ODOMETER ZURÜCKSETZEN ---
        elif type == "dist_reset":
            self.reset_distance()
            print("-> Odometer auf 0 gesetzt.")

        # --- C: GREIFER, LOCK & TRIGGER (Weiterleitung an Senior) ---
        elif type in ("arm", "trigger", "jaw", "lock"):
            # Servo-Aktionen blockieren die Loop kurz (bis ~1.2s) —
            # vorher den Antrieb sicher stoppen.
            self._target_speed = 0.0
            try:
                if hasattr(self.drive, "set_percent"):
                    self.drive.set_percent(0)
                else:
                    self.drive.set_speed_percent(0)
            except Exception as e:
                print("Antriebsstopp vor Servo-Aktion fehlgeschlagen:", e)
            if self.senior:
                try:
                    self.senior.handle_aux(type, data)
                except Exception as e:
                    print("Fehler im Senior-Aux:", e)
            else:
                print("Senior-Modul nicht aktiv.")

    def on_heartbeat(self) -> None:
        """Web-Callback: Heartbeat für Dead-Man aktualisieren."""
        self._last_heartbeat = time.ticks_ms()
        try:
            self.safety.touch_command(self._last_heartbeat)
        except AttributeError:
            pass

    # -------------------------------------------------------------------------
    # ODOMETRIE
    # -------------------------------------------------------------------------

    def get_distance_m(self) -> float:
        """Zurückgelegte Strecke in m seit Start bzw. letztem Reset."""
        return self._dist_m

    def reset_distance(self) -> None:
        """Odometer nullen (Kalibrierfahrt / Trial-Start)."""
        self._dist_m = 0.0
        self._dist_last_ms = time.ticks_ms()

    def _update_odometry(self, rpm, safe_pct, now) -> None:
        """Integriert die Drehzahl über die Loop-Zeit zu einer Strecke.

        Richtung wird gemerkt: bei safe_pct == 0 (Auslauf) zählt die zuletzt
        kommandierte Richtung weiter.
        """
        dt_odo = time.ticks_diff(now, self._dist_last_ms)
        self._dist_last_ms = now

        if safe_pct > 0:
            self._odo_dir = 1
        elif safe_pct < 0:
            self._odo_dir = -1

        if not (0 < dt_odo < 500):
            return

        step = (abs(rpm) / 60.0) * (dt_odo / 1000.0) \
               * 3.141592653589793 * self.kin["wheel_diameter"]
        self._dist_m += self._odo_dir * step

    # -------------------------------------------------------------------------
    # KOLLISIONSSCHUTZ (ToF, Continuous-Modus, nicht-blockierend)
    # -------------------------------------------------------------------------

    def _tof_init(self):
        """Adresstanz + Init + Start des Continuous-Modus (einmal beim Boot)."""
        try:
            xshut_r = _Pin(14, _Pin.OUT)
            xshut_l = _Pin(15, _Pin.OUT)
            xshut_r.value(0)
            xshut_l.value(0)
            time.sleep_ms(50)

            i2c = I2C(0, scl=_Pin(5), sda=_Pin(4), freq=100000)
            self._tof_i2c = i2c

            ADDR_DEF, ADDR_FRONT, ADDR_RIGHT = 0x29, 0x2A, 0x2B

            # Front (kein XSHUT) -> 0x2A
            try:
                i2c.writeto_mem(ADDR_DEF, 0x8A, bytes([ADDR_FRONT]))
            except OSError as e:
                print("[ToF] Front Adresswechsel fehlgeschlagen:", e)
            time.sleep_ms(10)

            # Right (XSHUT GP14) -> 0x2B
            xshut_r.value(1)
            time.sleep_ms(50)
            try:
                i2c.writeto_mem(ADDR_DEF, 0x8A, bytes([ADDR_RIGHT]))
            except OSError as e:
                print("[ToF] Right Adresswechsel fehlgeschlagen:", e)
            time.sleep_ms(10)

            # Left (XSHUT GP15) -> bleibt auf 0x29 (Default)
            xshut_l.value(1)
            time.sleep_ms(50)
            print("[ToF] Bus:", i2c.scan())

            for name, addr in (("front", ADDR_FRONT), ("right", ADDR_RIGHT), ("left", ADDR_DEF)):
                try:
                    tof = _VL53L0X(i2c, address=addr, io_timeout_ms=1000)
                    self._tof_sensors[name] = tof
                    self._tof_addr[name] = addr
                    print("[ToF] %s OK (0x%02X)" % (name, addr))
                except (OSError, RuntimeError) as e:
                    print("[ToF] %s FAILED (0x%02X): %s" % (name, addr, e))

            # 20ms-High-Speed-Budget setzen (falls der Treiber es anbietet)
            for tof in self._tof_sensors.values():
                fn = getattr(tof, "set_measurement_timing_budget", None)
                if callable(fn):
                    try:
                        fn(20000)
                    except Exception:
                        pass

            # Continuous-Modus starten
            for name, tof in self._tof_sensors.items():
                started = False
                fn = getattr(tof, "start_continuous", None) or getattr(tof, "start", None)
                if callable(fn):
                    try:
                        fn()
                        started = True
                    except Exception:
                        pass
                if not started:
                    try:
                        self._tof_i2c.writeto_mem(self._tof_addr[name], 0x00, b"\x02")
                        started = True
                    except OSError as e:
                        print("[ToF] %s Continuous-Start fehlgeschlagen: %s" % (name, e))
                if started:
                    print("[ToF] %s Continuous-Modus aktiv." % name)

        except Exception as e:
            print("[ToF] Init komplett fehlgeschlagen:", e)
            self._tof_sensors = {}

    def _tof_correct(self, name, raw_mm):
        c = self._tof_corr.get(name)
        return (raw_mm - c["offset"]) / c["slope"] if c else raw_mm

    @staticmethod
    def _median(vals):
        if not vals:
            return None
        s = sorted(vals)
        m = len(s) // 2
        return s[m] if len(s) % 2 else (s[m - 1] + s[m]) / 2

    def _tof_poll(self):
        """Nicht-blockierend: pro Sensor Data-Ready prüfen (1 Registerbyte);
        nur fertige Ergebnisse auslesen (~1-2ms). Es wird NIE gewartet."""
        for name, addr in self._tof_addr.items():
            try:
                ready = self._tof_i2c.readfrom_mem(addr, 0x13, 1)[0] & 0x07
                if not ready:
                    continue
                data = self._tof_i2c.readfrom_mem(addr, 0x14, 12)
                raw = (data[10] << 8) | data[11]
                self._tof_i2c.writeto_mem(addr, 0x0B, b"\x01")  # Interrupt löschen
            except OSError:
                continue

            # Sentinel (8190/8191 = kein Ziel) und implausible Werte verwerfen
            if raw >= 8190 or raw < 10:
                continue
            buf = self._tof_bufs[name]
            buf.append(self._tof_correct(name, raw))
            if len(buf) > 5:
                buf.pop(0)
            # Median erst ab 3 Werten als gültig betrachten
            if len(buf) >= 3:
                self._tof_vals[name] = self._median(buf)

    def _tof_check_proximity(self):
        """Setzt/löst self._tof_blocked anhand der Mediane, mit Hysterese."""
        jaw_open = bool(self.senior and getattr(self.senior, "jaw_open", False))

        margin = self.TOF_RELEASE_HYST_MM if self._tof_blocked else 0

        blocked = False
        for name, limit in (("left", self.TOF_STOP_SIDE_MM),
                            ("right", self.TOF_STOP_SIDE_MM),
                            ("front", self.TOF_STOP_FRONT_MM)):
            if name == "front" and jaw_open:
                continue
            val = self._tof_vals.get(name)
            if val is not None and val < (limit + margin):
                blocked = True
                break

        if blocked and not self._tof_blocked:
            print("[ToF] STOPP: Hindernis unter Schwelle (L:%s R:%s F:%s, Greifer %s)"
                  % (self._tof_vals["left"], self._tof_vals["right"],
                     self._tof_vals["front"], "offen" if jaw_open else "zu"))
        elif not blocked and self._tof_blocked:
            print("[ToF] Freigabe: Abstand wieder ausreichend.")

        self._tof_blocked = blocked

    def _tof_presweep_cap(self, target_m):
        """Blockierender Vor-Check (max ~0.7s): Front-Median frisch aufbauen
        und das Vorwärts-Ziel auf die freie Strecke begrenzen.
        Rückwärtsfahrten haben KEINEN Sensor — dort gibt es keinen Cap."""
        if not self._tof_sensors:
            return target_m
        self._tof_bufs["front"] = []
        self._tof_vals["front"] = None
        deadline = time.ticks_add(time.ticks_ms(), 700)
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            self._tof_poll()
            if len(self._tof_bufs["front"]) >= 5:
                break
            time.sleep_ms(5)
        front = self._tof_vals.get("front")
        if front is None:
            print("[MAN] Vor-Check: kein Frontecho (freie Bahn oder Sensor inaktiv).")
            return target_m
        allowed = (front - self.TOF_STOP_FRONT_MM) / 1000.0
        if allowed <= 0:
            print("[MAN] Vor-Check: Front bereits unter Schwelle (%.0f mm) — Fahrt verweigert." % front)
            return 0.0
        if target_m > allowed:
            print("[MAN] Vor-Check: Ziel %.2f m auf %.2f m gekappt (Front %.0f mm)."
                  % (target_m, allowed, front))
            return allowed
        return target_m

    # -------------------------------------------------------------------------
    # DISTANZ-MANÖVER (drive_dist)
    # -------------------------------------------------------------------------

    def _maneuver_start(self, target_m, direction) -> None:
        """Startet ein Distanz-Manöver relativ zum aktuellen Odometer-Stand."""
        if direction > 0 and _TOF_HW:
            target_m = self._tof_presweep_cap(target_m)
            if target_m <= 0:
                return

        self._man_ref_m    = self._dist_m   # Referenz merken, NICHT resetten
        self._man_target_m = target_m
        self._man_dir      = direction
        self._man_start_ms = time.ticks_ms()
        self._man_state    = "RUN"
        self._man_active   = True
        print("[MAN] drive_dist Start: Soll=%.3f m, Richtung=%s, Odometer=%.3f m"
              % (target_m, "vor" if direction > 0 else "zurueck", self._dist_m))

    def _maneuver_cancel(self, reason) -> None:
        """Bricht ein laufendes Manöver ab und stoppt den Antrieb."""
        if not self._man_active:
            return
        self._man_active = False
        self._target_speed = 0.0
        delta = abs(self._dist_m - self._man_ref_m)
        print("[MAN] Abbruch (%s) bei %.3f von %.3f m."
              % (reason, delta, self._man_target_m))

    def _maneuver_tick(self, now) -> None:
        """Ein Schritt der Manöver-Zustandsmaschine (nicht-blockierend)."""
        delta = abs(self._dist_m - self._man_ref_m)

        if self._man_state == "RUN":
            remaining = self._man_target_m - delta

            # Ziel erreicht -> Motor aus, Auslauf beobachten
            if remaining <= 0:
                self._target_speed = 0.0
                self._man_state = "COAST"
                self._man_coast_ms = now
                print("[MAN] Abschaltpunkt: %.3f m nach %d ms."
                      % (delta, time.ticks_diff(now, self._man_start_ms)))
                return

            # Watchdog: Ziel nicht erreichbar (Blockade, Encoderausfall)
            if time.ticks_diff(now, self._man_start_ms) > self.MAN_TIMEOUT_MS:
                self._maneuver_cancel("Timeout")
                return

            # Geschwindigkeitsprofil: Boost -> Cruise -> Kriechgang
            if time.ticks_diff(now, self._man_start_ms) < self.MAN_BOOST_MS:
                pct = self.MAN_BOOST_PCT
            elif self._man_target_m <= self.MAN_SLOW_ZONE_M:
                pct = self.MAN_SPEED_PCT          # kurze Ziele: kein Kriechgang
            elif remaining <= self.MAN_SLOW_ZONE_M:
                pct = self.MAN_SPEED_SLOW_PCT
            else:
                pct = self.MAN_SPEED_PCT
            self._target_speed = float(self._man_dir * pct)
            # HINWEIS: Lenkung wird bewusst NICHT zentriert —
            # drive_dist fährt entlang des eingestellten Lenkwinkels (Bogen).
            self._last_heartbeat = now

        elif self._man_state == "COAST":
            self._target_speed = 0.0
            stopped = abs(self._last_rpm) < 1.0
            timed_out = time.ticks_diff(now, self._man_coast_ms) > self.MAN_COAST_MAX_MS
            if stopped or timed_out:
                self._man_active = False
                print("[MAN] Endstand nach Auslauf: %.3f m (Soll %.3f m, Auslauf %.3f m)."
                      % (delta, self._man_target_m, delta - self._man_target_m))

    # -------------------------------------------------------------------------
    # TELEMETRIE
    # -------------------------------------------------------------------------

    def get_telemetry(self) -> dict:
        """Erstellt Telemetrie-Daten für das Frontend."""
        try:
            rpm = self.drive.get_rpm()
        except (AttributeError, OSError, RuntimeError) as e:
            print("get rpm fehlgeschlagen:", e)
            rpm = None

        try:
            m = self._cur_cache or self.current.read(window_ms=200, want=("avg",))
            currents = {"motor": m["motor"]["avg"], "system": m["system"]["avg"]}
        except (KeyError, AttributeError, OSError, RuntimeError) as e:
            print("telemetry: current read failed:", e)
            currents = {"motor": None, "system": None}

        data = {
            "speed_target": self._target_speed,
            "steer_target": self._target_steer,
            "rpm": rpm,
            "current": currents,
            "safety": getattr(self.safety, "status", "OK"),
            "speed_safe": self._safe_speed,
            "distance": round(self._dist_m, 3),
            "tof": {
                "front": round(self._tof_vals["front"]) if self._tof_vals["front"] is not None else None,
                "left":  round(self._tof_vals["left"])  if self._tof_vals["left"]  is not None else None,
                "right": round(self._tof_vals["right"]) if self._tof_vals["right"] is not None else None,
            },
            "ts": time.ticks_ms(),
        }
        if self.deathmatch_enabled:
            try:
                data["lives"] = int(self.crash.count)
            except (AttributeError, ValueError):
                pass
        return data

    # -------------------------------------------------------------------------
    # HAUPTSCHLEIFE
    # -------------------------------------------------------------------------

    def run(self) -> None:
        """Nicht-blockierende Hauptschleife.

        Pro Tick: 1) Webserver, 2) Autopilot, 2b) Manöver, 2c) Inch,
        3) Strom, 4) Dead-Man, 5) Deathmatch, 6) Safety, 6b) Odometrie,
        6c) ToF-Schutz, 7) Hardware-Ausgabe, 8) Takt, 9) LED.
        """

        self.current.calibrate_zero()

        next_tick = time.ticks_ms()

        while True:
            # 1) Webserver (holt manuelle Eingaben)
            try:
                self.web.poll_once()
            except OSError as e:
                print("web.poll_once failed:", e)

            # 2) AUTOPILOT (SENIOR) - PRIORITÄT VOR HARDWARE
            if self.mode == "AUTO" and self.senior:
                try:
                    rpm = self.drive.get_rpm()
                    s_speed, s_steer = self.senior.run_autopilot(rpm)
                    self._target_speed = float(s_speed)
                    self._target_steer = float(s_steer)
                    self._last_heartbeat = time.ticks_ms()
                except Exception as e:
                    print("Crash im Senior-Autopilot:", e)
                    self.mode = "MANUAL"
                    self._target_speed = 0

            now = time.ticks_ms()

            # 2b) Distanz-Manöver (nur im MANUAL-Modus)
            if self._man_active and self.mode == "MANUAL":
                self._maneuver_tick(now)

            # 2c) Inch-Impuls (zeitbegrenzter Schub; ToF-Schutz greift normal)
            if self._inch_until:
                if time.ticks_diff(self._inch_until, now) > 0:
                    self._target_speed = float(self.INCH_PCT)
                    self._last_heartbeat = now
                else:
                    self._inch_until = 0
                    self._target_speed = 0.0

            # 3) Strommessung periodisch auswerten
            if time.ticks_diff(now, self._last_adc) >= 120:
                try:
                    self._cur_cache = self.current.read(window_ms=200, want=("avg",))
                except (OSError, RuntimeError, AttributeError) as e:
                    print("current.read failed:", e)
                    self._cur_cache = None
                self._last_adc = now

            # 4) Dead-Man
            if time.ticks_diff(now, self._last_heartbeat) > self.HEARTBEAT_TIMEOUT_MS:
                if self._target_speed != 0.0:
                    self._target_speed = 0.0

            # 5) Deathmatch
            if self.deathmatch_enabled:
                try:
                    self.crash.tick()
                    st = self.crash.get_status()
                    if st.get("new"):
                        print("[HI %6d] DM: lives=%d" % (now, st.get("lives", -1)))
                    if self.crash.is_dead():
                        if not self.safety.is_locked():
                            self.safety.set_external_lock(True, reason="DEAD")
                            print("[HI %6d] DM: DEAD → lock & stop" % now)
                        self._target_speed = 0.0
                    else:
                        if self.safety.is_locked():
                            self.safety.set_external_lock(False)
                except (AttributeError, RuntimeError) as e:
                    print("deathmatch tick failed:", e)

            # 6) Safety anwenden
            try:
                rpm_for_safety = self.drive.get_rpm()
            except (AttributeError, RuntimeError, OSError):
                rpm_for_safety = 0.0
            self._last_rpm = rpm_for_safety

            try:
                m = self._cur_cache or self.current.read(window_ms=200, want=("avg",))
                imotor = m["motor"]["avg"]
            except (KeyError, AttributeError, RuntimeError, OSError):
                imotor = None

            try:
                safe_pct, status = self.safety.enforce(int(self._target_speed),
                                                       rpm_for_safety, imotor, now_ms=now)
                self._safe_speed = int(safe_pct)
            except (ValueError, RuntimeError) as e:
                print("safety.enforce failed:", e)
                safe_pct, status = int(self._target_speed), "OK"

            # 6b) Odometrie mit der frisch gelesenen Drehzahl aktualisieren
            self._update_odometry(rpm_for_safety, safe_pct, now)

            # 6c) ToF-Kollisionsschutz (passiv, nicht-blockierend):
            #     sperrt NUR Vorwärts-PWM; rückwärts und Lenkung bleiben frei.
            if _TOF_HW and self._tof_sensors:
                self._tof_poll()
                self._tof_check_proximity()
                if self._tof_blocked:
                    if self._man_active and self._man_dir > 0:
                        self._maneuver_cancel("Kollisionsschutz")
                    if self.mode == "AUTO":
                        self.mode = "MANUAL"
                        self._target_speed = 0.0
                        print("[ToF] AUTO -> MANUAL (Kollisionsschutz).")
                    if safe_pct > 0:
                        safe_pct = 0
                        self._safe_speed = 0

            # ruhiger log
            if (self._last_out["safe"] != safe_pct) or (self._last_out["safety"] != status):
                self._last_out["safe"] = safe_pct
                self._last_out["safety"] = status

            # 7) Hardware ausgeben
            try:
                if hasattr(self.drive, "set_percent"):
                    self.drive.set_percent(safe_pct)
                else:
                    self.drive.set_speed_percent(safe_pct)
            except (AttributeError, ValueError, RuntimeError, OSError) as e:
                print("drive.set_percent failed:", e)

            try:
                if hasattr(self.steering, "set_percent"):
                    self.steering.set_percent(self._target_steer)
                else:
                    self.steering.set_angle_percent(self._target_steer)
            except (AttributeError, ValueError, RuntimeError, OSError) as e:
                print("steering.set_percent failed:", e)

            # 8) Takt einhalten
            next_tick = time.ticks_add(next_tick, self.dt_ms)
            delay = time.ticks_diff(next_tick, time.ticks_ms())
            if delay > 0:
                time.sleep_ms(delay)
            else:
                if delay < -5:
                    next_tick = time.ticks_ms()

            # 9) LED-Muster (fast = nur AP, slow = Client/Heartbeat aktiv)
            has_client = False
            try:
                stations = self.net.stations()
                if isinstance(stations, (list, tuple)) and len(stations) > 0:
                    has_client = True
            except (AttributeError, RuntimeError, OSError):
                pass

            if not has_client and time.ticks_diff(now, self._last_heartbeat) <= 5000:
                has_client = True

            pat = "slow" if has_client else "fast"
            if pat != getattr(self, "_led_pat", None):
                self._led_pat = pat
            self.led.set_pattern(pat)
            self.led.tick()