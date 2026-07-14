# hipe.py
# MIT License — Tobias Bürmann, HAWK (Header gekürzt, Lizenztext unverändert gültig)
# -----------------------------------------------------------------------------
# HAUPT-ORCHESTRATOR
# Start: >>> from hipe import hipe ; h = hipe("PASSWORT") ; h.run()
# -----------------------------------------------------------------------------

import time

HIPE_REV = "2026-07-14a"   # Bei JEDER Änderung hochzählen!
print("hipe.py Revision:", HIPE_REV)

from modules.led import LedBlinker
from modules.net import NetworkManager
from modules.webserver import WebServer

from secure.drive import DriveController
from modules.steering import Steering
from secure.current_monitor import CurrentMonitor
from secure.safety import SafetyManager
from secure.crash_counter import CrashCounter

# --- ToF-Sensoren (Kollisionsvermeidung) --------------------------------------
try:
    from machine import I2C, Pin as _Pin, ADC as _ADC
    from modules.vl53l0x import VL53L0X as _VL53L0X
    _TOF_HW = True
except ImportError:
    print("VL53L0X-Treiber nicht gefunden – Kollisionsschutz deaktiviert.")
    _TOF_HW = False

try:
    from modules.senior import SeniorProject
    SENIOR_AVAILABLE = True
except ImportError:
    print("Kein Senior-Projekt gefunden (modules/senior.py fehlt).")
    SENIOR_AVAILABLE = False


class hipe:
    """Zentrale, nicht-blockierende Mainloop für Fahrzeug, Telemetrie und Web-UI."""

    def __init__(self, wifi_password: str) -> None:

        # --- Zeit/Loop-Parameter ---------------------------------------------
        self.loop_hz = 100
        self.dt_ms = max(1, int(1000 // max(1, int(self.loop_hz))))
        self._last_adc = 0
        self._cur_cache = None
        self.HEARTBEAT_TIMEOUT_MS = 800
        self._last_heartbeat = time.ticks_ms()

        # --- Zielwerte ---------------------------------------------------------
        self._target_speed = 0.0
        self._target_steer = 0.0
        self._safe_speed   = 0

        # --- LED, Netzwerk ------------------------------------------------------
        self.led = LedBlinker()
        self.web_root = "/www"
        self.port = 80
        self.net = NetworkManager(country="DE")
        self.wifi_password = wifi_password

        self.mode = "MANUAL"

        # ---------------------------------------------------------------------
        # HARDWARE: ANTRIEB (MOTOR + ENCODER)
        # ---------------------------------------------------------------------
        self.kin = {
            "pulses_per_rev": 16,    # Zählung erfolgt im frozen DriveController;
            "gear_ratio": 6.3,       # Datenblatt nennt 11 Pulse/U — Skalenfaktor
            "wheel_diameter": 0.02,  # wird über Distanz-Kalibrierung absorbiert.
            "invert_dir": False,
        }
        self.drive = DriveController(**self.kin)

        # --- Odometrie (RPM-Integration) --------------------------------------
        self._dist_m = 0.0
        self._dist_last_ms = time.ticks_ms()
        self._odo_dir = 1
        self._last_rpm = 0.0
        # Plausibilitätsgrenze gegen Encoder-Störimpulse (Bürstenrauschen):
        # Messwerte darüber sind Fiktion und werden komplett verworfen.
        self.ODO_MAX_RPM = 1000
        
        # #####################################################################
        # ##  PULS-ODOMETER (ersetzt RPM-Integration): kalibriert 2026-07-13 ##
        # #####################################################################
        self.M_PER_PULSE = 0.0026 * 2.2   # Busy-Loop-Undercount empirisch kompensiert
        # #####################################################################
        self._enc_pin = _Pin(0, _Pin.IN)   # DEC_A
        self._enc_last = self._enc_pin.value()
        self._enc_halfedge = 0

        # #####################################################################
        # ##  FAHRPROFIL — Launch-Boost + Cruise. HIER EIGENE TUNING-WERTE   ##
        # ##  EINTRAGEN (Defaults = letzter bekannter Stand)!                ##
        # #####################################################################
        self.MAN_BOOST_PCT      = 70     # Anfahr-Boost (%)
        self.MAN_BOOST_MS       = 100    # Boost-Dauer ab Start (ms)
        self.MAN_SPEED_PCT      = 20     # Cruise (%); wird vom UI-Slider gesetzt
        self.MAN_SPEED_SLOW_PCT = 20     # Kriechgang (nur drive_dist)
        self.MAN_SLOW_ZONE_M    = 0.0    # Kreichgang aus: Auslauf @20% ~1cm
        # #####################################################################
        self.MAN_TIMEOUT_MS     = 15000
        self.MAN_COAST_MAX_MS   = 2000

        # #####################################################################
        # ##  INCH — kurzer Vollgas-Impuls (Dauer kommt jetzt aus dem UI)    ##
        # #####################################################################
        self.INCH_PCT        = 100
        self.INCH_MS_DEFAULT = 80      # Fallback, falls UI keinen Wert sendet
        self.INCH_MS_MIN     = 30
        self.INCH_MS_MAX     = 500
        # #####################################################################
        self._inch_until = 0

        # --- HOLD-DRIVE (Fahren solange Button gedrückt, mit Heartbeat) ------
        self.HOLD_TIMEOUT_MS = 400   # MUSS > UI-Sendeintervall (180ms) sein,
                                     # sonst stottert die Fahrt selbst bei
                                     # perfekter Verbindung!
        self._hold_lockout_until = 0 # nach drive_stop/kill: noch gepufferte
                                     # drive_hold-Pakete verwerfen (Nachlauf-Fix)
        self._hold_until    = 0
        self._hold_dir      = 1
        self._hold_pct      = self.MAN_SPEED_PCT
        self._hold_start_ms = 0

        # --- Manöver-Zustandsvariablen (drive_dist bleibt für Autopilot) -----
        self._man_active   = False
        self._man_state    = "RUN"
        self._man_ref_m    = 0.0
        self._man_target_m = 0.0
        self._man_dir      = 1
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
            invert=True,
        )
        self.steering.center()

        # ---------------------------------------------------------------------
        # HARDWARE: STROMMESSUNG
        # ---------------------------------------------------------------------
        self.current = CurrentMonitor()
        self.current.start()

        # ---------------------------------------------------------------------
        # UMWELT-SENSORIK (Telemetrie: Temperatur + Irradianz)
        # ---------------------------------------------------------------------
        # NTC 10k/3950 an ADC0 (GP26); TCS3200 Clear-Kanal (S2=GP19 H, S3=GP17 L,
        # OUT=GP16). Abtastung alle ENV_PERIOD_MS in der Loop (blockiert max.
        # ~60ms pro Sample — bei 2.5s Takt unkritisch).
        self.ENV_PERIOD_MS = 2500
        self.TEMP_OFFSET_C = -2.0     # konstante Korrektur (empirisch bestätigt)
        self.IRR_RESPONSIVITY = 150   # Hz/(uW/cm2), Clear, 20%-Skalierung
        # #####################################################################
        # ##  FARBKLASSIFIKATION — Zentroiden aus 3 Kalibriersessions        ##
        # #####################################################################
        self.COLOR_REFS = {
            "red":    (0.54, 0.22, 0.26),
            "yellow": (0.50, 0.31, 0.22),
            "green":  (0.31, 0.35, 0.31),
            "blue":   (0.30, 0.27, 0.40),
            "white":  (0.47, 0.41, 0.47),
            "black":  (0.37, 0.29, 0.33),
        }
        self.COLOR_MAX_DIST = 0.06
        # #####################################################################
        self._ground_color = "unknown"
        # TCS3200 Filterwahl: (S2,S3)
        self._tcs_filters = {"red": (0, 0), "blue": (0, 1),
                             "clear": (1, 0), "green": (1, 1)}
        self._env_last_ms = 0
        self._temp_c = None           # int °C
        self._irr = None              # int uW/cm2
        self._env_hw = False
        try:
            self._ntc = _ADC(_Pin(26)) #REFERENCE ALL PINS FROM SENSOR_CALIBRATION.PY!
            self._tcs_s2 = _Pin(19, _Pin.OUT)   # S2 = GP19 (wie sensor_calibration.py!)
            self._tcs_s3 = _Pin(17, _Pin.OUT)   # S3 = GP17
            self._tcs_out = _Pin(16, _Pin.IN)
            self._tcs_s2.value(1)
            self._tcs_s3.value(0)
            self._env_hw = True
        except Exception as e:
            print("Umwelt-Sensorik-Init fehlgeschlagen:", e)

        # ---------------------------------------------------------------------
        # SICHERHEIT / DEATHMATCH
        # ---------------------------------------------------------------------
        self.safety = SafetyManager()
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
        # ##  KOLLISIONSSCHUTZ — ToF-Schwellen (mm, korrigierte Werte)       ##
        # #####################################################################
        self.TOF_STOP_FRONT_MM   = 250
        self.TOF_STOP_SIDE_MM    = 125
        self.TOF_RELEASE_HYST_MM = 30
        self._tof_override = False   # False = Schutz AKTIV (Normalzustand);
                                     # True nur via UI-Toggle (Notfall)
        # Im AUTO-Modus gelten NIEDRIGERE Notfall-Schwellen — die Ausweich-
        # logik des Autopiloten arbeitet OBERHALB davon (330/200):
        self.TOF_NAV_FLOOR_FRONT_MM = 120
        self.TOF_NAV_FLOOR_SIDE_MM  = 90
        # #####################################################################

        self._tof_corr = {
            "left":  {"offset": 48.5, "slope": 1.00},
            "right": {"offset": 20.0, "slope": 1.05},
            "front": {"offset": 19.5, "slope": 1.05},
        }
        self._tof_bufs = {"front": [], "left": [], "right": []}
        self._tof_vals = {"front": None, "left": None, "right": None}
        self._tof_blocked = False
        self._tof_sensors = {}
        self._tof_i2c = None
        self._tof_addr = {}
        if _TOF_HW:
            self._tof_init()

        self._last_out = {"safe": None, "safety": None}

        # ---------------------------------------------------------------------
        # PROJEKT-MODUL (SENIOR)
        # ---------------------------------------------------------------------
        self.senior = None
        if SENIOR_AVAILABLE:
            try:
                self.senior = SeniorProject()
                self.senior.hipe = self   # Zugriff auf ToF-Mediane & Farbsensor
                print("Senior-Projekt erfolgreich geladen.")
            except Exception as e:
                print("Fehler im Senior-Projekt Init:", e)

    # -------------------------------------------------------------------------
    # CALLBACKS AUS DEM WEBSERVER
    # -------------------------------------------------------------------------

    def on_control(self, spd, st) -> None:
        if self._man_active:
            self._maneuver_cancel("manuelle Eingabe")
        if spd > 100: spd = 100
        if spd < -100: spd = -100
        if st > 100: st = 100
        if st < -100: st = -100
        self._target_speed = float(spd)
        self._target_steer = float(st)
        self._last_heartbeat = time.ticks_ms()

    def on_aux_command(self, type, data) -> None:
        """Verarbeitet Zusatzbefehle vom Webserver (/aux?type=...&data=...)."""
        print(f"[AUX] Type: {type} | Data: {data}")

        # --- A: FAHRWERK SETUP (Lenkung) ---
        if type == "steer_config":
            try:
                parts = data.split(",")
                if len(parts) == 3 and self.steering:
                    self.steering.angle_min = int(parts[0])
                    self.steering.angle_max = int(parts[1])
                    self.steering.trim_deg = int(parts[2])
                    print("-> Lenkungskonfiguration aktualisiert.")
            except Exception as e:
                print(f"-> Fehler bei steer_config: {e}")

        # --- B: MODUS WECHSEL ---
        elif type == "mode":
            if data == "auto":
                self._maneuver_cancel("Moduswechsel AUTO")
                self._hold_until = 0
                self.mode = "AUTO"
                print("-> Modus: AUTONOM")
            else:
                self.mode = "MANUAL"
                self._target_speed = 0
                print("-> Modus: MANUELL")

        # --- H: LENKWINKEL DISKRET ---
        elif type == "steer_angle":
            try:
                deg = float(data)
                self._target_steer = max(-100.0, min(100.0, (deg / 90.0) * 100.0))
                self._last_heartbeat = time.ticks_ms()
                print("-> Lenkwinkel: %.0f°" % deg)
            except (ValueError, TypeError):
                print("-> steer_angle: ungültiger Wert:", data)

        # --- I: HOLD-DRIVE (fahren solange Heartbeats eintreffen) ---
        # data = "fwd:55" oder "bwd:40" (Richtung : Leistung in %)
        elif type == "drive_hold":
            if self.mode != "MANUAL":
                return
            if time.ticks_diff(self._hold_lockout_until, time.ticks_ms()) > 0:
                return   # veralteter Befehl aus der TCP-Warteschlange -> weg
            try:
                parts = data.split(":")
                direction = -1 if parts[0] == "bwd" else 1
                pct = int(float(parts[1])) if len(parts) > 1 else self.MAN_SPEED_PCT
            except (ValueError, TypeError, IndexError):
                direction, pct = 1, self.MAN_SPEED_PCT
            pct = max(10, min(100, pct))
            now = time.ticks_ms()
            if self._man_active:
                self._maneuver_cancel("Hold-Drive")
            if self._hold_until == 0:
                self._hold_start_ms = now   # neuer Hold -> Boost-Fenster startet
            self._hold_dir = direction
            self._hold_pct = pct
            self.MAN_SPEED_PCT = pct        # Slider setzt auch das Fahrprofil
            self._hold_until = time.ticks_add(now, self.HOLD_TIMEOUT_MS)
            self._last_heartbeat = now

        # --- J: HOLD-DRIVE STOPP (Button losgelassen) ---
        elif type == "drive_stop":
            self._hold_until = 0
            self._target_speed = 0.0
            self._hold_lockout_until = time.ticks_add(time.ticks_ms(), 600)

        # --- D: DISTANZ-MANÖVER (bleibt für Autopilot/REPL nutzbar) ---
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

        # --- E: INCH (Dauer in ms kommt als data) ---
        elif type == "inch":
            if self._man_active or self._hold_until:
                print("-> inch: Fahrbetrieb aktiv, ignoriert.")
                return
            try:
                ms = int(float(data))
            except (ValueError, TypeError):
                ms = self.INCH_MS_DEFAULT
            ms = max(self.INCH_MS_MIN, min(self.INCH_MS_MAX, ms))
            self._inch_until = time.ticks_add(time.ticks_ms(), ms)
            print("-> Inch: %d%% für %d ms." % (self.INCH_PCT, ms))

        # --- F: KILL-SWITCH ---
        elif type == "kill":
            self._maneuver_cancel("Kill-Switch")
            self.mode = "MANUAL"
            self._target_speed = 0.0
            self._inch_until = 0
            self._hold_until = 0
            self._hold_lockout_until = time.ticks_add(time.ticks_ms(), 600)
            print("-> KILL: Antrieb gestoppt, Modus MANUELL.")

        # --- G: ODOMETER ZURÜCKSETZEN ---
        elif type == "dist_reset":
            self.reset_distance()
            print("-> Odometer auf 0 gesetzt.")
            
        # --- K: ToF-Schutz an/aus (UI-Toggle; Schutz ist per Boot-Default AN)
        elif type == "tof_override":
            self._tof_override = (data == "off")
            print("-> ToF-Schutz:", "AUS (Override!)" if self._tof_override else "AN")

        # --- K2: ToF-Schwellwerte aus dem UI (data = "front_mm,side_mm")
        elif type == "tof_config":
            try:
                parts = data.split(",")
                self.TOF_STOP_FRONT_MM = int(float(parts[0]))
                self.TOF_STOP_SIDE_MM  = int(float(parts[1]))
                print("-> ToF-Schwellen: Front %dmm, Seite %dmm"
                      % (self.TOF_STOP_FRONT_MM, self.TOF_STOP_SIDE_MM))
            except (ValueError, IndexError):
                print("-> tof_config: ungültig:", data)

        # --- L: BALL-RETRY: Inch + Jaw sofort nachschliessen (ohne Statuswechsel)
        elif type == "ball_retry":
            if self.senior:
                self._inch_until = time.ticks_add(time.ticks_ms(), 100)
                self.senior.ball_retry_pending = time.ticks_add(time.ticks_ms(), 150)
                print("-> Ball-Retry: Inch 100ms + Jaw-Nachschluss.")

        # --- C: GREIFER, LOCK & TRIGGER ---
        elif type in ("arm", "trigger", "jaw", "lock", "nav_side"):
            self._target_speed = 0.0
            self._hold_until = 0
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
        self._last_heartbeat = time.ticks_ms()
        try:
            self.safety.touch_command(self._last_heartbeat)
        except AttributeError:
            pass

    # -------------------------------------------------------------------------
    # ODOMETRIE
    # -------------------------------------------------------------------------

    def get_distance_m(self) -> float:
        return self._dist_m

    def reset_distance(self) -> None:
        self._dist_m = 0.0
        self._dist_last_ms = time.ticks_ms()
        
    def _poll_encoder(self, ms):
        """Zählt Encoder-Flanken auf GP0 für ms Millisekunden (Busy-Poll,
        glitch-immun — IRQ zählt Bürstenfunken ~300x zu viel!). Läuft in
        der Idle-Zeit jedes Loop-Ticks statt sleep_ms."""
        if ms <= 0:
            return
        pin = self._enc_pin
        last = self._enc_last
        edges = 0
        deadline = time.ticks_add(time.ticks_us(), ms * 1000)
        while time.ticks_diff(deadline, time.ticks_us()) > 0:
            v = pin.value()
            if v != last:
                edges += 1
                last = v
        self._enc_last = last
        self._enc_halfedge += edges
        pulses = self._enc_halfedge // 2
        self._enc_halfedge -= pulses * 2
        self._dist_m += self._odo_dir * pulses * self.M_PER_PULSE    

    def _update_odometry(self, rpm, safe_pct, now) -> None:
        dt_odo = time.ticks_diff(now, self._dist_last_ms)
        self._dist_last_ms = now

        if safe_pct > 0:
            self._odo_dir = 1
        elif safe_pct < 0:
            self._odo_dir = -1

        if not (0 < dt_odo < 500):
            return

        if abs(rpm) > self.ODO_MAX_RPM:
            print("[ODO] Implausible RPM verworfen: %.0f" % rpm)
            return

        step = (abs(rpm) / 60.0) * (dt_odo / 1000.0) \
               * 3.141592653589793 * self.kin["wheel_diameter"]
        self._dist_m += self._odo_dir * step

    # -------------------------------------------------------------------------
    # UMWELT-SENSORIK (Temperatur + Irradianz für Telemetrie)
    # -------------------------------------------------------------------------

    def _read_temp_int(self):
        """NTC über B-Gleichung, konstante Korrektur, als int °C."""
        try:
            import math
            raw = self._ntc.read_u16()
            v = raw / 65535 * 3.3
            if v >= 3.29 or v <= 0.01:
                return None
            r = 10000 * v / (3.3 - v)
            t = 1.0 / (1.0 / 298.15 + math.log(r / 10000) / 3950) - 273.15
            return int(round(t + self.TEMP_OFFSET_C))
        except Exception:
            return None

    def _read_irr_int(self, periods=10, timeout_us=5000):
        """TCS3200 Clear-Kanal: Frequenz über n Perioden -> int uW/cm2.

        Misst mit DEAKTIVIERTEN Interrupts: der Soft-ISR des CurrentMonitor
        verschluckt sonst Flanken -> Frequenz ~3x unterschätzt (57 statt 180).
        Timeout kurz (5ms), da IRQs währenddessen aus sind; dunkelste Fläche
        (~11kHz) braucht <2ms für 10 Perioden.
        """
        try:
            # GP16 wird von einem frozen Modul umkonfiguriert -> Input-Modus
            # und Clear-Filter vor JEDER Messung neu erzwingen.
            self._tcs_out = _Pin(16, _Pin.IN)
            self._tcs_s2.value(1)
            self._tcs_s3.value(0)
            out = self._tcs_out

            import machine
            irq_state = machine.disable_irq()
            try:
                deadline = time.ticks_add(time.ticks_us(), timeout_us)

                def wait(level):
                    while out.value() != level:
                        if time.ticks_diff(deadline, time.ticks_us()) <= 0:
                            return False
                    return True

                if not (wait(0) and wait(1)):
                    return None
                t0 = time.ticks_us()
                for _ in range(periods):
                    if not (wait(0) and wait(1)):
                        return None
                dt = time.ticks_diff(time.ticks_us(), t0)
            finally:
                machine.enable_irq(irq_state)

            if dt <= 0:
                return None
            freq = 1_000_000 * periods / dt
            return int(round(freq / self.IRR_RESPONSIVITY))
        except Exception:
            return None
        
    def _tcs_freq(self, s2v, s3v, periods=10, timeout_us=5000):
        """Eine Filterfrequenz messen (IRQ-geschützt wie _read_irr_int)."""
        try:
            self._tcs_out = _Pin(16, _Pin.IN)
            self._tcs_s2.value(s2v)
            self._tcs_s3.value(s3v)
            time.sleep_ms(20)   # Filter-Settle
            out = self._tcs_out
            import machine
            irq_state = machine.disable_irq()
            try:
                deadline = time.ticks_add(time.ticks_us(), timeout_us)
                def wait(level):
                    while out.value() != level:
                        if time.ticks_diff(deadline, time.ticks_us()) <= 0:
                            return False
                    return True
                if not (wait(0) and wait(1)):
                    return None
                t0 = time.ticks_us()
                for _ in range(periods):
                    if not (wait(0) and wait(1)):
                        return None
                dt = time.ticks_diff(time.ticks_us(), t0)
            finally:
                machine.enable_irq(irq_state)
            return 1_000_000 * periods / dt if dt > 0 else None
        except Exception:
            return None

    def read_color_name(self):
        """Voller RGBC-Durchgang (~100ms, blockierend!) -> Farbname.
        Nur im Stand aufrufen. Aktualisiert self._ground_color."""
        f = {}
        for name, (a, b) in self._tcs_filters.items():
            f[name] = self._tcs_freq(a, b)
        if not f["clear"] or not f["red"] or not f["green"] or not f["blue"]:
            self._ground_color = "unknown"
            return "unknown"
        rr = f["red"] / f["clear"]
        gr = f["green"] / f["clear"]
        br = f["blue"] / f["clear"]
        best, best_d2 = "unknown", self.COLOR_MAX_DIST ** 2
        for cname, (cr, cg, cb) in self.COLOR_REFS.items():
            d2 = (rr - cr) ** 2 + (gr - cg) ** 2 + (br - cb) ** 2
            if d2 < best_d2:
                best, best_d2 = cname, d2
        self._ground_color = best
        return best

    # -------------------------------------------------------------------------
    # KOLLISIONSSCHUTZ (ToF, Continuous-Modus, nicht-blockierend)
    # -------------------------------------------------------------------------

    def _tof_init(self):
        try:
            xshut_r = _Pin(14, _Pin.OUT)
            xshut_l = _Pin(15, _Pin.OUT)
            xshut_r.value(0)
            xshut_l.value(0)
            time.sleep_ms(50)

            i2c = I2C(0, scl=_Pin(5), sda=_Pin(4), freq=100000)
            self._tof_i2c = i2c

            ADDR_DEF, ADDR_FRONT, ADDR_RIGHT = 0x29, 0x2A, 0x2B

            try:
                i2c.writeto_mem(ADDR_DEF, 0x8A, bytes([ADDR_FRONT]))
            except OSError as e:
                print("[ToF] Front Adresswechsel fehlgeschlagen:", e)
            time.sleep_ms(10)

            xshut_r.value(1)
            time.sleep_ms(50)
            try:
                i2c.writeto_mem(ADDR_DEF, 0x8A, bytes([ADDR_RIGHT]))
            except OSError as e:
                print("[ToF] Right Adresswechsel fehlgeschlagen:", e)
            time.sleep_ms(10)

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

            for tof in self._tof_sensors.values():
                fn = getattr(tof, "set_measurement_timing_budget", None)
                if callable(fn):
                    try:
                        fn(20000)
                    except Exception:
                        pass

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
        for name, addr in self._tof_addr.items():
            try:
                ready = self._tof_i2c.readfrom_mem(addr, 0x13, 1)[0] & 0x07
                if not ready:
                    continue
                data = self._tof_i2c.readfrom_mem(addr, 0x14, 12)
                raw = (data[10] << 8) | data[11]
                self._tof_i2c.writeto_mem(addr, 0x0B, b"\x01")
            except OSError:
                continue

            if raw >= 8190 or raw < 10:
                continue
            buf = self._tof_bufs[name]
            buf.append(self._tof_correct(name, raw))
            if len(buf) > 5:
                buf.pop(0)
            if len(buf) >= 3:
                self._tof_vals[name] = self._median(buf)

    def _tof_check_proximity(self):
        jaw_open = bool(self.senior and getattr(self.senior, "jaw_open", False))
        margin = self.TOF_RELEASE_HYST_MM if self._tof_blocked else 0

        blocked = False
        if self.mode == "AUTO":
            lim_f, lim_s = self.TOF_NAV_FLOOR_FRONT_MM, self.TOF_NAV_FLOOR_SIDE_MM
        else:
            lim_f, lim_s = self.TOF_STOP_FRONT_MM, self.TOF_STOP_SIDE_MM
        for name, limit in (("left", lim_s), ("right", lim_s), ("front", lim_f)):
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
        """Blockierender Vor-Check (max ~0.7s) für drive_dist vorwärts."""
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
    # DISTANZ-MANÖVER (drive_dist — für Autopilot/REPL)
    # -------------------------------------------------------------------------

    def _maneuver_start(self, target_m, direction) -> None:
        if direction > 0 and _TOF_HW:
            target_m = self._tof_presweep_cap(target_m)
            if target_m <= 0:
                return

        self._man_ref_m    = self._dist_m
        self._man_target_m = target_m
        self._man_dir      = direction
        self._man_start_ms = time.ticks_ms()
        self._man_state    = "RUN"
        self._man_active   = True
        print("[MAN] drive_dist Start: Soll=%.3f m, Richtung=%s, Odometer=%.3f m"
              % (target_m, "vor" if direction > 0 else "zurueck", self._dist_m))

    def _maneuver_cancel(self, reason) -> None:
        if not self._man_active:
            return
        self._man_active = False
        self._target_speed = 0.0
        delta = abs(self._dist_m - self._man_ref_m)
        print("[MAN] Abbruch (%s) bei %.3f von %.3f m."
              % (reason, delta, self._man_target_m))

    def _maneuver_tick(self, now) -> None:
        delta = abs(self._dist_m - self._man_ref_m)

        if self._man_state == "RUN":
            remaining = self._man_target_m - delta

            if remaining <= 0:
                self._target_speed = 0.0
                self._man_state = "COAST"
                self._man_coast_ms = now
                print("[MAN] Abschaltpunkt: %.3f m nach %d ms."
                      % (delta, time.ticks_diff(now, self._man_start_ms)))
                return

            if time.ticks_diff(now, self._man_start_ms) > self.MAN_TIMEOUT_MS:
                self._maneuver_cancel("Timeout")
                return

            # Boost: 70%/100ms — nur Anlaufschwelle überwinden, nicht überschiessen
            if time.ticks_diff(now, self._man_start_ms) < self.MAN_BOOST_MS:
                pct = self.MAN_BOOST_PCT
            elif remaining <= self.MAN_SLOW_ZONE_M:
                pct = self.MAN_SPEED_SLOW_PCT
            else:
                pct = self.MAN_SPEED_PCT
            self._target_speed = float(self._man_dir * pct)
            # Lenkung wird bewusst NICHT zentriert (Bogenfahrt möglich)
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
            "temp": self._temp_c,     # int °C (alle ~2.5s aktualisiert)
            "irr": self._irr,         # int uW/cm2 (alle ~2.5s aktualisiert)
            "ground_color": self._ground_color,
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
        self.current.calibrate_zero()
        next_tick = time.ticks_ms()

        while True:
            # 1) Webserver
            # 1) Webserver: Backlog LEEREN statt nur 1 Request/Tick —
            #    gepufferte Steuerbefehle wirken sonst sekundenlang nach
            #    (Motor-Nachlauf nach Loslassen!)
            for _ in range(8):
                try:
                    handled = self.web.poll_once()
                except OSError as e:
                    print("web.poll_once failed:", e)
                    break
                if not handled:
                    break

            # 2) AUTOPILOT
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
            
            # TEMP: Loop-Jitter-Diagnose — nach Auswertung wieder entfernen
            if not hasattr(self, "_jit_last"): self._jit_last, self._jit_max = now, 0
            _d = time.ticks_diff(now, self._jit_last)
            self._jit_last = now
            if _d > self._jit_max:
                self._jit_max = _d
                if _d > 15: print("[JIT] Tick %d ms (Soll 10)" % _d)

            # 2b) Distanz-Manöver
            if self._man_active and self.mode == "MANUAL":
                self._maneuver_tick(now)

            # 2c) Inch-Impuls
            if self._inch_until:
                if time.ticks_diff(self._inch_until, now) > 0:
                    self._target_speed = float(self.INCH_PCT)
                    self._last_heartbeat = now
                else:
                    self._inch_until = 0
                    self._target_speed = 0.0
                    
            # 2c2) Ball-Retry: verzögerter Jaw-Nachschluss nach Inch
            if self.senior and getattr(self.senior, "ball_retry_pending", 0):
                if time.ticks_diff(self.senior.ball_retry_pending, now) <= 0:
                    self.senior.ball_retry_pending = 0
                    self._target_speed = 0.0
                    try:
                        self.senior.jaw_reclose()
                    except Exception as e:
                        print("jaw_reclose fehlgeschlagen:", e)

            # 2d) HOLD-DRIVE: fahren solange Heartbeats frisch sind
            if self._hold_until:
                if time.ticks_diff(self._hold_until, now) > 0:
                    if time.ticks_diff(now, self._hold_start_ms) < self.MAN_BOOST_MS:
                        pct_eff = max(self._hold_pct, self.MAN_BOOST_PCT)
                    else:
                        pct_eff = self._hold_pct
                    self._target_speed = float(self._hold_dir * pct_eff)
                    self._last_heartbeat = now
                else:
                    self._hold_until = 0
                    self._target_speed = 0.0

            # 2e) Umwelt-Sensorik (Temp + Irradianz, alle ENV_PERIOD_MS)
            if self._env_hw and time.ticks_diff(now, self._env_last_ms) >= self.ENV_PERIOD_MS:
                self._env_last_ms = now
                self._temp_c = self._read_temp_int()
                self._irr = self._read_irr_int()

            # 3) Strommessung
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

            # 6) Safety
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

            # 6b) Odometer-Richtung aus Sollwert (Zählung selbst: siehe Step 8)
            if safe_pct > 0:
                self._odo_dir = 1
            elif safe_pct < 0:
                self._odo_dir = -1

            # 6c) ToF-Kollisionsschutz
            if _TOF_HW and self._tof_sensors and not self._tof_override:
                self._tof_poll()
                self._tof_check_proximity()
                if self._tof_blocked:
                    if self._man_active and self._man_dir > 0:
                        self._maneuver_cancel("Kollisionsschutz")
                    if self._hold_until and self._hold_dir > 0:
                        self._hold_until = 0
                        self._target_speed = 0.0
                    if self.mode == "AUTO":
                        self.mode = "MANUAL"
                        self._target_speed = 0.0
                        print("[ToF] AUTO -> MANUAL (Kollisionsschutz).")
                    if safe_pct > 0 and not self._inch_until:
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
            # Encoder-Polling GARANTIEREN: auch bei überzogenem Tick
            # mindestens 3ms zählen, sonst verhungert der Odometer
            # sobald ein Client verbunden ist (Tick-Überläufe!)
            self._poll_encoder(delay if delay > 2 else 3)

            # 9) LED-Muster
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