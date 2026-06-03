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
# Diese Datei ist das „Herz“ des Projekts. Sie startet alle Teilmodule
# (Antrieb, Lenkung, Strommessung, Sicherheit, Netzwerk, Webserver) und führt
# eine NICHT-BLOCKIERENDE Hauptschleife aus. „Nicht-blockierend“ bedeutet:
# Es wird in kleinen Schritten gearbeitet, sodass das System jederzeit schnell
# reagieren kann (z. B. auf neue Steuersignale).
#
# Für alle:
# - „%“ bedeutet hier immer ein Sollwert im Bereich -100..+100.
#   +100 = volle Vorwärtsfahrt, -100 = volle Rückwärtsfahrt, 0 = Stopp.
# - Die Lenkung nutzt ebenfalls -100..+100 (%), wobei der genaue Winkel
#   im Lenkungsmodul (Steering) festgelegt ist.
# - Ein „Heartbeat“ (Lebenszeichen) aus dem Web-Frontend verhindert, dass das
#   Fahrzeug bei Verbindungsabbruch weiterfährt (Dead-Man-Funktion).
#
# Start aus der REPL:
#   >>> from hipe import hipe
#   >>> h = hipe("Hier WLAN-Passwort angeben")
#   >>> h.run()
#
# Beenden:
#   - Strg+C in der REPL drücken (unterbricht die Loop).
# -----------------------------------------------------------------------------

"""Hauptorchestrator für Fahrzeugsteuerung, Telemetrie und Web-UI.

Startet die Teilmodule (Antrieb, Lenkung, Strommessung, Sicherheit, Netzwerk,
Webserver) und betreibt eine **nicht-blockierende** Hauptschleife. Die
Weboberfläche liefert Sollwerte (Speed/Steer) und Heartbeats; die ``SafetyManager``-Logik begrenzt und schützt den Antrieb.

Zusätzlich werden über den ``/aux``-Kanal Konfigurationsbefehle (Lenkungstrimmung)
und Steuerbefehle für Erweiterungen (Autopilot, Greifarm) verarbeitet.

Examples:

    from hipe import hipe
    h = hipe("Hier WLAN-Passwort angeben")
    h.run()
"""

import time

# --- Import der Teilmodule ----------------------------------------------------
# „modules.*“ sind die offenen Komponenten (sichtbarer Code).
# „secure.*“ sind eingefrorene/fest eingebundene Komponenten in der Firmware.
from modules.led import LedBlinker
from modules.net import NetworkManager
from modules.webserver import WebServer

from secure.drive import DriveController          # Motorsteuerung inkl. Encoder
from modules.steering import Steering             # Servo-Lenkung (offen)
from secure.current_monitor import CurrentMonitor # Zweikanal-Strommessung
from secure.safety import SafetyManager           # Sicherheitslogik
from secure.crash_counter import CrashCounter     # „Deathmatch“-Lebenszähler

# ProjektSenior
try:
    from modules.senior import SeniorProject
    SENIOR_AVAILABLE = True
except ImportError:
    print("Kein Senior-Projekt gefunden (modules/senior.py fehlt).")
    SENIOR_AVAILABLE = False


class hipe:
    """Zentrale, nicht-blockierende Mainloop für Fahrzeug, Telemetrie und Web-UI.

    Aufgaben:
        - Verbindet Weboberfläche und Hardware (Antrieb/Lenkung).
        - Erfasst Telemetrie (Drehzahl, Ströme, Status).
        - Erzwingt Sicherheit (Stromlimits, Totmannschaltung, Stall-Erkennung).
        - Verarbeitet Erweiterungsbefehle (Aux) für Setup und Autonomie.
        - Zeigt Aktivität per LED-Muster.

    Attributes:
        dt_ms (int): Ziel-Dauer eines Schleifendurchlaufs (ms).
        HEARTBEAT_TIMEOUT_MS (int): Zeitfenster, nach dem Sollwerte auf 0 gesetzt werden.
        led (LedBlinker): LED-Blinker für Aktivitätsanzeige.
        net (NetworkManager): Netzwerk-Manager (AP/STA, IP).
        web_root (str): Verzeichnis der Web-Assets.
        port (int): HTTP-Port des Webservers.
        drive (DriveController): Antriebscontroller (Motor + Encoder).
        steering (Steering): Lenkungscontroller (Servo).
        current (CurrentMonitor): Strommessung (zweikanalig).
        safety (SafetyManager): Sicherheitslogik (Begrenzungen, Dead-Man, Stall).
        crash (CrashCounter): Crash-/Lebenszähler (Deathmatch-Modus).
        mode (str): Betriebsmodus ("MANUAL" oder "AUTO") für Autonomie-Erweiterung.
        senior (SeniorProject | None): Instanz des Studenten-Codes.
        _target_speed (float): Zielgeschwindigkeit in % (−100..+100).
        _target_steer (float): Ziellenkung in % (−100..+100).
        _safe_speed (int): Von Safety begrenzter %-Wert für den Antrieb.
    """

    # -------------------------------------------------------------------------
    # INITIALISIERUNG
    # -------------------------------------------------------------------------
    def __init__(self, wifi_password: str) -> None:
        """Erzeugt eine Instanz, konfiguriert Hardware und startet AP + HTTP-Server.

        Args:
            wifi_password (str): Passwort für den WLAN-Access-Point (muss gesetzt werden).

        Returns:
            None
        """

        # --- Zeit/Loop-Parameter ---------------------------------------------
        self.loop_hz = 100 # Takt der Hauptschleife (z. B. 100 ⇒ ~10 ms/Tick).
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

        # --- Zustandsautomat (State Machine) ---------------------------------
        # MANUAL: Web-Joystick steuert direkt.
        # AUTO: Autopilot-Klasse übernimmt die Kontrolle (Erweiterung).
        self.mode = "MANUAL"


        # ---------------------------------------------------------------------
        # HARDWARE: ANTRIEB (MOTOR + ENCODER)
        # ---------------------------------------------------------------------
        self.drive = DriveController(
            # --- Kinematik (für Telemetrie) ---
            pulses_per_rev=16,     # Flanken A-rising pro Motorumdrehung (laut Datenblatt)
            gear_ratio=6.3,        # Getriebe Motorwelle:Ausgangswelle (laut Datenblatt)
            wheel_diameter=0.02,    # Raddurchmesser in m
            invert_dir=False        # Motordrehrichtung umkehren
        )

        # ---------------------------------------------------------------------
        # HARDWARE: LENKUNG (SERVO)
        # ---------------------------------------------------------------------
        self.steering = Steering(
            pin=6,                 # Servo-Pin (GPIO 6)
            pwm_freq_hz=50,        # Normale Servo-Frequenz
            min_us=900, max_us=2100,
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
        # „DEATHMATCH“-MODUS
        # ---------------------------------------------------------------------
        self.deathmatch_enabled = False
        self.crash = CrashCounter()

        # ---------------------------------------------------------------------
        # NETZ & WEB
        # ---------------------------------------------------------------------
        # 1) WLAN-Access-Point
        try:
            ap_ip = self.net.start_ap(password=self.wifi_password, channel=None)  # Auto-Kanal
            print("SSID =", getattr(self.net, "ap_ssid", "<unknown>"))
            print("AP aktiv: IP =", ap_ip)
            self.led.set_pattern("fast")
        except (OSError, RuntimeError, ValueError) as e:
            print("AP start fehlgeschlagen:", e)
            self.led.set_pattern("off")

        # 2) HTTP-Server (nicht-blockierend, wird in der Loop gepollt)
        try:
            # Der Webserver erhält zwei Callbacks:
            # - on_control: Für hochfrequente Joystick-Daten (/control)
            # - on_aux: Für Konfiguration und Sonderfunktionen (/aux)
            self.web = WebServer(
                port=self.port,
                web_root=self.web_root,
                on_control=self.on_control,
                on_aux=self.on_aux_command, # <--- Handler für Setup & Erweiterungen
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
        """Web-Callback: neue Zielwerte setzen (Speed/Steer).

        Wird vom Endpoint ``/control`` aufgerufen (Joystick).
        Klemmt Werte in die UI-Spanne (−100..+100) und aktualisiert den
        Heartbeat-Zeitstempel (Totmannschaltung).

        Args:
            spd (float | int): gewünschte Geschwindigkeit in % (−100..+100).
            st  (float | int): gewünschte Lenkung in % (−100..+100).

        Returns:
            None
        """
        # Werte „einfangen“ und begrenzen
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
        """Verarbeitet Zusatzbefehle vom Webserver (Setup & Erweiterungen).

        Wird über den Endpunkt ``/aux?type=...&data=...`` aufgerufen.
        Dient zur Laufzeit-Konfiguration (Lenkung) und Steuerung von
        Zusatzmodulen (Autopilot, Greifarm).

        Args:
            type (str): Befehlstyp (z. B. "steer_config", "mode", "trigger").
            data (str): Nutzdaten (z. B. "-90,90,0" oder "auto").
        """
        # Debugging
        print(f"[AUX] Type: {type} | Data: {data}")

        # --- A: FAHRWERK SETUP (Lenkung) ---
        if type == "steer_config":
            # Erwartet: "min,max,trim" (z.B. "-40,40,5")
            try:
                parts = data.split(",")
                if len(parts) == 3:
                    # Wir greifen direkt auf das Steering-Objekt zu
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
                self.mode = "AUTO"
                print("-> Modus: AUTONOM")
            else:
                self.mode = "MANUAL"
                self._target_speed = 0  # Sicherheitsstopp
                print("-> Modus: MANUELL")

        # --- C: GREIFARM & TRIGGER (Weiterleitung an Senior) ---
        elif (type == "arm" or type == "trigger"):
            # Wenn das Senior-Modul geladen ist, wird es weitergeleitet
            if self.senior:
                try:
                    self.senior.handle_aux(type, data)
                except Exception as e:
                    print("Fehler im Senior-Aux:", e)
            else:
                print("Senior-Modul nicht aktiv.")

    def on_heartbeat(self) -> None:
        """Web-Callback: Heartbeat für Dead-Man aktualisieren.

        Setzt internen Zeitstempel und pingt die Safety (falls vorhanden).

        Returns:
            None
        """
        self._last_heartbeat = time.ticks_ms()
        try:
            self.safety.touch_command(self._last_heartbeat)
        except AttributeError:
            # safety hat evtl. keinen touch_command() (oder wurde ersetzt)
            pass

    # -------------------------------------------------------------------------
    # TELEMETRIE
    # -------------------------------------------------------------------------

    def get_telemetry(self) -> dict:
        """Erstellt Telemetrie-Daten für das Frontend.

        Returns:
            dict: Felder:
                - ``speed_target`` (float): Zielgeschwindigkeit in %.
                - ``steer_target`` (float): Ziellenkung in %.
                - ``rpm`` (float | None): Drehzahl (U/min), falls verfügbar.
                - ``current`` (dict): Mittelwerte der Ströme ``{"motor": A, "system": A}``.
                - ``safety`` (str): Safety-Status (OK/LIMIT/STALL/TIMEOUT/DEAD).
                - ``speed_safe`` (int): Von Safety begrenzter Wert in %.
                - ``ts`` (int): Zeitstempel (ms).
                - ``lives`` (int, optional): Restleben im Deathmatch-Modus.
        """
        # Drehzahl
        try:
            rpm = self.drive.get_rpm()
        except (AttributeError, OSError, RuntimeError) as e:
            print("get rpm fehlgeschlagen:", e)
            rpm = None

        # Ströme (Cache spart CPU)
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
        """Führt die nicht-blockierende Hauptschleife aus.

        Ablauf pro Tick (vereinfacht):

            1. Webserver bedienen (max. 1 Anfrage).
            2. Autopilot (Senior) Logik ausführen (Überschreibt ggf. Manual).
            3. Stromfenster mitteln und cachen (~120 ms).
            4. Dead-Man: Bei altem Heartbeat → Zielgeschwindigkeit auf 0.
            5. Deathmatch: Leben/„DEAD“ prüfen und ggf. sperren.
            6. Safety anwenden → sicheren %-Wert berechnen.
            7. PWM (Antrieb) und Position (Lenkung) ausgeben.
            8. Takt einhalten (schlafen, falls Zeit übrig).
            9. LED-Muster je nach Client/Heartbeat.

        Returns:
            None
        """

        self.current.calibrate_zero()  # Stromkalibrierung direkt nach dem Start

        next_tick = time.ticks_ms()

        while True:
            # 1) Webserver (holt manuelle Eingaben)
            try:
                self.web.poll_once()
            except OSError as e:
                print("web.poll_once failed:", e)

            # -----------------------------------------------------------------
            # 2) AUTOPILOT (SENIOR) - PRIORITÄT VOR HARDWARE
            # -----------------------------------------------------------------
            # Wenn der Autopilot läuft, überschreibt er die manuellen Inputs
            # des Webservers, bevor diese an die Hardware gehen.
            if self.mode == "AUTO" and self.senior:
                try:
                    # a. Daten holen
                    rpm = self.drive.get_rpm()

                    # b. Senior-Logik fragen ("Wohin willst du?")
                    s_speed, s_steer = self.senior.run_autopilot(rpm)

                    # c. Werte setzen (Überschreibt ggf. Joystick-Werte)
                    self._target_speed = float(s_speed)
                    self._target_steer = float(s_steer)

                    # d. WICHTIG: Heartbeat füttern (sonst bremst Safety)
                    self._last_heartbeat = time.ticks_ms()

                except Exception as e:
                    print("Crash im Senior-Autopilot:", e)
                    self.mode = "MANUAL"  # Notaus bei Code-Fehler
                    self._target_speed = 0

            # -----------------------------------------------------------------
            # REST DER LOOP (Strom, Safety, Ausgabe)
            # -----------------------------------------------------------------
            now = time.ticks_ms()

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