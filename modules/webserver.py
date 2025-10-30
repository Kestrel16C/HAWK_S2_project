# modules/webserver.py
# created by Tobias Bürmann, HAWK

"""WebServer – sehr schlanker, nicht-blockierender HTTP-Server für MicroPython.

Der Server bedient einfache GET-Endpunkte (Steuerung/Telemetrie) und liefert statische Dateien aus einem konfigurierbaren Web-Root aus. Kurze Socket-Timeouts sorgen dafür, dass die Mainloop responsiv bleibt. Optional kann ein
externer Safety-Manager per Heartbeat beruhigt werden.

Examples:

    ws = WebServer(
        web_root="/www",
        on_control=lambda speed, steer: ctrl(speed, steer),
        on_center=steering.center,
        get_telemetry=lambda: read_telemetry(),
        on_heartbeat=heartbeat,
        safety=safety_manager,
    )
    ws.setup_server()
    while True:
        ws.poll_once()  # blockiert nicht – mit Mainloop kompatibel
"""

import time
import socket

try:
    import ujson as json
except ImportError:  # CPython-Tests
    import json

try:
    import ure as re
except ImportError:  # CPython-Tests
    import re


class WebServer:
    """Minimaler HTTP-Server mit kurzen Timeouts (nicht-blockierend).

    Der Server unterstützt u. a. folgende Endpunkte:

    - ``GET /`` → statische Dateien (``index.html``) aus ``web_root``.
    - ``GET /telemetry`` → JSON, ergänzt um ``speed_target``/``steer_target``
      und optional ``safety_raw``/``safety``.
    - ``GET /control?speed=&steer=`` → kombinierter Setter (−100..+100).
    - ``GET /center`` → Lenkung zentrieren.
    - ``GET /set_zero?ch=&v=`` / ``/set_scale?ch=&v=`` / ``/calibrate_zero?n=`` → Strommessungs-APIs.
    - ``GET /steering?value=`` / ``GET /speed?value=`` → separate Setter.

    Args:
        port (int): TCP-Port, Standard: ``80``.
        web_root (str): Verzeichnis für statische Dateien (z. B. ``"/www"``).
        on_control (Callable[[float, float], None] | None): Kombi-Setter für
            Geschwindigkeit und Lenkung.
        on_center (Callable[[], None] | None): Lenkung auf Mitte fahren.
        on_calib_zero (Callable[..., None] | None): Entweder ``(ch:int, v:float)`` oder ``(n:int)``.
        on_set_scale (Callable[[int, float], None] | None): Skala (A/V) je Kanal setzen.
        get_telemetry (Callable[[], dict] | None): Liefert Telemetrie-Daten.
        on_heartbeat (Callable[[], None] | None): Wird bei **jedem** HTTP-Request aufgerufen.
        on_set_steer (Callable[[float], None] | None): Setter nur für Lenkung.
        on_set_speed (Callable[[float], None] | None): Setter nur für Antrieb.
        steering (object | None): Optionaler Verweis auf Lenk-Subsystem.
        current (object | None): Optionaler Verweis auf Strommess-Subsystem.
        safety (object | None): Optionaler Safety-Manager (z. B. mit ``status``/``status_ui()``).

    Attributes:
        ACCEPT_TIMEOUT_S (float): Accept-Timeout (Sek.) für nicht-blockierenden Betrieb.
        CLIENT_TIMEOUT_S (float): Client-Timeout (Sek.) je Anfrage.
        MAX_REQ (int): Max. eingehende Request-Größe (Bytes).
        web_root (str): Wurzel für statische Auslieferung.
        port (int): TCP-Port.
        speed_target (int): Letzter Zielwert Speed (UI-Target).
        steer_target (int): Letzter Zielwert Steer (UI-Target).
        safety, steering, current: Referenzen auf optionale Subsysteme.
    """

    # kurze Timeouts → Server blockiert die Mainloop nicht
    ACCEPT_TIMEOUT_S = 0.02
    CLIENT_TIMEOUT_S = 1.5
    MAX_REQ = 4096  # reicht für GET + knappe Header

    def __init__(self, *,
                 port=80,
                 web_root="/www",
                 on_control=None,
                 on_center=None,
                 on_calib_zero=None,
                 on_set_scale=None,
                 get_telemetry=None,
                 on_heartbeat=None,
                 on_set_steer=None,
                 on_set_speed=None,
                 steering=None,
                 current=None,
                 safety=None):
        """Siehe Klassenbeschreibung für Parameterdetails."""
        # Pfad normalisieren
        if not web_root.startswith("/"):
            web_root = "/" + web_root
        self.web_root = web_root
        self.port = int(port)

        # Callbacks (No-Ops als Fallback)
        self.on_control     = on_control     or (lambda s, t: None)
        self.on_center      = on_center      or (lambda: None)
        self.on_calib_zero  = on_calib_zero  or (lambda *args, **kw: None)
        self.on_set_scale   = on_set_scale   or (lambda ch, v: None)
        self.get_telemetry  = get_telemetry  or (lambda: {})
        self.on_heartbeat   = on_heartbeat   or (lambda: None)
        self.on_set_steer   = on_set_steer   or (lambda v: None)
        self.on_set_speed   = on_set_speed   or (lambda v: None)

        # Direkte Subsysteme (optional)
        self.safety   = safety
        self.steering = steering
        self.current  = current

        # UI-Targets (damit die Seite zuverlässig aktuelle Werte zeigt)
        self.speed_target = 0
        self.steer_target = 0

        # intern
        self._sock = None
        self._re_path_qs = re.compile(r"^([^\?]+)(?:\?(.*))?$")

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------
    def setup_server(self) -> None:
        """Initialisiert den Server-Socket (nicht-blockierender Accept).

        Stellt einen lauschenden TCP-Socket auf ``0.0.0.0:<port>`` bereit
        und konfiguriert ein kurzes Accept-Timeout.

        Returns:
            None
        """
        addr = socket.getaddrinfo("0.0.0.0", self.port)[0][-1]
        s = socket.socket()
        try:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        except OSError:
            pass
        s.bind(addr)
        s.listen(8)
        s.settimeout(self.ACCEPT_TIMEOUT_S)
        self._sock = s

    def start(self) -> None:
        """Startet den Server im blockierenden Dauermodus.

        Normalerweise wird stattdessen ``poll_once()`` zyklisch aus der
        Mainloop aufgerufen.

        Returns:
            None
        """
        self.setup_server()
        while True:
            self.poll_once()

    # -------------------------------------------------------------------------
    # Haupt-Loop: genau 1 Anfrage bedienen (falls vorhanden)
    # -------------------------------------------------------------------------
    def poll_once(self) -> None:
        """Bedient maximal **eine** HTTP-Anfrage; kehrt sonst sofort zurück.

        Nicht-blockierend: Wenn gerade kein Client anklopft, endet der Aufruf
        ohne Wartezeit.

        Returns:
            None
        """
        if not self._sock:
            return None
        try:
            client, addr = self._sock.accept()

        except OSError:
            return None # kein Client in diesem Tick

        try:
            client.settimeout(self.CLIENT_TIMEOUT_S)

            # Nur die erste Header-Chunk lesen (reicht für GET + kurze Header)
            req = client.recv(self.MAX_REQ)
            if not req:
                return None

            # zentraler Heartbeat → Dead-Man im Orchestrator beruhigen
            try:
                self.on_heartbeat()
            except Exception:
                pass  # pylint: disable=broad-exception-caught

            if self.safety and hasattr(self.safety, "touch_command"):
                try:
                    self.safety.touch_command()
                except Exception:
                    pass  # pylint: disable=broad-exception-caught

            method, path = self._parse_request_line(req)
            path, qs = self._split_qs(path)

            if method != "GET":
                return self._send_response(client, 405, "text/plain", b"Method Not Allowed")

            # ---- API-Routen ----
            if path == "/telemetry":
                return self._handle_telemetry(client)

            if path == "/control":
                return self._handle_control(client, qs)

            if path == "/center":
                return self._handle_center(client)

            if path == "/set_zero":
                return self._handle_set_zero(client, qs)

            if path == "/set_scale":
                return self._handle_set_scale(client, qs)

            if path == "/calibrate_zero":
                return self._handle_calibrate_zero(client, qs)

            if path == "/steering":
                return self._handle_steering(client, qs)

            if path == "/speed":
                return self._handle_speed(client, qs)

            # ---- Statische Dateien ----
            return self._serve_static(client, path)

        except (UnicodeError, ValueError, OSError) as e:
            try:
                code = 400 if isinstance(e, (UnicodeError, ValueError)) else 500
                self._send_response(client, code, "text/plain", str(e).encode())
            except OSError:
                pass
        finally:
            try:
                client.close()
            except OSError:
                pass

    # -------------------------------------------------------------------------
    # Handlers
    # -------------------------------------------------------------------------
    def _handle_telemetry(self, client) -> None:
        """Sendet Telemetrie als JSON und ergänzt UI-/Safety-Felder.

        Ergänzt fehlende UI-Targets (``speed_target``/``steer_target``) sowie
        den Safety-Status (``safety_raw``/``safety``) falls ein Safety-Manager
        konfiguriert ist.

        Returns:
            None
        """
        data = self.get_telemetry() or {}

        # UI-Targets (Default, falls Orchestrator sie nicht liefert)
        data.setdefault("speed_target", int(self.speed_target))
        data.setdefault("steer_target", int(self.steer_target))

        # Safety-Status → "safety_raw" (intern) + "safety" (UI-Map)
        if self.safety:
            try:
                raw = getattr(self.safety, "status", None) or "OK"
            except Exception:
                raw = "OK" # pylint: disable=broad-exception-caught
            ui = None
            if hasattr(self.safety, "status_ui"):
                try:
                    ui = self.safety.status_ui()
                except Exception:
                    ui = None  # pylint: disable=broad-exception-caught

            if ui is None:
                ui = "OK" if raw == "OK" else ("WARN" if raw == "LIMIT" else "ERROR")
            data["safety_raw"] = raw
            data["safety"] = ui
        else:
            data.setdefault("safety_raw", "OK")
            data.setdefault("safety", "OK")

        data.setdefault("ts", time.ticks_ms())

        body = json.dumps(data).encode()
        # JSON nicht cachen
        self._send_response(
            client, 200, "application/json", body,
            extra_headers={
                "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
                "X-Content-Type-Options": "nosniff",
            },
        )

    def _handle_control(self, client, qs) -> None:
        """Kombi-Setter: Speed und Steer in einem Request.

        Query-Parameter:
            - ``speed`` (float, −100..+100)
            - ``steer`` (float, −100..+100)

        Returns:
            None

        Sends:
            200 OK mit ``"OK"`` bei Erfolg, sonst ``400 Bad Request``.
        """
        p = self._parse_qs(qs)
        try:
            speed = float(p.get("speed", "0"))
            steer = float(p.get("steer", "0"))
        except (TypeError, ValueError):
            return self._send_response(client, 400, "text/plain", b"Bad params")

        # in UI-Spanne klemmen
        speed = 100 if speed > 100 else -100 if speed < -100 else speed
        steer = 100 if steer > 100 else -100 if steer < -100 else steer

        # UI-Targets merken
        self.speed_target = int(speed)
        self.steer_target = int(steer)

        # weiterreichen
        try:
            self.on_control(speed, steer)
        except Exception:
            # fallback: getrennte Setter versuchen
            try:
                self.on_set_speed(speed)
                self.on_set_steer(steer)
            except Exception:
                pass  # pylint: disable=broad-exception-caught

        self._send_text(client, "OK")
        return None

    def _handle_center(self, client) -> None:
        try:
            self.on_center()
        except Exception:
            pass  # pylint: disable=broad-exception-caught
        self._send_text(client, "OK")
        return None

    def _handle_set_zero(self, client, qs) -> None:
        """Nullpunkt pro Kanal setzen (oder an externen Handler delegieren).

        Query-Parameter:
            - ``ch`` (int): Kanalindex (UI ggf. 1-basiert → implementiert als 0-basiert).
            - ``v`` / ``val`` (float): Zielwert.

        Returns:
            None
        """
        p = self._parse_qs(qs)
        try:
            ch = int(p.get("ch", "0"))
            val = float(p.get("v", p.get("val", "0")))
            if self.current and hasattr(self.current, "set_zero"):
                # Falls UI 1-basiert zählt → 0-basiert umrechnen:
                self.current.set_zero(ch - 1, val)
            else:
                # Fallback auf alten Callback
                self.on_calib_zero(ch, val)
            return self._send_text(client, "OK")
        except (TypeError, ValueError):
            return self._send_response(client, 400, "text/plain", b"Bad params")

    def _handle_set_scale(self, client, qs) -> None:
        """A/V-Skala pro Kanal setzen (Skalierungsfaktor).

        Query-Parameter:
            - ``ch`` (int): Kanalindex (UI ggf. 1-basiert → implementiert als 0-basiert).
            - ``v`` / ``val`` (float): Faktor.

        Returns:
            None
        """
        p = self._parse_qs(qs)
        try:
            ch = int(p.get("ch", "0"))
            val = float(p.get("v", p.get("val", "1")))
            if self.current and hasattr(self.current, "set_scale"):
                self.current.set_scale(ch - 1, val)  # siehe Hinweis oben
            else:
                self.on_set_scale(ch, val)
            return self._send_text(client, "OK")
        except (TypeError, ValueError):
            return self._send_response(client, 400, "text/plain", b"Bad params")

    def _handle_calibrate_zero(self, client, qs) -> None:
        """Zero-Offsets beider Kanäle über N Samples ermitteln.

        Query-Parameter:
            - ``n`` (int): Anzahl der Samples, Standard: ``64``.

        Returns:
            None
        """
        p = self._parse_qs(qs)
        try:
            n = int(p.get("n", "64"))
            if self.current and hasattr(self.current, "calibrate_zero"):
                self.current.calibrate_zero(n)
            else:
                self.on_calib_zero(n)
            return self._send_text(client, "OK")
        except (TypeError, ValueError):
            return self._send_response(client, 400, "text/plain", b"Bad params")

    def _handle_steering(self, client, qs) -> None:
        """Lenkung separat setzen.

        Endpunkt:
            ``/steering?value=`` (Alias: ``v``/``steer``), Werte −100..+100.

        Returns:
            None
        """
        p = self._parse_qs(qs)
        try:
            val = float(p.get("value", p.get("v", p.get("steer", "0"))))
            val = 100 if val > 100 else -100 if val < -100 else val
            self.steer_target = int(val)
            self.on_set_steer(val)
            return self._send_text(client, "OK")
        except (TypeError, ValueError):
            return self._send_response(client, 400, "text/plain", b"Bad params")

    def _handle_speed(self, client, qs) -> None:
        """Geschwindigkeit separat setzen.

        Endpunkt:
            ``/speed?value=`` (Alias: ``v``/``speed``), Werte −100..+100.

        Returns:
            None
        """
        p = self._parse_qs(qs)
        try:
            val = float(p.get("value", p.get("v", p.get("speed", "0"))))
            val = 100 if val > 100 else -100 if val < -100 else val
            self.speed_target = int(val)
            self.on_set_speed(val)
            return self._send_text(client, "OK")
        except (TypeError, ValueError):
            return self._send_response(client, 400, "text/plain", b"Bad params")

    # -------------------------------------------------------------------------
    # Statische Dateien
    # -------------------------------------------------------------------------
    def _serve_static(self, client, path) -> None:
        """Statische Datei aus ``web_root`` ausliefern (HTML → ``no-store``).

        Beinhaltet eine einfache Härtung gegen Directory-Traversal.

        Args:
            client: Socket des verbundenen Clients.
            path (str): Angeforderter Pfad (z. B. ``"/"`` oder ``"/index.html"``).

        Returns:
            None

        Sends:
            200 OK + Dateiinhalt oder 404/Not Found.
        """
        if path == "/":
            path = "/index.html"

        # sehr einfache Traversal-Guard
        if ".." in path or path.startswith("/.."):
            return self._send_response(client, 404, "text/plain", b"Not Found")

        full = self.web_root + path

        try:
            with open(full, "rb") as f:
                body = f.read()
        except OSError:
            return self._send_response(client, 404, "text/plain", b"Not Found")

        ctype = self._guess_type(path)
        extra = {"X-Content-Type-Options": "nosniff"}
        if ctype.startswith("text/html"):
            extra["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
        return self._send_response(client, 200, ctype, body, extra_headers=extra)

    # -------------------------------------------------------------------------
    # Parser / Utilities
    # -------------------------------------------------------------------------
    @staticmethod
    def _parse_request_line(raw_bytes) -> tuple[str, str]:
        """Parst die Request-Line (z. B. ``GET /path?qs HTTP/1.1``).

        Fällt bei Fehlern auf ``("GET", "/")`` zurück.

        Args:
            raw_bytes (bytes): Rohdaten des Requests.

        Returns:
            tuple[str, str]: ``(method, path_with_qs)``.
        """
        try:
            first = raw_bytes.split(b"\r\n", 1)[0].decode()
            parts = first.split(" ")
            method = parts[0].upper()
            path = parts[1] if len(parts) > 1 else "/"
            return method, path
        except (UnicodeError, IndexError):
            return "GET", "/"

    def _split_qs(self, path):
        """Teilt ``/path?qs`` in Pfad und Querystring.

        Args:
            path (str): Pfad inkl. optionalem Querystring.

        Returns:
            tuple[str, str]: ``(path, qs)``.
        """
        m = self._re_path_qs.match(path)
        if not m:
            return path, ""
        return m.group(1), (m.group(2) or "")

    def _parse_qs(self, qs) -> dict[str, str]:
        """Einfacher Querystring-Parser (``%HH`` und ``+`` → Leerzeichen).

        Args:
            qs (str): Roh-Querystring ohne Fragezeichen.

        Returns:
            dict[str, str]: Schlüssel/Wert-Paare (ohne Typkonvertierung).
        """
        out = {}
        if not qs:
            return out
        for pair in qs.split("&"):
            if not pair:
                continue
            if "=" in pair:
                k, v = pair.split("=", 1)
            else:
                k, v = pair, ""
            out[self._url_unquote(k)] = self._url_unquote(v)
        return out

    @staticmethod
    def _url_unquote(s) -> str:
        """Minimales URL-Decoding (``%HH`` und ``+`` → Leerzeichen).

        Args:
            s (str): Kodierter String.

        Returns:
            str: Dekodierter String.
        """
        res = []
        i = 0
        while i < len(s):
            ch = s[i]
            if ch == "%" and i + 2 < len(s):
                try:
                    res.append(chr(int(s[i+1:i+3], 16)))
                    i += 3
                    continue
                except ValueError:
                    pass
            if ch == "+":
                res.append(" ")
            else:
                res.append(ch)
            i += 1
        return "".join(res)

    @staticmethod
    def _guess_type(path) -> str:
        """Sehr einfache Content-Type-Heuristik (Dateiendung).

        Args:
            path (str): Angeforderter Pfad/Dateiname.

        Returns:
            str: Content-Type.
        """
        p = path.lower()
        if p.endswith(".html") or p.endswith(".htm"):   return "text/html"
        if p.endswith(".css"):                          return "text/css"
        if p.endswith(".js"):                           return "application/javascript"
        if p.endswith(".png"):                          return "image/png"
        if p.endswith(".jpg") or p.endswith(".jpeg"):   return "image/jpeg"
        if p.endswith(".ico"):                          return "image/x-icon"
        return "application/octet-stream"

    # -------------------------------------------------------------------------
    # Senden
    # -------------------------------------------------------------------------
    def _send_text(self, client, s) -> None:
        """Sendet einen einfachen 200/OK-Text-Response (UTF-8).

        Args:
            client: Client-Socket.
            s (str): Payload-Text.

        Returns:
            None
        """
        self._send_response(client, 200, "text/plain", s.encode())

    def _send_response(self, client, code, content_type, body, extra_headers=None) -> None:
        """Sendet eine vollständige HTTP-Antwort (inkl. kurzer Writes-Handling).

        Args:
            client: Client-Socket.
            code (int): HTTP-Statuscode.
            content_type (str): Inhaltstyp (``text/*`` → automatisch UTF-8).
            body (bytes): Antwortkörper.
            extra_headers (dict[str, str] | None): Zusätzliche Header.

        Returns:
            None
        """
        reason = {
            200: "OK",
            400: "Bad Request",
            404: "Not Found",
            405: "Method Not Allowed",
            500: "Internal Server Error",
        }.get(code, "ERR")

        if content_type.startswith("text/") and "charset=" not in content_type:
            content_type += "; charset=utf-8"

        hdr = "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nConnection: close\r\n" % (
            code, reason, content_type, len(body)
        )
        if extra_headers:
            for k, v in extra_headers.items():
                hdr += "%s: %s\r\n" % (k, v)
        hdr += "\r\n"

        sent_hdr = self._send_all(client, hdr.encode())
        sent_body = self._send_all(client, body)

    @staticmethod
    def _send_all(client, buf) -> int:
        """Sendet alle Bytes robust (mit kurzem Warten bei blockierendem Socket).

        Args:
            client: Client-Socket.
            buf (bytes): Zu sendender Puffer.

        Returns:
            int: Anzahl tatsächlich gesendeter Bytes.
        """
        mv = memoryview(buf)
        total = 0
        while total < len(mv):
            try:
                n = client.send(mv[total:])
                if n is None:
                    n = 0
                total += n
            except OSError:
                try:
                    time.sleep_ms(5)
                except AttributeError:
                    pass  # MicroPython/host unterscheiden sich  # pylint: disable=broad-exception-caught

        return total
