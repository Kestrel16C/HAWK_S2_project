# modules/net.py
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

"""NetworkManager – einfache AP/STA-Verwaltung für MicroPython (z. B. Pico W).

Dieses Modul stellt die Klasse `NetworkManager` bereit, um Access-Point (AP)
und Station (STA) konsistent zu konfigurieren und zu starten/stoppen. Optional
kann im AP-Modus eine einfache 1/6/11-Kanalheuristik genutzt werden.

Examples:

    from modules.net import NetworkManager

    net = NetworkManager(
            country="DE",
            ap_password="12345678"
            )

    ip_ap = net.start_ap(channel=None)  # Auto-Channel
    sta_ip = net.connect_sta()          # nutzt sta_ssid/sta_password (falls gesetzt)

Hinweis:
    AP = Pico fungiert als Access Point.
    STA = Pico verbindet sich mit vorhandenem WLAN.
"""

import time
try:
    import network
except ImportError:
    network = None  # Laufzeit ohne WLAN-Stack (z. B. beim Doku-Build)


def _set_country(code="DE") -> None:
    """Setzt die Regulatory Domain (falls vom Port unterstützt).

    Args:
        code (str): Zweibuchstabige Länderkennung, z. B. "DE".

    Returns:
        None
    """
    try:
        import rp2
        rp2.country(code)
    except (ImportError, AttributeError, OSError, RuntimeError):
        # unkritisch – nicht jeder Port unterstützt das
        pass


def _uid_hex() -> str:
    """Liefert die Chip-UID als Hex-String.

    Fällt bei fehlender Unterstützung auf eine Pseudo-ID zurück.

    Returns:
        str: Hexadezimale UID (Fallback: Pseudo-ID oder "unknown").
    """
    try:
        import machine, ubinascii
        return ubinascii.hexlify(machine.unique_id()).decode()
    except (ImportError, AttributeError):
        # Fallback: CPU-Ticks (MicroPython) → kann auf CPython fehlen
        try:
            return "%08x" % (time.ticks_cpu() & 0xFFFFFFFF)
        except (AttributeError, TypeError, ValueError):
            return "unknown"


__all__ = ["NetworkManager"]


class NetworkManager:
    """Kleiner Helfer, um AP/STA konsistent zu starten/stoppen.

    Args:
        country (str): Länderkennung für WLAN-Bereich (Regulatory Domain).
        ssid_prefix (str): Präfix der automatisch generierten AP-SSID.
        ap_password (str): Passwort für den AP-Modus (mind. 8 Zeichen empfohlen).
        ap_channel (int | None): Kanalwahl (2.4 GHz); ``None`` = Auto-Channel (1/6/11-Heuristik).
        ap_hidden (bool): SSID verbergen.
        ap_max_clients (int): Maximale Anzahl AP-Clients.
        ap_ip_wait_ms (int): Wartezeit (ms) bis IP im AP-Interface verfügbar ist.
        sta_ssid (str | None): SSID für den STA-Modus (optional).
        sta_password (str | None): Passwort für den STA-Modus (optional).
        sta_timeout_ms (int): Verbindungs-Timeout (ms) im STA-Modus.

    Attributes:
        country (str): Konfigurierte Regulatory Domain.
        uid (str): Gerätespezifische UID (Hex).
        ap_ssid (str): Vorgeschlagene/standardmäßige AP-SSID.
        ap_password (str): Standardpasswort für AP.
        ap_channel (int | None): Vorgegebener AP-Kanal oder ``None``.
        ap_hidden (bool): Sichtbarkeit der SSID.
        ap_max_clients (int): Maximal zulässige AP-Clients.
        ap_ip_wait_ms (int): Wartezeit bis IP im AP-Mode.
        sta_ssid (str | None): Vorkonfigurierte STA-SSID.
        sta_password (str | None): Vorkonfiguriertes STA-Passwort.
        sta_timeout_ms (int): STA-Timeout in Millisekunden.
        ap: Laufzeit-Handle auf `network.WLAN(network.AP_IF)` oder ``None``.
        sta: Laufzeit-Handle auf `network.WLAN(network.STA_IF)` oder ``None``.
    """

    def __init__(self, *,
                 country="DE",
                 ssid_prefix="PicoCar-",
                 # AP-Defaults
                 ap_password="12345678",
                 ap_channel=None,
                 ap_hidden=False,
                 ap_max_clients=4,
                 ap_ip_wait_ms=2000,
                 # STA-Defaults
                 sta_ssid=None,
                 sta_password=None,
                 sta_timeout_ms=10000):
        # Basis / Identität
        self.country = country
        self.uid = _uid_hex()
        self.ap_ssid = (ssid_prefix or "") + self.uid

        # AP-Defaults
        self.ap_password = ap_password
        self.ap_channel = ap_channel
        self.ap_hidden = bool(ap_hidden)
        self.ap_max_clients = ap_max_clients
        self.ap_ip_wait_ms = int(ap_ip_wait_ms)

        # STA-Defaults
        self.sta_ssid = sta_ssid
        self.sta_password = sta_password
        self.sta_timeout_ms = int(sta_timeout_ms)

        # Laufzeit-Interfaces
        self.ap = None
        self.sta = None

    # ---------------- interne Hilfen ----------------

    @staticmethod
    def _pick_best_channel(uid: str) -> int:
        """Deterministische Kanalwahl für viele APs: verteilt über mehrere Kanäle.

        Nutzt eine UID-basierte Verteilung auf [1,3,5,7,9,11,13], um Co-Channel-
        Interference zu reduzieren, wenn viele Pico-APs gleichzeitig laufen.
        """
        cands = [1, 3, 5, 7, 9, 11, 13]
        try:
            x = int(uid[-2:], 16)  # letzte 2 Hex-Zeichen → 0..255
        except Exception:
            # Fallback falls UID-Format unerwartet ist
            try:
                x = time.ticks_cpu() & 0xFF
            except Exception:
                x = 0
        return cands[x % len(cands)]

    # ---------------- Access Point ----------------

    def start_ap(self,
                 ssid=None,
                 password=None,
                 channel=None,
                 hidden=None,
                 max_clients=None,
                 ip_wait_ms=None) -> str:
        """Startet den AP und gibt seine IP zurück (z. B. ``"192.168.4.1"``).

        Args:
            ssid (str | None): SSID des AP; ``None`` verwendet die generierte.
            password (str | None): Passwort; ``None`` verwendet das Standardpasswort.
            channel (int | None): Kanal (2.4 GHz); ``None`` = Auto-Channel.
            hidden (bool | None): SSID verbergen; ``None`` verwendet den Standard.
            max_clients (int | None): Maximale Clientanzahl.
            ip_wait_ms (int | None): Wartezeit (ms) bis IP bereit.

        Returns:
            str: Zugeteilte IP-Adresse des AP-Interfaces (Fallback: ``"192.168.4.1"``).

        Raises:
            RuntimeError: Wenn das `network`-Modul nicht verfügbar ist.
        """
        if network is None:
            raise RuntimeError("network module not available")

        # Effektive Parameter (Defaults + Overrides)
        ssid        = self.ap_ssid if ssid is None else ssid
        password    = self.ap_password if password is None else password
        channel     = self.ap_channel if channel is None else channel
        hidden      = self.ap_hidden if hidden is None else bool(hidden)
        max_clients = self.ap_max_clients if max_clients is None else max_clients
        ip_wait_ms  = self.ap_ip_wait_ms if ip_wait_ms is None else int(ip_wait_ms)

        _set_country(self.country)

        # Auto-Channel?
        if channel is None:
            try:
                channel = self._pick_best_channel(self.uid)
            except (OSError, RuntimeError, ValueError, AttributeError):
                channel = 1

        # Interface neu aufsetzen
        self.ap = network.WLAN(network.AP_IF)
        try:
            self.ap.active(False)
        except (OSError, AttributeError, RuntimeError):
            pass

        use_pw = bool(password and len(password) >= 8)

        # Konfigurationsvarianten (Port-Kompatibilität)
        cfgs = []
        if use_pw:
            cfgs.append(dict(ssid=ssid, key=password, channel=channel, hidden=hidden))
            cfgs.append(dict(essid=ssid, password=password, channel=channel, hidden=hidden))
        else:
            cfgs.append(dict(ssid=ssid, channel=channel, hidden=hidden))
            cfgs.append(dict(essid=ssid, channel=channel, hidden=hidden))

        if max_clients is not None:
            for cfg in cfgs:
                cfg.setdefault("max_clients", max_clients)

        configured = False
        for i, cfg in enumerate(cfgs, 1):
            try:
                self.ap.config(**cfg)
                configured = True
                break
            except (TypeError, ValueError, OSError, RuntimeError) as e:
                pass

        if not configured:
            # Minimaler Fallback
            try:
                if use_pw:
                    self.ap.config(ssid=ssid, key=password)
                else:
                    self.ap.config(ssid=ssid)
            except (TypeError, ValueError, OSError, RuntimeError) as e:
                print("start_ap fallback:", e)

        # Aktivieren
        try:
            self.ap.active(True)
            time.sleep_ms(200)
        except (OSError, RuntimeError, ValueError, AttributeError) as e:
            print("start_ap failed:", e)
            raise

        # kurze Wartezeit, bis IP steht
        t0 = time.ticks_ms()
        ip = None
        while time.ticks_diff(time.ticks_ms(), t0) <= ip_wait_ms:
            try:
                ip, _mask, _gw, _dns = self.ap.ifconfig()
                if ip and ip != "0.0.0.0":
                    break
            except (OSError, RuntimeError, ValueError, AttributeError):
                pass
            time.sleep_ms(100)

        if not ip:
            ip = "192.168.4.1"  # üblicher AP-Default

        return ip

    def stop_ap(self) -> None:
        """Deaktiviert den Access Point (falls aktiv).

        Returns:
            None
        """
        if self.ap:
            try:
                self.ap.active(False)
            except (OSError, RuntimeError, AttributeError) as e:
                print("stop_ap failed:", e)

    # ---------------- Station (optional) ----------------

    def connect_sta(self, ssid=None, password=None, timeout_ms=None) -> str | None:
        """Verbindet im Station-Mode mit einem WLAN und gibt die IP zurück.

        Args:
            ssid (str | None): Ziel-SSID; ``None`` verwendet die vorkonfigurierte.
            password (str | None): Passwort; ``None`` verwendet das vorkonfigurierte.
            timeout_ms (int | None): Verbindungs-Timeout (ms).

        Returns:
            str | None: Zugewiesene IP-Adresse oder ``None``, falls unbekannt.

        Raises:
            RuntimeError: Wenn das `network`-Modul nicht verfügbar ist.
            ValueError: Wenn keine SSID angegeben/konfiguriert ist.
            TimeoutError: Wenn innerhalb von ``timeout_ms`` keine Verbindung zustande kommt.
            Exception: Weitergereichte Fehler des Ports beim Verbindungsaufbau.
        """
        if network is None:
            raise RuntimeError("network module not available")

        # Effektive Parameter
        ssid = self.sta_ssid if ssid is None else ssid
        password = self.sta_password if password is None else password
        timeout_ms = self.sta_timeout_ms if timeout_ms is None else int(timeout_ms)

        if not ssid:
            raise ValueError("STA SSID fehlt (setzen im Konstruktor oder beim Aufruf)")

        _set_country(self.country)

        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)

        if not self.sta.isconnected():
            try:
                if password:
                    self.sta.connect(ssid, password)
                else:
                    self.sta.connect(ssid)
            except (OSError, RuntimeError, ValueError):
                # Port-spezifischer Verbindungsfehler: unverändert weiterreichen
                raise

            t0 = time.ticks_ms()
            while not self.sta.isconnected():
                if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                    raise TimeoutError("STA connect timeout")
                time.sleep_ms(100)

        try:
            ip = self.sta.ifconfig()[0]
        except (OSError, RuntimeError, AttributeError, IndexError):
            ip = None

        return ip

    def disconnect_sta(self) -> None:
        """Trennt die Station-Verbindung (falls aktiv).

        Returns:
            None
        """
        if self.sta:
            try:
                if self.sta.isconnected():
                    self.sta.disconnect()
                self.sta.active(False)
            except (OSError, RuntimeError, AttributeError) as e:
                print("disconnect_sta failed:", e)

    # ---------------- Helpers ----------------

    def ip(self) -> str | None:
        """Liefert bevorzugt die STA-IP, sonst die AP-IP, sonst ``None``.

        Returns:
            str | None: Aktuelle IP-Adresse oder ``None``.
        """
        try:
            if self.sta and self.sta.isconnected():
                return self.sta.ifconfig()[0]
        except (OSError, RuntimeError, AttributeError, IndexError):
            pass
        try:
            if self.ap and getattr(self.ap, "active", lambda: False)():
                return self.ap.ifconfig()[0]
        except (OSError, RuntimeError, AttributeError, IndexError):
            pass
        return None

    def stations(self) -> list[tuple] | None:
        """Liste verbundener AP-Clients (falls vom Port unterstützt).

        Returns:
            list[tuple] | None: Port-spezifisches Format (z. B. MAC/RSSI) oder ``None``
            falls nicht unterstützt oder kein AP aktiv.
        """
        try:
            if not self.ap:
                return None
            s = self.ap.status("stations")
            return s
        except (OSError, RuntimeError, AttributeError, ValueError):
            return None
