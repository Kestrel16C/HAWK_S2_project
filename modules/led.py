# modules/led.py
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

"""Nicht-blockierender LED-Statusblinker für Mainloops.

Dieses Modul stellt die Klasse `LedBlinker` bereit, um eine Status-LED ohne Blockierungen zu steuern. Ein periodischer Aufruf von `tick()` in der Hauptschleife übernimmt das Umschalten je nach gewähltem Muster.

Examples:

    from modules.led import LedBlinker
    led = LedBlinker()
    led.set_pattern("fast")
    while True:
        led.tick()
        # ... weitere Mainloop-Arbeiten ...

Hinweis:
    - Muster: ``"off"`` (aus), ``"solid"`` (an), ``"fast"`` (~5 Hz), ``"slow"`` (~0,8 Hz).
    - Die Klasse blockiert nie; sie toggelt nur an berechneten Zeitpunkten.
"""

from machine import Pin
import time

__all__ = ["LedBlinker"]

class LedBlinker:
    """Leichter, nicht-blockierender LED-Blinker für Statusanzeigen.

    Args:
        pin_name (str | int): Name oder Nummer des LED-Pins. Auf RP2040 meist ``"LED"``; Fallback ist Pin ``25``.
        active_high (bool): ``True`` = LED an bei Pegel 1, ``False`` = invertierte Polarität.

    Attributes:
        _pattern (str): Aktuelles Muster (``"off"``, ``"solid"``, ``"fast"``, ``"slow"``).
        _state (int): Interner LED-Zustand (0 = aus, 1 = an).
        _next (int): Nächster Umschaltzeitpunkt in ``ticks_ms()``.
    """

    def __init__(self, pin_name="LED", active_high=True) -> None:
        self._active_high = bool(active_high)

        # Hardware initialisieren – robust mit Fallback
        self._led = None
        try:
            self._led = Pin(pin_name, Pin.OUT)  # "LED" alias oder int
        except (ValueError, OSError, RuntimeError, TypeError):
            # Fallback auf den üblichen Onboard-LED-Pin (RP2040: 25)
            try:
                self._led = Pin(25, Pin.OUT)
            except (ValueError, OSError, RuntimeError, TypeError) as exc:
                print("[LED ERROR]", exc)
                self._led = None

        # Zustand
        self._pattern = "off"
        self._state = 0            # 0=aus, 1=an
        self._next = 0             # nächster Umschaltzeitpunkt (ticks_ms)

        self._apply()  # sicher aus

    # ---------------- Öffentliche API ----------------

    def set_pattern(self, pattern: str) -> None:
        """Wechselt das Blinkmuster.

        Unbekannte Muster werden auf ``"slow"`` normalisiert.

        Args:
            pattern (str): ``"off"``, ``"solid"``, ``"fast"``, ``"slow"``.

        Returns:
            None
        """
        if pattern not in ("off", "solid", "fast", "slow"):
            pattern = "slow"
        if pattern == self._pattern:
            return  # keine Änderung

        prev = self._pattern
        self._pattern = pattern

        if pattern == "off":
            self._state = 0
            self._apply()
        elif pattern == "solid":
            self._state = 1
            self._apply()
        else:
            # blinkende Muster starten sichtbar mit „an“
            self._state = 1
            self._apply()
            self._next = time.ticks_add(time.ticks_ms(), self._period_ms())

    def tick(self) -> None:
        """Regelmäßig in der Mainloop aufrufen; schaltet nur bei Fälligkeit um.

        Returns:
            None
        """
        if not self._led or self._pattern in ("off", "solid"):
            return

        now = time.ticks_ms()
        if time.ticks_diff(now, self._next) >= 0:
            self._state ^= 1
            self._apply()
            self._next = time.ticks_add(now, self._period_ms())

    # ---------------- Interna ----------------

    def _apply(self) -> None:
        """Schreibt den aktuellen Zustand auf die Hardware-LED (unter Beachtung der Polarität).

        Returns:
            None
        """
        try:
            if self._led:
                level = self._state if self._active_high else (0 if self._state else 1)
                self._led.value(level)
        except Exception as e:
            print("[LED ERROR]", e)

    def _period_ms(self) -> int:
        """Gibt die Blinkperiode (ms) abhängig vom aktuellen Muster zurück.

        Returns:
            int: Periode in Millisekunden.
        """
        if self._pattern == "fast":
            return 200   # ~5 Hz (100 ms an/aus)
        if self._pattern == "slow":
            return 1200  # ~0.8 Hz (0.6 s an/aus)
        return 1000      # Default (wird bei 'off'/'solid' nicht genutzt)
