# modules/steering.py
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

"""PWM-Servosteuerung für ein Lenkservo.

Dieses Modul stellt die Klasse `Steering` bereit, um ein RC-Lenkservo über PWM anzusteuern.
Pulsbreiten (µs) sind kalibrierbar, eine Hysterese unterdrückt
Jitter, und die Eingabe kann wahlweise in Prozent oder Grad erfolgen.

Examples:

    from modules.steering import Steering
    st = Steering(
            pin=15, pwm_freq_hz=50,
            min_us=900, max_us=2100, center_us=1500,
            deadband_us=10,
            angle_min=-40, angle_max=40,
            trim_deg=0.0,
            invert=False,
        )
    st.set_percent(+20)  # ~leicht rechts
    st.set_angle(-30)    # 30° links
    st.center()          # zurück zur Mitte
"""

from machine import Pin, PWM
import time

__all__ = ["Steering"]

class Steering:
    """Steuerung eines Lenkservos über PWM.

    Args:
        pin (int): GPIO-Pin (PWM-fähig).
        pwm_freq_hz (int): PWM-Frequenz in Hz (bei Servos üblich: 50 Hz).
        min_us (int): Pulsbreite in µs am linken Anschlag.
        max_us (int): Pulsbreite in µs am rechten Anschlag.
        center_us (int): Pulsbreite in µs in Mittelstellung.
        deadband_us (int): Kleinste Pulsbreitenänderung (µs), die ausgegeben wird.
        angle_min (float): Kleinster Winkel (Grad), typ. negativ (z. B. −40).
        angle_max (float): Größter Winkel (Grad), typ. positiv (z. B. +40).
        trim_deg (float): Mittelpunktversatz in Grad (additiv auf den Winkel).
        invert (bool): Drehrichtung invertieren.

    Raises:
        ValueError: Wenn `min_us < center_us < max_us` nicht erfüllt ist
            oder `angle_min >= angle_max`.

    Attributes:
        us_min (int): Aktuelle minimale Pulsbreite in µs.
        us_mid (int): Aktuelle Pulsbreite der Mittelstellung in µs.
        us_max (int): Aktuelle maximale Pulsbreite in µs.
        angle_min (float): Aktueller minimaler Winkel in Grad.
        angle_max (float): Aktueller maximaler Winkel in Grad.
        servo (PWM): PWM-Objekt des Servos.
        _period_us (int): PWM-Periodendauer in µs (z. B. 20_000 bei 50 Hz).
        _hyst (int): Hysterese in Duty-Schritten (16 Bit).
        _last_duty (int): Zuletzt gesetzter Duty-Wert (0..65535).
        _last_update (int): Zeitstempel (ms) der letzten Ausgabe.
        _invert (bool): Ob die Richtung invertiert wird.
        _trim_deg (float): Trim-Wert in Grad.
    """

    def __init__(
        self,
        *,
        pin: int,
        pwm_freq_hz: int,
        min_us: int,
        max_us: int,
        center_us: int,
        deadband_us: int,
        angle_min: float,
        angle_max: float,
        trim_deg: float,
        invert: bool,
    ) -> None:
        # --- Parameter prüfen/speichern -------------------------------------
        if not (min_us < center_us < max_us):
            raise ValueError("Ungültige Pulsbreiten: erwarte min_us < center_us < max_us")

        self._invert = bool(invert)
        self._trim_deg = float(trim_deg)

        self.us_min = int(min_us)
        self.us_mid = int(center_us)
        self.us_max = int(max_us)

        self.angle_min = float(angle_min)
        self.angle_max = float(angle_max)
        if self.angle_min >= self.angle_max:
            raise ValueError("angle_min muss < angle_max sein")

        # --- PWM init --------------------------------------------------------
        self.servo = PWM(Pin(pin))
        self.servo.freq(int(pwm_freq_hz))

        # Periodendauer in µs (z. B. 20_000 µs bei 50 Hz)
        self._period_us = int(1_000_000 // int(pwm_freq_hz))

        # Deadband (µs) in Duty-Schritte umrechnen
        self._hyst = self._us_to_duty(int(deadband_us))

        # Zustand
        self._last_duty = self._us_to_duty(self.us_mid)
        self._last_update = time.ticks_ms()

        # --- Slew-Rate-Begrenzung (Servo-/Getriebeschutz) -------------------
        # Begrenzt die Winkelgeschwindigkeit, damit die träge Motor-Masse
        # nicht mit vollem Servo-Tempo herumgerissen wird (Getriebe-Schnappen
        # beim abrupten Stopp). Konvergiert, weil die Hauptschleife den
        # Sollwert mit ~100 Hz wiederholt setzt. 0 oder None = deaktiviert.
        self.slew_deg_per_s = 200.0
        self._cur_angle = 0.0
        self._last_slew_ms = time.ticks_ms()

        # Start in Mittelstellung
        self.center()

    # ---------------- Öffentliche API ----------------

    def set_percent(self, percent: float) -> None:
        """Lenkwinkel per Prozent setzen.

        Mappt linear ``-100 .. +100`` auf ``[angle_min .. angle_max]``,
        berücksichtigt ``invert`` und ``trim``.

        Args:
            percent (float): Prozentwert (wird auf ``[-100, 100]`` begrenzt).

        Returns:
            None
        """
        p = max(-100.0, min(100.0, float(percent)))
        if self._invert:
            p = -p

        # -100 → angle_min, 0 → 0°, +100 → angle_max
        if p >= 0.0:
            angle = (p / 100.0) * self.angle_max
        else:
            angle = (p / 100.0) * (-self.angle_min)  # p<0 → skaliert auf |angle_min|
        self.set_angle(angle)

    def set_angle_percent(self, percent: float) -> None:
        """Alias für Kompatibilität — entspricht ``set_percent()``.

        Args:
            percent (float): Prozentwert (``-100 .. +100``).

        Returns:
            None
        """
        self.set_percent(percent)

    def set_angle(self, angle_deg: float) -> None:
        """Lenkwinkel in Grad setzen (clamp + Trim, symmetrisch um Mitte).

        Args:
            angle_deg (float): Gewünschter Winkel (Grad).

        Notes:
            - Der effektive Winkel ist ``angle_deg + trim_deg`` und wird auf
              ``[angle_min, angle_max]`` begrenzt.
            - Die Pulsbreite wird für negative/positive Winkel getrennt
              gegen ``min_us``/``max_us`` interpoliert.

        Returns:
            None
        """
        a = float(angle_deg) + self._trim_deg
        if a < self.angle_min:
            a = self.angle_min
        elif a > self.angle_max:
            a = self.angle_max
            
        # --- Slew-Rate-Begrenzung: nur schrittweise Richtung Ziel bewegen ---
        now = time.ticks_ms()
        dt = time.ticks_diff(now, self._last_slew_ms)
        self._last_slew_ms = now
        if self.slew_deg_per_s and 0 < dt < 500:
            max_step = self.slew_deg_per_s * (dt / 1000.0)
            diff = a - self._cur_angle
            if diff > max_step:
                a = self._cur_angle + max_step
            elif diff < -max_step:
                a = self._cur_angle - max_step
        self._cur_angle = a

        # Symmetrische Interpolation um die Mitte:
        if a >= 0:
            # 0..angle_max → center..max
            span = (self.us_max - self.us_mid)
            us = self.us_mid + (a / self.angle_max) * span if self.angle_max != 0 else self.us_mid
        else:
            # angle_min..0 → min..center
            span = (self.us_mid - self.us_min)
            us = self.us_mid + (a / abs(self.angle_min)) * span if self.angle_min != 0 else self.us_mid

        duty = self._us_to_duty(us)

        # Hysterese: nur schreiben, wenn Änderung groß genug ist
        if abs(duty - self._last_duty) > self._hyst:
            self.servo.duty_u16(duty)
            now = time.ticks_ms()
            self._last_duty = duty
            self._last_update = now

    def center(self) -> None:
        """Servo in die Mittelstellung (0°) fahren.

        Returns:
            None
        """
        self.set_angle(0.0)

    # ------- Laufzeit-Anpassungen / Kalibrierung -------

    def calibrate(self, *, min_us=None, center_us=None, max_us=None) -> None:
        """Pulsbreiten zur Laufzeit anpassen.

        Nur geänderte Werte übergeben; nach der Kalibrierung wird aus
        Sicherheitsgründen in die Mitte gefahren.

        Args:
            min_us (int | None): Neue minimale Pulsbreite in µs.
            center_us (int | None): Neue Mittel-Pulsbreite in µs.
            max_us (int | None): Neue maximale Pulsbreite in µs.

        Returns:
            None

        Raises:
            ValueError: Wenn die Relation ``min < center < max`` verletzt ist.
        """
        new_min = int(min_us if min_us is not None else self.us_min)
        new_mid = int(center_us if center_us is not None else self.us_mid)
        new_max = int(max_us if max_us is not None else self.us_max)
        if not (new_min < new_mid < new_max):
            raise ValueError("Ungültige Kalibrierung (min < center < max erforderlich)")
        self.us_min, self.us_mid, self.us_max = new_min, new_mid, new_max
        self.center()

    def set_limits(self, *, angle_min=None, angle_max=None, trim_deg=None) -> None:
        """Grenzwinkel/Trim zur Laufzeit anpassen.

        Args:
            angle_min (float | None): Neuer Minimalwinkel in Grad.
            angle_max (float | None): Neuer Maximalwinkel in Grad.
            trim_deg (float | None): Neuer Trim-Wert in Grad.

        Returns:
            None

        Raises:
            ValueError: Wenn ``angle_min >= angle_max``.
        """
        if angle_min is not None:
            self.angle_min = float(angle_min)
        if angle_max is not None:
            self.angle_max = float(angle_max)
        if self.angle_min >= self.angle_max:
            raise ValueError("angle_min muss < angle_max sein")
        if trim_deg is not None:
            self._trim_deg = float(trim_deg)

    # ---------------- Interna ----------------

    def _us_to_duty(self, pulse_us: float) -> int:
        """Wandelt eine Pulsbreite (µs) in einen 16-bit-Duty-Wert für ``duty_u16()`` um.

        Formel:
            ``duty = 65535 * (pulse_us / period_us)``

        Args:
            pulse_us (float): Pulsbreite in µs.

        Returns:
            int: Duty-Wert (0..65535), auf die PWM-Periode begrenzt.
        """
        pu = max(0, min(int(pulse_us), self._period_us))  # Begrenzen auf die PWM-Periode
        return int((pu * 65535) // self._period_us)
