# motor_encoder_test.py
# ISOLIERTER Motor-/Encoder-Test: kein Webserver, kein CurrentMonitor,
# kein ToF, kein Safety. Nur DriveController + Lenkservo-Zentrierung.
# ACHTUNG: KEIN Kollisionsschutz aktiv — freie Strecke, Kabel im Blick!
#
# REPL-Befehle:
#   drive(0.5)          # 0.5 m vorwärts mit Standard-Power
#   drive(0.5, pct=70)  # mit 70% Power
#   drive(-0.3)         # rückwärts
#   rpm()               # 50 rohe RPM-Samples anzeigen (Motor von Hand drehen
#                       #   oder während roll() in zweitem Anlauf)
#   roll(2000, pct=60)  # 2s Zeitfahrt, Distanz wird mitprotokolliert
#   stop()              # Not-Halt

from machine import Pin, PWM
from secure.drive import DriveController
import time

# --- Kinematik (wie hipe.py — Faktor wird ja gerade kalibriert) --------------
KIN = {
    "pulses_per_rev": 16,
    "gear_ratio": 6.3,
    "wheel_diameter": 0.02,
    "invert_dir": False,
}
ODO_MAX_RPM = 1000          # Plausibilitätsgrenze (Spikes verwerfen + zählen)
LOOP_MS     = 10

drv = DriveController(**KIN)

# --- Lenkservo hart zentrieren (1500us, direkt, ohne Steering-Modul) ---------
_servo = PWM(Pin(6))
_servo.freq(50)
_servo.duty_u16(int(1500 * 65535 // 20000))
print("Lenkung zentriert (1500us auf GP6).")


def _set(pct):
    if hasattr(drv, "set_percent"):
        drv.set_percent(pct)
    else:
        drv.set_speed_percent(pct)


def stop():
    _set(0)
    print("STOP.")


def rpm(n=50, dt_ms=20):
    """Rohe get_rpm()-Samples anzeigen — Rauschen/Spikes direkt sichtbar."""
    for i in range(n):
        print("%3d: %.1f" % (i, drv.get_rpm()))
        time.sleep_ms(dt_ms)


def drive(dist_m, pct=55, boost_pct=80, boost_ms=250, timeout_ms=15000):
    """Fährt dist_m (negativ = rückwärts) und protokolliert alles."""
    direction = 1 if dist_m > 0 else -1
    target = abs(dist_m)
    circ = 3.141592653589793 * KIN["wheel_diameter"]

    dist = 0.0
    rejected = 0
    max_rpm = 0.0
    t_start = time.ticks_ms()
    t_last = t_start
    print("[TEST] Start: Soll=%.3f m, pct=%d (Boost %d/%dms)" % (target, pct, boost_pct, boost_ms))

    try:
        # --- Fahrphase ---
        while dist < target:
            now = time.ticks_ms()
            if time.ticks_diff(now, t_start) > timeout_ms:
                print("[TEST] TIMEOUT bei %.3f m" % dist)
                break

            p = boost_pct if time.ticks_diff(now, t_start) < boost_ms else pct
            _set(direction * p)

            r = abs(drv.get_rpm())
            dt = time.ticks_diff(now, t_last)
            t_last = now
            if r > max_rpm:
                max_rpm = r
            if r > ODO_MAX_RPM:
                rejected += 1
            elif 0 < dt < 500:
                dist += (r / 60.0) * (dt / 1000.0) * circ

            time.sleep_ms(LOOP_MS)

        cutoff = dist
        t_cut = time.ticks_diff(time.ticks_ms(), t_start)
        _set(0)

        # --- Auslauf beobachten ---
        t_coast = time.ticks_ms()
        while True:
            now = time.ticks_ms()
            r = abs(drv.get_rpm())
            dt = time.ticks_diff(now, t_last)
            t_last = now
            if r > ODO_MAX_RPM:
                rejected += 1
            elif 0 < dt < 500:
                dist += (r / 60.0) * (dt / 1000.0) * circ
            if r < 1.0 or time.ticks_diff(now, t_coast) > 2000:
                break
            time.sleep_ms(LOOP_MS)

    finally:
        _set(0)   # Motor IMMER aus, auch bei Ctrl-C

    print("[TEST] Abschaltpunkt: %.3f m nach %d ms" % (cutoff, t_cut))
    print("[TEST] Endstand inkl. Auslauf: %.3f m (Auslauf %.3f m)" % (dist, dist - cutoff))
    print("[TEST] max RPM: %.0f | verworfene Spikes: %d" % (max_rpm, rejected))
    print("[TEST] --> Jetzt REAL gefahrene Strecke messen!")
    print("[TEST]     Faktor = real_m / %.3f  (auf wheel_diameter anwenden)" % dist)


def roll(ms, pct=55):
    """Zeitfahrt: pct für ms Millisekunden, Distanz wird integriert."""
    circ = 3.141592653589793 * KIN["wheel_diameter"]
    dist = 0.0
    t_start = time.ticks_ms()
    t_last = t_start
    try:
        _set(pct)
        while time.ticks_diff(time.ticks_ms(), t_start) < ms:
            now = time.ticks_ms()
            r = abs(drv.get_rpm())
            dt = time.ticks_diff(now, t_last)
            t_last = now
            if r <= ODO_MAX_RPM and 0 < dt < 500:
                dist += (r / 60.0) * (dt / 1000.0) * circ
            time.sleep_ms(LOOP_MS)
    finally:
        _set(0)
    print("[ROLL] %d ms @ %d%% -> integriert %.3f m" % (ms, pct, dist))


print("Bereit: drive(m, pct=..), roll(ms, pct=..), rpm(), stop()")