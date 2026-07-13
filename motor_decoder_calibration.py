# motor_encoder_test.py  v3
# ISOLIERTER Motor-/Encoder-Test — Pulszählung per Busy-Polling auf GP0.
# (PIO-Ansatz verworfen: Ressourcenkonflikt mit frozen DriveController
#  vermutet; Polling nachweislich zuverlässig bei unseren Pulsraten.)
#
# REPL-Befehle:
#   drive(0.5)            # fährt bis Zähler 0.5 m meldet (Standard 20%)
#   drive(0.5, pct=30)
#   roll(2000, pct=20)    # Zeitfahrt, zählt Pulse
#   irq_test()            # Entscheidungstest für hipe-Integration (s.u.)
#   stop()
#
# KALIBRIERUNG: drive/roll fahren, REAL messen, dann:
#   M_PER_PULSE = real_m / gezählte Pulse   -> unten eintragen.

from machine import Pin
from secure.drive import DriveController
import time

# #############################################################################
# ##  KALIBRIERWERT — nach Messfahrt ersetzen: M_PER_PULSE = real_m / Pulse ##
# #############################################################################
M_PER_PULSE = 0.00272     # Startschätzung (60mm-Rad, 69.3 Pulse/Umdrehung)
# #############################################################################

KIN = {"pulses_per_rev": 16, "gear_ratio": 6.3,
       "wheel_diameter": 0.02, "invert_dir": False}

drv = DriveController(**KIN)

from machine import PWM
_servo = PWM(Pin(6))
_servo.freq(50)
_servo.duty_u16(int(1500 * 65535 // 20000))
print("Lenkung zentriert (1500us auf GP6).")

_enc = Pin(0, Pin.IN)


def _set(pct):
    if hasattr(drv, "set_percent"):
        drv.set_percent(pct)
    else:
        drv.set_speed_percent(pct)


def stop():
    _set(0)
    print("STOP.")


def _count_edges_for(duration_ms, until_pulses=None, base=0):
    """Busy-Poll: zählt Flanken auf GP0. Stoppt nach duration_ms ODER wenn
    (base + Pulse) >= until_pulses. Gibt (Pulse, abgelaufene_ms) zurück."""
    edges = 0
    last = _enc.value()
    t0 = time.ticks_ms()
    check = 0
    while True:
        v = _enc.value()
        if v != last:
            edges += 1
            last = v
        check += 1
        if check >= 64:            # Zeit nur alle 64 Iterationen prüfen (Tempo!)
            check = 0
            el = time.ticks_diff(time.ticks_ms(), t0)
            if el >= duration_ms:
                break
            if until_pulses is not None and base + edges // 2 >= until_pulses:
                break
    return edges // 2, time.ticks_diff(time.ticks_ms(), t0)


def drive(dist_m, pct=20, boost_pct=80, boost_ms=250, timeout_ms=15000):
    """Fährt bis der Pulszähler dist_m meldet. Zählt lückenlos weiter
    durch Boost, Cruise und Auslauf."""
    direction = 1 if dist_m > 0 else -1
    target_pulses = int(abs(dist_m) / M_PER_PULSE)
    total = 0
    print("[TEST] Start: Soll=%.3f m = %d Pulse, pct=%d (Boost %d/%dms)"
          % (abs(dist_m), target_pulses, pct, boost_pct, boost_ms))

    try:
        # Boost-Phase (zählt mit)
        _set(direction * boost_pct)
        p, _ = _count_edges_for(boost_ms, until_pulses=target_pulses, base=0)
        total += p

        # Cruise-Phase bis Zielpulse
        if total < target_pulses:
            _set(direction * pct)
            p, el = _count_edges_for(timeout_ms, until_pulses=target_pulses, base=total)
            total += p
            if total < target_pulses:
                print("[TEST] TIMEOUT bei %d Pulsen" % total)

        cutoff = total
        _set(0)

        # Auslauf: weiterzählen bis 300ms Ruhe
        while True:
            p, _ = _count_edges_for(300)
            if p == 0:
                break
            total += p
    finally:
        _set(0)

    print("[TEST] Abschaltpunkt: %d Pulse (%.3f m)" % (cutoff, cutoff * M_PER_PULSE))
    print("[TEST] Endstand:      %d Pulse (%.3f m), Auslauf %d Pulse"
          % (total, total * M_PER_PULSE, total - cutoff))
    print("[TEST] --> real messen!  M_PER_PULSE = real_m / %d" % total)


def roll(ms, pct=20):
    total = 0
    try:
        _set(pct)
        p, _ = _count_edges_for(ms)
        total += p
    finally:
        _set(0)
    while True:
        p, _ = _count_edges_for(300)
        if p == 0:
            break
        total += p
    print("[ROLL] %d ms @ %d%% -> %d Pulse = %.3f m" % (ms, pct, total, total * M_PER_PULSE))


def irq_test():
    """ENTSCHEIDUNGSTEST für die hipe-Integration: verträgt sich ein
    Pin-IRQ-Zähler auf GP0 mit dem frozen DriveController, oder
    zerstört er dessen get_rpm()? 2s Fahrt bei 30%."""
    cnt = [0]
    def _h(pin):
        cnt[0] += 1
    _enc.irq(trigger=Pin.IRQ_RISING, handler=_h)
    rpms = []
    try:
        _set(30)
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < 2000:
            rpms.append(drv.get_rpm())
            time.sleep_ms(100)
    finally:
        _set(0)
        _enc.irq(handler=None)
    print("[IRQ] Pulse via IRQ: %d" % cnt[0])
    print("[IRQ] get_rpm-Samples währenddessen:", [int(r) for r in rpms])
    print("[IRQ] Beide gesund -> IRQ-Zähler kann in hipe.py;")
    print("[IRQ] get_rpm tot (nur 0en) -> DriveController-Konflikt, Fallback nötig.")


print("Bereit: drive(m, pct=..), roll(ms, pct=..), irq_test(), stop()")