from hipe import hipe
from machine import Pin
import time

if __name__ == "__main__":
    try:
        h = hipe("1688_Group19")
        try:
            import os
            os.remove("/crash.log")   # alter Log weg: Datei existiert nur nach echtem Crash
        except OSError:
            pass
        h.run()
    except Exception as e:
        # Kein USB angeschlossen -> kein Shell-Feedback möglich.
        # Vollständigen Traceback (Datei + Zeile!) sichern und per
        # unverwechselbarem LED-Muster signalisieren.
        import sys, io
        buf = io.StringIO()
        sys.print_exception(e, buf)
        tb = buf.getvalue()
        print(tb)                      # sichtbar, falls doch USB dran ist
        try:
            with open("/crash.log", "w") as f:
                f.write(tb)
        except Exception:
            pass

        led = Pin("LED", Pin.OUT)
        while True:
            # Doppel-Blitz + Pause: eindeutig vom normalen fast/slow-Muster
            # der LedBlinker-Betriebsanzeige unterscheidbar
            for _ in range(2):
                led.value(1); time.sleep_ms(80)
                led.value(0); time.sleep_ms(120)
            time.sleep_ms(600)