from hipe import hipe
from machine import Pin
import time

if __name__ == "__main__":
    try:
        h = hipe("1688_Group19")
        h.run()
    except Exception as e:
        # Kein USB angeschlossen -> kein Shell-Feedback möglich.
        # Onboard-LED schnell blinken lassen als Fehler-Indikator,
        # Fehler in eine Datei schreiben fürs spätere Auslesen per USB.
        try:
            with open("/crash.log", "w") as f:
                f.write(str(e))
        except Exception:
            pass

        led = Pin("LED", Pin.OUT)
        while True:
            led.toggle()
            time.sleep_ms(100)