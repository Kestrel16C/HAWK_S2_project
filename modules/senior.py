# modules/senior.py
from machine import Pin, PWM
import time


class SeniorProject:
    def __init__(self):
        print("🎓 Projekt (Senior) wurde gestartet.")

        # --- 1. HARDWARE SETUP ---
        # Hier initialisieren die Seniorprojekt-Studierenden ihre Servos & Sensoren.
        # WICHTIG: Die Pins müssen mit der Verkabelung übereinstimmen!

        #try:
        #    # Beispiel: 3 Servos an Pin 10, 11, 12
        #    self.servo_base = PWM(Pin(10));
        #    self.servo_base.freq(50)
        #    self.servo_lift = PWM(Pin(11));
        #    self.servo_lift.freq(50)
        #    self.servo_grip = PWM(Pin(12));
        #    self.servo_grip.freq(50)
        #
        #    # Startpositionen (damit nichts wackelt)
        #    self.set_angle(self.servo_base, 90)
        #    self.set_angle(self.servo_lift, 45)
        #    self.set_angle(self.servo_grip, 0)
        #except Exception as e:
        #    print(f"Warnung: Hardware konnte nicht initialisiert werden: {e}")

        # --- 2. ZUSTANDSSPEICHER (für Autopilot) ---
        self.state = "IDLE"
        self.timer_start = 0

    # -------------------------------------------------------------------------
    # TEIL A: MANUELLE STEUERUNG (Slider & Buttons)
    # -------------------------------------------------------------------------
    def handle_aux(self, command, data):
        """
        Wird aufgerufen, wenn im Web-UI etwas gedrückt/geschoben wird.
        command: "arm" (Slider) oder "trigger" (Buttons)
        data:    z.B. "s1:90" oder "1"
        """
        # --- Slider für Servos ---
        #if command == "arm":
        #    try:
        #        # Format "s1:90" zerlegen
        #        part, val_str = data.split(":")
        #        angle = int(val_str)
        #
        #        if part == "s1":
        #            self.set_angle(self.servo_base, angle)
        #        elif part == "s2":
        #            self.set_angle(self.servo_lift, angle)
        #        elif part == "s3":
        #            self.set_angle(self.servo_grip, angle)
        #    except:
        #        pass

        # --- Buttons für Sonderfunktionen ---
        #elif command == "trigger":
        #    if data == "1":
        #        print("Senior-Aktion A: Greifer-Sequenz starten")
        #        # Beispiel: Sequenz starten
        #        self.set_angle(self.servo_grip, 180)  # Zu
        #        time.sleep_ms(200)  # Kurz warten (nur in Aux erlaubt, nicht im Loop!)
        #        self.set_angle(self.servo_lift, 90)  # Hoch

        #    elif data == "2":
        #        print("Senior-Aktion B: Alles zurücksetzen")
        #        self.set_angle(self.servo_base, 90)
        #        self.set_angle(self.servo_lift, 45)
        #        self.set_angle(self.servo_grip, 0)

    # -------------------------------------------------------------------------
    # TEIL B: AUTONOMES FAHREN (wird ständig aufgerufen)
    # -------------------------------------------------------------------------
    def run_autopilot(self, current_rpm):
        """
        Muss bei jedem Aufruf speed (-100..100) und steer (-100..100) zurückgeben.
        Darf NICHT blockieren (kein time.sleep)!
        """
        speed = 0
        steer = 0

        # --- ZUSTANDSMASCHINE ---
        # Beispiel: 2 Sekunden fahren, dann drehen

        if self.state == "IDLE":
            # Wenn wir im Auto-Modus starten, gehen wir in den Drive-Modus
            self.state = "DRIVE"
            self.timer_start = time.ticks_ms()

        elif self.state == "DRIVE":
            speed = 40  # Vorwärts
            steer = 0  # Geradeaus

            # Nach 2 Sekunden wechseln
            if time.ticks_diff(time.ticks_ms(), self.timer_start) > 2000:
                self.state = "TURN"
                self.timer_start = time.ticks_ms()

        elif self.state == "TURN":
            speed = 30
            steer = 100  # Voll rechts

            # Nach 1 Sekunde wieder geradeaus
            if time.ticks_diff(time.ticks_ms(), self.timer_start) > 1000:
                self.state = "DRIVE"
                self.timer_start = time.ticks_ms()

        return speed, steer

    # -------------------------------------------------------------------------
    # HILFSFUNKTIONEN
    # -------------------------------------------------------------------------
    def set_angle(self, servo, angle):
        """Wandelt 0-180 Grad in PWM Duty Cycle um."""
        if angle < 0: angle = 0
        if angle > 180: angle = 180
        # Werte für Standard-Servos (ca. 1ms bis 2ms Pulsweite)
        # Duty berechnet auf Basis von 65535 (16-bit)
        duty = int(3000 + (angle / 180.0 * 4000))
        servo.duty_u16(duty)