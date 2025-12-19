# Projekt Junior – Starterpaket (Raspberry Pi Pico 2 W)

Willkommen! Dieses Repository enthält die benötigten Artefakte, um das Fahrzeug aus dem Juniorprojekt im ersten Semester des Studiengangs Ingenieurwissenschaften mit MicroPython auf dem **Raspberry Pi Pico 2 W** zu betreiben.

---

Die aktuelle Firmware ist unter Releases erhältlich:
[Download Firmware](https://git.i.hawk.de/informatik/projektjunior/-/releases/permalink/latest)

---


## Schnelleinstieg

1. **Firmware flashen**
   - Pico mit gedrückter **BOOTSEL**-Taste per USB verbinden → erscheint als Massenspeicher.
   - Firmware (z. B. `HAWK_ING_FW-…uf2`) auf das Pico-Laufwerk **kopieren**.
   - Der Pico startet automatisch neu.

   *Falls Probleme auftreten:* zunächst (optional) `tools/flash_nuke.uf2` flashen, anschließend **sofort** die eigentliche Firmware nochmals aufspielen.

2. **Offenen Code übertragen**
   - Mit **Thonny** oder **PyCharm** (MicroPython-IDEs) verbinden.
   - Folgendes auf den Pico kopieren:
     - `hipe.py`
     - Ordner `modules/`
     - Ordner `www/`

3. **Starten**
   - **Manuell (REPL):**
     ```python
     from hipe import hipe
     h = hipe("Hier WLAN-Passwort angeben")
     h.run()
     ```
   - **Autostart (optional):** auf dem Pico eine `main.py` anlegen:
     ```python
     from hipe import hipe

     if __name__ == "__main__":
        h = hipe("Hier WLAN-Passwort angeben")
        h.run()
     ```

4. **Web-UI aufrufen**
   - Smartphone mit WiFi des Picos verbinden.
   - Pico-IP im Browser öffnen (je nach Betriebsart AP/Client).
   - Lenkung/Motor/Telemetrie prüfen.

---

## Inhalte

- `hipe.py` – Einstiegspunkt (änderbar).
- `modules/` – offene Module (änderbar).
- `www/` – Weboberfläche (HTML/JS/CSS).

> **Hinweis:** `secure/*` ist in der Firmware integriert und nicht veränderbar.

---

## Sicherheit

- Fahrzeug für erste Tests **aufbocken** (frei drehende Räder).
- Akku **geladen** und **Polarität** korrekt.
- Leitungen mechanisch sichern; keine losen Schrauben oder Kurzschlüsse.
- Drehrichtung bei Bedarf **softwareseitig** invertieren, nicht durch Vertauschen der Versorgung.
- **Wichtig:** Lenkungs-Servo auf Freigängigkeit kontrollieren und ggf. Lenkwinkel **softwareseitig** einschränken.

---

## Hilfe & Dokumentation

- **Projektdokumentation:** https://pages.i.hawk.de/informatik/projektjunior/
- **Stud.IP-Veranstaltung:**
  https://studip.hawk.de/dispatch.php/course/details?sem_id=5751ae910432d8bf2f69fc498e4b9ade&again=yes
- **MicroPython-Doku:** https://docs.micropython.org/en/latest/

---

## Troubleshooting (Kurz)

- **Keine Web-UI erreichbar:** Pico-IP/Betriebsmodus prüfen, Browser-Cache leeren.
- **Import-Fehler:** Liegen `hipe.py`, `modules/`, `www/` wirklich auf dem Pico?
- **Motor reagiert nicht:** Duty-Limit/Pinbelegung/Versorgung prüfen; Akku laden.
- **angezeigte Stromwerte auffällig:** Sensor-Verdrahtung, Kalibrierung und Parameter gemäß Doku prüfen.

Viel Erfolg beim Aufbau und Testen!

---

## CI/CD

Pipeline-Status:  
![pipeline status](https://git.i.hawk.de/informatik/projektjunior/badges/main/pipeline.svg)

---

## Lizenz

Dieses Projekt (HIPE – HAWK Ingenieurwissenschaften Projekt Erstsemester) wird unter der **MIT License** veröffentlicht.

MIT License

Copyright (c) 2025
Tobias Bürmann, HAWK – Hochschule für angewandte Wissenschaft und Kunst

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

### Drittanbieter-Lizenzen (MIT)

Dieses Projekt enthält Komponenten, die auf externer MIT-lizenzierter Software basieren:

- **MicroPython (MIT License)**  
  Lizenztext unter:  
  `THIRD_PARTY/micropython_LICENSE.txt`

- **TM1637 MicroPython-Treiber (MIT License)**  
  Basierend auf: https://github.com/mcauser/micropython-tm1637  
  Lizenztext unter:  
  `THIRD_PARTY/tm1637_LICENSE.txt`

Alle Erweiterungen und Änderungen wurden von **Tobias Bürmann (HAWK)** vorgenommen.

### Firmware & Secure-Module

Die Firmware-Images (Releases) enthalten:
- MicroPython (MIT)  
- Secure-Module (MIT)  
- Offene Module & Web-UI (MIT)  
- Optional: flash_nuke.uf2 (MIT)

Alle Bestandteile sind MIT-kompatibel. Die resultierende Firmware ist ebenfalls insgesamt MIT-lizenziert.

---

## Kontakt

Projektbetreuung: Dipl.-Ing. (FH) Tobias Bürmann, Prof. Dr. Roman Grothausmann, Prof. Dr. Tobias Sprodowski  
Fakultät Ingenieurwissenschaften und Gesundheit - Informatik  
HAWK – Hochschule für angewandte Wissenschaft und Kunst Hildesheim/Holzminden/Göttingen

---