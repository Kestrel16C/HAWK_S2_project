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
     hipe(loop_hz=100, web_root="/www", port=80).run()
     ```
   - **Autostart (optional):** auf dem Pico eine `main.py` anlegen:
     ```python
     from hipe import hipe

     if __name__ == "__main__":
         hipe(loop_hz=100, web_root="/www", port=80).run()
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
- `tools/flash_nuke.uf2` – optionales Zurücksetzen des Speichers.

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

Dieses Projekt ist ausschließlich für den internen Gebrauch in der Lehre vorgesehen. Weitergabe oder Nutzung außerhalb der HAWK bedarf der Zustimmung.

---

## Kontakt

Projektbetreuung: Dipl.-Ing. (FH) Tobias Bürmann, Prof. Dr. Roman Grothausmann, Prof. Dr. Tobias Sprodowski  
Fakultät Ingenieurwissenschaften und Gesundheit - Informatik  
HAWK – Hochschule für angewandte Wissenschaft und Kunst Hildesheim/Holzminden/Göttingen

---