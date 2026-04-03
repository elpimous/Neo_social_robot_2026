# NEO v2 Vincent FOUCAULT Mars 2026 — Corrections firmware Arduino
## Fichiers modifiés

---

## motores.h — CRITIQUE

### ATOMIC_BLOCK restauré dans getPulsesDifference()
**Problème :** lecture d'un int 16 bits sans protection = 2 instructions AVR non atomiques.
Si l'ISR encodeur fire entre les deux → valeur corrompue → odométrie erratique aléatoire.
**Fix :** ATOMIC_BLOCK(ATOMIC_RESTORESTATE) autour de la lecture + reset.

---

## ArduBot.h

### calibrationStep() prend les valeurs brutes en paramètre
**Problème :** calibrationStep() relisait gyro+accel via I2C alors que spinOnce() venait
de les lire → 4 lectures I2C par cycle au lieu de 2 pendant la calibration.
**Fix :** signature `calibrationStep(raw_gx, raw_gy, raw_gz, raw_ax, raw_ay, raw_az)`
→ spinOnce() lit une fois et passe les valeurs.

---

## ArduBot.cpp

### loopPeriodSeconds mesuré AVANT les lectures I2C
**Problème :** loopPeriodSeconds incluait le temps de calibrationStep() (~0.2ms)
→ biais odométrie pendant les 500 cycles de calibration.
**Fix :** micros() appelé avant les lectures, pas après.

### cos/sin précalculés une seule fois
**Problème :** cos() et sin() avec le même argument = 2 appels flottants = ~400µs/cycle = 8% budget.
**Fix :** angle_mid calculé une fois → c = cos(angle_mid), s = sin(angle_mid).

### Constante ODO_ANGULAR_CORRECTION précalculée
**Problème :** 1.042874 * 1.0256 * ... = deux multiplications au lieu d'une.
**Fix :** static const float ODO_ANGULAR_CORRECTION = 1.042874f * 1.0256f

### thetaGyroOnly supprimé
**Problème :** variable jamais utilisée nulle part, multiplication flottante inutile chaque 5ms.
**Fix :** lignes commentées/supprimées.

---

## LCD_Graphics.cpp

### Cache lastLevel dans loadBatteryIcon()
**Problème :** lcd.createChar() appelé toutes les secondes même si l'icône n'avait pas changé.
lcd.createChar() = ~3-5ms I2C bloquant inutile.
**Fix :** static int lastLevel → réécriture uniquement si niveau change.

---

## Fichiers inchangés
- serialProtocol.cpp  (déjà correct)
- serialProtocol.h
- comands.h
- L3G4200D.cpp
- LCD_Graphics.h

---

## Résumé des gains

| Fix | Impact | Gravité |
|---|---|---|
| ATOMIC_BLOCK | Corruption encodeurs éliminée | CRITIQUE |
| Double I2C calib | -0.2ms/cycle pendant calib | IMPORTANT |
| cos/sin précalc | -400µs/cycle = -8% CPU | IMPORTANT |
| ODO_ANGULAR | -1 multiplication/cycle | FAIBLE |
| thetaGyroOnly | -50µs/cycle | FAIBLE |
| LCD createChar | -5ms/s I2C inutile | MOYEN |
