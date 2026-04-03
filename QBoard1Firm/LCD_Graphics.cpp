
#include "LCD_Graphics.h"

byte batteryIconFrame = 0;

byte pc[8]   = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000 };
byte sect[8] = { 0b01010, 0b01010, 0b11111, 0b01110, 0b01110, 0b00100, 0b11100, 0b10000 };
byte hp[8]   = { 0b00000, 0b00011, 0b00111, 0b11111, 0b11111, 0b00111, 0b00011, 0b00000 };

void loadBatteryIcon(LCD03 &lcd, int level) {
    // vince : cache du dernier niveau affiché
    // Anciennement : lcd.createChar() appelé à chaque seconde même si le niveau n'avait pas changé
    // lcd.createChar() = ~3-5ms I2C bloquant → inutile si l'icône est identique
    // Après : réécriture uniquement si le niveau change réellement
    static int lastLevel = -1;
    if (level == lastLevel) return;
    lastLevel = level;

    byte buffer[8];
    memcpy(buffer, battIcons[level], 8);
    lcd.createChar(0, buffer);
}

void displayBatteryIcon(LCD03 &lcd, byte charg, float batV) {
    if (charg == 1 || charg == 2) {
        lcd.display();
        lcd.setCursor(15, 3);
        lcd.write(7);
        batteryIconFrame = (batteryIconFrame + 1) % 5;
        lcd.setCursor(17, 3);
        loadBatteryIcon(lcd, batteryIconFrame);
        lcd.write(0);
    } else if (charg == 3) {
        lcd.display();
        lcd.setCursor(15, 3);
        lcd.write(7);
        lcd.setCursor(17, 3);
        loadBatteryIcon(lcd, 5);
        lcd.write(0);
    } else if (charg == 4) {
        lcd.noDisplay();
        lcd.setCursor(15, 3);
        lcd.print(" ");
        lcd.setCursor(17, 3);
        int level = 0;
        if      (batV >= 12.9) level = 5;
        else if (batV >= 12.7) level = 4;
        else if (batV >= 12.5) level = 3;
        else if (batV >= 12.3) level = 2;
        else if (batV >= 12.1) level = 1;
        else                   level = 0;
        loadBatteryIcon(lcd, level);
        lcd.write(0);
    }
}
