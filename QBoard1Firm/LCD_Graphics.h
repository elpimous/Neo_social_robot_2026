
#ifndef LCD_Graphics_h
#define LCD_Graphics_h

#include <Arduino.h>    // Pour Serial
#include <LCD03.h>

extern byte pc[8];
extern byte hp[8];
extern byte sect[8];


// 📌 Icônes de batterie dynamiques
const byte battIcons[6][8] = {
    {0b00110, 0b01111, 0b01001, 0b01001, 0b01001, 0b01001, 0b01001, 0b01111},  // 0%
    {0b00110, 0b01111, 0b01001, 0b01001, 0b01001, 0b01001, 0b01111, 0b01111}, // 20%
    {0b00110, 0b01111, 0b01001, 0b01001, 0b01001, 0b01111, 0b01111, 0b01111}, // 40%
    {0b00110, 0b01111, 0b01001, 0b01001, 0b01111, 0b01111, 0b01111, 0b01111}, // 60%
    {0b00110, 0b01111, 0b01001, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111}, // 80%
    {0b00110, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111, 0b01111} // Full
};

// 📌 Fonctions (passage de `lcd` en paramètre)
void loadBatteryIcon(LCD03 &lcd, int level);
void displayBatteryIcon(LCD03 &lcd, byte charg, float batV);

#endif
