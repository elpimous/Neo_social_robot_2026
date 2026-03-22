
#include "LCD_Graphics.h"
// #include <avr/pgmspace.h>

byte batteryIconFrame = 0;

// 📌 Icônes fixes
byte pc[8] = { 0b00000, 0b01010, 0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000 };
byte sect[8] = {0b01010, 0b01010, 0b11111, 0b01110, 0b01110, 0b00100, 0b11100, 0b10000};
byte hp[8] = {0b00000, 0b00011, 0b00111, 0b11111, 0b11111, 0b00111, 0b00011, 0b00000};

// 📌 Charge l'icône batterie selon son niveau
void loadBatteryIcon(LCD03 &lcd, int level) {
    byte buffer[8];  // Tampon pour stocker l’icône
    memcpy(buffer, battIcons[level], 8);  // Copie l'icône correcte en RAM
    lcd.createChar(0, buffer);  // Charge l'icône directement
}

// 📌 Affiche l'icône de batterie en fonction de l'état de charge
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
        lcd.write(7);   //Affichage du secteur (Prise de courant)
        lcd.setCursor(17, 3);
        loadBatteryIcon(lcd, 5);
        lcd.write(0); //Affichage de la batterie 100%
    } else if (charg == 4) {
        lcd.noDisplay();
        lcd.setCursor(15, 3);
        lcd.print(" ");
        lcd.setCursor(17, 3);
        if (batV >= 12.9){
          loadBatteryIcon(lcd, 5);
          lcd.write(0);
        } 
        else if (batV >= 12.7) {
          loadBatteryIcon(lcd, 4);
          lcd.write(0);
        }
        else if (batV >= 12.5) {
          loadBatteryIcon(lcd, 3);
          lcd.write(0);
        }
        else if (batV >= 12.3) {
          loadBatteryIcon(lcd, 2);
          lcd.write(0);
        }
        else if (batV >= 12.1) {
          loadBatteryIcon(lcd, 1);
          lcd.write(0);
        }
        else {
          lcd.write(0);
          loadBatteryIcon(lcd, 0);
        }
    }
}
