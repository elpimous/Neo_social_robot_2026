// Accelerometer
// Un accéléromètre, est un capteur qui mesure l’accélération linéaire – 
// c’est-à-dire les variations de vitesse dans une direction donnée – sur un ou plusieurs axes.
// Il peut détecter à la fois les accélérations dynamiques (mouvements) et l’accélération statique due à la gravité.
#include "LIS35DE.h"


LIS35DE::LIS35DE(uint8_t address) {
    deviceAddress = address;
}

bool LIS35DE::begin() {
    Wire.begin();

    // Test de communication : lecture d'un registre valide
    int test = readRegister(LIS35DE_CTRL_REG1);
    if (test == -1) {
        Serial.println("❌ Erreur : Impossible de communiquer avec le LIS35DE !");
        return false;
    }

    if (setup() != 0) {
        Serial.println("Erreur d'initialisation du LIS35DE !");
        return false;
    }

    Serial.println("✅ LIS35DE prêt !");
    return true;
}


void LIS35DE::writeRegister(uint8_t address, uint8_t val) {
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.write(val);
    uint8_t status = Wire.endTransmission();
    
    if (status != 0) {
        Serial.print("Erreur I2C lors de l'écriture sur le registre 0x");
        Serial.println(address, HEX);
    }
}

int LIS35DE::readRegister(uint8_t address) {
    int v = -1;  // Retourne -1 en cas d'échec

    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    uint8_t status = Wire.endTransmission();
    if (status != 0) {
        Serial.print("Erreur I2C lors de la lecture du registre 0x");
        Serial.println(address, HEX);
        return v;
    }

    Wire.requestFrom(deviceAddress, (uint8_t)1);
    if (Wire.available()) {
        v = Wire.read();
    } else {
        Serial.println("Erreur : Aucun octet reçu !");
    }

    return v;
}

void LIS35DE::getAccelerometerValues(int8_t& x, int8_t& y, int8_t& z) {
    Wire.beginTransmission(deviceAddress);
    // On démarre la lecture à partir de l'adresse 0x29 en activant l'auto-incrémentation
    Wire.write(0x29 | 0x80);
    uint8_t status = Wire.endTransmission();
    
    if (status != 0) {
        Serial.println("Erreur I2C lors de la lecture des valeurs de l'accéléromètre !");
        return;
    }

    // On demande 5 octets : OutX, registre inutilisé, OutY, registre inutilisé, OutZ
    Wire.requestFrom(deviceAddress, (uint8_t)5);
    if (Wire.available() >= 5) {
        x = Wire.read();    // OutX
        Wire.read();        // Ignorer le registre inutilisé (0x2A)
        y = Wire.read();    // OutY
        Wire.read();        // Ignorer le registre inutilisé (0x2C)
        z = Wire.read();    // OutZ
    } else {
        Serial.println("Erreur : Données incomplètes reçues !");
    }
}



int LIS35DE::setup(){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(LIS35DE_CTRL_REG1, 0b01000111); // Activation X, Y, Z

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(LIS35DE_CTRL_REG2, 0b00000000); // Désactivation des filtres HPF

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(LIS35DE_CTRL_REG3, 0b00000000); // Pas d'interruptions

  return 0;  // Indique que l'initialisation s'est bien déroulée
}
