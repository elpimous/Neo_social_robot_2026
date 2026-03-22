// Gyroscope :
// Un gyroscope est un capteur qui mesure la vitesse de rotation ou le taux de changement d’orientation autour d’un ou plusieurs axes.
// Il s’appuie sur le principe de la conservation du moment angulaire, ce qui lui permet de détecter les rotations même en l'absence de repères extérieurs.

#include "L3G4200D.h"

L3G4200D::L3G4200D(uint8_t address) {
    deviceAddress = address;  // Stocke l'adresse I2C du capteur
}

bool L3G4200D::begin(int scale) {
    Wire.begin();
    
    // Test de communication : lecture d'un registre valide
    int test = readRegister(L3G4200D_CTRL_REG1);
    if (test == -1) {
        Serial.println("❌ Erreur : Impossible de communiquer avec le L3G4200D !");
        return false;
    }

    if (setupScale(scale) != 0) {
        Serial.println("Erreur d'initialisation du L3G4200D !");
        return false;
    }

    Serial.println("✅ L3G4200D prêt !");
    return true;
}

void L3G4200D::writeRegister(uint8_t address, uint8_t val) {

    //I2c.write(deviceAddress, address, val);
    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    Wire.write(val);
    uint8_t status = Wire.endTransmission();
    
    if (status != 0) {
        Serial.print("Erreur I2C lors de l'écriture sur le registre 0x");
        Serial.println(address, HEX);
    }
} 

int L3G4200D::readRegister(uint8_t address){

    int v = -1;

    Wire.beginTransmission(deviceAddress);
    Wire.write(address);
    uint8_t status = Wire.endTransmission();
    if (status != 0) {
        Serial.print("Erreur I2C lors de la lecture du registre 0x");
        Serial.println(address, HEX);
        return v;  // Retourne -1 en cas d'erreur
    }

    Wire.requestFrom(deviceAddress, (uint8_t)1);
    if (Wire.available()) {
        v = Wire.read();
    } else {
        Serial.println("Erreur : aucun octet reçu !");
    }

    return v;
}

void L3G4200D::getGyroValues(int& x, int& y, int& z){

  Wire.beginTransmission(deviceAddress);
    
    Wire.write(0x28 | 0x80); // Auto-incrémentation des adresses
    uint8_t status = Wire.endTransmission();
    
    if (status != 0) {
        Serial.println("Erreur I2C lors de la lecture des valeurs du gyroscope !");
        return;
    }

    Wire.requestFrom(deviceAddress, (uint8_t)6);
    if (Wire.available() >= 6) {
        uint8_t xLSB = Wire.read();
        uint8_t xMSB = Wire.read();
        uint8_t yLSB = Wire.read();
        uint8_t yMSB = Wire.read();
        uint8_t zLSB = Wire.read();
        uint8_t zMSB = Wire.read();

        x = ((int16_t)(xMSB << 8 | xLSB));
        y = ((int16_t)(yMSB << 8 | yLSB));
        z = ((int16_t)(zMSB << 8 | zLSB));
    } else {
        Serial.println("Erreur : Données incomplètes reçues !");
    }
}

int L3G4200D::setupScale(int scale) {
    // Activer X, Y, Z et désactiver le mode power down
    writeRegister(L3G4200D_CTRL_REG1, 0b01101111);  // 200Hz, all axes enabled

    // Configuration de CTRL_REG2 (Filtrage passe-haut activé, filtre à 8Hz)
    writeRegister(L3G4200D_CTRL_REG2, 0b00001000);

    // Configuration de CTRL_REG3 (DRDY activé pour vérifier les données prêtes)
    writeRegister(L3G4200D_CTRL_REG3, 0b00001000);  
    // Serial.println(scale);
    // Configuration de CTRL_REG4 (choix du scale)
    switch (scale) {
        case 250:
            writeRegister(L3G4200D_CTRL_REG4, 0b00000000); // ±250 dps
            break;
        case 500:
            writeRegister(L3G4200D_CTRL_REG4, 0b00010000); // ±500 dps
            break;
        case 2000:
            writeRegister(L3G4200D_CTRL_REG4, 0b00110000); // ±2000 dps
            break;
        default:
            Serial.println("Erreur : Valeur de scale invalide !");
            return -1;
    }

    // Configuration de CTRL_REG5 (Filtrage activé et FIFO désactivé)
    writeRegister(L3G4200D_CTRL_REG5, 0b00010000);  // Active LPF et filtre passe-haut

    return 0;  // Retourne 0 si tout est OK
}

