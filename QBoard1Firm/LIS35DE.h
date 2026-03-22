// Accelerometer
#ifndef LIS35DE_h
#define LIS35DE_h

#include <Wire.h>
#include <Arduino.h>    // Pour Serial


#define LIS35DE_CTRL_REG1 0x20
#define LIS35DE_CTRL_REG2 0x21
#define LIS35DE_CTRL_REG3 0x22

#include <inttypes.h>

class LIS35DE {
  public:
    LIS35DE(uint8_t address = 0x1D);  // Adresse I2C par défaut 0x1D
    bool begin();  // Nouvelle méthode d'initialisation
    void writeRegister(uint8_t address, uint8_t val);
    int readRegister(uint8_t address);
    void getAccelerometerValues(int8_t& x, int8_t& y, int8_t& z);

  private:
    int setup();  // Reste privé car ne doit être utilisé que par `begin()`
    uint8_t deviceAddress;
};

#endif

