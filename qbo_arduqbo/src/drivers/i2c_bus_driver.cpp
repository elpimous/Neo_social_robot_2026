#include "qbo_arduqbo/drivers/i2c_bus_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <iostream>

I2CBusDriver::I2CBusDriver(const std::string& device_path, uint8_t address)
    : device_path_(device_path), address_(address) {}

bool I2CBusDriver::readBytes(uint8_t* buffer, size_t length) {
    int file = open(device_path_.c_str(), O_RDWR);
    if (file < 0) {
        std::perror("❌ Erreur ouverture I2C");
        return false;
    }

    if (ioctl(file, I2C_SLAVE, address_) < 0) {
        std::perror("❌ Erreur configuration I2C_SLAVE");
        close(file);
        return false;
    }

    if (read(file, buffer, length) != static_cast<ssize_t>(length)) {
        std::perror("❌ Erreur lecture I2C");
        close(file);
        return false;
    }

    close(file);
    return true;
}
