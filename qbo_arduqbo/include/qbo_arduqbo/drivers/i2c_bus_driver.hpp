#pragma once

#include <memory>
#include <string>
#include <cstdint>

class I2CBusDriver {
public:
    I2CBusDriver(const std::string& device_path, uint8_t address);
    bool readBytes(uint8_t* buffer, size_t length);

private:
    std::string device_path_;
    uint8_t address_;
};
