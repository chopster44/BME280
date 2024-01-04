/*
    eMAKER BME280 driver
    Copyright (C) 2024  chopster44 for eMAKER Ltd

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef BME280_H

#define BME280_H

#include <Arduino.h>
#include <Wire.h>

//I2C Address
#define BME280_DEFAULT_ADDRESS      (0x77)
#define BME280_ALTERNATE_ADDRESS    (0x76)
#define BME280_WHO_AM_I             (0x60)

// Registers
// DATA

// CONFIG
#define BME280_WHO_AM_I_REG         (0xD0)

class BME280 {
    public:
        bool begin(uint8_t addr = BME280_DEFAULT_ADDRESS, TwoWire *theWire = &Wire);
        uint8_t whoami();
    private:
        TwoWire *_wire;
        uint8_t _addr;
        bool write(uint8_t reg, uint8_t *buffer, uint8_t len);
        bool readRegister(uint8_t reg, uint8_t *buffer, uint8_t len = 1);
};

#endif