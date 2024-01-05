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

enum BME280_Reg_Map {
    BME280_REG_DIG_T1 = 0x88,
    BME280_REG_DIG_T2 = 0x8A,
    BME280_REG_DIG_T3 = 0x8C,
    
    BME280_REG_DIG_P1 = 0x8E,
    BME280_REG_DIG_P2 = 0x90,
    BME280_REG_DIG_P3 = 0x92,
    BME280_REG_DIG_P4 = 0x94,
    BME280_REG_DIG_P5 = 0x96,
    BME280_REG_DIG_P6 = 0x98,
    BME280_REG_DIG_P7 = 0x9A,
    BME280_REG_DIG_P8 = 0x9C,
    BME280_REG_DIG_P9 = 0x9E,

    BME280_REG_DIG_H1 = 0xA1,
    BME280_REG_DIG_H2 = 0xE1,    
    BME280_REG_DIG_H3 = 0xE3,
    BME280_REG_DIG_H4 = 0xE4,    
    BME280_REG_DIG_H5 = 0xE5,
    BME280_REG_DIG_H6 = 0xE7,

    BME280_REG_HUM    = 0xFD,
    BME280_REG_TEMP   = 0xFA,
    BME280_REG_PRES   = 0xF7,

    BME280_REG_CONFIG = 0xF5,
    BME280_CTRL_MEAS  = 0xF4,
    BME280_REG_STATUS = 0xF3,
    BME280_CTRL_HUM   = 0xF2,

    BME280_REG_RESET  = 0xE0,
    BME280_REG_WHOAMI = 0xD0    
};

// types

typedef struct {
  uint16_t dig_T1; ///< temperature compensation value
  int16_t dig_T2;  ///< temperature compensation value
  int16_t dig_T3;  ///< temperature compensation value

  uint16_t dig_P1; ///< pressure compensation value
  int16_t dig_P2;  ///< pressure compensation value
  int16_t dig_P3;  ///< pressure compensation value
  int16_t dig_P4;  ///< pressure compensation value
  int16_t dig_P5;  ///< pressure compensation value
  int16_t dig_P6;  ///< pressure compensation value
  int16_t dig_P7;  ///< pressure compensation value
  int16_t dig_P8;  ///< pressure compensation value
  int16_t dig_P9;  ///< pressure compensation value

  uint8_t dig_H1; ///< humidity compensation value
  int16_t dig_H2; ///< humidity compensation value
  uint8_t dig_H3; ///< humidity compensation value
  int16_t dig_H4; ///< humidity compensation value
  int16_t dig_H5; ///< humidity compensation value
  int8_t dig_H6;  ///< humidity compensation value
} bme280_trim_data;


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