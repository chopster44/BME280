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

#include <BME280.h>

bool BME280::begin(uint8_t addr, TwoWire *theWire) {
    _wire = theWire;
    _addr = addr;
    uint8_t whoAmI = whoami();
    if (whoAmI == BME280_WHO_AM_I) {
        return true;
    } else {
        return false;
    }
}

uint8_t BME280::whoami() {
    uint8_t buffer[1];
    if (readRegister(BME280_WHO_AM_I_REG, buffer))
        return buffer[0];
    return -1;
}

bool BME280::write(uint8_t reg, uint8_t *buffer, uint8_t len) {
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    for(uint8_t i = 0; i < len; i++) {
        _wire->write(buffer[i]);
    }
    if(_wire->endTransmission() != 0)
        return false;
    return true;
}

bool BME280::readRegister(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t rx_bytes = 0;

    _wire->beginTransmission(_addr);
    _wire->write(reg);
    uint8_t err = _wire->endTransmission();
    if (err!=0) {
        return false;
    }
    rx_bytes = _wire->requestFrom(_addr, len);
    if (rx_bytes != len){
        return false;
    }
    for (uint8_t i =0; i < len; i++) {
        buffer[i] = _wire->read();
    }
    return true;
}