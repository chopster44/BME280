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
    if (whoAmI != BME280_WHO_AM_I) {
        return false;
    }

    return calibrationSetup();
}

bool BME280::calibrationSetup() {
    // reset the sensor
    // make sure everythig is base settings
    uint8_t writeBuffer = 0xB6;
    if (!write(BME280_REG_RESET, &writeBuffer, 1))
        return false;

    // wait for the reset to finish
    delay(10);

    // verify the chip is ready
    while (!isReady())
        delay(10);
    
    // read the Factory trim values
    bmeTrim = readTrim();
    // set the Sampling values
    setSampling();
    // wait for boot
    delay(100);
    return true;
}

void BME280::setSampling(sensorMode mode, sensorSampling tempSampling,
                         sensorSampling pressSampling, sensorSampling humSampling,
                         sensorFilter filter, standbyDuration duration) {
    // put sensor to sleep so that the settings are written properly
    uint8_t buffer = BME280_SLEEP;
    write(BME280_CTRL_MEAS, &buffer, 1);

    // write the given values
    // to humidity settings
    buffer = humSampling;
    write(BME280_CTRL_HUM, &buffer, 1);
    // to main conf
    // bits 7 6 5 are standby setting, bits 4 3 2 are filter and 0 sets spi on
    buffer = ((uint8_t)duration << 5) | ((uint8_t)filter << 3) | 0b00;
    write(BME280_REG_CONFIG, &buffer, 1);
    // to sensor conf
    // 7 6 5 are temp, 4 3 2 are pressure and 1 0 are mode
    buffer = ((uint8_t)tempSampling << 5) | ((uint8_t)pressSampling << 3) | (uint8_t)mode;
    write(BME280_CTRL_MEAS, &buffer, 1);
    // store the sampling settings for later use if the filter is off
    if (BME280_FILTER_OFF == filter) {
        BME280::tempSampling = tempSampling;
        presSampling = pressSampling;
        BME280::humSampling = humSampling;
    }
}

bme280TrimData BME280::readTrim() {
    bme280TrimData trim;
    // read Temperature trim;
    trim.dig_T1 = read16(BME280_REG_DIG_T1);
    trim.dig_T2 = readSigned16(BME280_REG_DIG_T2);
    trim.dig_T3 = readSigned16(BME280_REG_DIG_T3);

    // read Pressure trim;
    trim.dig_P1 = read16(BME280_REG_DIG_P1);
    trim.dig_P2 = readSigned16(BME280_REG_DIG_P2);
    trim.dig_P3 = readSigned16(BME280_REG_DIG_P3);
    trim.dig_P4 = readSigned16(BME280_REG_DIG_P4);
    trim.dig_P5 = readSigned16(BME280_REG_DIG_P5);
    trim.dig_P6 = readSigned16(BME280_REG_DIG_P6);
    trim.dig_P7 = readSigned16(BME280_REG_DIG_P7);
    trim.dig_P8 = readSigned16(BME280_REG_DIG_P8);
    trim.dig_P9 = readSigned16(BME280_REG_DIG_P9);

    // read Humidity trim;
    trim.dig_H1 = read8(BME280_REG_DIG_H1);
    trim.dig_H2 = readSigned16(BME280_REG_DIG_H2);
    trim.dig_H3 = read8(BME280_REG_DIG_H3);
    trim.dig_H4 = ((int8_t)read8(BME280_REG_DIG_H4) << 4) |
                    ((int8_t)read8(BME280_REG_DIG_H4+1) & 0xF);
    trim.dig_H5 = (((int8_t)read8(BME280_REG_DIG_H5) & 0xF0) << 8) |
                    ((int8_t)read8(BME280_REG_DIG_H5+1));
    trim.dig_H6 = read8(BME280_REG_DIG_H6);

    return trim;
}

uint32_t BME280::getRawPres() {
    // read 0xF7 0xF8  some of 0xF9 (bit 7,6,5,4)
    uint32_t rawPressure = 0;
    uint8_t presByte[1];
    if (!readRegister(BME280_REG_PRES, presByte))
        return -1;
    rawPressure |= presByte[0] << (16 - (presSampling - 1));
    if (!readRegister(BME280_REG_PRES+1, presByte))
        return -1;
    rawPressure |= presByte[0] << (8 - (presSampling - 1));
    if (!readRegister(BME280_REG_PRES+2, presByte))
        return -1;
    rawPressure |= presByte[0];
    return rawPressure;
}

uint32_t BME280::getRawTemp() {
    // read 0xFA 0xFB  some of 0xFC (bit 7,6,5,4)
    uint32_t rawTemperature = 0;
    uint8_t tempByte[1];
    if (!readRegister(BME280_REG_TEMP, tempByte))
        return -1;
    rawTemperature |= tempByte[0] << (16 - (presSampling - 1));
    if (!readRegister(BME280_REG_TEMP+1, tempByte))
        return -1;
    rawTemperature |= tempByte[0] << (8 - (presSampling - 1));
    if (!readRegister(BME280_REG_TEMP+2, tempByte))
        return -1;
    rawTemperature |= tempByte[0];
    return rawTemperature;
}

uint16_t BME280::getRawHum() {
    // read 0xFD 0x FE
    uint16_t rawHum = 0;
    uint8_t humByte[1];
    if (!readRegister(BME280_REG_HUM, humByte))
        return -1;
    rawHum |= humByte[0] << 8;
    if (!readRegister(BME280_REG_HUM+1, humByte))
        return -1;
    rawHum |= humByte[0];
    return rawHum;
}

bool BME280::isReady() {
    uint8_t status[1];
    readRegister(BME280_REG_STATUS, status);

    return (status[0] & 1) == 0;
}

uint8_t BME280::whoami() {
    uint8_t buffer[1];
    if (readRegister(BME280_REG_WHOAMI, buffer))
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

uint8_t BME280::read8(uint8_t reg) {
    uint8_t buffer[1];
    readRegister(reg, buffer);
    return buffer[0];
}

uint16_t BME280::read16(uint8_t reg) {
    uint8_t buffer[1];
    uint16_t res;
    readRegister(reg, buffer);
    res = buffer[0] << 8;
    readRegister(reg +1, buffer);
    res |= buffer[0];
    return res;
}

int16_t BME280::readSigned16(uint8_t reg) {
    return (int16_t)read16(reg);
}