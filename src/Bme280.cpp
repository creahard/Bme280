/* Class for handling Bme280 temperature compensated pressure sensors.
   Simply pass the address to the I2C object that has already been
   initialized and it will do the rest.

   Copyright (c) 2022 Chris Reahard
 */

#include "Bme280.hpp"

Bme280::Bme280(TwoWire *myWire) {
  this->myWire = myWire;
  this->address = BME280_ADDR;
  this->error = BME280_ERROR_NONE;
}

Bme280::Bme280(TwoWire *myWire, uint8_t addr) {
  this->myWire = myWire;
  this->address = addr;
  this->error = BME280_ERROR_NONE;
}

bool Bme280::begin() {
  uint8_t data;
  if ( this->myWire->status() != 0 ) {
    this->error = BME280_ERROR_I2C_READ;
    return false;
  }
  if ( !this->readByte(BME280_REG_ID, &data) ) {
    this->error = BME280_ERROR_NO_ANSWER;
    return false;
  }
  if (data == BME280_DEV_ID) {
    //Device ID matches Bme280. Reset the device.
    if ( this->sendByte(BME280_REG_RESET, 0xB6) != 1 ) {
      this->error = BME280_ERROR_NO_RESET;
      return false;
    }
    delay(250);
    //Load the compensation table from the device for calculations.
    if ( !this->readTable(BME280_REG_COMP_TABLE, sizeof(this->compensation.data), this->compensation.data) ) {
      this->error = BME280_ERROR_NO_TABLE;
      return false;
    }
    //First byte of the humidity compensation table is OFP.
    if ( !this->readByte(BME280_REG_COMP_H1, &data) ) {
      this->error = BME280_ERROR_NO_TABLE_HUMIDITY;
      return false;
    } else {
        this->H1 = data;
    }
    //Finally, the rest of the humidity compensation table is in another block of registers.
    if ( !this->readTable(BME280_REG_COMP_H2_MSB, sizeof(this->compH.data), this->compH.data) ) {
      this->error = BME280_ERROR_NO_TABLE;
      return false;
    }

    // Now we have to correct for the fact that there is some bit packing with the last 3 values.
    this->compH.var.H6 = (int8_t)(this->compH.data[6]);
    uint8_t b1, b2, b3;
    b1 = ((uint8_t)this->compH.data[3]); // H4
    b2 = ((uint8_t)this->compH.data[4]); // H4_5
    b3 = ((uint8_t)this->compH.data[5]); // H5
    this->compH.var.H4 = (b1 << 4) | (b2 & 0x0F);
    this->compH.var.H5 = (b3 << 4) | (b2 >> 4);

    return true;
  } else {
    this->error = BME280_ERROR_NO_ID;
    return false;
  }
}

bool Bme280::configure(uint8_t humid, uint8_t pres_temp, uint8_t config) {
  // This took TOO long to get in the right order.
  if(!this->sendByte(BME280_REG_CTRL_HUMI, humid)) {
    this->error = BME280_ERROR_I2C_WRITE;
    return false;
  }
  if(!this->sendByte(BME280_REG_CONFIG, config)) {
    this->error = BME280_ERROR_I2C_WRITE;
    return false;
  }
  if(!this->sendByte(BME280_REG_CTRL_MEAS, pres_temp)) {
    this->error = BME280_ERROR_I2C_WRITE;
    return false;
  }
  return true;
}

bool Bme280::standby() {
  uint8_t data;
  if(!this->readByte(BME280_REG_CTRL_MEAS, &data)) {
    this->error = BME280_ERROR_I2C_READ;
    return false;
  }
  this->sendByte(BME280_REG_CTRL_MEAS, data & 0xFC);
  return true;
}

bool Bme280::measure() {
  uint8_t data;
  if(!this->readByte(BME280_REG_CTRL_MEAS, &data)) {
    this->error = BME280_ERROR_I2C_READ;
    return false;
  }
  if(!this->sendByte(BME280_REG_CTRL_MEAS, (data & 0xFC) | 0x02)) {
    this->error = BME280_ERROR_I2C_WRITE;
    return false;
  }
  return true;
}

bool Bme280::continuous() {
  uint8_t data;
  if(!this->readByte(BME280_REG_CTRL_MEAS, &data)) {
    this->error = BME280_ERROR_I2C_READ;
    return false;
  }
  if(!this->sendByte(BME280_REG_CTRL_MEAS, data | 0x03)) {
    this->error = BME280_ERROR_I2C_WRITE;
    return false;
  }
  return true;
}

bool Bme280::read() {
  uint8_t data[8];
  uint32_t temperature, pressure;
  uint16_t humid;
  int32_t temp, pres, t_fine, humi;
  int32_t var1, var2;
  int64_t varl1, varl2, p;

  temperature = pressure = humid = 0;

  // Block read all ADC registers, then fill out variables for calculations.
  if(!this->readTable(BME280_REG_PRES_MSB, sizeof(data), data)) {
    return false;
  }

  pressure = (data[0] << 16 | data[1] << 8 | data[2]) >> 4;
  temperature = (data[3] << 16 | data[4] << 8 | data[5]) >> 4;
  humid = data[6] << 8 | data[7];

  // Move the values into signed integers for calculations.
  temp = temperature;
  pres = pressure;
  humi = humid;

  // Calculate temperature. Result is in Celsius. 
  var1 = ((((temp >> 3) - ((int32_t)this->compensation.var.T1 << 1))) * ((int32_t)this->compensation.var.T2)) >> 11;
  var2 = (((((temp >> 4) - ((int32_t)this->compensation.var.T1)) * ((temp >> 4) - ((int32_t)this->compensation.var.T1))) >> 12 ) * ((int32_t)this->compensation.var.T3)) >> 14;
  t_fine = var1 + var2;
  this->temperature = ((t_fine * 5 + 128) >> 8) / 100.0;

  // Calculate pressure. Result is in pascals.  
  varl1 = ((int64_t)t_fine) - 128000;
  varl2 = varl1 * varl1 * (int64_t)this->compensation.var.P6;
  varl2 = varl2 + ((varl1 * (int64_t)this->compensation.var.P5) << 17);
  varl2 = varl2 + (((int64_t)this->compensation.var.P4) << 35);
  varl1 = ((varl1 * varl1 * (int64_t)this->compensation.var.P3) >> 8) + ((varl1 * (int64_t)this->compensation.var.P2) << 12);
  varl1 = (((((int64_t)1) << 47) + varl1)) * ((int64_t)this->compensation.var.P1) >> 33;
  if (varl1 == 0) {
    this->pressure = 0.0;
  } else {
    p = 1048576 - pres;
    p = (((p << 31) - varl2) * 3125) / varl1;
    varl1 = (((int64_t)this->compensation.var.P9) * (p >> 13) * (p >> 13)) >> 25;
    varl2 = (((int64_t)this->compensation.var.P8) * p) >> 19;
    p = ((p + varl1 + varl2) >> 8) + (((int64_t)this->compensation.var.P7) << 4);
    this->pressure = p/256.0;
  }

  // Calculate relative humidity.
  var1 = (t_fine - ((int32_t)76800));
  var1 = (((((humi << 14) - (((int32_t)this->compH.var.H4) << 20) - (((int32_t)this->compH.var.H5) * var1)) + ((int32_t)16384)) >> 15) * (((((((var1 * ((int32_t)this->compH.var.H6)) >> 10) * (((var1 * ((int32_t)this->compH.var.H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)this->compH.var.H2) + 8192) >> 14));
  var1 = (var1 - (((((var1 >> 15) * (var1 >> 15)) >> 7) * ((int32_t)this->H1)) >> 4));
  var1 = (var1 < 0 ? 0 : var1);
  var1 = (var1 > 419430400 ? 419430400 : var1);
  this->humidity = ((uint32_t)(var1 >> 12)) / 1024.0;

  return true;
}

bool Bme280::ready() {
  static uint8_t status;

  if (this->error != BME280_ERROR_NONE) {
    return false;
  }
  if(!this->readByte(BME280_REG_STATUS, &status)) {
    this->error = BME280_ERROR_NO_ANSWER;
    return false;
  }
  if (status == 0) {
    return true;
  } else {
    this->error = BME280_ERROR_NONE;
    return false;
  }
}

bool Bme280::readByte(uint8_t reg, uint8_t *data) {
  uint8_t count = 0;
  this->myWire->beginTransmission(this->address);
  if(this->myWire->write(reg) != 1) {
    this->error = BME280_ERROR_I2C_WRITE;
    return false;
  }
  this->myWire->endTransmission();
  this->myWire->beginTransmission(this->address);
  count = this->myWire->requestFrom(this->address, 1);
  this->myWire->endTransmission();
  if (count != 1) {
    this->error = BME280_ERROR_I2C_READ;
    return false;
  } else {
    *data = this->myWire->read();
    return true;
  }
}

bool Bme280::sendByte(uint8_t reg, uint8_t data) {
  this->myWire->beginTransmission(this->address);
  if(this->myWire->write(reg) != 1) {
    this->error = BME280_ERROR_I2C_WRITE;
  }
  if(this->myWire->write(data) != 1) {
    this->error = BME280_ERROR_I2C_WRITE;
  }
  this->myWire->endTransmission();
  if(this->error == BME280_ERROR_I2C_WRITE) {
    return false;
  } else {
    return true;
  }
}

bool Bme280::readTable(uint8_t reg, size_t len, uint8_t *data) {
  uint8_t count = 0;
  this->myWire->beginTransmission(this->address);
  if(this->myWire->write(reg) != 1) {
    this->error = BME280_ERROR_I2C_WRITE;
  }
  this->myWire->endTransmission();
  if(this->error == BME280_ERROR_I2C_WRITE) {
    return false;
  }
  this->myWire->beginTransmission(this->address);
  count = this->myWire->requestFrom(this->address, len);
  this->myWire->endTransmission();
  if (count != len) {
    this->error = BME280_ERROR_I2C_READ;
    return false;
  }
  for (uint8_t i = 0; i < count; i++) {
    data[i] = this->myWire->read();
  }
  return true;
}