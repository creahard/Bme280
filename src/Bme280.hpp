/* BME280 Temperature compensated pressure sensor */

#ifndef BME280_H
#define BME280_H
#pragma once

#include <inttypes.h>
#include <Wire.h>

#define BME280_ADDR             0x76
#define BME280_DEV_ID           0x60
#define BME280_REG_COMP_TABLE   0x88
#define BME280_REG_COMP_T1_MSB  0x88
#define BME280_REG_COMP_T1_LSB  0x89
#define BME280_REG_COMP_T2_MSB  0x8A
#define BME280_REG_COMP_T2_LSB  0x8B
#define BME280_REG_COMP_T3_MSB  0x8C
#define BME280_REG_COMP_T3_LSB  0x8D
#define BME280_REG_COMP_P1_MSB  0x8E
#define BME280_REG_COMP_P1_LSB  0x8F
#define BME280_REG_COMP_P2_MSB  0x90
#define BME280_REG_COMP_P2_LSB  0x91
#define BME280_REG_COMP_P3_MSB  0x92
#define BME280_REG_COMP_P3_LSB  0x93
#define BME280_REG_COMP_P4_MSB  0x94
#define BME280_REG_COMP_P4_LSB  0x95
#define BME280_REG_COMP_P5_MSB  0x96
#define BME280_REG_COMP_P5_LSB  0x97
#define BME280_REG_COMP_P6_MSB  0x98
#define BME280_REG_COMP_P6_LSB  0x99
#define BME280_REG_COMP_P7_MSB  0x9A
#define BME280_REG_COMP_P7_LSB  0x9B
#define BME280_REG_COMP_P8_MSB  0x9C
#define BME280_REG_COMP_P8_LSB  0x9D
#define BME280_REG_COMP_P9_MSB  0x9E
#define BME280_REG_COMP_P9_LSB  0x9F
#define BME280_REG_COMP_H1      0xA1
#define BME280_REG_COMP_H2_MSB  0xE1
#define BME280_REG_COMP_H2_LSB  0xE2
#define BME280_REG_COMP_H3      0xE3
#define BME280_REG_COMP_H4_MSB  0xE4
#define BME280_REG_COMP_H4_5    0xE5
#define BME280_REG_COMP_H5_LSB  0xE6
#define BME280_REG_COMP_H6      0xE7

#define BME280_REG_ID           0xD0
#define BME280_REG_RESET        0xE0
#define BME280_REG_CTRL_HUMI    0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_PRES_MSB     0xF7
#define BME280_REG_PRES_LSB     0xF8
#define BME280_REG_PRES_XLSB    0xF9
#define BME280_REG_TEMP_MSB     0xFA
#define BME280_REG_TEMP_LSB     0xFB
#define BME280_REG_TEMP_XLSB    0xFC
#define BME280_REG_HUMI_MSB     0xFD
#define BME280_REG_HUMI_LSB     0xFE

#define BME280_ERROR_NONE                 0
#define BME280_ERROR_NO_ANSWER            0x10
#define BME280_ERROR_NO_ID                0x20
#define BME280_ERROR_NO_RESET             0x30
#define BME280_ERROR_NO_TABLE             0x40
#define BME280_ERROR_NO_CONF              0x50
#define BME280_ERROR_MODE                 0x60
#define BME280_ERROR_NO_MEAS              0x70
#define BME280_ERROR_NO_TABLE_HUMIDITY    0x80
#define BME280_ERROR_I2C_READ             1
#define BME280_ERROR_I2C_WRITE            2

struct compData_t {
  uint16_t T1;
  int16_t  T2;
  int16_t  T3;
  uint16_t P1;
  int16_t  P2;
  int16_t  P3;
  int16_t  P4;
  int16_t  P5;
  int16_t  P6;
  int16_t  P7;
  int16_t  P8;
  int16_t  P9;
};

struct compHumid_t {
  int16_t  H2;
  uint8_t  H3;
  int16_t  H4;
  int16_t  H5;
  int8_t   H6;
};

union compTable_t {
  compData_t var;
  uint8_t data[sizeof(compData_t)];
};

union compTableH_t {
  compHumid_t var;
  uint8_t data[sizeof(compHumid_t)];
};

class Bme280 {
  private:
    TwoWire *myWire;
    uint8_t address;
    compTable_t compensation;
    uint8_t H1;
    compTableH_t compH;
    bool readByte(uint8_t reg, uint8_t *data);
    bool sendByte(uint8_t reg, uint8_t data);
    bool readTable(uint8_t reg, size_t len, uint8_t *data);
  public:
    Bme280(TwoWire *myWire);
    Bme280(TwoWire *myWire, uint8_t addr);
    uint8_t error;
    bool begin();
    bool configure(uint32_t confRegs);
    bool standby();
    bool continuous();
    bool measure();
    bool read();
    double temperature;
    double pressure;
    double humidity;
    bool ready();
};

#endif // BME280_H
