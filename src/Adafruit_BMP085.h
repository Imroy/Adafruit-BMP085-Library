/***************************************************
  This is a library for the Adafruit BMP085/BMP180 Barometric Pressure + Temp sensor

  Designed specifically to work with the Adafruit BMP085 or BMP180 Breakout
  ----> http://www.adafruit.com/products/391
  ----> http://www.adafruit.com/products/1603

  These displays use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef ADAFRUIT_BMP085_H
#define ADAFRUIT_BMP085_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>

#define BMP085_DEBUG 0

#define BMP085_I2CADDR 0x77

class Adafruit_BMP085 {
public:
  enum mode {
    ULTRALOWPOWER,
    STANDARD,
    HIGHRES,
    ULTRAHIGHRES,
  };

private:
  uint8_t oversampling;

  int16_t ac1, ac2, ac3;
  uint16_t ac4, ac5, ac6;
  int16_t b1, b2, mb, mc, md;

  uint16_t UT;
  uint32_t UP;

  bool error;

  enum reg {
    CAL_AC1		= 0xAA,  // R   Calibration data (16 bits)
    CAL_AC2		= 0xAC,  // R   Calibration data (16 bits)
    CAL_AC3		= 0xAE,  // R   Calibration data (16 bits)
    CAL_AC4		= 0xB0,  // R   Calibration data (16 bits)
    CAL_AC5		= 0xB2,  // R   Calibration data (16 bits)
    CAL_AC6		= 0xB4,  // R   Calibration data (16 bits)
    CAL_B1		= 0xB6,  // R   Calibration data (16 bits)
    CAL_B2		= 0xB8,  // R   Calibration data (16 bits)
    CAL_MB		= 0xBA,  // R   Calibration data (16 bits)
    CAL_MC		= 0xBC,  // R   Calibration data (16 bits)
    CAL_MD		= 0xBE,  // R   Calibration data (16 bits)

    CHECK		= 0xD0,

    CONTROL		= 0xF4,
    DATA		= 0xF6,
    DATA_XLSB		= 0xF8,
  };

  enum command {
    READTEMP		= 0x2E,
    READPRESSURE	= 0x34,
  };

  uint8_t read8(reg addr);
  uint16_t read16(reg addr);
  void write_cmd(reg addr, uint8_t cmd);
  int32_t computeB5(void);

 public:
  Adafruit_BMP085();
  boolean check(void);
  boolean begin(mode m = ULTRAHIGHRES);  // by default go highres

  // 4.5 ms
  unsigned long measureTemperature(void) {
    write_cmd(CONTROL, READTEMP);
    return 5;
  }
  // 4.5/7.5/13.5/25.5 ms
  unsigned long measurePressure(void) {
    write_cmd(CONTROL, READPRESSURE | (oversampling << 6));
    switch (oversampling) {
    case 0:
      return 5;
    case 1:
      return 8;
    case 2:
      return 14;
    default:
      return 26;
    }
  }

  bool readRawTemperature(void);
  bool readRawPressure(void);

  uint16_t rawTemperature(void) const { return UT; }
  uint32_t rawPressure(void) const { return UP; }
  float temperature(void);
  int32_t pressure(void);
  int32_t sealevelPressure(float altitude_meters = 0);
  float altitude(float sealevelPressure = 101325); // std atmosphere

};


#endif //  ADAFRUIT_BMP085_H
