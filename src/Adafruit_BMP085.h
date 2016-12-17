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
  enum class Mode : uint8_t {
    UltraLowPower,
    Standard,
    HighRes,
    UltraHighRes,
  };

private:
  uint8_t _oversampling;
  uint16_t _oss_mult, _oss_scale;

  float _ac1, _ac2, _ac3, _ac4, _ac5, _ac6;
  float _b1, _b2, _b5, _mb, _mc, _md;
  bool _have_b5;

  uint16_t _ut;
  uint32_t _up;

  bool _error;

  enum class Register : uint8_t {
    Cal_AC1		= 0xaa,  // R   Calibration data (16 bits)
    Cal_AC2		= 0xac,  // R   Calibration data (16 bits)
    Cal_AC3		= 0xae,  // R   Calibration data (16 bits)
    Cal_AC4		= 0xb0,  // R   Calibration data (16 bits)
    Cal_AC5		= 0xb2,  // R   Calibration data (16 bits)
    Cal_AC6		= 0xb4,  // R   Calibration data (16 bits)
    Cal_B1		= 0xb6,  // R   Calibration data (16 bits)
    Cal_B2		= 0xb8,  // R   Calibration data (16 bits)
    Cal_MB		= 0xba,  // R   Calibration data (16 bits)
    Cal_MC		= 0xbc,  // R   Calibration data (16 bits)
    Cal_MD		= 0xbe,  // R   Calibration data (16 bits)

    Check		= 0xd0,

    Control		= 0xf4,
    Data		= 0xf6,
    Data_XLSB		= 0xf8,
  };

  enum class Command : uint8_t {
    ReadTemperature	= 0x2e,
    ReadPressure	= 0x34,
  };

  uint8_t _read8(Register addr);
  uint16_t _read16(Register addr);
  void _write_cmd(Register addr, Command cmd);
  void _computeB5(void);

 public:
  Adafruit_BMP085();
  boolean check(void);
  boolean begin(Mode m = Mode::UltraHighRes);  // by default go highres

  // 4.5 ms
  unsigned long measureTemperature(void) {
    _have_b5 = false;
    _write_cmd(Register::Control, Command::ReadTemperature);
    return 5;
  }

  // 4.5/7.5/13.5/25.5 ms
  unsigned long measurePressure(void) {
    _write_cmd(Register::Control, static_cast<Command>(static_cast<uint8_t>(Command::ReadPressure)
						       | (_oversampling << 6)));
    switch (_oversampling) {
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

  uint16_t rawTemperature(void) const { return _ut; }
  uint32_t rawPressure(void) const { return _up; }
  float temperature(void);
  float pressure(void);
  float sealevelPressure(float altitude_meters = 0);
  float altitude(float sealevelPressure = 101325); // std atmosphere

};


#endif //  ADAFRUIT_BMP085_H
