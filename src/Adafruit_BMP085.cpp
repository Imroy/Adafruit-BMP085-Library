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

#include "Adafruit_BMP085.h"

Adafruit_BMP085::Adafruit_BMP085() {
}

boolean Adafruit_BMP085::check(void) {
  uint8_t d = _read8(Register::Check);
  if (_error) return false;
  return d == 0x55;
}

boolean Adafruit_BMP085::begin(Mode m) {
  if (m > Mode::UltraHighRes)
    m = Mode::UltraHighRes;
  _oversampling = static_cast<uint8_t>(m);
  _oss_mult = 1 << _oversampling;
  _oss_scale = 50000UL >> _oversampling;

  if (!check())
    return false;

  /* read calibration data */
  _ac1 = static_cast<float>(_read16(Register::Cal_AC1)) * 4;
  if (_error) return false;
  _ac2 = static_cast<float>(_read16(Register::Cal_AC2)) / 32;
  if (_error) return false;
  _ac3 = static_cast<float>(_read16(Register::Cal_AC3)) / 128;
  if (_error) return false;
  _ac4 = static_cast<float>(_read16u(Register::Cal_AC4)) / 32768;
  if (_error) return false;
  _ac5 = static_cast<float>(_read16u(Register::Cal_AC5)) / 32768;
  if (_error) return false;
  _ac6 = static_cast<float>(_read16u(Register::Cal_AC6));
  if (_error) return false;

  _b1 = static_cast<float>(_read16(Register::Cal_B1)) / 65536;
  if (_error) return false;
  _b2 = static_cast<float>(_read16(Register::Cal_B2)) / 2048;
  if (_error) return false;

  _mb = static_cast<float>(_read16(Register::Cal_MB));
  if (_error) return false;
  _mc = static_cast<float>(_read16(Register::Cal_MC)) * 2048;
  if (_error) return false;
  _md = static_cast<float>(_read16(Register::Cal_MD));
  if (_error) return false;
#if (BMP085_DEBUG == 1)
  Serial.print("ac1 = "); Serial.println(_ac1, DEC);
  Serial.print("ac2 = "); Serial.println(_ac2, DEC);
  Serial.print("ac3 = "); Serial.println(_ac3, DEC);
  Serial.print("ac4 = "); Serial.println(_ac4, DEC);
  Serial.print("ac5 = "); Serial.println(_ac5, DEC);
  Serial.print("ac6 = "); Serial.println(_ac6, DEC);

  Serial.print("b1 = "); Serial.println(_b1, DEC);
  Serial.print("b2 = "); Serial.println(_b2, DEC);

  Serial.print("mb = "); Serial.println(_mb, DEC);
  Serial.print("mc = "); Serial.println(_mc, DEC);
  Serial.print("md = "); Serial.println(_md, DEC);
#endif

  return true;
}

void Adafruit_BMP085::_computeB5(void) {
  float X1 = (_ut - _ac6) * _ac5;
  float X2 = _mc / (X1 + _md);
  _b5 = (X1 + X2) / 64;		// Now 1/64th the scale
  _have_b5 = true;

#if BMP085_DEBUG == 1
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
  Serial.print("B5 = "); Serial.println(_b5);
#endif
}

bool Adafruit_BMP085::readRawTemperature(void) {
  uint16_t d = _read16(Register::Data);
  if (_error) return false;
  _ut = d;

#if BMP085_DEBUG == 1
  Serial.print("Raw temp: "); Serial.println(_ut);
#endif
  return true;
}

bool Adafruit_BMP085::readRawPressure(void) {
  uint16_t high = _read16(Register::Data);
  if (_error) return false;

  uint8_t low = _read8(Register::Data_XLSB);
  if (_error) return false;

  _up = (static_cast<uint32_t>(high) << 8) | low;
  _up >>= (8 - _oversampling);

#if BMP085_DEBUG == 1
  Serial.print("Raw pressure: "); Serial.println(_up);
#endif
  return true;
}


float Adafruit_BMP085::pressure(void) {
  if (!_have_b5)
    _computeB5();

  // do pressure calcs
  float B6 = _b5 - 62.5;	// Now also 1/64th the scale
  float X1 = _b2 * sq(B6);
  float X2 = _ac2 * B6;
  float X3 = X1 + X2;
  float B3 = (((_ac1 + X3) * _oss_mult) + 2) / 4;

#if BMP085_DEBUG == 1
  Serial.print("B6 = "); Serial.println(B6);
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
  Serial.print("X3 = "); Serial.println(X3);
  Serial.print("B3 = "); Serial.println(B3);
#endif

  X1 = _ac3 * B6;
  X2 = _b1 * sq(B6);
  X3 = ((X1 + X2) + 2) / 4;
  float B4 = _ac4 * (X3 + 32768);
  float B7 = (_up - B3) * _oss_scale;

#if BMP085_DEBUG == 1
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
  Serial.print("X3 = "); Serial.println(X3);
  Serial.print("B4 = "); Serial.println(B4);
  Serial.print("B7 = "); Serial.println(B7);
#endif

  float p = B7 * 2 / B4;
  X1 = sq(p / 256);
  X1 = (X1 * 3038) / 65536;
  X2 = (-7357 * p) / 65536;

#if BMP085_DEBUG == 1
  Serial.print("p = "); Serial.println(p);
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
#endif

  p += (X1 + X2 + 3791) / 16;
#if BMP085_DEBUG == 1
  Serial.print("p = "); Serial.println(p);
#endif
  return p;
}

float Adafruit_BMP085::sealevelPressure(float altitude_meters) {
  float p = pressure();
  return p / pow(1 - altitude_meters / 44330, 5.255);
}

float Adafruit_BMP085::temperature(void) {
  if (!_have_b5)
    _computeB5();

  return (_b5 + 0.125) * 0.4;
}

float Adafruit_BMP085::altitude(float sealevelPressure) {
  float p = pressure();

  return 44330 * (1 - pow(p / sealevelPressure, 0.1903));
}

uint8_t Adafruit_BMP085::_read8(Register addr) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
  Wire.write(static_cast<uint8_t>(addr)); // sends register address to read from
  Wire.endTransmission(); // end transmission

  _error = false;
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
  if (Wire.requestFrom(BMP085_I2CADDR, 1) < 1) { // send data n-bytes read
    _error = true;
    return 0;
  }

  int data = Wire.read(); // receive DATA
  if (data == -1) {
    _error = true;
    return 0;
  }
  Wire.endTransmission(); // end transmission

  return data & 0xff;
}

uint16_t Adafruit_BMP085::_read16u(Register addr) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
  Wire.write(static_cast<uint8_t>(addr)); // sends register address to read from
  Wire.endTransmission(); // end transmission

  _error = false;
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
  if (Wire.requestFrom(BMP085_I2CADDR, 2) < 2) { // send data n-bytes read
    _error = true;
    return 0;
  }

  int high = Wire.read();
  if (high == -1) {
    _error = true;
    return 0;
  }

  int low = Wire.read();
  if (low == -1) {
    _error = true;
    return 0;
  }

  Wire.endTransmission(); // end transmission

  return ((high & 0xff)  << 8) | (low & 0xff);
}

void Adafruit_BMP085::_write_cmd(Register addr, Command cmd) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device
  Wire.write(static_cast<uint8_t>(addr)); // sends register address to read from
  Wire.write(static_cast<uint8_t>(cmd));  // write command
  Wire.endTransmission(); // end transmission
}
