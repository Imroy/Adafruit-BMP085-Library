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
  uint8_t d = read8(0xD0);
  if (error) return false;
  return d == 0x55;
}

boolean Adafruit_BMP085::begin(uint8_t mode) {
  if (mode > BMP085_ULTRAHIGHRES) 
    mode = BMP085_ULTRAHIGHRES;
  oversampling = mode;

  if (!check())
    return false;

  /* read calibration data */
  ac1 = read16(BMP085_CAL_AC1);
  if (error) return false;
  ac2 = read16(BMP085_CAL_AC2);
  if (error) return false;
  ac3 = read16(BMP085_CAL_AC3);
  if (error) return false;
  ac4 = read16(BMP085_CAL_AC4);
  if (error) return false;
  ac5 = read16(BMP085_CAL_AC5);
  if (error) return false;
  ac6 = read16(BMP085_CAL_AC6);
  if (error) return false;

  b1 = read16(BMP085_CAL_B1);
  if (error) return false;
  b2 = read16(BMP085_CAL_B2);
  if (error) return false;

  mb = read16(BMP085_CAL_MB);
  if (error) return false;
  mc = read16(BMP085_CAL_MC);
  if (error) return false;
  md = read16(BMP085_CAL_MD);
  if (error) return false;
#if (BMP085_DEBUG == 1)
  Serial.print("ac1 = "); Serial.println(ac1, DEC);
  Serial.print("ac2 = "); Serial.println(ac2, DEC);
  Serial.print("ac3 = "); Serial.println(ac3, DEC);
  Serial.print("ac4 = "); Serial.println(ac4, DEC);
  Serial.print("ac5 = "); Serial.println(ac5, DEC);
  Serial.print("ac6 = "); Serial.println(ac6, DEC);

  Serial.print("b1 = "); Serial.println(b1, DEC);
  Serial.print("b2 = "); Serial.println(b2, DEC);

  Serial.print("mb = "); Serial.println(mb, DEC);
  Serial.print("mc = "); Serial.println(mc, DEC);
  Serial.print("md = "); Serial.println(md, DEC);
#endif

  return true;
}

int32_t Adafruit_BMP085::computeB5(void) {
  int32_t X1 = (UT - (int32_t)ac6) * ((int32_t)ac5) >> 15;
  int32_t X2 = ((int32_t)mc << 11) / (X1+(int32_t)md);
  return X1 + X2;
}

bool Adafruit_BMP085::readRawTemperature(void) {
  write8(BMP085_CONTROL, BMP085_READTEMPCMD);
  delay(5);
  uint16_t d = read16(BMP085_TEMPDATA);
  if (error) return false;
  UT = d;

#if BMP085_DEBUG == 1
  Serial.print("Raw temp: "); Serial.println(UT);
#endif
  return true;
}

bool Adafruit_BMP085::readRawPressure(void) {
  write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (oversampling << 6));

  if (oversampling == BMP085_ULTRALOWPOWER) 
    delay(5);
  else if (oversampling == BMP085_STANDARD) 
    delay(8);
  else if (oversampling == BMP085_HIGHRES) 
    delay(14);
  else 
    delay(26);

  uint16_t d;
  d = read16(BMP085_PRESSUREDATA);
  if (error) return false;
  UP = (uint32_t)d << 8;
  d = read8(BMP085_PRESSUREDATA+2);
  if (error) return false;
  UP |= d;
  UP >>= (8 - oversampling);

#if BMP085_DEBUG == 1
  Serial.print("Raw pressure: "); Serial.println(UP);
#endif
  return true;
}


int32_t Adafruit_BMP085::pressure(void) {
  int32_t B3, B5, B6, X1, X2, X3, p;
  uint32_t B4, B7;

  B5 = computeB5();

#if BMP085_DEBUG == 1
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
  Serial.print("B5 = "); Serial.println(B5);
#endif

  // do pressure calcs
  B6 = B5 - 4000;
  X1 = ((int32_t)b2 * ( (B6 * B6)>>12 )) >> 11;
  X2 = ((int32_t)ac2 * B6) >> 11;
  X3 = X1 + X2;
  B3 = ((((int32_t)ac1*4 + X3) << oversampling) + 2) / 4;

#if BMP085_DEBUG == 1
  Serial.print("B6 = "); Serial.println(B6);
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
  Serial.print("B3 = "); Serial.println(B3);
#endif

  X1 = ((int32_t)ac3 * B6) >> 13;
  X2 = ((int32_t)b1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  B4 = ((uint32_t)ac4 * (uint32_t)(X3 + 32768)) >> 15;
  B7 = ((uint32_t)UP - B3) * (uint32_t)( 50000UL >> oversampling );

#if BMP085_DEBUG == 1
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
  Serial.print("B4 = "); Serial.println(B4);
  Serial.print("B7 = "); Serial.println(B7);
#endif

  if (B7 < 0x80000000) {
    p = (B7 * 2) / B4;
  } else {
    p = (B7 / B4) * 2;
  }
  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;

#if BMP085_DEBUG == 1
  Serial.print("p = "); Serial.println(p);
  Serial.print("X1 = "); Serial.println(X1);
  Serial.print("X2 = "); Serial.println(X2);
#endif

  p = p + ((X1 + X2 + (int32_t)3791)>>4);
#if BMP085_DEBUG == 1
  Serial.print("p = "); Serial.println(p);
#endif
  return p;
}

int32_t Adafruit_BMP085::sealevelPressure(float altitude_meters) {
  float p = pressure();
  return (int32_t)(p / pow(1.0-altitude_meters/44330, 5.255));
}

float Adafruit_BMP085::temperature(void) {
  int32_t B5;     // following ds convention
  float temp;

  B5 = computeB5();
  temp = (B5+8) >> 4;
  temp /= 10;
  
  return temp;
}

float Adafruit_BMP085::altitude(float sealevelPressure) {
  float p = pressure();

  return 44330 * (1.0 - pow(p / sealevelPressure, 0.1903));
}


/*********************************************************************/

uint8_t Adafruit_BMP085::read8(uint8_t a) {
  uint8_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 1);// send data n-bytes read
  int d;
  error = false;
#if (ARDUINO >= 100)
  d = Wire.read(); // receive DATA
#else
  d = Wire.receive(); // receive DATA
#endif
  if (d == -1) {
    error = true;
    return 0;
  }
  ret = d;
  Wire.endTransmission(); // end transmission

  return ret;
}

uint16_t Adafruit_BMP085::read16(uint8_t a) {
  uint16_t ret;

  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
#else
  Wire.send(a); // sends register address to read from
#endif
  Wire.endTransmission(); // end transmission
  
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
  Wire.requestFrom(BMP085_I2CADDR, 2);// send data n-bytes read
  int d;
  error = false;
#if (ARDUINO >= 100)
  d = Wire.read();
  if (d == -1) {
    error = true;
    return 0;
  }
  ret = (uint8_t)d << 8;
  d = Wire.read();
  if (d == -1) {
    error = true;
    return 0;
  }
  ret |= (uint8_t)d;
#else
  d = Wire.receive();
  if (d == -1) {
    error = true;
    return 0;
  }
  ret = (uint8_t)d << 8;
  d = Wire.receive();
  if (d == -1) {
    error = true;
    return 0;
  }
  ret |= (uint8_t)d;
#endif
  Wire.endTransmission(); // end transmission

  return ret;
}

void Adafruit_BMP085::write8(uint8_t a, uint8_t d) {
  Wire.beginTransmission(BMP085_I2CADDR); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(a); // sends register address to read from
  Wire.write(d);  // write data
#else
  Wire.send(a); // sends register address to read from
  Wire.send(d);  // write data
#endif
  Wire.endTransmission(); // end transmission
}
