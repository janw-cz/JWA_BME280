#include <Arduino.h>

#include <limits.h>
#include <SPI.h>
#include <Wire.h>

#include "Bme280BoschWrapper.h"

#define DOUBLE_NOT_CALCULATED -1000.0

int Bme280BoschWrapper::_cs = -1;

Bme280BoschWrapper::Bme280BoschWrapper(bool forced)
{
  this->forced = forced;
}

bool Bme280BoschWrapper::beginI2C(u8 dev_addr)
{
  I2CInit();
  bme280.dev_addr = dev_addr;
  s8 ret = bme280_init(&bme280);
  return (ret == SUCCESS);
}

bool Bme280BoschWrapper::beginSPI(int8_t cspin)
{
  Bme280BoschWrapper::_cs = cspin;

  SPIInit();
  pinMode(_cs, OUTPUT);

  s8 ret = bme280_init(&bme280);

  return (ret == SUCCESS);
}
  
bool Bme280BoschWrapper::measure()
{
  temperature = LONG_MIN;
  humidity = ULONG_MAX;
  pressure = ULONG_MAX;

  dTemperature = DOUBLE_NOT_CALCULATED;
  dHumidity = DOUBLE_NOT_CALCULATED;
  dPressure = DOUBLE_NOT_CALCULATED;

  pressurePrec = ULONG_MAX;
   
  s8 ret = SUCCESS;

  ret += bme280_set_oversamp_humidity(BME280_OVERSAMP_16X);
  ret += bme280_set_oversamp_pressure(BME280_OVERSAMP_16X);
  ret += bme280_set_oversamp_temperature(BME280_OVERSAMP_16X);

  if(!forced)
  {
    ret += bme280_read_uncomp_pressure_temperature_humidity(&rawPressure, &rawTemperature, &rawHumidity);
  }
  else
  {
    ret += bme280_get_forced_uncomp_pressure_temperature_humidity(&rawPressure, &rawTemperature, &rawHumidity);
  } 

  return (ret == SUCCESS);
}

s32 Bme280BoschWrapper::getRawTemperature()
{
  return rawTemperature;
}

s32 Bme280BoschWrapper::getRawHumidity()
{
  return rawHumidity;
}

s32 Bme280BoschWrapper::getRawPressure()
{
  return rawPressure;
}

s32 Bme280BoschWrapper::getTemperature()
{
  if(temperature == LONG_MIN)
  {
    temperature = bme280_compensate_temperature_int32(rawTemperature);
    dTemperature = DOUBLE_NOT_CALCULATED;
  }
  
  return temperature;
}

u32 Bme280BoschWrapper::getHumidity()
{
  if(humidity == ULONG_MAX)
  {
    if(temperature == LONG_MIN)
    {
      //temperature calculation prepares value for humidity calculation
      getTemperature();    
    }
  
    humidity = bme280_compensate_humidity_int32(rawHumidity);
  }
  
  return humidity;
}

u32 Bme280BoschWrapper::getPressure()
{
  if(pressure == ULONG_MAX)
  {
    if(temperature == LONG_MIN)
    {
      //temperature calculation prepares value for pressure calculation
      getTemperature();    
    }
  
    pressure = bme280_compensate_pressure_int32(rawPressure);
  }
  
  return pressure;
}

double Bme280BoschWrapper::getTemperatureDouble()
{
  if(dTemperature == DOUBLE_NOT_CALCULATED)
  {
    dTemperature = bme280_compensate_temperature_double(rawTemperature);
    temperature = LONG_MIN;
  }
  
  return dTemperature;
}

double Bme280BoschWrapper::getHumidityDouble()
{
  if(dHumidity == DOUBLE_NOT_CALCULATED)
  {
    if(dTemperature == DOUBLE_NOT_CALCULATED)
    {
      //temperature calculation prepares value for humidity calculation
      getTemperatureDouble();    
    }
  
    dHumidity = bme280_compensate_humidity_double(rawHumidity);
  }

  return dHumidity;
}

double Bme280BoschWrapper::getPressureDouble()
{
  if(dPressure == DOUBLE_NOT_CALCULATED)
  {
    if(dTemperature == DOUBLE_NOT_CALCULATED)
    {
      //temperature calculation prepares value for pressure calculation
      getTemperatureDouble();    
    }
  
    dPressure = bme280_compensate_pressure_double(rawPressure);
  }

  return dPressure;
}

u32 Bme280BoschWrapper::getPressurePrec()
{
  if(pressurePrec == ULONG_MAX)
  {
    if(dTemperature == DOUBLE_NOT_CALCULATED)
    {
      //temperature calculation prepares value for pressure calculation
      getTemperatureDouble();    
    }
  
    pressurePrec = bme280_compensate_pressure_int64(rawPressure);
  }
  
  return pressurePrec;
}

/**
 * Wrapper functions for Bosch BME280 driver.
 */
#define SPI_READ  0x80
#define SPI_WRITE 0x7F

SPISettings bme280SpiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

void Bme280BoschWrapper::I2CInit() 
{
  bme280.bus_write = Bme280BoschWrapper::I2CWrite;
  bme280.bus_read = Bme280BoschWrapper::I2CRead;
  bme280.dev_addr = BME280_I2C_ADDRESS2;
  bme280.delay_msec = Bme280BoschWrapper::delaymsec;

  Wire.begin();
}

void Bme280BoschWrapper::SPIInit() 
{
  bme280.bus_write = Bme280BoschWrapper::SPIWrite;
  bme280.bus_read = Bme280BoschWrapper::SPIRead;
  bme280.delay_msec = Bme280BoschWrapper::delaymsec;

  SPI.begin();
}

s8 Bme280BoschWrapper::I2CRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
//  Serial.println("I2C_bus_read");
  s8 ret = SUCCESS;

//  Serial.println(dev_addr, HEX);
  Wire.beginTransmission(dev_addr);
  
//  Serial.println(reg_addr, HEX);
  Wire.write(reg_addr);
  Wire.endTransmission();
  
  Wire.requestFrom(dev_addr, cnt);
  
  u8 available = Wire.available();
  if(available != cnt)
  {
    ret = ERROR;
  }
  
  for(u8 i = 0; i < available; i++)
  {
    if(i < cnt) 
    {
      *(reg_data + i) = Wire.read();
//      Serial.print(*(reg_data + i), HEX);
//      Serial.print(" ");
    }
    else
      Wire.read();
  }

//  Serial.println();
  
  return ret;
}

s8 Bme280BoschWrapper::I2CWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{  
//  Serial.println("I2C_bus_write");
  s8 ret = SUCCESS;

//  Serial.println(dev_addr, HEX);
  Wire.beginTransmission(dev_addr);

//  Serial.println(reg_addr, HEX);
  Wire.write(reg_addr);
  Wire.write(reg_data, cnt);
  Wire.endTransmission();
  
  return ret;
}

s8 Bme280BoschWrapper::SPIRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
//  Serial.println("SPI_bus_read");
  s32 ret = SUCCESS;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);

//  Serial.println(reg_addr | SPI_READ, HEX);

  SPI.transfer(reg_addr | SPI_READ);
  for (u8 i = BME280_INIT_VALUE; i < cnt; i++) {
    *(reg_data + i) = SPI.transfer(0);
    
//    Serial.print(*(reg_data + i), HEX);
//    Serial.print(" ");
  }

//  Serial.println();
  
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

s8 Bme280BoschWrapper::SPIWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
//  Serial.println("SPI_bus_write");
  s8 ret = SUCCESS;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);
  for (u8 i = 0; i < cnt; i++) 
  {
    u8 addr = (reg_addr++) & SPI_WRITE;
    u8 data = *(reg_data + i);

//    Serial.print(addr, HEX);
//    Serial.print(" ");
//    Serial.print(data, HEX);

    SPI.transfer(addr);
    SPI.transfer(data);
  }
//  Serial.println();

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

void Bme280BoschWrapper::delaymsec(u32 msec)
{
  delay(msec);
}

