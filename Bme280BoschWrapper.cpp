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

bool Bme280BoschWrapper::beginI2C(uint8_t dev_addr)
{
  I2CInit();
  bme280.dev_id = dev_addr;

  int8_t ret = bme280_init(&bme280);

  setSensorSettings();

  return (ret == BME280_OK);
}

bool Bme280BoschWrapper::beginSPI(int8_t cspin)
{
  Bme280BoschWrapper::_cs = cspin;

  SPIInit();
  pinMode(_cs, OUTPUT);

  int8_t ret = bme280_init(&bme280);

  setSensorSettings();

  return (ret == BME280_OK);
}
  
bool Bme280BoschWrapper::measure()
{
  int8_t ret = BME280_OK;

  if(forced)
  {
    setSensorSettings();
    bme280.delay_ms(255);
    ret += bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, &bme280);
  }
  else
  {
    ret += bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, &bme280);
  }

  if(ret != BME280_OK) {
    error = true;
  }

  return (ret == BME280_OK);
}

int32_t Bme280BoschWrapper::getTemperature()
{
  return comp_data.temperature;
}

u32 Bme280BoschWrapper::getHumidity()
{
  return comp_data.humidity;
}

u32 Bme280BoschWrapper::getPressure()
{
  return comp_data.pressure;
}

/**
 * Wrapper functions for Bosch BME280 driver.
 */
#define SPI_READ  0x80
#define SPI_WRITE 0x7F

SPISettings bme280SpiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

void Bme280BoschWrapper::I2CInit() 
{
  bme280.intf = BME280_I2C_INTF;
  bme280.write = Bme280BoschWrapper::I2CWrite;
  bme280.read = Bme280BoschWrapper::I2CRead;
  bme280.delay_ms = Bme280BoschWrapper::delaymsec;

  Wire.begin();
}

void Bme280BoschWrapper::SPIInit() 
{
  bme280.dev_id = 0;
  bme280.intf = BME280_SPI_INTF;
  bme280.write = Bme280BoschWrapper::SPIWrite;
  bme280.read = Bme280BoschWrapper::SPIRead;
  bme280.delay_ms = Bme280BoschWrapper::delaymsec;

  SPI.begin();
}

int8_t Bme280BoschWrapper::I2CRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
//  Serial.println("I2C_bus_read");
  int8_t ret = BME280_OK;

//  Serial.println(dev_addr, HEX);
  Wire.beginTransmission(dev_addr);
  
//  Serial.println(reg_addr, HEX);
  Wire.write(reg_addr);
  Wire.endTransmission();
  
  Wire.requestFrom((int)dev_addr, (int)cnt);
  
  uint8_t available = Wire.available();
  if(available != cnt)
  {
    ret = BME280_E_COMM_FAIL;
  }
  
  for(uint8_t i = 0; i < available; i++)
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

int8_t Bme280BoschWrapper::I2CWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{  
//  Serial.println("I2C_bus_write");
  int8_t ret = BME280_OK;

//  Serial.println(dev_addr, HEX);
  Wire.beginTransmission(dev_addr);

//  Serial.println(reg_addr, HEX);
  Wire.write(reg_addr);
  Wire.write(reg_data, cnt);
  Wire.endTransmission();
  
  return ret;
}

int8_t Bme280BoschWrapper::SPIRead(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
//  Serial.println("SPI_bus_read");
  int32_t ret = BME280_OK;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);

//  Serial.println(reg_addr | SPI_READ, HEX);

  SPI.transfer(reg_addr | SPI_READ);
  for (uint8_t i = 0; i < cnt; i++) {
    *(reg_data + i) = SPI.transfer(0);
    
//    Serial.print(*(reg_data + i), HEX);
//    Serial.print(" ");
  }

//  Serial.println();
  
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

int8_t Bme280BoschWrapper::SPIWrite(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
//  Serial.println("SPI_bus_write");
  int8_t ret = BME280_OK;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);
  for (uint8_t i = 0; i < cnt; i++) 
  {
    uint8_t addr = (reg_addr++) & SPI_WRITE;
    uint8_t data = *(reg_data + i);

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

int8_t Bme280BoschWrapper::setSensorSettings()
{
  int8_t ret = BME280_OK;

  uint8_t settings_sel;

  bme280.settings.osr_h = BME280_OVERSAMPLING_16X;
  bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
  bme280.settings.osr_t = BME280_OVERSAMPLING_16X;

  settings_sel = BME280_OSR_PRESS_SEL|BME280_OSR_TEMP_SEL|BME280_OSR_HUM_SEL;

  ret += bme280_set_sensor_settings(settings_sel, &bme280);

  if(forced)
    ret += bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
  else
    ret += bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);

  return ret;
}
