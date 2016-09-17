#ifndef __BMP280_BW_H__
#define __BMP280_BW_H__

extern "C" {        
#define __STDC_VERSION__ 201112L
#define uint16_t unsigned short 
#include <bme280.h>
#undef __STDC_VERSION__ 
#undef uint16_t
}

class Bme280BoschWrapper
{
  public:
    //true: uses forced mode, sensore measures values on demand
    //false: uses continuous measuring mode
    Bme280BoschWrapper(bool forcedMode);

    bool beginI2C(u8 dev_addr = 0x77);
    bool beginSPI(int8_t cspin);

    //this method performs measurement
    //be sure to call it before reading values
    bool measure();

    //Temperature in degrees of Celsius * 100
    s32 getTemperature();

    //Relative humidity in % * 1024
    u32 getHumidity();

    //Air pressure in Pa
    u32 getPressure();

    //double calculations are more CPU intensive
    //Temperature in degrees of Celsius
    double getTemperatureDouble();

    //Relative humidity in %
    double getHumidityDouble();

    //Air pressure in Pa
    double getPressureDouble();

    //Air pressure in Pa * 256
    u32 getPressurePrec();

    //Raw values as returned from sensor
    //without calculation into any specific scale
    s32 getRawTemperature();
    s32 getRawHumidity();
    s32 getRawPressure();

  private:
    void I2CInit();
    void SPIInit();
  
    static s8 I2CRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
    static s8 I2CWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
    static s8 SPIRead(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
    static s8 SPIWrite(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
    static void delaymsec(u32 msec);

    static int _cs;

    s32 rawTemperature;
    s32 rawHumidity;
    s32 rawPressure;

    s32 temperature;
    u32 humidity;
    u32 pressure;

    double dTemperature;
    double dPressure;
    double dHumidity;

    u32 pressurePrec;

    struct bme280_t bme280;

    bool forced;
};

#endif

