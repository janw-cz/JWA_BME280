# BME280_Bosch_Wrapper
## News 27-3-2020
Updated BME280 driver to version 3.4.3.

## News 1-4-2018
Updated BME280 driver to version 3.3.4.

## News 21-7-2017
Updated BME280 driver to version 3.3.0.

## News 28-6-2017
Updated BME280 driver to version 3.2.0.
* int humidity is now returned in 1/1000% (previous version used 1/1024%)
* Value conversion using float or long values is not supported anymore.

## About
This is Arduino library for BME280 environmental sensor. This library is based on reference code released by Bosch Sensortec.

BME280 is very small sensor for measuring air temperature, relative humidity and pressure. It features I2C and SPI connection. https://www.bosch-sensortec.com/bst/products/all_products/bme280

Bosch driver repository: https://github.com/BoschSensortec/BME280_driver

## Function
The original Bosch driver is included in this package and it has not been modified in any way. I have only supplied wrapper code which enables compilation with Arduino IDE and provides implementation of Arduino I2C and SPI for the driver.

This driver seems to report slighly different values than some other Arduino BME280 drivers, with the difference for temperature being about 0.1 - 0.5 °C. The difference is caused by the driver, however I have not yet figured out which exact part of the code is causing that.

I tried to compare the temperature with temperature from SMT-172 (sensor with manufacturer declared accuracy +-0.1°C) and the difference caused by the driver meant that the value was closer to what SMT-172 measured, so I assume the Bosch driver is correct.

## Compatibility
I have tested this with Arduino UNO. Both I2C and SPI are implemented and working on my UNO. I have received success reports from people using Arduino on various boards based on ESP8266.

If you are able to use this driver with different boards, please let me know.

## Copyright
Files bme280.c, bme280.h and bme280_defs.h are Copyright (c) 2013 - 2017 Bosch Sensortec GmbH

All other files are written by Jan Wasserbauer and are licensed under the terms of GNU GPL v3.

