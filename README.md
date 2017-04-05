# BME280_Bosch_Wrapper
Arduino library for BME280 based on original code supplied by Bosch.

BME280 is a very small sensor for measuring air temperature, relative humidity and pressure. It features I2C and SPI connection. https://www.bosch-sensortec.com/bst/products/all_products/bme280

This code wraps the original driver from Bosch so that it can be used unmodified with Arduino.

Bosch driver repository: https://github.com/BoschSensortec/BME280_driver

#Function
The original Bosch driver is included in this package and it has not been modified in any way. I have only supplied wrapper code which enables compilation with Arduino IDE and provides implementation of Arduino I2C and SPI for the driver.

This driver seems to report slighly different values than some other Arduino BME280 drivers, with the difference for temperature being about 0.1 - 0.5 °C. The difference is caused by the driver, however I have not yet figured out which exact part of the code is causing that.

I tried to compare the temperature with temperature from SMT-172 (sensor with manufacturer declared accuracy +-0.1°C) and the difference caused by the driver meant that the value was closer to what SMT-172 measured, so I assume the Bosch driver is correct.

#Compatibility
I have tested this with Arduino UNO. Both I2C and SPI are implemented and working on my UNO. I have received success reports from people using Arduino on various boards based on ESP8266.

If you are able to use this driver with different boards, please let me know.

#Copyright
Files bme280.c and bme280.h are Copyright (C) 2013 - 2016 Bosch Sensortec GmbH

All other files are written by Jan Wasserbauer and are licensed under the terms of GNU GPL v3.

