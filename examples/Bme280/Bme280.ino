#include <Bme280BoschWrapper.h>

Bme280BoschWrapper bme280(true);

void setup() {
  Serial.begin(9600);
  Serial.println("BME280 Bosch test");
  
  while(!bme280.beginI2C(0x77))
  {
    Serial.println("Cannot find sensor.");
    delay(1000);
  }
}

void loop() {
  bool ok = bme280.measure();

  if(ok)
  {
    Serial.print("Temperature: ");
    Serial.print(bme280.getTemperature() / 100.0);
    Serial.println(" [C]");
    Serial.print("Humidity: ");
    Serial.print(bme280.getHumidity() / 1024.0);
    Serial.println(" [%]");
    Serial.print("Pressure: ");
    Serial.print(bme280.getPressure());
    Serial.println(" [Pa]");
    Serial.println();
  }
  else
  {
    Serial.println("Measuring failed.");
  }
  
  delay(5000);
}
