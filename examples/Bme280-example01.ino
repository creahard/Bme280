#include <Arduino.h>
#include <Bme280.hpp>

#define SDA_PIN   2
#define SCL_PIN   0

TwoWire *myWire;
Bme280 *bme280;

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();

  myWire = new TwoWire();
  myWire->begin(SDA_PIN, SCL_PIN);
  bme280 = new Bme280(myWire);
 
  Serial.println("Setting up TWI/I2C.");
  if(myWire->status() != 0) {
    Serial.println("I2C bus NOT OK.");
  }
  if(bme280->begin()) {
    if(bme280->configure(BME280_HUMID_OS_4, (BME280_TEMP_OS_4 | BME280_PRES_OS_4 | BME280_MODE_NORMAL), BME280_DELAY_500) == false) {
      Serial.println("Failed to configure BME280 sensor!");
      Serial.print("Error code: ");
      Serial.println(bme280->error);
    }
    Serial.println("BME280 setup for one shot measurements.");
  } else {
    Serial.println("No BME280 sensor detected!");
    Serial.print("Error code: ");
    Serial.println(bme280->error);
  }
}

void loop() {
  uint8_t status = myWire->status();
  switch (status) {
    case 0:
      if (true) { //bme280->ready()) {
        if (bme280->measure()) {
          delay(500);
          bme280->read();
          Serial.print("Temperature: ");
          Serial.print(bme280->temperature * 9/5 + 32);
          Serial.print("F Humitidy: ");
          Serial.print(bme280->humidity);
          Serial.print("% Pressure: ");
          Serial.print(bme280->pressure * 0.0002953);
          Serial.println("inmg");
        } else {
          Serial.println("BME280 failed to take measurement!");
          Serial.print("Error code: ");
          Serial.println(bme280->error);
        }
      } else {
        Serial.println("BME280 is not ready!");
        Serial.print("Error code: ");
        Serial.println(bme280->error);
      }
      break;
    case 1:
      Serial.println("I2C: SCL Line held low.");
      break;
    case 2:
      Serial.println("I2C: SCL Line held low after read.");
      break;
    case 3:
      Serial.println("I2C: SDA Line held low.");
      break;
    default:
      Serial.print("I2C: Unknown status number: ");
      Serial.println(status);
      break;
  }
  delay(5000);
}