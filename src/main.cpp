//INCLUDE LIBRARY
#include <DHT.h>

//DECLARE PIN DHT22
const int dhtPin = 13;
DHT dhtSensor(dhtPin, DHT22);

void setup() {
  Serial.begin(115200);

  //INITIALIZE DHT22
  dhtSensor.begin();
  Serial.println("Hello, ESP32!");
}

void loop() {
  delay(500); // this speeds up the simulation
  //FETCH DATA SENSOR
  float humidity = dhtSensor.readHumidity();
  float temperature = dhtSensor.readTemperature();

  //DISPLAY DATA TO SERIAL
  Serial.println("Temp: " + String(temperature) + "°C");
  Serial.println("Humidity: " + String(humidity) + "%");
}
