
#include <DHT.h>

const int ldrPin = 34;
const int ledPin2 = 4;
const int pushButton = 35;
const int dhtPin = 13;
const int ledPin1 =  22;
int analogValue = 0;
bool isButtonPressed = false;

// LDR Characteristics
const float GAMMA = 0.7;
const float RL10 = 50;

DHT dhtSensor(dhtPin, DHT22);
void lamp() {
  analogValue = analogRead(ldrPin);
  float voltage = analogValue / 4096. * 5;
  float resistance = 2000 * voltage / (1 - voltage / 5);
  float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

  if (digitalRead(pushButton) == HIGH) {
    isButtonPressed = !isButtonPressed;  
    delay(500);  
  }
  if (isButtonPressed) {
    digitalWrite(ledPin2, HIGH);
  }else{
    if (lux < 100) {
      digitalWrite(ledPin2, HIGH);
    }else {
      digitalWrite(ledPin2, LOW);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin1, OUTPUT);
  Serial.begin(115200);
  dhtSensor.begin();
  Serial.println("Hello, ESP32!");
  pinMode(ledPin2, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(pushButton, INPUT);  
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(500); // this speeds up the simulation
  float humidity = dhtSensor.readHumidity();
  float temperature = dhtSensor.readTemperature();
  Serial.println("Temp: " + String(temperature) + "°C");
  Serial.println("Humidity: " + String(humidity) + "%");
  lamp();
if (temperature >= 34) {
  Serial.println("Temperature is above 34°C");
  digitalWrite(ledPin1, HIGH);
} else {
  Serial.println("Temperature is below 34°C");
  digitalWrite(ledPin1, LOW);
}
}