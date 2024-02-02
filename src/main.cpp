#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

//MQTT
const char* wifiSsid = "Wokwi-GUEST";
const char* wifiPassword = "";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;

const char* temperatureTopic = "ESP32/temperature";
const char* humidityTopic = "ESP32/humidity";
const char* statusTopic = "ESP32/status";
const char* controlTopic = "ESP32/control";

WiFiClient espClient;
PubSubClient client(espClient);

int analogValue = 0;
bool isButtonPressed = false;
const int sensorReadingInterval = 1000;

//DHT22 CODE
const int dhtPin = 13;
const int ledPin1 =  22;
DHT dhtSensor(dhtPin, DHT22);
void dht22(){
    float humidity = dhtSensor.readHumidity();
    float temperature = dhtSensor.readTemperature();
    Serial.println("Temp: " + String(temperature) + "°C");
    Serial.println("Humidity: " + String(humidity) + "%");
    if (temperature >= 34) {
    Serial.println("Temperature is above 34°C");
    digitalWrite(ledPin1, HIGH);
    } else {
    Serial.println("Temperature is below 34°C");
    digitalWrite(ledPin1, LOW);
    }
}

//LDR LAMP CODE
const int ldrPin = 34;
const int ledPin2 = 4;
const int pushButton = 35;
// LDR Characteristics
const float GAMMA = 0.7;
const float RL10 = 50;
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

unsigned long lastSensorReadingTime = 0;
bool activeStatus = false;

void showStatus(bool status) {
    String statusString;
    if (status) {
        digitalWrite(ledPin1, HIGH);
        statusString = "ON";
    } else {
        digitalWrite(ledPin1, LOW);
        statusString = "OFF";
    }
    Serial.printf("Status: %s\r\n", statusString.c_str());

    char statusMessage[5];
    snprintf(statusMessage, 5, "%s", statusString.c_str());
    client.publish(statusTopic, statusMessage);
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("Message received: %s\r\n", topic);
    char statusInput = (char)payload[0];
    Serial.printf("Payload: %c\r\n", statusInput);

    if (statusInput == '0') {
        activeStatus = false;
    } else if (statusInput == '1') {
        activeStatus = true;
    }
    showStatus(activeStatus);
}

void mqttConnect() {
    while (!client.connected()) {
        Serial.print("MQTT connecting ... ");
        char clientId[20];
        sprintf(clientId, "ESP32Client-%ld", random(1000));
        if (client.connect(clientId)) {
            Serial.print("Connected with id: ");
            Serial.println(clientId);
            client.subscribe(controlTopic);
        } else {
            Serial.print("failed, state: ");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    //SETUP DHT22
    pinMode(ledPin1, OUTPUT);
    dhtSensor.begin();

    //SETUP LDR LAMP
    pinMode(ledPin2, OUTPUT);
    pinMode(ldrPin, INPUT);
    pinMode(pushButton, INPUT);  

    Serial.printf("Connecting to %s ", wifiSsid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSsid, wifiPassword);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.printf("\r\nWiFi connected, IP address: %s\r\n", WiFi.localIP().toString().c_str());

    client.setServer(mqttServer, mqttPort);
    client.setCallback(receivedCallback);
}

void loop() {
    delay(500); // this speeds up the simulation
    // lamp();
    if (!client.connected()) {
      mqttConnect();
    }
    client.loop();

    unsigned long currentTime = millis();

    bool shouldMeasure = (currentTime - lastSensorReadingTime) > sensorReadingInterval;
    if (activeStatus && shouldMeasure) {
        lastSensorReadingTime = currentTime;
        float humidity = dhtSensor.readHumidity();
        float temperature = dhtSensor.readTemperature();
		
        Serial.printf("Humidity: %f %%, Temperature: %f C\r\n", humidity, temperature);

        char statusMessage[5];
        snprintf(statusMessage, 5, "%f", humidity);
        client.publish(humidityTopic, statusMessage);
        snprintf(statusMessage, 5, "%f", temperature);
        client.publish(temperatureTopic, statusMessage);
    }
    // dht22();
}

