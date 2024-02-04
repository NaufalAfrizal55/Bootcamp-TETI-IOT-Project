#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>

//MQTT
const char* wifiSsid = "Wokwi-GUEST";
const char* wifiPassword = "";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;

//CONTROL TOPIC => TO ON/OFF THE IOT
const char* controlTopic = "Kel5_ESP32/control";

//TEMP & HUM => MONITORING TEMPERATURE & HUMIDITY
const char* temperatureTopic = "Kel5_ESP32/temperature";
const char* humidityTopic = "Kel5_ESP32/humidity";
const char* statusTopic = "Kel5_ESP32/status";

//DOOR LOCKING => CONTROL TO LOCK/UNLOCK DOOR
const char* doorLockingTopic = "Kel5_ESP32/locking";
const char* doorStatusTopic = "Kel5_ESP32/doorstatus";

//LDR LAMP
const char* ldrLampTopic = "Kel5_ESP32/ldrlamp";
const char* ldrButtonTopic = "Kel5_ESP32/ldrbutton";

//WATER => MONITORING WATER LEVEL 
const char* waterTopic = "Kel5_ESP32/waterlevel";

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
		
    // Serial.printf("Humidity: %f %%, Temperature: %f C\r\n", humidity, temperature);

    //PUBLISH TO MQTT TEMP & HUM TOPIC
    char statusMessage[5];
    snprintf(statusMessage, 5, "%f", humidity);
    client.publish(humidityTopic, statusMessage);
    snprintf(statusMessage, 5, "%f", temperature);
    client.publish(temperatureTopic, statusMessage);
}

//LDR LAMP CODE
const int ldrPin = 34;
const int ledPin2 = 4;
const int pushButton = 35;
// LDR Characteristics
const float GAMMA = 0.7;
const float RL10 = 50;
void lamp() {
    String ldrStatus;
    analogValue = analogRead(ldrPin);
    float voltage = analogValue / 4096. * 5;
    float resistance = 2000 * voltage / (1 - voltage / 5);
    float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

    //PUBLISH TO MQTT LDR LAMP TOPIC
    char statusMessage[5];
    snprintf(statusMessage, 5, "%f", lux);
    client.publish(ldrLampTopic, statusMessage);

    if (digitalRead(pushButton) == HIGH) {
        isButtonPressed = !isButtonPressed;  
        ldrStatus = "ON";
        delay(500);  
    }
    if (isButtonPressed) {
        digitalWrite(ledPin2, HIGH);
        ldrStatus = "ON";
    } else if (lux < 100) {
        digitalWrite(ledPin2, HIGH);
        ldrStatus = "ON";
    } else {
        digitalWrite(ledPin2, LOW);
        ldrStatus = "OFF";
    }
    char statusMessage2[5];
    snprintf(statusMessage2, 5, "%s", ldrStatus.c_str());
    client.publish(ldrButtonTopic, statusMessage2);
}

//ULTRASONIC HC-SR04
#define Trig_pin 14
#define Echo_pin 27
#define buzzer 26
float durasi;
float jarak;
int batas_kosong=80;
int batas_penuh=10;

void ukur_jarak() //distance calculaion...
{
  digitalWrite(Trig_pin, LOW);
  delay(10);
  digitalWrite(Trig_pin, HIGH);
  delay(10);
  digitalWrite(Trig_pin, LOW);
  
  durasi = pulseIn(Echo_pin, HIGH);
  jarak = durasi * 0.034 / 2;

  Serial.println(jarak);
}

void output_jarak(){
  if(jarak > batas_kosong) {
    digitalWrite(10,HIGH);// Pump On...
    tone(buzzer, 450);//digitalWrite(buzzer, HIGH);//Buzzer beeping......
    delay(2000);
    Serial.print("pump on\n");
  }
  else if(jarak <= batas_penuh) {
    digitalWrite(10,LOW);// pump off...
    noTone(buzzer);//digitalWrite(buzzer, LOW);
    Serial.print("pump off\n");
    delay(100);
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
        dht22();
        lamp();
        ukur_jarak();
        output_jarak();
    }
}

