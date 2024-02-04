#include <DHT.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

//MQTT
const char* wifiSsid = "Wokwi-GUEST";
const char* wifiPassword = "";
const char* mqttServer = "broker.emqx.io";
const int mqttPort = 1883;

//CONTROL TOPIC => TO ON/OFF THE IOT
const char* controlTopic = "Kel5_ESP32/control";
const char* statusTopic = "Kel5_ESP32/status";

//TEMP & HUM => MONITORING TEMPERATURE & HUMIDITY
const char* temperatureTopic = "Kel5_ESP32/temperature";
const char* humidityTopic = "Kel5_ESP32/humidity";
const char* acStatusTopic = "Kel5_ESP32/acstatus";

//DOOR LOCKING => CONTROL TO LOCK/UNLOCK DOOR
const char* doorLockingTopic = "Kel5_ESP32/locking";
const char* doorStatusTopic = "Kel5_ESP32/doorstatus";

//LDR LAMP
const char* ldrLampTopic = "Kel5_ESP32/ldrlamp";
const char* ldrStatusTopic = "Kel5_ESP32/ldrstatus";
const char* ldrButtonTopic = "Kel5_ESP32/ldrbutton";

//WATER => MONITORING WATER LEVEL 
const char* waterTopic = "Kel5_ESP32/waterlevel";
const char* waterStatusTopic = "Kel5_ESP32/waterstatus";

WiFiClient espClient;
PubSubClient client(espClient);

int analogValue = 0;
bool isButtonPressed = false;
const int sensorReadingInterval = 1000;
unsigned long lastSensorReadingTime = 0;
bool activeStatus = false;
bool ldrActiveStatus = false;

/*-----------DHT22 FUNCTION---------------*/
const int dhtPin = 13;
const int ledPin1 =  22;
DHT dhtSensor(dhtPin, DHT22);
void dht22(){
    String statusString;
    float humidity = dhtSensor.readHumidity();
    float temperature = dhtSensor.readTemperature();
	
    if(temperature > 28){
        digitalWrite(ledPin1, HIGH);
        statusString = "ON";
    } else {
        digitalWrite(ledPin1, LOW);
        statusString = "OFF";        
    }
    // Serial.printf("Humidity: %f %%, Temperature: %f C\r\n", humidity, temperature);

    //PUBLISH TEMP & HUM TOPIC TO MQTT
    char statusMessage[5];
    snprintf(statusMessage, 5, "%f", humidity);
    client.publish(humidityTopic, statusMessage);
    snprintf(statusMessage, 5, "%f", temperature);
    client.publish(temperatureTopic, statusMessage);
    snprintf(statusMessage, 5, "%s", statusString.c_str());
    client.publish(acStatusTopic, statusMessage);
}
/*----------END DHT22 FUNCTION-----------------*/

/*---------DOOR LOCKING----------*/
//DOOR LOCKING
bool doorLockingStatus = false;
const int servoPin = 32;
const int relay = 12;
Servo servo; 
void doorStatus(bool status) {
    String statusString;
    if (status) {
        statusString = "LOCKED";
    } else {
        statusString = "UNLOCKED";
    }
    Serial.printf("Door Status: %s\r\n", statusString.c_str());

    //PUBLISH LDR LAMP STATUS TO MQTT
    char statusMessage[5];
    snprintf(statusMessage, 5, "%s", statusString.c_str());
    client.publish(doorStatusTopic, statusMessage);
}

void doorLock(bool doorLockingStatus){
    if (doorLockingStatus){
        digitalWrite(relay, HIGH);
        servo.write(90);
        delay(10); 
    } else {
        digitalWrite(relay, LOW);
        servo.write(180);
        delay(10); 
    }
}
/*---------END DOOR LOCKING----------*/

/*-------------LDR LAMP FUNCTION-------------------*/
//LDR LAMP CODE
const int ldrPin = 34;
const int ledPin2 = 4;
// LDR Characteristics
const float GAMMA = 0.7;
const float RL10 = 50;
void lamp() {
    String ldrStatus;
    analogValue = analogRead(ldrPin);
    float voltage = analogValue / 4096. * 5;
    float resistance = 2000 * voltage / (1 - voltage / 5);
    float lux = pow(RL10 * 1e3 * pow(10, GAMMA) / resistance, (1 / GAMMA));

    //PUBLISH LDR LAMP TOPIC TO MQTT
    char statusMessage[5];
    snprintf(statusMessage, 5, "%f", lux);
    client.publish(ldrLampTopic, statusMessage);

//PRIORITY LAMP ON : BUTTON DASHBOARD > BUTTON > LUX
    if(ldrActiveStatus){
        digitalWrite(ledPin2, HIGH);
        ldrStatus = "ON";        
    } else if(!ldrActiveStatus){
            if(lux <= 100){
                digitalWrite(ledPin2, HIGH);
                ldrStatus = "ON"; 
            } 
            else {
                digitalWrite(ledPin2, LOW);
                ldrStatus = "OFF"; 
            }
    }

    char statusMessage2[5];
    snprintf(statusMessage2, 5, "%s", ldrStatus.c_str());
    client.publish(ldrStatusTopic, statusMessage2);
}

//LDR LAMP STATUS
void ldrStatus(bool status) {
    String statusString;
    if (status) {
        digitalWrite(ledPin2, HIGH);
        statusString = "ON";
    } else {
        digitalWrite(ledPin2, LOW);
        statusString = "OFF";
    }
    Serial.printf("Lamp Status : %s\r\n", statusString.c_str());

    //PUBLISH LDR LAMP STATUS TO MQTT
    char statusMessage[5];
    snprintf(statusMessage, 5, "%s", statusString.c_str());
    client.publish(ldrStatusTopic, statusMessage);
}
/*----------------END LDR LAMP FUNCTION-------------------*/

/*-----------ULTRASONIC HC-SR04 (WATER PUMP) FUNCTION----------------*/
const int Trig_pin = 14;
const int Echo_pin = 27;
const int buzzer = 26;
float durasi;
float jarak;
int batas_kosong=80;
int batas_penuh=10;
int freq = 2000;
int channel = 1;
int resolution = 8;
float waterTubHeight = 100;
String pumpStatus;

void waterStatus(float status) {
    if (status > batas_kosong) {
        pumpStatus = "ON";
    } else if(status <= batas_penuh){
        pumpStatus = "OFF";
    }

    char statusMessage[5];
    snprintf(statusMessage, 5, "%s", pumpStatus.c_str());
    client.publish(waterStatusTopic, statusMessage);
}

void ukur_jarak() //distance calculaion...
{
    digitalWrite(Trig_pin, LOW);
    delay(10);
    digitalWrite(Trig_pin, HIGH);
    delay(10);
    digitalWrite(Trig_pin, LOW);
    
    durasi = pulseIn(Echo_pin, HIGH);
    jarak = durasi * 0.034 / 2;
    waterStatus(jarak);
    float waterHeight = waterTubHeight - jarak;
    if(waterHeight < 0){
        waterHeight = 0;
    }
    //PUBLISH LDR LAMP TOPIC TO MQTT
    char statusMessage[5];
    snprintf(statusMessage, 5, "%f", waterHeight);
    client.publish(waterTopic, statusMessage);
}

void myTone(int pin)
{
  ledcAttachPin(pin, 1);             // pin, channel
  ledcWriteNote(1, NOTE_F, 4);    // channel, frequency, octave
}

void myNoTone(int pin)
{
  ledcDetachPin(pin);
}

void output_jarak(){
  if(jarak > batas_kosong) {
    myTone(buzzer);//digitalWrite(buzzer, HIGH);//Buzzer beeping......
    delay(2000);
    // Serial.print("pump on\n");
  }
  else if(jarak <= batas_penuh) {
    myNoTone(buzzer);//digitalWrite(buzzer, LOW);
    // Serial.print("pump off\n");
    delay(100);
    digitalWrite (buzzer , LOW);
  }
}
/*--------END WATER PUMP FUNCTION-----------*/

/*---------CONFIG MQTT------------*/
//IOT STATUS (ON OR OFF)
void showStatus(bool status) {
    String statusString;
    if (status) {
        statusString = "ON";
    } else {
        statusString = "OFF";
    }
    Serial.printf("IOT Status : %s\r\n", statusString.c_str());

    char statusMessage[5];
    snprintf(statusMessage, 5, "%s", statusString.c_str());
    client.publish(statusTopic, statusMessage);
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
    Serial.printf("Message received: %s\r\n", topic);
    char statusInput = (char)payload[0];
    Serial.printf("Payload: %c\r\n", statusInput);

    //CALLBACK WHEN PAYLOAD FROM CONTROL TOPIC
    if (strcmp(topic, controlTopic) == 0) {
        if (statusInput == '0') {
            activeStatus = false;
        }
        if (statusInput == '1') {
            activeStatus = true;
        }
        showStatus(activeStatus);
    }
    //CALLBACK WHEN PAYLOAD FROM LDR BUTTON TOPIC
    if (strcmp(topic, ldrButtonTopic) == 0) {
        if (statusInput == '0') {
            ldrActiveStatus = false;
        }
        if (statusInput == '1') {
            ldrActiveStatus = true;
        }
        ldrStatus(ldrActiveStatus);
    }
    //CALLBACK WHEN PAYLOAD FROM DOOR LOCKING TOPIC
    if (strcmp(topic, doorLockingTopic) == 0) {
        if (statusInput == '0') {
            doorLockingStatus = false;
        }
        if (statusInput == '1') {
            doorLockingStatus = true;
        }
        doorStatus(doorLockingStatus);
    }
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
            client.subscribe(ldrButtonTopic);
            client.subscribe(doorLockingTopic);
        } else {
            Serial.print("failed, state: ");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}
/*-------------END CONFIG MQTT----------------*/

void setup() {
    delay(500);
    Serial.begin(115200);

    //SETUP DHT22
    pinMode(ledPin1, OUTPUT);
    dhtSensor.begin();

    //SETUP DOOR LOCKING
    pinMode(relay, OUTPUT);
    servo.attach(servoPin);

    //SETUP LDR LAMP
    pinMode(ledPin2, OUTPUT);
    pinMode(ldrPin, INPUT); 

    //Setup Water Pump
    pinMode(Trig_pin, OUTPUT);
    pinMode(Echo_pin,INPUT);
    pinMode(buzzer, OUTPUT);
    ledcSetup(channel, freq, resolution);
    ledcAttachPin(buzzer, channel);

    //SETUP MQTT 
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
    //MQTT CONNECTING
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
        doorLock(doorLockingStatus);
    }

    if (activeStatus) {
        ukur_jarak();
        output_jarak();
    } else {
        digitalWrite(buzzer,LOW);// pump off...
        myNoTone(buzzer);//digitalWrite(buzzer, LOW);
    }
}

