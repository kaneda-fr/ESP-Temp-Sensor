/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp8266-nodemcu-mqtt-publish-ds18b20-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"


#define LED BUILTIN_LED //Led in NodeMCU at pin GPIO16 (D0) 
#define LED_RX D4
#define LED_INTERVAL_OK 5000
#define LED_INTERVAL_ERROR 500

// Raspberri Pi Mosquitto MQTT Broker
#define MQTT_HOST IPAddress(192, 168, 100, 60)
// For a cloud MQTT broker, type the domain name
//#define MQTT_HOST "example.com"
#define MQTT_PORT 1883

// Temperature MQTT Topics
#define MQTT_PUB_TEMP "esp/temperature"


// GPIO where the Water flow meter is connected to
#define FLOW_SENSOR  D2
// GPIO where the DS18B20 is connected to
#define TEMP_SENSOR  D1

const int oneWireBus = TEMP_SENSOR;          
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);
// Temperature value
float temp;

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

// Led blink variables
unsigned long  previousLEDMillis = 0;   // Stores last time LED was Blinked
boolean ledState = true;

// Flow meter variables
float calibrationFactor = 4; // Pulse per litre
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned long flowMilliLitres;
unsigned int totalMilliLitres;
float flowLitres;
float totalLitres;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void onWifiConnect(const WiFiEventStationModeGotIP& event) {
  Serial.println("Connected to Wi-Fi.");
  connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected& event) {
  Serial.println("Disconnected from Wi-Fi.");
  mqttReconnectTimer.detach(); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  wifiReconnectTimer.once(2, connectToWifi);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

/*void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}*/

void onMqttPublish(uint16_t packetId) {
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

// Intrupt for counting water flow meter pulse
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
  digitalWrite(LED_RX, LOW); 
  //Serial.println(pulseCount);
}

void setup() {

  pinMode(LED, OUTPUT); //LED pin as output
  pinMode(LED_RX, OUTPUT); //LED pin as output
  digitalWrite(LED, LOW); //turn the led on
  digitalWrite(LED_RX, LOW); 

  sensors.begin();
  Serial.begin(115200);
  Serial.println();
  
  wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
  wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  // If your broker requires authentication (username and password), set them below
  //mqttClient.setCredentials("REPlACE_WITH_YOUR_USER", "REPLACE_WITH_YOUR_PASSWORD");
  
  connectToWifi();

  int deviceCount = 0;
  deviceCount = sensors.getDeviceCount();
  Serial.print(deviceCount, DEC);
  Serial.println(" devices.");
  Serial.println("");
  
  if (deviceCount > 0)
    digitalWrite(LED, HIGH); //turn the led off

  // Setup flow meter
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;

  // enable PULLUP on flow sensor interupt pin
  pinMode(FLOW_SENSOR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), pulseCounter, FALLING);
  digitalWrite(LED_RX, HIGH); //turn the led off
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long calc_interval = 0;
  int count=0;
  // good for about 10 sensors
  StaticJsonDocument<192> payload;
  char output[512];

  // Turn  LED Off
  digitalWrite(LED_RX, HIGH); 

  // Every X number of seconds (interval = 10 seconds) 
  // it publishes a new MQTT message
  if (currentMillis - previousMillis >= interval) {
    payload.clear();

    calc_interval = currentMillis - previousMillis;

    // Flow calculation
    payload["flow"]["pulse"] = pulseCount;
    pulse1Sec = pulseCount;
    pulseCount = 0;

    flowRate = ((1000.0 / calc_interval) * pulse1Sec) / calibrationFactor;


    // Divide the flow rate in litres/minute by 60 to determine how many litres have
    // passed through the sensor in this 1 second interval, then multiply by 1000 to
    // convert to millilitres.
    flowMilliLitres = (flowRate / 60) * calc_interval;
    flowLitres = (flowRate / 60) * calc_interval / 1000;
 
    // Add the millilitres passed in this second to the cumulative total
    totalMilliLitres += flowMilliLitres;
    totalLitres += flowLitres;

    payload["flow"]["rate"] = flowRate;
    payload["flow"]["volume"] = flowLitres;
    
    // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(float(flowRate));  // Print the integer part of the variable
    Serial.print("L/min");
    Serial.print("\t");  

    // Save the last time a new reading was published
    previousMillis = currentMillis;

    // New temperature readings
    sensors.requestTemperatures(); 


    //payload["ds18b20"]=[];
    for (int deviceIndex =0; deviceIndex <sensors.getDeviceCount(); deviceIndex++ ){
        DeviceAddress deviceAddress;
        String dev_addr="";
        String pub="";

      Serial.print("Device Adress: ");
      // Temperature in Celsius degrees
      sensors.getAddress(deviceAddress, deviceIndex);

      for (uint8_t i = 0; i < 8; i++)
      {
        if (deviceAddress[i] < 0x10) {
          Serial.print("0");
          dev_addr += String("0");
        }
        //Serial.print(deviceAddress[i], HEX);
        dev_addr += String(deviceAddress[i], HEX);
      }
      Serial.print(dev_addr);
      Serial.println();
      temp = sensors.getTempCByIndex(deviceIndex);
      // Temperature in Fahrenheit degrees
      //temp = sensors.getTempFByIndex(0);
    
      // Publish an MQTT message on topic esp/temperature/sensorID
      //pub.concat(MQTT_PUB_TEMP);
      payload["ds18b20"][dev_addr] = temp;
    }
  
    serializeJson(payload, output, 512);
    uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, output);                            
    Serial.printf("Publishing on topic %s at QoS 1, packetId: %i ", MQTT_PUB_TEMP, packetIdPub1);
    Serial.printf("Message: %s \n", output);


    // Blink LED
    if (currentMillis - previousLEDMillis >= (sensors.getDeviceCount() == 0 ? LED_INTERVAL_ERROR : LED_INTERVAL_OK)) {
      previousLEDMillis = currentMillis;
      digitalWrite(LED_BUILTIN, ledState);
      ledState = !ledState;
    }
  }
}