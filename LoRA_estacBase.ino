/**
 * @file LoRaP2P_RX.ino
 * @author rakwireless.com
 * @brief Receiver node for LoRa point to point communication
 * @version 0.1
 * @date 2021-08-21
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <SX126x-Arduino.h> //http://librarymanager/All#SX126x
#include <SPI.h>
// MQTT client  https://github.com/knolleary/pubsubclient/archive/master.zip
#include <PubSubClient.h> // http://librarymanager/All#PubSubClient

// Function declarations
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnRxTimeout(void);
void OnRxError(void);

// Create passinfo.h in your local repository if it does not exists (I'm not adding it for obvious reasons)
//  passinfo.h contains the following two lines:
// #define WIFI_SSID  "my_ssid"
// #define WIFI_PASS  "******"
#include "passinfo.h"

// Change the credentials below, so your ESP8266 connects to your router
const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

// MQTT broker IP address (Raspberry Pi IP address)
#define MQTT_SERVER  "192.168.1.11"
#define ZONE "RAK11200"

const char* mqtt_server = MQTT_SERVER;

// Initializes the espClient. You should change the espClient name if you have multiple ESPs running in your home automation system
const char* mqttClientId = "RAK11200Client_";
WiFiClient espClient;
PubSubClient mqttClient(espClient);
// if need username and password to connect mqtt server, they cannot be NULL.
const char* mqttUsername = NULL;
const char* mqttPassword = NULL;

// Define LoRa parameters
#define RF_FREQUENCY 868300000	// Hz
#define TX_OUTPUT_POWER 22		// dBm
#define LORA_BANDWIDTH 0		// [0: 125 kHz, 1: 250 kHz, 2: 500 kHz, 3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1		// [1: 4/5, 2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8	// Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0	// Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 3000
#define TX_TIMEOUT_VALUE 3000

static RadioEvents_t RadioEvents;

static uint8_t RcvBuffer[200];
int val = 0;
JsonDocument jsonMessage;
JsonDocument jsonGPSinfo;
JsonDocument jsonAccInfo;
// static uint8_t jsonSerial[200];
static char jsonSerial[200];
int count = 0;


void setup()
{
  pinMode(WB_IO2, OUTPUT);
  digitalWrite(WB_IO2, HIGH);
	// Initialize Serial for debug output
	time_t timeout = millis();
	Serial.begin(115200);
	while (!Serial)
	{
		if ((millis() - timeout) < 5000)
		{
            delay(100);
        }
        else
        {
            break;
        }
	}

  setup_wifi();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);

	Serial.println("=====================================");
	Serial.println("LoRaP2P Rx Test");
	Serial.println("=====================================");

	// Initialize the Radio callbacks
	RadioEvents.TxDone = NULL;
	RadioEvents.RxDone = OnRxDone;
	RadioEvents.TxTimeout = NULL;
	RadioEvents.RxTimeout = OnRxTimeout;
	RadioEvents.RxError = OnRxError;
	RadioEvents.CadDone = NULL;
	// Initialize LoRa chip.
	lora_rak13300_init();
	// Initialize the Radio
	Radio.Init(&RadioEvents);

	// Set Radio channel
	Radio.SetChannel(RF_FREQUENCY);

	// Set Radio RX configuration
	Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
					  LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
					  LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
					  0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

	// Start LoRa
	Serial.println("Starting Radio.Rx");
	Radio.Rx(0);
}

void loop()
{
 // Put your application tasks here, like reading of sensors,
  // Controlling actuators and/or other functions. 
  // delay(10000);
  // val = digitalRead(WB_IO2);
  // Serial.print("WB_IO2=");
  // Serial.println(val);

  // if (!mqttClient.connected()) {
  //   reconnect();
  // }
  // if(!mqttClient.loop()) {
  //   const char* nombrecliente = (nombreini + WiFi.localIP().toString()).c_str();
  //   mqttClient.connect(nombrecliente);
  // }
  if (!mqttClient.connected())
  {
    reconnect();
  }
  mqttClient.loop();
}

/**@brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
	Serial.println("OnRxDone");
	delay(10);
	memcpy(RcvBuffer, payload, size);
  DeserializationError error = deserializeMsgPack(jsonMessage, RcvBuffer);
  // deserializeJson(jsonMessage, RcvBuffer);
  // Test if parsing succeeded.
  if (error) {
    Serial.print("deserializeMsgPack() failed: ");
    Serial.println(error.f_str());
    return;
  }
	Serial.printf("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
	// Serial.printf("Payload: %s\n", RcvBuffer);
  serializeJson(jsonMessage, Serial);
	// for (int idx = 0; idx < size; idx++)
	// {
	// 	Serial.printf("%02X ", RcvBuffer[idx]);
	// }
	// Serial.println("");
  // Publishes to MQTT
  // {"la":"37.41","lo":"-6.07","al":"218","sp":"0","hd":"297.08","si":"6","ct":"0034","t":"16.35","h":"46.52","xg":"0.112","yg":"-0.016","zg":"-0.993"}
  String topic = "lora/" + jsonMessage["cli"].as<String>() + "/datetime";
  mqttClient.publish(topic.c_str(), jsonMessage["time"]);
  // mqttClient.publish("lora/" ZONE "/temperature", jsonMessage["temp"]);
  topic = "lora/" + jsonMessage["cli"].as<String>() + "/temperature";
  mqttClient.publish(topic.c_str() , jsonMessage["temp"]);
  topic = "lora/" + jsonMessage["cli"].as<String>() + "/humidity";
  mqttClient.publish(topic.c_str(), jsonMessage["RH"]);
  // mqttClient.publish("lora/" ZONE "/humidity", jsonMessage["RH"]);

  // mqttClient.publish("lora/" ZONE "/latitude", jsonMessage["la"]);
  // mqttClient.publish("lora/" ZONE "/longitude", jsonMessage["lo"]);
  // mqttClient.publish("lora/" ZONE "/altitude", jsonMessage["al"]);
  // mqttClient.publish("lora/" ZONE "/speed", jsonMessage["sp"]);
  // mqttClient.publish("lora/" ZONE "/heading", jsonMessage["hd"]);
  // mqttClient.publish("lora/" ZONE "/satinview", jsonMessage["si"]);
  jsonGPSinfo["time"] = jsonMessage["time"];
  jsonGPSinfo["lat"] = jsonMessage["lat"];
  jsonGPSinfo["lon"] = jsonMessage["lon"];
  jsonGPSinfo["alt"] = jsonMessage["alt"];
  jsonGPSinfo["speed"] = jsonMessage["speed"];
  jsonGPSinfo["heading"] = jsonMessage["hdg"];
  // char buff[20] ;
  // strcpy(buff, "satinview: ");
  // strcat(buff, jsonMessage["SIV"]);
  // jsonGPSinfo["tooltip"] = buff;
  String tooltip = "satinview: " + jsonMessage["SIV"].as<String>();
  jsonGPSinfo["tooltip"] = tooltip.c_str();

  count = serializeJson(jsonGPSinfo, jsonSerial);
  // mqttClient.publish("lora/" ZONE "/gpsinfo", jsonSerial);
  topic = "lora/" + jsonMessage["cli"].as<String>() + "/gpsinfo";
  mqttClient.publish(topic.c_str(), jsonSerial);
  // mqttClient.publish("lora/" ZONE "/x_acceleration", jsonMessage["xg"]);
  // mqttClient.publish("lora/" ZONE "/y_acceleration", jsonMessage["yg"]);
  // mqttClient.publish("lora/" ZONE "/z_acceleration", jsonMessage["zg"]);
  jsonAccInfo["xg"] = jsonMessage["xg"];
  jsonAccInfo["yg"] = jsonMessage["yg"];
  jsonAccInfo["zg"] = jsonMessage["zg"];
  count = serializeJson(jsonAccInfo, jsonSerial);
  // mqttClient.publish("lora/" ZONE "/acceleration", jsonSerial);
  topic = "lora/" + jsonMessage["cli"].as<String>() + "/acceleration";
  mqttClient.publish(topic.c_str(), jsonSerial);
  // mqttClient.publish("lora/" ZONE "/msgcount", jsonMessage["ct"]);

	Radio.Rx(0);
}

/**@brief Function to be executed on Radio Rx Timeout event
 */
void OnRxTimeout(void)
{
	Serial.println("OnRxTimeout");
	Radio.Rx(0);
  // val = digitalRead(WB_IO2);
  // Serial.print("WB_IO2=");
  // Serial.println(val);
}

/**@brief Function to be executed on Radio Rx Error event
 */
void OnRxError(void)
{
	Serial.println("OnRxError");
	Radio.Rx(0);
}

// WIFI
void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("WiFi connected - ESP IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until reconnected
  while (!mqttClient.connected())
  {
    Serial.print("Connecting to  MQTT broker...");
    String strcli = mqttClientId + WiFi.localIP().toString();
    const char* nombrecliente = strcli.c_str();
    if (mqttClient.connect(nombrecliente, mqttUsername, mqttPassword))
    // if (mqttClient.connect(mqttClientId, mqttUsername, mqttPassword))
    {
      Serial.println("connected");

      // Subscribe topics which you want to receive.
      // mqttClient.subscribe("RAK11200/led");
      mqttClient.subscribe("lora/" ZONE "/lamp");
      // you can add other Subscribe here
    }
    else
    {
      Serial.print("failed, code=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

// This functions is executed when some device publishes a message to a topic that your ESP8266 is subscribed to
// Change the function below to add logic to your program, so when a device publishes a message to a topic that 
// your ESP8266 is subscribed you can actually do something
void callback(char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT

  // If a message is received on the topic garden/zona1/lamp, you check if the message is either on or off. Turns the lamp GPIO according to the message
  if(topic=="lora/" ZONE "/lamp"){
      Serial.print("Changing zone lamp to ");
      if(messageTemp == "on"){
        // digitalWrite(lamp, HIGH);
        Serial.print("On");
      }
      else if(messageTemp == "off"){
        // digitalWrite(lamp, LOW);
        Serial.print("Off");
      }
  }
  Serial.println();
}
