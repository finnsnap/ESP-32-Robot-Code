#include "../src/async-mqtt-client/src/AsyncMqttClient.h"
#include "../src/ArduinoJson/ArduinoJson-v6.16.1.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include "../Leds/Leds.cpp"

WiFiClient wifiClient;

AsyncMqttClient mqttClient;
//TimerHandle_t mqttReconnectTimer;
//TimerHandle_t wifiReconnectTimer;

// Storing names of stuff
char robotName[4];
char espMacAddress[18];
const char* storedSsid;
const char* storedPassword;

bool updateStatus = false;
String filename = "";

// Storing wireless info
IPAddress ip;
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
bool connectingToWifi = false;


/**
 * Updates the esp32 code over http by connecting to a remote webserver. It defaults to a web path of %"http://192.168.4.1:8080/public/binaries/" + filename% passed to reprogram command
 */
void checkForUpdate() {
  if (updateStatus) {
    // Disconnect from MQTT client so there are no interruptions
    mqttClient.disconnect(true);

    // Flash LEDs twice and then turn them off to let the user know robot is reprogramming itself
    flashLeds();
    flashLeds();
    ledsOff();

    // Info for serial port if it connected
    Serial.println("MQTT disconnected");
    Serial.println("Reprogramming with: ");
    Serial.println("http://192.168.4.1:8080/public/binaries/" + filename);

    // Run the httpUpdate funciton to start the update process
    t_httpUpdate_return ret = httpUpdate.update(wifiClient, "http://192.168.4.1:8080/public/binaries/" + filename);
    
    // Get and handle any errors returned by the update process
    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        break;
    }
  }
}


/**
 * Sets the IP address of the robot using the given number as the last number in the IP 
 * Eg. 192.168.4.XXX
 * @param number The number to be set as the last part of the IP address
 */
void setIPAddress(int number) {
  ip = IPAddress(192, 168, 4, number);
  //Serial.print("IPAddress of robot: ");
  //ip.printTo(Serial);
  //Serial.println();
}


/**
 * Connect to WiFi. No autoreconnect, persistance, using static ip and stored ssid and password
 * See https://www.bakke.online/index.php/2017/05/22/reducing-wifi-power-consumption-on-esp8266-part-3/ for explanation and reason for static IP
 * @return int If the wifi connected (0) or if it failed (1)
 */
int connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  
  // Flag so WiFi event does not cause disconnect
  connectingToWifi = true;
  
  // Start the wifi connection process with the correct settings
  WiFi.disconnect();
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(storedSsid, storedPassword);

  // Wait 10 seconds to give the wifi a chance to connect
  for(int i = 0; i < 10; i++) {
    if(WiFi.status() == WL_CONNECTED) {
      connectingToWifi = false;
      return 0;
    }
    else {
      Serial.println("Connecting..........");
    }

    delay(1000);
  }

  // WiFi did not connect so disconnect
  WiFi.disconnect();
  connectingToWifi = false;
  return 1;
}


/**
 * Connect to the MQTT server
 */
void connectToMqtt() {
  //Serial.println("Connecting to MQTT...");
  mqttClient.disconnect(true);
  mqttClient.connect();
}


/**
 * Callback function for a WiFiEvent
 * @param event The WiFi event
 */
void WiFiEvent(WiFiEvent_t event) {
  //Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        // Flag to ensure that event does not fire during wifi setup process
        if(!connectingToWifi) {
          WiFi.disconnect();
          Serial.println("WiFi disconnected");
        }

        break;
    }
}


/**
 * Callback function for when connected to the MQTT server
 * @param sessionPresent Wheather the session is present or not
 */
void onMqttConnect(bool sessionPresent) {
  uint16_t packetIdSub = mqttClient.subscribe(robotName, 2);

}


/**
 * Callback function for when a message is recieved from the MQTT server. Used for recieving remote commands like reprogramming
 * @param topic The topic that the message was recieved on
 * @param payload The actual message itself
 * @param properties If the message has any properties
 * @param len The length of the message
 * @param index Index
 * @param total Total
 */
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  char messageCommand;
  String messageTemp;
  
  //Serial.print(" payload: ");
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  if (*topic == *robotName) {
    //Serial.print("Changing output to ");

    int index = 0;
    messageCommand = messageTemp[0];
    String messageInput = messageTemp.substring(messageTemp.indexOf('-') + 1);

    if(messageCommand == 'p'){
      filename = messageInput;
      updateStatus = true;
    }
    // else if(messageCommand == 'r'){
    //   Serial.print("Name read from EEPROM:");
    //   Serial.print(robotName);
    //   Serial.print("\n");
    // }
    // else if(messageCommand == 'w'){
    //   writeStoredName(messageInput);
    //   Serial.print("Name written to EEPROM: ");
    //   Serial.print(messageInput);
    //   Serial.print("\n");
    // }
    else {
      //Serial.println("Invalid message");
    }
  }
}


/**
 * Sets up all the required parts before starting any of the wireless functionality. Must be run before any other functions from this file are called as it sets variables that are used in the other functions. 
 * @param ssid The name of the WiFi network to connect to
 * @param passwod The password of the WiFi network
 * @param mqttHost The server that the mqtt host is on
 * @param mqttPort The port that the mqtt server is listening too
 * @param name The name of the robot
 */
void wirelessSetup(const char* ssid, const char* password, const char* mqttHost, const uint16_t mqttPort, char* name) {
  // Connects wifi events to the event handler
  WiFi.onEvent(WiFiEvent);

  // Sets the mqtt connection and message functions as well as the server
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(mqttHost, mqttPort);

  // Stores the ssid and password for connecting to the correct wifi network
  storedSsid = ssid;
  storedPassword = password;
  
  // Stores the esp MAC address and robot name to send to the webserver
  strcpy(espMacAddress, WiFi.macAddress().c_str());
  strcpy(robotName, name);
}


/**
 * Sends the robot data over MQTT to the webserver
 * @param tackleStatus The current status of the tackle sensor
 * @param contollerStatus The current status of the contoller
 */
void sendRobotData(String codeVersion, char* tackleStatus, char* contollerStatus, int batteryLevel) {
  if(WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    
    // JSON variables for sending data to webserver
    // Max size of packet is 256 chars including brackets and other parts
    StaticJsonDocument<JSON_OBJECT_SIZE(20)> data;
    char buffer[256];

    // From initilization
    data["robotNumber"] = robotName;
    data["espMacAddress"] = espMacAddress;

    // Stuff from function input
    data["batteryLevel"] =  batteryLevel;
    data["tackleStatus"] = tackleStatus;
    data["contollerStatus"] = contollerStatus;
    data["ipAddress"] = WiFi.localIP().toString();
    data["codeVersion"] = codeVersion;

    // Serialize JSON into the buffer array
    size_t n = serializeJson(data, buffer);

    // Print json data to serial port
    serializeJson(data, Serial);
    Serial.println();

    if (mqttClient.publish("esp32/output", 0, false, buffer, n) == 0) {
      //Serial.print(" Error sending message\n");
    }
    else {
      //Serial.print(" Success sending message\n");
    }
  }
  else if(WiFi.status() != WL_CONNECTED) {
    //Serial.println("Failed to send robot data. WiFi disconnected");
  }
  else if(!mqttClient.connected()) {
    //Serial.println("Failed to send robot data. MQTT disconnected");
  }
  else {
    //Serial.println("Failed to send robot data.");

  }
  
}