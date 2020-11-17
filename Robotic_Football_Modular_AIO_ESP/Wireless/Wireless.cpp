#include "../src/async-mqtt-client/src/AsyncMqttClient.h"
#include "../src/ArduinoJson/ArduinoJson-v6.16.1.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include "../Leds/Leds.cpp"

WiFiClient wifiClient;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

char robotName[4];
char espMacAddress[18];

const char* storedSsid;
const char* storedPassword;

bool updateStatus = false;
String filename = "";

IPAddress ip;
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);



/**
 * Updates the esp32 code over http by connecting to a remote webserver. It defaults to a web path of "http://192.168.4.1:8080/public/binaries/" + filename passed to reprogram command
 */
void checkForUpdate() {
  if (updateStatus) {
    // Disconnect from MQTT client and stop wifi and mqtt reconnect timers to be safe
    mqttClient.disconnect(true);
    xTimerStop(mqttReconnectTimer, 0);
    xTimerStop(wifiReconnectTimer, 0);

    // Flash LEDs and then turn them off to let the user know robot is reprogramming itself
    flashLeds();
    ledsOff();

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
 */
void connectToWifi() {
  //Serial.println("Connecting to Wi-Fi...");
  //WiFi.disconnect();
  /* See https://www.bakke.online/index.php/2017/05/22/reducing-wifi-power-consumption-on-esp8266-part-3/ for explanation and reason for static IP*/

  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(storedSsid, storedPassword);
}

/**
 * Callback function for a WiFiEvent
 * @param event The WiFi event
 */
void WiFiEvent(WiFiEvent_t event) {
    //Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        xTimerStop(wifiReconnectTimer, 0);
        //Serial.println("WiFi connected");
        //Serial.println("IP address: ");
        //Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        //Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		    xTimerStart(wifiReconnectTimer, 0);
        break;
    }
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
 * Callback function for when connected to the MQTT server
 * @param sessionPresent Wheather the session is present or not
 */
void onMqttConnect(bool sessionPresent) {
  xTimerStop(mqttReconnectTimer, 0);
  //Serial.println("Connected to MQTT.");
  //Serial.print("Session present: ");
  //Serial.println(sessionPresent);
  
  uint16_t packetIdSub = mqttClient.subscribe(robotName, 2);
  //Serial.print("Subscribing at QoS 2, packetId: ");
  //Serial.println(packetIdSub);
  
  // mqttClient.publish("esp32/output", 0, true, "test 1");
  // Serial.println("Publishing at QoS 0");
  // uint16_t packetIdPub1 = mqttClient.publish("esp32/output", 1, true, "test 2");
  // Serial.print("Publishing at QoS 1, packetId: ");
  // Serial.println(packetIdPub1);
  // uint16_t packetIdPub2 = mqttClient.publish("esp32/output", 2, true, "test 3");
  // Serial.print("Publishing at QoS 2, packetId: ");
  // Serial.println(packetIdPub2);
}


/**
 * Callback function for when the MQTT client is disconnected from the server
 * @param reason The reason why the mqtt client was disconnected
 */
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  //Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}


/**
 * Callback function for when a message is recieved from the MQTT server
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
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(500), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(3000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  //mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(mqttHost, mqttPort);

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
void sendRobotData(char* tackleStatus, char* contollerStatus) {
  if(WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    
    // JSON variables for sending data to webserver
    StaticJsonDocument<JSON_OBJECT_SIZE(8)> data;
    char buffer[256];

    // From initilization
    data["robotNumber"] = robotName;
    data["espMacAddress"] = espMacAddress;

    // Stuff from function input
    data["batteryLevel"] =  "22";
    data["tackleStatus"] = tackleStatus;
    data["contollerStatus"] = contollerStatus;

    // Serialize JSON into the buffer array
    size_t n = serializeJson(data, buffer);

    // Print json data to serial port
    //serializeJson(data, Serial);

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