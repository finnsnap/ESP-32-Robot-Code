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

void setIPAddress(int number) {
  ip = IPAddress(192, 168, 4, number);
  Serial.print("IPAddress of robot: ");
  ip.printTo(Serial);
  Serial.println();
}
    /**
 * Updates the esp32 code over http by connecting to a remote webserver
 */
    void checkForUpdate()
{
  if (updateStatus) {
    mqttClient.disconnect(true);
    xTimerStop(mqttReconnectTimer, 0);
    xTimerStop(wifiReconnectTimer, 0);

    flashLeds();
    ledsOff();

    Serial.println("MQTT disconnected");

    Serial.println("Reprogramming with: ");
    Serial.println("http://192.168.4.1:8080/public/binaries/" + filename);

    t_httpUpdate_return ret = httpUpdate.update(wifiClient, "http://192.168.4.1:8080/public/binaries/" + filename);
    
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
 * Connect to WiFi
 */
void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.disconnect();
  // WiFi.setAutoReconnect(true);
  /* See https://www.bakke.online/index.php/2017/05/22/reducing-wifi-power-consumption-on-esp8266-part-3/ for explanation */

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);
  WiFi.begin(storedSsid, storedPassword);
}

/**
 * Connect to the MQTT server
 */
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.disconnect();
  mqttClient.connect();
}

/**
 * Callback function for a WiFiEvent
 * @param event The WiFi event
 */
void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        xTimerStop(wifiReconnectTimer, 0);
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		    xTimerStart(wifiReconnectTimer, 0);
        break;
    }
}




// void WiFiEvent(WiFiEvent_t event)
// {
//     Serial.printf("[WiFi-event] event: %d\n", event);
//
//     switch (event) {
//         case SYSTEM_EVENT_WIFI_READY: 
//             Serial.println("WiFi interface ready");
//             break;
//         case SYSTEM_EVENT_SCAN_DONE:
//             Serial.println("Completed scan for access points");
//             break;
//         case SYSTEM_EVENT_STA_START:
//             Serial.println("WiFi client started");
//             break;
//         case SYSTEM_EVENT_STA_STOP:
//             Serial.println("WiFi clients stopped");
//             break;
//         case SYSTEM_EVENT_STA_CONNECTED:
//             Serial.println("Connected to access point");
//             break;
//         case SYSTEM_EVENT_STA_DISCONNECTED:
//             Serial.println("Disconnected from WiFi access point");
//             break;
//         case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
//             Serial.println("Authentication mode of access point has changed");
//             break;
//         case SYSTEM_EVENT_STA_GOT_IP:
//             Serial.print("Obtained IP address: ");
//             Serial.println(WiFi.localIP());
//             break;
//         case SYSTEM_EVENT_STA_LOST_IP:
//             Serial.println("Lost IP address and IP address is reset to 0");
//             break;
//         case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
//             Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
//             break;
//         case SYSTEM_EVENT_STA_WPS_ER_FAILED:
//             Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
//             break;
//         case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
//             Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
//             break;
//         case SYSTEM_EVENT_STA_WPS_ER_PIN:
//             Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
//             break;
//         case SYSTEM_EVENT_AP_START:
//             Serial.println("WiFi access point started");
//             break;
//         case SYSTEM_EVENT_AP_STOP:
//             Serial.println("WiFi access point  stopped");
//             break;
//         case SYSTEM_EVENT_AP_STACONNECTED:
//             Serial.println("Client connected");
//             break;
//         case SYSTEM_EVENT_AP_STADISCONNECTED:
//             Serial.println("Client disconnected");
//             break;
//         case SYSTEM_EVENT_AP_STAIPASSIGNED:
//             Serial.println("Assigned IP address to client");
//             break;
//         case SYSTEM_EVENT_AP_PROBEREQRECVED:
//             Serial.println("Received probe request");
//             break;
//         case SYSTEM_EVENT_GOT_IP6:
//             Serial.println("IPv6 is preferred");
//             break;
//         case SYSTEM_EVENT_ETH_START:
//             Serial.println("Ethernet started");
//             break;
//         case SYSTEM_EVENT_ETH_STOP:
//             Serial.println("Ethernet stopped");
//             break;
//         case SYSTEM_EVENT_ETH_CONNECTED:
//             Serial.println("Ethernet connected");
//             break;
//         case SYSTEM_EVENT_ETH_DISCONNECTED:
//             Serial.println("Ethernet disconnected");
//             break;
//         case SYSTEM_EVENT_ETH_GOT_IP:
//             Serial.println("Obtained IP address");
//             break;
//         default: break;
//     }}






/**
 * Callback function for when connected to the MQTT server
 * @param sessionPresent Wheather the session is present or not
 */
void onMqttConnect(bool sessionPresent) {
  xTimerStop(mqttReconnectTimer, 0);
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  uint16_t packetIdSub = mqttClient.subscribe(robotName, 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  
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
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

/**
 * Callback function for acknowledging a subscription to a topic
 */
void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

/**
 * Callback function for acknowledging an unsubscription from a topic
 */
void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
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
      Serial.println("Invalid message");
    }
  }
}

void wirelessSetup(const char* ssid, const char* password, const char* mqttHost, const uint16_t mqttPort, char* name) {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(500), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(500), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  //mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setServer(mqttHost, mqttPort);

  storedSsid = ssid;
  storedPassword = password;

  connectToWifi();
  
  Serial.print("\nMAC ADDRESS: ");
  Serial.println(WiFi.macAddress().c_str());
  strcpy(espMacAddress, WiFi.macAddress().c_str());
  strcpy(robotName, name);
}


void sendRobotData(char* tackleStatus, char* contollerStatus) {
  if(WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
    // JSON variables for sending data to webserver
    const int capacity = JSON_OBJECT_SIZE(8);
    StaticJsonDocument<capacity> data;
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
      Serial.print(" Error sending message\n");
    }
    else {
      //Serial.print(" Success sending message\n");
    }
  }
  else if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to send robot data. WiFi disconnected");
  }
  else if(!mqttClient.connected()) {
    //Serial.println("Failed to send robot data. MQTT disconnected");
  }
  else {
    Serial.println("Failed to send robot data.");

  }
  
}