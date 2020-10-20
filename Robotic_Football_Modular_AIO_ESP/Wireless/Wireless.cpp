#include "../src/async-mqtt-client/src/AsyncMqttClient.h"
#include "../src/ArduinoJson/ArduinoJson-v6.16.1.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

WiFiClient wifiClient;
//WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// JSON variables for sending data to webserver
const int capacity = JSON_OBJECT_SIZE(8);
StaticJsonDocument<capacity> data;
char buffer[256];

char robotName;

/**
 * Updates the esp32 code over http by connecting to a remote webserver
 */
void update() {
  mqttClient.disconnect();
  Serial.println("MQTT disconnected");

  t_httpUpdate_return ret = httpUpdate.update(wifiClient, "http://192.168.137.211:8080/public/esp32.bin");
  
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



/**
 * Connect to WiFi
 */
void connectToWifi(const char* ssid, const char* password) {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
}

/**
 * Connect to the MQTT server
 */
void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
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
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        data["espMacAddress"] = WiFi.macAddress();
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
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  
  uint16_t packetIdSub = mqttClient.subscribe("esp32/input", 2);
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
  Serial.println("Publish received.");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print(" payload: ");
  Serial.println(payload);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);

  char messageCommand;
  String messageTemp;
  String messageInput;

  for (int i = 0; i < len; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }

  if (String(topic) == "esp32/input") {
    //Serial.print("Changing output to ");

    //payload = messageTemp[0]; // Maybe change up to pointers to make more efficient?
    int index = 0;
    for (int i = 1; i < len; i++) if (messageTemp[i] == '-') index = i + 1;
    for (int i = index; i < len; i++) messageInput += messageTemp[i];
    
    if(messageTemp == "p"){
      Serial.println("Reprogramming");
      update();
    }
    else if(messageCommand == 'r'){
      Serial.print("Name read from EEPROM:");
      Serial.print(robotName);
      Serial.print("\n");
    }
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

/**
 * Callback function to ackknowledge the client sending a message
 * @param packetID The ID of the message that was sent
 */
void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}


void wirelessSetup(const char* ssid, const char* password, const char* mqttHost, const uint16_t mqttPort, char* name) {
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(mqttHost, mqttPort);

  connectToWifi(ssid, password);

  robotName = *name;
  data["robotNumber"] = name;
}

void setData(char* key, char* value) {
  data[key] = value;
  Serial.print("\n\nKey: ");
  Serial.print(key);
  Serial.print("\nValue: ");
  Serial.print(value);
  Serial.print("\ndata value: ");
  Serial.print((const char*) data[key]);
  Serial.println();
}

// void setData(char* key, const char* value) {
//   data[key] = value;
// }

// void setData(char* key, int value) {
//   data[key] = value;

// }

// void setData(char* key, byte* value) {
//   data[key] = value;
// }

void sendData() {

    //data["batteryLevel"] = "22";
    setData("batteryLevel", "22");

    size_t n = serializeJson(data, buffer);

    // Print json data to serial port
    serializeJson(data, Serial);

    if (mqttClient.publish("esp32/output", 2, false, buffer, n) == 0) {
      Serial.print(" Error sending message\n");
    }
    else {
      Serial.print(" Success sending message\n");
    }
}