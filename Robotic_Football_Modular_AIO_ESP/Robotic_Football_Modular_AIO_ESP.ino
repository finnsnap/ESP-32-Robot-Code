#include "src/esp32-ps3-develop/src/Ps3Controller.h"
#include "src/ESP32Servo/src/ESP32Servo.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

#include <EEPROM.h>

#include "src/pubsubclient/src/PubSubClient.h"
#include "src/ArduinoJson/ArduinoJson-v6.16.1.h"


//===========Uncomment a LED===========================
#include "Leds/Leds.cpp"
//#include "Leds/OldLeds.cpp"

//===========Uncomment a drive train===================
#include "DriveTrains/BasicDrive.cpp"
//#include "DriveTrains/CenterDrive.cpp"
//#include "DriveTrains/SquareOmniDrive.cpp"

//===========Uncomment for tackle sensor===============
#define TACKLE

//===========Uncomment to choose a Position============
//#define WR
//#define Center
//#define QB
//#define Kicker

//===========Uncomment if not using bag motors=========
//#define OldMotors

//===========Uncomment for debug modes=================
//#define SHOW_CONTROLLER_INPUT
//#define SHOW_EXECUTION_TIME

//===================================
//Includes the right peripheral file for specified position
#ifdef WR
    #define PERIPHERAL
    #include "Peripherals/WRPeripheral.cpp"
#endif

#ifdef Center
    #define PERIPHERAL
    #include "Peripherals/CenterPeripheral.cpp"
#endif

#ifdef QB
    #define PERIPHERAL
    #include "Peripherals/QBPeripheral.cpp"
#endif

#ifdef Kicker
    #define PERIPHERAL
    #include "Peripherals/KickerPeripheral.cpp"
#endif

//This just enables and disables the old motors
#ifdef OldMotors
  int motorType = -1;
#else
  int motorType = 1;
#endif
//===================================

// Tackle sensor is wired to pin 13
#define TACKLE_INPUT 13           
bool hasIndicated = false;
bool stayTackled = false;

// Default handicap is 3 with no kids mode
int handicap = 3;       
bool kidsMode = false;

// Define joystick variables and other useful variables
int leftX, leftY, rightX, rightY;
int newconnect = 0;
ps3_cmd_t cmd;
float exeTime;


// Read battery is analog read pin 35

// Name and password of the WiFi network to connect to
//const char* ssid = "RoboticFootballRasPi";
//const char* password = "FootballRobots";
const char* ssid = "PHILIP-DESKTOP";
const char* password = "18o06(W6";
WiFiClient wifiClient;
//WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;

// IP address of the MQTT broker to connect to
const char* mqtt_server = "192.168.137.223"; //"192.168.4.1";
PubSubClient mqttClient(wifiClient);

// Storing timing values
unsigned long lastMsg = 0;
unsigned long now = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;


// Change name and contoller address for each robot
char name[4];
char* macaddress;

const char* robotNames[18] = {"r3",  "r7",  "r9", 
                          "r12", "r16", "r35", 
                          "r40", "r42", "r44", 
                          "r53", "r68", "r74", 
                          "r75", "r81", "r82", 
                          "r85", "r88", "rK9"};

char* macAddresses[18] = {"00:15:83:f3:e8:d8", "00:15:83:3d:0a:57", "00:1b:dc:0f:aa:58", 
                             "00:1b:dc:0f:f3:59", "5c:f3:70:78:51:d6", "00:1b:dc:0f:d3:f1", 
                             "00:1b:dc:0f:f3:59", "00:15:83:f3:e0:09", "00:1b:dc:0f:dc:32", 
                             "00:1b:dc:0f:dc:3e", "00:1b:dc:0f:d3:e8", "5c:f3:70:78:51:d0", 
                             "01:02:03:04:05:06", "00:15:83:f3:e8:cd", "5c:f3:70:6e:5f:df", 
                             "00:1b:dc:0f:dc:2d", "00:1b:dc:0f:e8:af", "00:15:83:f3:e8:e8"};


// JSON variables for sending data to webserver
const int capacity = JSON_OBJECT_SIZE(6);
StaticJsonDocument<capacity> data;
char buffer[256];

// Contoller battery level
int battery = 0;

#define EEPROM_SIZE 16

void writeStoredName(String data) {
  int stringSize = data.length();
  if (stringSize <= 3){
    for(int i = 0; i < stringSize; i++) {
      EEPROM.write(i, data[i]);
      // name[i] = data[i];
    }
    EEPROM.write(stringSize,'\0');   //Add termination null character
    // name[stringSize] = '\0';

    EEPROM.commit();

    
  }
}

void readStoredName() {
  int len=0;
  unsigned char k;
  k = EEPROM.read(0);
  while(k != '\0' && len < EEPROM_SIZE)   //Read until null character
  {
    k = EEPROM.read(len);
    name[len] = k;
    len++;
  }
  name[len]='\0';
  
  for (int i = 0; i < 18; i++) { // Change 18 to make work with any length array
      
      if (strcmp(name, robotNames[i]) == 0) {
        macaddress = macAddresses[i];
        Serial.println(robotNames[i]);
        Serial.println(macaddress);
        break;
      }
      
    }
}

// void writeString(int address, char* data)
// {
//   int stringSize = strlen(data);
//   for(int i=0; i < stringSize; i++) {
//     EEPROM.write(address+i, data[i]);
//   }
//   EEPROM.write(address + stringSize,'\0');   //Add termination null character
//   EEPROM.commit();
// }

// String readString(int address) {
//   char data[EEPROM_SIZE];
//   int len=0;
//   unsigned char k;
//   k = EEPROM.read(address);
//   while(k != '\0' && len < EEPROM_SIZE)   //Read until null character
//   {
//     k = EEPROM.read(address + len);
//     data[len] = k;
//     len++;
//   }
//   data[len]='\0';
//   return data;
// }


void callback(const char* topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(" Message: ");
  char messageCommand;
  String messageTemp;
  String messageInput;
  Serial.print("\n");
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }

  if (String(topic) == "esp32/input") {
    //Serial.print("Changing output to ");

    messageCommand = messageTemp[0]; // Maybe change up to pointers to make more efficient?
    int index = 0;
    for (int i = 1; i < length; i++) if (messageTemp[i] == '-') index = i + 1;
    for (int i = index; i < length; i++) messageInput += messageTemp[i];
    
    if(messageTemp == name){
      Serial.println("Reprogramming");
      update();
    }
    else if(messageCommand == 'r'){
      readStoredName();
      Serial.print("Name read from EEPROM:");
      Serial.print(name);
      Serial.print("\n");
    }
    else if(messageCommand == 'w'){
      writeStoredName(messageInput);
      Serial.print("Name written to EEPROM: ");
      Serial.print(messageInput);
      Serial.print("\n");
    }
    else {
      Serial.println("Invalid message");
    }
  }
}

/**
 * Tries to connect to the MQTT server and subscribe to the esp32/(robotName) topic
 */
void reconnect() {
  // Attempt to connect
  if (mqttClient.connect(name)) {
    Serial.println("MQTT connected");
    // Subscribe
    mqttClient.subscribe("esp32/input");
  } 
  else {
    Serial.print("MQTT failed, rc=");
    Serial.print(mqttClient.state());
    Serial.print("\n");
  }
}

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
 * Rumbles the right and left of the contoller based on the given intensity and length values. 
 * 
 * @param rightDuration The duration of the right side rumble in ms
 * @param rightPower The strength of the right side rumble 
 * @param leftDuration The duration of the left side rumble in ms
 * @param leftPower The strength of the left side rumble
 */
void rumbleContoller(uint8_t rightDuration, uint8_t rightPower, uint8_t leftDuration, uint8_t leftPower) {
    cmd.rumble_right_duration = rightDuration;
    cmd.rumble_right_intensity = rightPower;
    cmd.rumble_left_intensity = leftPower;
    cmd.rumble_left_duration = leftDuration;

    ps3Cmd(cmd);
}

/**
 * Callback function for when the contoller connects. This will vibrate the contoller only on the first connection after the robot is turned on.
 */
void onControllerConnect(){
    // Vibrates controller when you connect
    if (newconnect == 0) {
      rumbleContoller(50, 255, 50, 255);
      newconnect++;
      ps3SetLed(1);
    }

    Serial.println("Controller is connected!");
}


// void WiFiEvent(WiFiEvent_t event)
// {
//     Serial.printf("[WiFi-event] event: %d\n", event);

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


void setup() {// This is stuff for connecting the PS3 controller.
  Serial.begin(115200);       //Begin Serial Communications
  ledsSetup();          //Setup the leds
  flashLeds();
  Serial.println("ESP32 starting up");

  EEPROM.begin(EEPROM_SIZE);
  readStoredName();

  // Setup WiFi event handlers and begin WiFi connection
  // WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);

  // Start mqtt server
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
  
  // Attached the contoller connect function to connection callback and start contoller connection
  Ps3.attachOnConnect(onControllerConnect);
  Ps3.begin(macaddress);
  // while(!Ps3.isConnected()){
  //   Serial.println("Controller not connected");
  //   blue();
  // }
  
  //Setup the drive train, peripherals, tackle sensor, and changes leds to green once complete
  driveSetup(motorType);

  #ifdef PERIPHERAL
    peripheralSetup();
  #endif

  #ifdef TACKLE
    pinMode(TACKLE_INPUT, INPUT);
  #endif

  #ifdef TACKLE
      green();
  #else
      blue();
  #endif   
}

void loop() {
  #ifdef SHOW_EXECUTION_TIME
    exeTime = micros();
  #endif

  data["tackleStatus"] = " ";

  if (WiFi.status() == WL_CONNECTED){
    data["espMacAddress"] = WiFi.macAddress();
  }

  Serial.println("Running contoller");
  // Run if the controller is connected
  if (Ps3.isConnected()) {
    data["contollerStatus"] = "Connected";
    
    // if( battery != Ps3.data.status.battery ){
    //     battery = Ps3.data.status.battery;
    //     Serial.print("The controller battery is ");
    //     if( battery == ps3_status_battery_charging )      Serial.println("CHARGING");
    //     else if( battery == ps3_status_battery_full )     Serial.println("FULL");
    //     else if( battery == ps3_status_battery_high )     Serial.println("HIGH");
    //     else if( battery == ps3_status_battery_low)       Serial.println("LOW");
    //     else if( battery == ps3_status_battery_dying )    Serial.println("DYING");
    //     else if( battery == ps3_status_battery_shutdown ) Serial.println("SHUTDOWN");
    //     else Serial.println("UNDEFINED");
    // }

    // // Press the PS button to disconnect the controller
    // if (Ps3.data.button.ps && newconnect == 1) {
    // PS3.disconnect();
    //   newconnect = 0;
    // }

    //====================Get Controller Input=================================
    // Reads and maps joystick values from -90 to 90
    leftX = map(Ps3.data.analog.stick.lx, -128, 127, -90, 90);
    leftY = map(Ps3.data.analog.stick.ly, -128, 127, -90, 90);
    rightX = map(Ps3.data.analog.stick.rx, -128, 127, -90, 90);
    rightY = map(Ps3.data.analog.stick.ry, -128, 127, -90, 90);
    
    // Deals with stickness from joysticks
    if (abs(leftX) < 8) leftX = 0;
    if (abs(leftY) < 8) leftY = 0;
    if (abs(rightX) < 8) rightX = 0;
    if (abs(rightY) < 8) rightY = 0;
    
    #ifdef SHOW_CONTROLLER_INPUT
      Serial.print(leftX);    
      Serial.print("\t");
      Serial.print(leftY);    
      Serial.print("\t");
      Serial.print(rightX);    
      Serial.print("\t");
      Serial.print(rightY);    
      Serial.print("\n");
    #endif

    //====================Specify the handicap=================================
    //Toggle in and out of kidsmode
    if (Ps3.data.button.start) {
      if (kidsMode == true) {
        kidsMode = false;
        handicap = 3;
        ps3SetLed(5);     // ON OFF OFF ON          
        rumbleContoller(5, 255, 5, 255);      // vibrate both, then left, then right
      } else if (kidsMode == false) {
        kidsMode = true;
        handicap = 7;
        ps3SetLed(1);     // OFF OFF OFF ON
        rumbleContoller(5, 255, 5, 255);      // vibrate both, then left, then right
      }
    }

    if (kidsMode == false) {
      // Press R2 to boost
      if (Ps3.data.button.r2) {
        handicap = 1;
      }
      // Press L2 to slow down
      else if (Ps3.data.button.l2) {
        handicap = 6;
      } 
      // Sets default handicap
      else {
        handicap = 3;
      }
    }

    //===============================Adjust motors=============================
    if (Ps3.data.button.left){
      correctMotor(1);
      Serial.println("Left button clicked");
      rumbleContoller(0, 0, 5, 255);
    }
    if (Ps3.data.button.right){
      correctMotor(-1);
      Serial.println("Right button clicked");
      rumbleContoller(5, 255, 0, 0);
    }

    //=================================Tackle Sensor===========================

    #ifdef TACKLE
      // NORMAL OPERATION MODE
      // for the if statement for whether or not
      // tackle is enabled. cool stuff
      if (Ps3.data.button.left) {
        if (stayTackled == true) {
          stayTackled = false;
        } 
        else {
          stayTackled = true;
        }
        rumbleContoller(30, 255, 30, 255);
      }
      if (!digitalRead(TACKLE_INPUT))
      {
        red();
        data["tackleStatus"] = "Tackled";
        if (!hasIndicated) {
          rumbleContoller(10, 255, 10, 255);
          hasIndicated = true;
        }
      }
      else
      {
        if (stayTackled == false) {
          hasIndicated = false;
          green();
          data["tackleStatus"] = "Not tackled";
        }
      }

    #endif
    //===============================================================================================

    // Drives the robot according to joystick input
    driveCtrl(handicap, leftX, leftY, rightX, rightY);

    #ifdef PERIPHERAL
        peripheral(Ps3);//Call the peripheral
    #endif
  }
  else { // If the controller is not connected, LEDs blue and stop robot
    blue();
    driveStop();
    data["contollerStatus"] = "Disconnected";
  } 

  #ifdef SHOW_EXECUTION_TIME
    Serial.print("Exe exeTime: ");
    Serial.println(micros() - exeTime);
  #endif

  // Stores the current time
  now = millis();

  mqttClient.loop();

  if (WiFi.status() != WL_CONNECTED) {
    lastWiFiAttempt = now;
    //Serial.println("WiFi Disconnected");
    
    // if wifi is down, try reconnecting every 30 seconds
    // if ((WiFi.status() != WL_CONNECTED) && (millis() > 30000)) {
    //   Serial.println("Reconnecting to WiFi...");
    //   WiFi.disconnect();
    //   WiFi.begin(SSID, PASS);
    //   check_wifi = millis() + 30000;
    // }
  } 
  else if (!mqttClient.connected()) {
    //Serial.println("Trying to connect to MQTT");
    lastMQTTAttempt = now;
    reconnect();
  } 
  else if (now  - lastMsg > 200) {
    lastMsg = now;

    data["robotNumber"] = name;
    data["batteryLevel"] = "22";

    size_t n = serializeJson(data, buffer);

    // Print json data to serial port
    serializeJson(data, Serial);

    if (mqttClient.publish("esp32/output", buffer, n)) {
      Serial.print(" Success sending message\n");
    }
    else {
      Serial.print(" Error sending message\n");
    }
  }

}
