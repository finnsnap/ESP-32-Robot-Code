#include "src/esp32-ps3-develop/src/Ps3Controller.h"
#include "src/ESP32Servo/src/ESP32Servo.h"

#include <WiFi.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

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

#define TACKLE_INPUT    13           // Tackle sensor is wired to pin 6
bool hasIndicated = false;
bool stayTackled = false;
int handicap = 3;
bool kidsMode = false;
int newconnect = 0;
int leftX, leftY, rightX, rightY;
ps3_cmd_t cmd;
float exeTime;


// Read battery is analog read pin 35


const char* ssid = "RoboticFootballRasPi";
const char* password = "FootballRobots";

// Add your MQTT Broker IP address, example:
//const char* mqtt_server = "192.168.1.144";
const char* mqtt_server = "192.168.4.1";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long lastMsg = 0;
unsigned long now = 0;
unsigned long lastWiFiAttempt = 0;
unsigned long lastMQTTAttempt = 0;

char msg[50];
int value = 0;

// Change name and contoller address for each robot
char name[] = "rK9";
char macaddress[] = "00:15:83:f3:e8:e8";

const int capacity = JSON_OBJECT_SIZE(6);
StaticJsonDocument<capacity> data;
char buffer[256];

int battery = 0;

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

  if (String(topic) == "esp32/input") {
    Serial.print("Changing output to ");
    if(messageTemp == "on"){
      Serial.println("on");
    }
    else if(messageTemp == "off"){
      Serial.println("off");
    }
  }
}

void reconnect() {
  // Attempt to connect
  if (mqttClient.connect(name)) {
    Serial.println("MQTT connected");
    // Subscribe
    char topic[7 + strlen(name)] = "esp32/";
    strcat(topic, name); /* add the extension */
    mqttClient.subscribe(topic);
  } 
  else {
    Serial.print("MQTT failed, rc=");
    Serial.print(mqttClient.state());
    Serial.print("\n");
  }
}

void update() {
  // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    // httpUpdate.setLedPin(LED_BUILTIN, LOW);

    t_httpUpdate_return ret = httpUpdate.update(wifiClient, "http://server/file.bin");
    // Or:
    //t_httpUpdate_return ret = httpUpdate.update(mqttClient, "server", 80, "/file.bin");

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

void setRumbleOn(uint8_t rightDuration, uint8_t rightPower, uint8_t leftDuration, uint8_t leftPower) {
    cmd.rumble_right_duration = rightDuration;    
    cmd.rumble_right_intensity = rightPower;
    cmd.rumble_left_intensity = leftPower;
    cmd.rumble_left_duration = leftDuration;

    ps3Cmd(cmd);
}

void onControllerConnect(){
    // Vibrates controller when you connect
    if (newconnect == 0) {
      setRumbleOn(50, 255, 50, 255);
      newconnect++;
    }

    Serial.println("Controller is connected!");
}

void setup() {// This is stuff for connecting the PS3 controller.
  Serial.begin(115200);       //Begin Serial Communications
  ledsSetup();          //Setup the leds
  flashLeds();
  Serial.println("ESP32 starting up");

  // Connect to WiFi network
  WiFi.begin(ssid, password);

  // Start mqtt server
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
  
  // Wait for contoller to be connected to the esp
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

  // Run if the controller is connected
  if (Ps3.isConnected()) {
    //Ps3.setPlayer(1);

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
        Ps3.setPlayer(5);     // ON OFF OFF ON          
        //setRumbleOn(5, 255, 5, 255);      // vibrate both, then left, then right
      } else if (kidsMode == false) {
        kidsMode = true;
        handicap = 7;
        Ps3.setPlayer(1);     // OFF OFF OFF ON
        //setRumbleOn(5, 255, 5, 255);      // vibrate both, then left, then right
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
    }
    if (Ps3.data.button.right){
      correctMotor(-1);
      Serial.println("right button clicked");
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
        setRumbleOn(30, 255, 30, 255);
        /*
          Ps3.data.cmd.rumble_right_duration(30);
          Ps3.data.cmd.rumble_right_intensity(255);
          Ps3.data.cmd.rumble_left_duration(30);
          Ps3.data.cmd.rumble_left_intensity(255);
        */
        //PS3.//setRumbleOn(30, 255, 30, 255);
      }
      if (!digitalRead(TACKLE_INPUT))
      {
        red();
        data["tackleStatus"] = "Tackled";
        if (!hasIndicated) {
          setRumbleOn(10, 255, 10, 255);
          //PS3.//setRumbleOn(10, 255, 10, 255);
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
    Serial.println("WiFi Disconnected");
  } 
  else if (!mqttClient.connected()) {
    Serial.println("Trying to connect to MQTT");
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
