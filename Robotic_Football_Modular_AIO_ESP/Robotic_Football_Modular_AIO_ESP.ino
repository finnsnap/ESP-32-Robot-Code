#include "src/esp32-ps3-develop/src/Ps3Controller.h"
#include "src/ESP32Servo/src/ESP32Servo.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

// #include <EEPROM.h>
#include <Preferences.h>

#include "Wireless/Wireless.cpp"

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
unsigned long exeTime;
unsigned long now;
unsigned long lastMsg;



#define WIRELESS



// Read battery is analog read pin 35

// Name and password of the WiFi network to connect to
//const char* ssid = "RoboticFootballRasPi";
//const char* password = "FootballRobots";
const char* ssid = "PHILIP-DESKTOP";
const char* password = "18o06(W6";

const char* mqttHost = "192.168.137.223";
const uint16_t mqttPort = 1883;

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


// Contoller battery level
int battery = 0;

#define EEPROM_SIZE 16

Preferences preferences;

void writeStoredName(String data) {
  preferences.begin("RobotName");
  preferences.putString("name", data);
  preferences.end();
}

// Add check for a valid name that matches an already given name
void readStoredName() {
  preferences.begin("RobotName", false);
  String data = preferences.getString("name");
  preferences.end();

  Serial.print("Read name: ");
  Serial.print(data);
  Serial.print("\n");

  if (data.length() == 3) {
    strcpy(name, data.c_str());
  }

  Serial.print("Read name: ");
  Serial.print(name);
  Serial.print("\n");

  for (int i = 0; i < 18; i++) { // Change 18 to make work with any length mac address array
    if (strcmp(name, robotNames[i]) == 0) {
      macaddress = macAddresses[i];
      Serial.println(robotNames[i]);
      Serial.println(macaddress);
      break;
    }
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



void setup() {// This is stuff for connecting the PS3 controller.
  Serial.begin(115200);       //Begin Serial Communications
  ledsSetup();          //Setup the leds
  flashLeds();
  Serial.println("ESP32 starting up");
  // String dadfa = "rK9";
  // writeStoredName(dadfa);
  readStoredName();

  #ifdef WIRELESS
    wirelessSetup(ssid, password, mqttHost, mqttPort, name);
  #endif

// Add check for valid mac address

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

  setData("tackleStatus", " ");
  //data["tackleStatus"] = " ";

  // Run if the controller is connected
  if (Ps3.isConnected()) {
    setData("contollerSStatus", "Connected");
    //data["contollerStatus"] = "Connected";
    
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
        setData("tackleStatus", "Tackled");
        //data["tackleStatus"] = "Tackled";
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
          setData("tackleStatus", "Not tackled");
          //data["tackleStatus"] = "Not tackled";
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
    setData("contollerStatus", "Disconnected");
    //data["contollerStatus"] = "Disconnected";
  } 

  #ifdef SHOW_EXECUTION_TIME
    Serial.print("Exe exeTime: ");
    Serial.println(micros() - exeTime);
  #endif

  // Stores the current time
  now = millis();


  if (now  - lastMsg > 200) {
    lastMsg = now;
    sendData();
  }
}
