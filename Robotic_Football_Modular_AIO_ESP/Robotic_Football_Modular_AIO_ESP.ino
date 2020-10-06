#include "src/esp32-ps3-develop/src/Ps3Controller.h"
#include "src/ESP32Servo/src/ESP32Servo.h"

#define REMOTE
//#define CONTROLLER

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

#define TACKLE_INPUT    35           // Tackle sensor is wired to pin 6
bool hasIndicated = false;
bool stayTackled = false;
int handicap = 3;
bool kidsMode = false;
int newconnect = 0;
int leftX, leftY, rightX, rightY;
ps3_cmd_t cmd;
float exeTime;


#ifdef REMOTE
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <ESPAsyncWebServer.h>
  #include <SPIFFS.h>
  #include <Update.h>
  #include "src/pubsubclient/src/PubSubClient.h"

  const char* ssid = "RoboticFootballRasPi";
  const char* password = "FootballRobots";

  // Add your MQTT Broker IP address, example:
  //const char* mqtt_server = "192.168.1.144";
  const char* mqtt_server = "192.168.4.1";

  WiFiClient espClient;
  PubSubClient client(espClient);
  long lastMsg = 0;
  char msg[50];
  int value = 0;

  AsyncWebServer server(80);
#endif

// // handle the upload of the firmware
// void handleUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final)
// {
//     // handle upload and update
//     if (!index)
//     {
//         Serial.printf("Update: %s\n", filename.c_str());
//         if (!Update.begin(UPDATE_SIZE_UNKNOWN))
//         { //start with max available size
//             Update.printError(Serial);
//         }
//     }

//     /* flashing firmware to ESP*/
//     if (len)
//     {
//         Update.write(data, len);
//     }

//     if (final)
//     {
//         if (Update.end(true))
//         { //true to set the size to the current progress
//             Serial.printf("Update Success: %ub written\nRebooting...\n", index+len);
//         }
//         else
//         {
//             Update.printError(Serial);
//         }
//     }
//     // alternative approach
//     // https://github.com/me-no-dev/ESPAsyncWebServer/issues/542#issuecomment-508489206
// }

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

  // If a message is received on the topic esp32/output, you check if the message is either "on" or "off". 
  // Changes the output state according to the message
  if (String(topic) == "esp32/output") {
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
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ES32Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/input");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
  }
}

void setup() {// This is stuff for connecting the PS3 controller.
  Serial.begin(115200);       //Begin Serial Communications
  ledsSetup();          //Setup the leds
  flashLeds();

  #ifdef REMOTE
    
    if(!SPIFFS.begin(true)){
      Serial.println("An Error has occurred while mounting SPIFFS");
      return;
    }
    
    // Connect to WiFi network
    WiFi.begin(ssid, password);
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/loginindex.html", "text/html");
    });
    
    // Route to load jquery file
    server.on("/jquery.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/jquery/jquery.min.js", "text/javascript");
    });

    server.on("/serverIndex", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(SPIFFS, "/serverindex.html", "text/html");
    });

    // /*handling uploading firmware file */
    // server.on("/update", HTTP_POST, []() {
    //   server.sendHeader("Connection", "close");
    //   server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    //   ESP.restart();
    // }, []() {
    //   HTTPUpload& upload = server.upload();
    //   if (upload.status == UPLOAD_FILE_START) {
    //     Serial.printf("Update: %s\n", upload.filename.c_str());
    //     if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
    //       Update.printError(Serial);
    //     }
    //   } else if (upload.status == UPLOAD_FILE_WRITE) {
    //     /* flashing firmware to ESP*/
    //     if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
    //       Update.printError(Serial);
    //     }
    //   } else if (upload.status == UPLOAD_FILE_END) {
    //     if (Update.end(true)) { //true to set the size to the current progress
    //       Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
    //     } else {
    //       Update.printError(Serial);
    //     }
    //   }
    // });

    // Start server
    server.begin();

    client.setServer(mqtt_server, 1883);
    //client.setCallback(callback);
  #endif

  
  #ifdef CONTROLLER
    // Wait for contoller to be connected to the esp
    Ps3.begin("00:1b:dc:0f:f3:59");
    while(!Ps3.isConnected()){
      Serial.println("Controller not connected");
      blue();
    }
    Serial.println("Controller is connected!");
  #endif

  // Vibrates controller when you first connect
  if (newconnect == 0) {
    cmd.rumble_left_intensity = 0xff;
    cmd.rumble_right_intensity = 0xff;
    cmd.rumble_right_duration = 750;
    cmd.rumble_left_duration = 750;
    newconnect++;
    ps3Cmd(cmd);
  }

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

  // Variable(boolean) for connection to ps3, also activates rumble
  int newconnect = 0;         
}


void loop() {
   if (!client.connected()) {
    reconnect();
  }
  //client.loop();
  
  long now = millis();
  if (now  - lastMsg > 5000) {
    lastMsg = now;
    char timeString[8];
    gcvt (millis(), 6, timeString);
    Serial.print("Time: ");
    Serial.println(timeString);
    client.publish("esp32/output", timeString);
  }
  // Run if the controller is connected
  if (Ps3.isConnected()) {
    #ifdef SHOW_EXECUTION_TIME
      exeTime = micros();
    #endif

    // !QUESTION! - What is the point of this code?
    // // Press the PS button to disconnect the controller
    // if (Ps3.data.button.ps && newconnect == 1) {
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
      Serial.print("\t");
    #endif

    //====================Specify the handicap=================================
    //Toggle in and out of kidsmode
    if (Ps3.data.button.start) {
      if (kidsMode == true) {
        kidsMode = false;
        handicap = 3;
        //PS3.setLedRaw(1);               // ON OFF OFF ON
        //PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
      } else if (kidsMode == false) {
        kidsMode = true;
        handicap = 7;
        //PS3.setLedRaw(9);               // OFF OFF OFF ON
        //PS3.setRumbleOn(5, 255, 5, 255);// vibrate both, then left, then right
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
        /*
          Ps3.data.cmd.rumble_right_duration(30);
          Ps3.data.cmd.rumble_right_intensity(255);
          Ps3.data.cmd.rumble_left_duration(30);
          Ps3.data.cmd.rumble_left_intensity(255);
        */
        //PS3.setRumbleOn(30, 255, 30, 255);
      }
      if (!digitalRead(TACKLE_INPUT))
      {
        red();
        if (!hasIndicated) {
          //PS3.setRumbleOn(10, 255, 10, 255);
          hasIndicated = true;
        }
      }
      else
      {
        if (stayTackled == false) {
          hasIndicated = false;
          green();
        }
      }
    #endif
    //===============================================================================================

    // Drives the robot according to joystick input
    driveCtrl(handicap, leftX, leftY, rightX, rightY);

    #ifdef PERIPHERAL
        peripheral(Ps3);//Call the peripheral
    #endif

    #ifdef SHOW_EXECUTION_TIME
      Serial.print("Exe exeTime: ");
      Serial.print(micros() - exeTime);
      Serial.print("\t");
    #endif

    Serial.println();
  }

  // If the controller is not connected, LEDs blue and stop robot
  if (!Ps3.isConnected()) {
    blue();
    driveStop();
  }
}
