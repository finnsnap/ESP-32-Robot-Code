Robotic-Football-All-In-One
=====


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

You need to add the ESP32 board in the Ardunio IDE before you can compile and upload this code. Below is a link to a tutorial on how to do this.

[Installing the ESP32 Board in Arduino IDE (Windows, Mac OS X, Linux)](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/)


### Installation

1. Clone the repo
```sh
git clone https://github.com/finnsnap/ESP-32-Robot-Code.git
```
2. Open the [Robotic_Football_Modular_AIO_ESP.ino](Robotic_Football_Modular_AIO_ESP/Robotic_Football_Modular_AIO_ESP.ino) file in the Arduino IDE

3. Comment out/uncomment the necessary include files as detailed below in the [Configuring a Robot](#configuring-a-robot) section.

3. Select the **ESP32 Dev Module** in the **Tools > Board** menu

4. Select the COM port that the ESP32 is connected to in the **Tools > Port** menu

4. Upload the code to the board

### Configuring a Robot

The code is structured to include only the necessary files to enable different configurations of a robot to be made. This allows different robots with differnt abilities to easily be programmed. The abilities of the robot to be programmed are chosen from a list of define statements at the top of the main Robotic_Football_Modular_AIO_ESP.ino file, like so:

#### This example is for a robot with LEDs, the basic drive train, and a tackle sensor.

````c++
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
````

#### This example would create a robot that uses an omniwheel drivetrain, is the quarterback, and has the led lights and tackle sensor enabled.

````c++
//===========Uncomment a LED===========================
#include "Leds/Leds.cpp"
//#include "Leds/OldLeds.cpp"

//===========Uncomment a drive train===================
//#include "DriveTrains/BasicDrive.cpp"
//#include "DriveTrains/CenterDrive.cpp"
#include "DriveTrains/SquareOmniDrive.cpp"

//===========Uncomment for tackle sensor===============
#define TACKLE

//===========Uncomment to choose a Position============
//#define WR
//#define Center
#define QB
//#define Kicker

//===========Uncomment if not using bag motors=========
//#define OldMotors
````



### Useful Information

* Might have to switch partition scheme to minimal SPIFFS if the app is too large to fit on the defualt partition scheme.
* It is useful to enable verbose mode under Core Debug Info to see what is happening with the esp32 especially when errors are occurring.


#### Useful Libraries

1. (PS3 Contoller) [esp32-ps3](https://github.com/jvpernis/esp32-ps3)
2. (Motor Library) [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)

#### Controls
---
  - **Basic Drivetrain**
    - _Up/Down Left Joystick_ - Forward and Backward movement
    - _Left/Right Right Joystick_ - Turning
    - _R2_ - activates "boost"
    - _Start_ - Puts robot in "kids mode". The speed is reduced, boost is disabled, and the leds will change
    - _Select_ and _Left/Right D-Pad_ - Corrects the robot for drift in the left or right direction
    <!-- - _Select _- Calibration mode - disables drivetrain while changes are made
      - _Up/Down D-Pad_ - compensates for drag left or right
      - _Select_ - exit Calibration Mode to regular drive mode -->

  - **Center**
    - Center currently uses basic drivetrain
    - _TRIANGLE_ - raise the center release servo
    - _CROSS_ - lower the center release servo
  - **Omniwheel Drivetrain**
    - _Up/Down/Left/Right Left Joystick_ - Lateral movement in any direction
    - _Up/Down/Left/Right D-Pad_- Lateral Movement along compass directions at full power
    - _Left/Right Right Joystick_ - Turning - as of version 1.0.3 this will disable rotation correction
    - _R3_ (Right Joystick Press) - Re-engage rotation correction
    - _R2_ - slow down speed
    - _L1_ - reverse directions (make back of robot front and vise versa)
    - Throwing
      - _SQUARE_ - Handoff throw
      - _CROSS_ - Reciever handoff throw/weak toss
      - _CIRCLE_ - mid range throw
      - _TRIANGLE_ - max power throw
      - _R1_ - return thrower to down position
      - _L2_ - hold to enable throw offset 
        - _Up/Down D-Pad_ - adjust power of all throws but triangle
  - **Kicker**
    - Kicker - currently uses basic drivetrain
    - _CROSS_ - kick
    - _TRIANGLE_ - reload





<!-- USEFUL LINKS -->
## Useful Links

This is a list of useful links and tutorials that were used.
* [How to use ArduinoJson with PubSubClient?](https://arduinojson.org/v6/how-to/use-arduinojson-with-pubsubclient/)
* [WiFiClientEvents.ino](https://github.com/espressif/arduino-esp32/blob/master/libraries/WiFi/examples/WiFiClientEvents/WiFiClientEvents.ino)
* [async-mqtt-client](https://github.com/marvinroger/async-mqtt-client)
* [esp32 low power wifi](https://www.bakke.online/index.php/2017/05/22/reducing-wifi-power-consumption-on-esp8266-part-3/)


## TODO

* ~~Replace EEPROM with esp32-Preferences~~
* ~~Replace pubsubclient with async-mqtt-client~~
* Make IP depend on robot name. Right now all IP are the same so they will conflict
* Read IP address from EEPROM, has to be unique to each robot, faster if static but can be allocated by DHCP
* Fix esp32 crashing when there is no robot name stored
* Move contoller joystick reading code into drive files?
