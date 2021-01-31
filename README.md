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
* Add a check for a valid name in the readStoredName function

* Change it so that you have 5 seconds on poweron to connect to wifi and afterwards it wont try
* If wifi ever gets disconnected during operation then just give up on it

* Check wifi every cycle? Maybe faster than waiting for a wifi event callback.

## Performance Testing
No wireless - 1130 microseconds
Wireless with no WiFi connection - 
Wireless With WiFi connection - 1130uS spikes up to 1650uS

## Issues
* Contoller disconnects after some amount of time if esp32 cannot connect to wifi. The contoller will not reconnect at all. For some reason if the contoller is connected and you move the joysticks, the esp32 will panic and reboot
````
ASSERT_PARAM(1 0), in ld_pscan.c at line 235
Guru Meditation Error: Core  0 panic'ed (Interrupt wdt timeout on CPU0)
Core 0 register dump:
PC      : 0x40087c6d  PS      : 0x00060e34  A0      : 0x8003edb5  A1      : 0x3ffbe2b0  
A2      : 0x00000001  A3      : 0x00000000  A4      : 0x00000000  A5      : 0x60008054  
A6      : 0x3ffbdd44  A7      : 0x60008050  A8      : 0x80087c6d  A9      : 0x3ffbe290  
A10     : 0x00000004  A11     : 0x00000000  A12     : 0x6000804c  A13     : 0xffffffff  
A14     : 0x00000000  A15     : 0xfffffffc  SAR     : 0x00000004  EXCCAUSE: 0x00000005  
EXCVADDR: 0x00000000  LBEG    : 0x40087ba5  LEND    : 0x40087bac  LCOUNT  : 0x00000000  
Core 0 was running in ISR context:
EPC1    : 0x4008183e  EPC2    : 0x00000000  EPC3    : 0x00000000  EPC4    : 0x40087c6d

Backtrace: 0x40087c6d:0x3ffbe2b0 0x4003edb2:0x3ffbe2d0 0x40089206:0x3ffbe320 0x4008bcba:0x3ffbe340 0x4008c513:0x3ffbe360 0x400847e9:0x3ffbe380 0x401caad3:0x3ffbc140 0x400e1c76:0x3ffbc160 0x4008fe71:0x3ffbc180 0x4008e68d:0x3ffbc1a0

Core 1 register dump:
PC      : 0x400029ac  PS      : 0x00060234  A0      : 0x800d4f6d  A1      : 0x3ffce410  
A2      : 0x00000000  A3      : 0x00000000  A4      : 0x00000000  A5      : 0x00190000  
A6      : 0x7ff00000  A7      : 0x00000000  A8      : 0x00000000  A9      : 0x00000034  
A10     : 0x00008000  A11     : 0x00000000  A12     : 0x3ffe49f4  A13     : 0x00000000  
A14     : 0x00000000  A15     : 0x3ffcc580  SAR     : 0x0000001f  EXCCAUSE: 0x00000005  
EXCVADDR: 0x00000000  LBEG    : 0x400029ac  LEND    : 0x400029cb  LCOUNT  : 0x00000004  

Backtrace: 0x400029ac:0x3ffce410 0x400d4f6a:0x3ffce420 0x400d4f9e:0x3ffce450 0x400d4fd8:0x3ffce470 0x400d2049:0x3ffce490 0x400d2a82:0x3ffce4b0 0x400de8d9:0x3ffce4d0 0x4008e68d:0x3ffce4f0
````

