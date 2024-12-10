# vTally32-Multi - VISCA converter for vMix
# (c) 2024 by VID-PRO

This is a VISCA converter based on an esp32 and the vMix TCP API.
It can convert 4 VISCA UDP Control Signals to VISCA over serial.
It is tested on cisco PHD1080p12x cameras.

## Installation

### Hardware

To use it as tally you need a WT32-ETH01 board and a spezial Add On shield.  
For using the converter function you need the shield, see folder shield.

### Software

#### 1. libraries needed

Adafruit_NeoPixel
ESPoftwareSerial
ESPAsyncUDP

#### 2. Uploading static files

Connect the Arduino to the computer with a USB cable.  
The static files in the vTally32-Multi/data folder must be uploaded using the Tools > ESP32 Sketch Data Upload in the Arduino IDE.  

#### 3. Uploading firmware

Compile and upload the vTally.ino to the WT32-ETH01. The converter will restart in Connecting mode.  

## Getting Started

### LED color and meaning

#### 1. Connecting

| Color  | Meaning      | Led brightness | 
|--------|--------------|----------------|
| blue   | Connecting   | Dim            |

In this state the tally is trying to connect to WiFi and vMix based on the settings.  

#### 2. Access point

| Color  | Meaning      | Led brightness | 
|--------|--------------|----------------|
| purple | Settings     | Dim            |

In this state the tally was unable to connect to WiFi and it turned itself to access point mode. It can be accessed by connecting to the WiFi network with the SSID *vMix_Tally_#* (# is the tally number) and password *vMix_Tally_#_pwd* (# is the tally number). Once connected the settings can be changed by going to the webpage on IP address 192.168.4.1.  

#### 3. Tally

| Color  | Meaning      | Led brightness | 
|--------|--------------|----------------|
| green  | Preview      | Full           |
| orange | Live/program | Full           |
| blue   | Off          | Dim            |

In this state the tally is connected to WiFi and vMix. It detects new tally states and shows them using the rgb led.  
You can adjust the colors and brightness on the web page.

### Setup over Webpage

Network and tally settings can be edited on the built-in webpage. Just navigate to the IP address in a browser.  
On this webpage the WiFi SSID, WiFi password, vMix hostname, tally number, LED colors, LED brightness and VISCA settings can be changed. It also shows some basic information of the device. 

![WebGUI](https://github.com/pvtex/vTally32-Multi/raw/master/images/web.png)

## Pictures of the shield PCB
![PCB_2D](https://github.com/pvtex/vTally32-Multi/blob/main/shield/PCB_2D.png?raw=true) ![PCB_3D](https://github.com/pvtex/vTally32-Multi/blob/main/shield/PCB_3D.png?raw=true)
