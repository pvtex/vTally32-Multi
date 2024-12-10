/*
  vTally-Multi for vMix
  Copyright 2024 VID-PRO
*/

#include "Arduino.h"
#include <EEPROM.h>
#include "WiFi.h"
#include "Arduino.h"
#include "WebServer.h"
#include <WiFiClient.h>
#include <Adafruit_NeoPixel.h>

#include <SoftwareSerial.h>
#include <ESPAsyncUDP.h>
#include <FS.h>
#include "SPIFFS.h"

// Constants
const float vers = 2.1;

const int SsidMaxLength = 24;
const int PassMaxLength = 24;
const int HostNameMaxLength = 24;
const int TallyNumberMaxValue = 64;

// LED setting
//#define LED_DATA D8
//bool data_state = false;
#define LED_PIN 0
#define LED_NUM 4
Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

// Settings object
struct Settings
{
  char ssid[SsidMaxLength];
  char pass[PassMaxLength];
  char hostName[HostNameMaxLength];
  int tallyNumber;
  int intensFull;
  int intensDim;
  int prgred;
  int prggreen;
  int prgblue;
  int prvred;
  int prvgreen;
  int prvblue;
  int offred;
  int offgreen;
  int offblue;
  unsigned int viscabaud;
  uint16_t viscaport;
};

// Default settings object
Settings defaultSettings = {
  "SSID",
  "PASSWORD",
  "vmix_hostname",
  1,
  254,
  128,
  0,
  254,
  0,
  254,
  128,
  0,
  0,
  0,
  254,
  9600,
  52381
};

Settings settings;

// HTTP Server settings
WebServer httpServer(80);
char deviceName[32];
int status = WL_IDLE_STATUS;
bool apEnabled = false;
char apPass[64];

// vMix settings
int port = 8099;

//// Tally info
char currentState1 = -1;
char oldState1 = -1;
char currentState2 = -1;
char oldState2 = -1;
char currentState3 = -1;
char oldState3 = -1;
char currentState4 = -1;
char oldState4 = -1;
const char tallyStateProgram = 1;
const char tallyStatePreview = 2;

// The WiFi client
WiFiClient client;
const int timeout = 10;
const int delayTime = 10000;
int vmixcon = 0;

// Time measure
const int interval = 5000;
unsigned long lastCheck = 0;

// VISCAoIP 2 serial
EspSoftwareSerial::UART viscaSerial1;
EspSoftwareSerial::UART viscaSerial2;
EspSoftwareSerial::UART viscaSerial3;
EspSoftwareSerial::UART viscaSerial4;
int udpstate = 0;

//// RS232 Serial Settings
const int txpin1 = 4;
const int rxpin1 = 2;
const int txpin2 = 14;
const int rxpin2 = 12;
const int txpin3 = 15;
const int rxpin3 = 14;
const int txpin4 = 39;
const int rxpin4 = 36;

//// Use the following constants and functions to modify the speed of PTZ commands
const double ZOOMMULT = 0.3;      // speed multiplier for the zoom functions
const double ZOOMEXP = 1.5;       // exponential curve for the speed modification
const double PTZMULT = 0.3;       // speed multiplier for the pan and tilt functions
const double PTZEXP = 1.0;        // exponential curve for the speed modification

//// STATE VARIABLES
AsyncUDP udp;
int lastclientport = 0;
IPAddress lastclientip;
bool pwr_is_on = false;

//// memory buffers for VISCA commands
size_t lastudp_len = 0;
uint8_t lastudp_in[16];
size_t lastser_len = 0;
uint8_t lastser_in[16];

//// quick use VISCA commands
const uint8_t pwr_on[] = {0x81, 0x01, 0x04, 0x00, 0x02, 0xff};
const uint8_t pwr_off[] = {0x81, 0x01, 0x04, 0x00, 0x03, 0xff};
const uint8_t addr_set[] = {0x88, 0x30, 0x01, 0xff};            // address set
const uint8_t if_clear[] = {0x88, 0x01, 0x00, 0x01, 0xff};      // if clear
const uint8_t ifClear[] = {0x88, 0x01, 0x00, 0x01, 0xff}; // Checks to see if communication line is clear
const uint8_t irOff[] = {0x81, 0x01, 0x06, 0x09, 0x03, 0xff}; // Turn off IR control (required for speed control of Pan/Tilt on TelePresence cameras)
const uint8_t callLedOn[] = {0x81, 0x01, 0x33, 0x01, 0x01, 0xff};
const uint8_t callLedOff[] = {0x81, 0x01, 0x33, 0x01, 0x00, 0xff};
const uint8_t callLedBlink[] = {0x81, 0x01, 0x33, 0x01, 0x02, 0xff};
const uint8_t ack[] = {0x90, 0x41, 0xff};
const uint8_t ack2[] = {0x90, 0x42, 0xff};
const uint8_t complete[] = {0x90, 0x51, 0xff};
const uint8_t complete2[] = {0x90, 0x52, 0xff};
const uint8_t ifclearcompl[] = {0x90, 0x50, 0xff};
int ifclearcomplength = 3;

/*
   Video formats values:
   Value    HDMI    SDI
   0x00     1080p25 1080p25
   0x01     1080p30 1080p30
   0x02     1080p50 720p50
   0x03     1080p60 720p60
   0x04     720p25  720p25
   0x05     720p30  720p30
   0x06     720p50  720p50
   0x07     720p60  720p60
*/
const uint8_t format = 0x01;
const uint8_t videoFormat[] = { 0x81, 0x01, 0x35, 0x00, format, 0x00, 0xff }; // 8x 01 35 0p 0q 0r ff p = reserved, q = video mode, r = Used in PrecisionHD 720p camera.

void restart()
{
  saveSettings();

  Serial.println(F(""));
  Serial.println(F("+--------------------+"));
  Serial.println(F("|       RESTART      |"));
  Serial.println(F("+--------------------+"));
  Serial.println(F(""));

  ESP.restart();
  //start();
}

// Load settings from EEPROM
void loadSettings()
{
  Serial.println(F("+--------------------+"));
  Serial.println(F("|  Loading settings  |"));
  Serial.println(F("+--------------------+"));

  long ptr = 0;

  for (int i = 0; i < SsidMaxLength; i++)
  {
    settings.ssid[i] = EEPROM.read(ptr);
    ptr++;
  }

  for (int i = 0; i < PassMaxLength; i++)
  {
    settings.pass[i] = EEPROM.read(ptr);
    ptr++;
  }

  for (int i = 0; i < HostNameMaxLength; i++)
  {
    settings.hostName[i] = EEPROM.read(ptr);
    ptr++;
  }

  settings.tallyNumber = EEPROM.read(ptr);

  ptr++;
  settings.intensFull = EEPROM.read(ptr);

  ptr++;
  settings.intensDim = EEPROM.read(ptr);

  ptr++;
  settings.prgred = EEPROM.read(ptr);
  ptr++;
  settings.prggreen = EEPROM.read(ptr);
  ptr++;
  settings.prgblue = EEPROM.read(ptr);

  ptr++;
  settings.prvred = EEPROM.read(ptr);
  ptr++;
  settings.prvgreen = EEPROM.read(ptr);
  ptr++;
  settings.prvblue = EEPROM.read(ptr);

  ptr++;
  settings.offred = EEPROM.read(ptr);
  ptr++;
  settings.offgreen = EEPROM.read(ptr);
  ptr++;
  settings.offblue = EEPROM.read(ptr);

  ptr++;
  byte low = EEPROM.read(ptr);
  ptr++;
  byte high = EEPROM.read(ptr);
  settings.viscabaud = low + ((high << 8) & 0xFF00);
  ptr++;
  low = EEPROM.read(ptr);
  ptr++;
  high = EEPROM.read(ptr);
  settings.viscaport = low + ((high << 8) & 0xFF00);


  if (strlen(settings.ssid) == 0 || strlen(settings.pass) == 0 || strlen(settings.hostName) == 0 || settings.tallyNumber == 0 || settings.intensFull == 0 || settings.intensDim == 0 || settings.viscabaud == 0 || settings.viscaport == 0)
  {
    Serial.println(F("| No settings found"));
    Serial.println(F("| Loading default settings"));
    settings = defaultSettings;
    saveSettings();
    restart();
  }
  else
  {
    Serial.println(F("| Settings loaded"));
    printSettings();
    Serial.println(F("+---------------------"));
  }
}

// Save settings to EEPROM
void saveSettings()
{
  Serial.println(F("+--------------------+"));
  Serial.println(F("|  Saving settings   |"));
  Serial.println(F("+--------------------+"));

  long ptr = 0;

  for (int i = 0; i < 512; i++)
  {
    EEPROM.write(i, 0);
  }

  for (int i = 0; i < SsidMaxLength; i++)
  {
    EEPROM.write(ptr, settings.ssid[i]);
    ptr++;
  }

  for (int i = 0; i < PassMaxLength; i++)
  {
    EEPROM.write(ptr, settings.pass[i]);
    ptr++;
  }

  for (int i = 0; i < HostNameMaxLength; i++)
  {
    EEPROM.write(ptr, settings.hostName[i]);
    ptr++;
  }

  EEPROM.write(ptr, settings.tallyNumber);

  ptr++;
  EEPROM.write(ptr, settings.intensFull);

  ptr++;
  EEPROM.write(ptr, settings.intensDim);

  ptr++;
  EEPROM.write(ptr, settings.prgred);
  ptr++;
  EEPROM.write(ptr, settings.prggreen);
  ptr++;
  EEPROM.write(ptr, settings.prgblue);

  ptr++;
  EEPROM.write(ptr, settings.prvred);
  ptr++;
  EEPROM.write(ptr, settings.prvgreen);
  ptr++;
  EEPROM.write(ptr, settings.prvblue);

  ptr++;
  EEPROM.write(ptr, settings.offred);
  ptr++;
  EEPROM.write(ptr, settings.offgreen);
  ptr++;
  EEPROM.write(ptr, settings.offblue);

  ptr++;
  EEPROM.write(ptr, settings.viscabaud & 0xFF);
  ptr++;
  EEPROM.write(ptr, (settings.viscabaud >> 8) & 0xFF);

  ptr++;
  EEPROM.write(ptr, settings.viscaport & 0xFF);
  ptr++;
  EEPROM.write(ptr, (settings.viscaport >> 8) & 0xFF);

  EEPROM.commit();

  Serial.println(F("| Settings saved"));
  printSettings();
  Serial.println(F("+---------------------"));
}

// Print settings
void printSettings()
{
  Serial.print(F("| SSID            : "));
  Serial.println(settings.ssid);
  Serial.print(F("| SSID Password   : "));
  Serial.println(settings.pass);
  Serial.print(F("| vMix Hostname/IP: "));
  Serial.println(settings.hostName);
  Serial.print(F("| Tally number    : "));
  Serial.print(settings.tallyNumber);
  Serial.print(F(", "));
  Serial.print(settings.tallyNumber+1);
  Serial.print(F(", "));
  Serial.print(settings.tallyNumber+2);
  Serial.print(F(", "));
  Serial.println(settings.tallyNumber+3);
  Serial.print(F("| Intensity Full  : "));
  Serial.println(settings.intensFull);
  Serial.print(F("| Intensity Dim   : "));
  Serial.println(settings.intensDim);
  Serial.print(F("| Program-Color   : "));
  Serial.print(settings.prgred);
  Serial.print(F(","));
  Serial.print(settings.prggreen);
  Serial.print(F(","));
  Serial.println(settings.prgblue);
  Serial.print(F("| Preview-Color   : "));
  Serial.print(settings.prvred);
  Serial.print(F(","));
  Serial.print(settings.prvgreen);
  Serial.print(F(","));
  Serial.println(settings.prvblue);
  Serial.print(F("| Off-Color       : "));
  Serial.print(settings.offred);
  Serial.print(F(","));
  Serial.print(settings.offgreen);
  Serial.print(F(","));
  Serial.println(settings.offblue);
  Serial.print(F("| VISCA baud      : "));
  Serial.println(settings.viscabaud);
  Serial.print(F("| VISCA port      : "));
  Serial.println(settings.viscaport);
}

// Set led intensity from 0 to 255
void ledSetIntensity(int intensity)
{
  leds.setBrightness(intensity);
}

// Set LED's off
void ledSetOff(int port )
{
  leds.setPixelColor(port - 1, leds.Color(0, 0, 0));
  ledSetIntensity(0);
  leds.show();
}

// Draw corner dots
void ledSetCornerDots(int port)
{
  leds.setPixelColor(port - 1, leds.Color(settings.offred, settings.offgreen, settings.offblue));
  ledSetIntensity(settings.intensDim);
  leds.show();
}

// Draw L(ive) with LED's
void ledSetProgram(int port)
{
  leds.setPixelColor(port - 1, leds.Color(settings.prgred, settings.prggreen, settings.prgblue));
  ledSetIntensity(settings.intensFull);
  leds.show();
}

// Draw P(review) with LED's
void ledSetPreview(int port)
{
  leds.setPixelColor(port - 1, leds.Color(settings.prvred, settings.prvgreen, settings.prvblue));
  ledSetIntensity(settings.intensFull);
  leds.show();
}

// Draw C(onnecting) with LED's
void ledSetConnecting(int port)
{
  leds.setPixelColor(port - 1, leds.Color(0, 255, 255));
  ledSetIntensity(settings.intensDim);
  leds.show();
}

// Draw S(ettings) with LED's
void ledSetSettings(int port)
{
  leds.setPixelColor(port - 1, leds.Color(255, 0, 255));
  ledSetIntensity(settings.intensDim);
  leds.show();
}
void send_visca1(uint8_t *c, size_t len)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial1.print(elem);
  } while (i < len && elem != 0xff);
  Serial.println(F("| VISCA IP->SER1"));
}
void send_visca2(uint8_t *c, size_t len)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial2.print(elem);
  } while (i < len && elem != 0xff);
  Serial.println(F("| VISCA IP->SER2"));
}
void send_visca3(uint8_t *c, size_t len)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial3.print(elem);
  } while (i < len && elem != 0xff);
  Serial.println(F("| VISCA IP->SER3"));
}
void send_visca4(uint8_t *c, size_t len)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial4.print(elem);
  } while (i < len && elem != 0xff);
  Serial.println(F("| VISCA IP->SER4"));
}

void send_visca1(const uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial1.print(elem);
  } while (elem != 0xff);
  Serial.println(F("| VISCA IP->SER1"));
}
void send_visca2(const uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial2.print(elem);
  } while (elem != 0xff);
  Serial.println(F("| VISCA IP->SER2"));
}
void send_visca3(const uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial3.print(elem);
  } while (elem != 0xff);
  Serial.println(F("| VISCA IP->SER3"));
}
void send_visca4(const uint8_t *c)
{
  int i = 0;
  uint8_t elem;
  do
  {
    elem = c[i++];
    viscaSerial4.print(elem);
  } while (elem != 0xff);
  Serial.println(F("| VISCA IP->SER4"));
}

// Set tally to off
void tallySetOff(int port)
{
  Serial.println(F("| Tally off"));

  ledSetCornerDots(port);
  if (port == 1)
  {
    send_visca1(callLedOff);
  } else if (port == 2)
  {
    send_visca2(callLedOff);
  } else if (port == 3)
  {
    send_visca3(callLedOff);
  } else if (port == 4)
  {
    send_visca4(callLedOff);
  }
}

// Set tally to program
void tallySetProgram(int port)
{
  Serial.println(F("| Tally program"));

  ledSetProgram(port);
  if (port == 1)
  {
    send_visca1(callLedOn);
  } else if (port == 2)
  {
    send_visca2(callLedOn);
  } else if (port == 3)
  {
    send_visca3(callLedOn);
  } else if (port == 4)
  {
    send_visca4(callLedOn);
  }
}

// Set tally to preview
void tallySetPreview(int port)
{
  Serial.println(F("| Tally preview"));

  ledSetPreview(port);
  if (port == 1)
  {
    send_visca1(callLedBlink);
  } else if (port == 2)
  {
    send_visca2(callLedBlink);
  } else if (port == 3)
  {
    send_visca3(callLedBlink);
  } else if (port == 4)
  {
    send_visca4(callLedBlink);
  }
}

// Set tally to connecting
void tallySetConnecting()
{
  ledSetConnecting(1);
  ledSetConnecting(2);
  ledSetConnecting(3);
  ledSetConnecting(4);
}

// Handle incoming data
void handleData(String data)
{
  // Check if server data is tally data
  if (data.indexOf("TALLY OK") == 0)
  {
    vmixcon = 1;
    char newState1 = data.charAt(settings.tallyNumber + 8);
    char newState2 = data.charAt(settings.tallyNumber + 9);
    char newState3 = data.charAt(settings.tallyNumber + 10);
    char newState4 = data.charAt(settings.tallyNumber + 11);

    // Check if tally state has changed
    if (currentState1 != newState1 || currentState2 != newState2 || currentState3 != newState3 || currentState4 != newState4)
    {
      currentState1 = newState1;
      currentState2 = newState2;
      currentState3 = newState3;
      currentState4 = newState4;

      switch (currentState1)
      {
        case '0':
          tallySetOff(1);
          break;
        case '1':
          tallySetProgram(1);
          break;
        case '2':
          tallySetPreview(1);
          break;
        default:
          tallySetOff(1);
      }
      switch (currentState2)
      {
        case '0':
          tallySetOff(2);
          break;
        case '1':
          tallySetProgram(2);
          break;
        case '2':
          tallySetPreview(2);
          break;
        default:
          tallySetOff(2);
      }
      switch (currentState3)
      {
        case '0':
          tallySetOff(3);
          break;
        case '1':
          tallySetProgram(3);
          break;
        case '2':
          tallySetPreview(3);
          break;
        default:
          tallySetOff(3);
      }
      switch (currentState4)
      {
        case '0':
          tallySetOff(4);
          break;
        case '1':
          tallySetProgram(4);
          break;
        case '2':
          tallySetPreview(4);
          break;
        default:
          tallySetOff(4);
      }
      /*
            if (data_state) {
              digitalWrite(LED_DATA, data_state);
              data_state = false;
            } else {
              digitalWrite(LED_DATA, data_state);
              data_state = true;
            }
      */
    }
  }
  else
  {
    vmixcon = 1;
    Serial.print(F("| Response from vMix: "));
    Serial.println(data);

    //Serial.print(F("| FreeHeap: "));
    //Serial.println(ESP.getFreeHeap(),DEC);
  }
}

// Start access point
void apStart()
{
  ledSetSettings(1);
  ledSetSettings(2);
  ledSetSettings(3);
  ledSetSettings(4);
  Serial.println(F("+--------------------+"));
  Serial.println(F("|       AP Start     |"));
  Serial.println(F("+--------------------+"));
  Serial.print(F("| AP SSID         : "));
  Serial.println(deviceName);
  Serial.print(F("| AP password     : "));
  Serial.println(apPass);

  WiFi.mode(WIFI_AP);
  WiFi.hostname(deviceName);
  WiFi.softAP(deviceName, apPass);
  delay(100);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print(F("| IP address      : "));
  Serial.println(myIP);
  Serial.println(F("+---------------------"));


  apEnabled = true;
}

// Handle http server Tally request
void tallyPageHandler()
{
  String response_message = F("<!DOCTYPE html>");

  response_message += F("<html lang='en'>");
  response_message += F("<head>");
  response_message += F("<title>vTally-Multi by VID-PRO - ");
  response_message += String(deviceName);
  response_message += F(" +1/+2/+3</title>");
  response_message += F("<meta name='viewport' content='width=device-width, initial-scale=1, shrink-to-fit=no'>");
  response_message += F("<meta charset='utf-8'>");
  response_message += F("<link rel='icon' type='image/x-icon' href='favicon.ico'>");
  response_message += F("<link rel='stylesheet' href='styles.css'>");
  response_message += F("<style>body {width: 100%;height: 100%;padding: 0px;}</style>");
  response_message += F("</head>");

  response_message += F("<body class='bg-light'>");

  response_message += F("<table class='table'><tbody  style='border-radius: 0px 0px 10px 10px;background-color:#d5dadd;'>");

  response_message += F("<tr><th style='text-align:center' width='25%'>");
  response_message += String(settings.tallyNumber);
  response_message += F("</th><th style='text-align:center' width='25%'>");
  response_message += String(settings.tallyNumber+1);
  response_message += F("</th><th style='text-align:center' width='25%'>");
  response_message += String(settings.tallyNumber+2);
  response_message += F("</th><th style='text-align:center' width='25%'>");
  response_message += String(settings.tallyNumber+3);
  response_message += F("</th></tr>");

  response_message += F("<tr><th style='background-color:rgb(");
  if (vmixcon == 0) {
    currentState1 = '3';
  }
  if (vmixcon == 1 && (currentState1 != '0' && currentState1 != '1' && currentState1 != '2')) {
    currentState1 = 4;
  }
  if (vmixcon == 1 && (currentState2 != '0' && currentState2 != '1' && currentState2 != '2')) {
    currentState2 = 4;
  }
  if (vmixcon == 1 && (currentState3 != '0' && currentState3 != '1' && currentState3 != '2')) {
    currentState3 = 4;
  }
  if (vmixcon == 1 && (currentState4 != '0' && currentState4 != '1' && currentState4 != '2')) {
    currentState4 = 4;
  }
  switch (currentState1)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th>");

  response_message += F("<th style='background-color:rgb(");
  if (vmixcon == 0) {
    currentState1 = '3';
  }
  if (vmixcon == 1 && (currentState1 != '0' && currentState1 != '1' && currentState1 != '2')) {
    currentState1 = 4;
  }
  if (vmixcon == 1 && (currentState2 != '0' && currentState2 != '1' && currentState2 != '2')) {
    currentState2 = 4;
  }
  if (vmixcon == 1 && (currentState3 != '0' && currentState3 != '1' && currentState3 != '2')) {
    currentState3 = 4;
  }
  if (vmixcon == 1 && (currentState4 != '0' && currentState4 != '1' && currentState4 != '2')) {
    currentState4 = 4;
  }
  switch (currentState2)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th>");

  response_message += F("<th style='background-color:rgb(");
  if (vmixcon == 0) {
    currentState1 = '3';
  }
  if (vmixcon == 1 && (currentState1 != '0' && currentState1 != '1' && currentState1 != '2')) {
    currentState1 = 4;
  }
  if (vmixcon == 1 && (currentState2 != '0' && currentState2 != '1' && currentState2 != '2')) {
    currentState2 = 4;
  }
  if (vmixcon == 1 && (currentState3 != '0' && currentState3 != '1' && currentState3 != '2')) {
    currentState3 = 4;
  }
  if (vmixcon == 1 && (currentState4 != '0' && currentState4 != '1' && currentState4 != '2')) {
    currentState4 = 4;
  }
  switch (currentState3)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th>");

  response_message += F("<th style='background-color:rgb(");
  if (vmixcon == 0) {
    currentState1 = '3';
  }
  if (vmixcon == 1 && (currentState1 != '0' && currentState1 != '1' && currentState1 != '2')) {
    currentState1 = 4;
  }
  if (vmixcon == 1 && (currentState2 != '0' && currentState2 != '1' && currentState2 != '2')) {
    currentState2 = 4;
  }
  if (vmixcon == 1 && (currentState3 != '0' && currentState3 != '1' && currentState3 != '2')) {
    currentState3 = 4;
  }
  if (vmixcon == 1 && (currentState4 != '0' && currentState4 != '1' && currentState4 != '2')) {
    currentState4 = 4;
  }
  switch (currentState4)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th></tr>");

  response_message += F("<tr>");


  response_message += F("<th style='text-align:center;background-color:rgb(");
  switch (currentState1)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");color:");
  switch (currentState1)
  {
    case '0':
      response_message += F("#ffffff"); //off
      break;
    case '1':
      response_message += F("#ffffff"); //prg
      break;
    case '2':
      response_message += F("#ffffff"); //prv
      break;
    case '3':
      response_message += F("#ffffff"); // no vMix Server
      break;
    case '4':
      response_message += F("#ffffff"); // no vMix Server
      break;
    default:
      response_message += F("#ffffff"); //default off
  }
  response_message += F("'>");
  switch (currentState1)
  {
    case '0':
      response_message += F("OFF"); //off
      break;
    case '1':
      response_message += F("PROGRAM"); //prg
      break;
    case '2':
      response_message += F("PREVIEW"); //prv
      break;
    case '3':
      response_message += F("vMix Server not found!"); // no vMix Server
      break;
    case '4':
      response_message += F("connected to vMix Server, waiting for data."); // no vMix Server
      break;
    default:
      response_message += F("OFF"); //default off
  }
  response_message += F("</th>");

  response_message += F("<th style='text-align:center;background-color:rgb(");
  switch (currentState2)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");color:");
  switch (currentState2)
  {
    case '0':
      response_message += F("#ffffff"); //off
      break;
    case '1':
      response_message += F("#ffffff"); //prg
      break;
    case '2':
      response_message += F("#ffffff"); //prv
      break;
    case '3':
      response_message += F("#ffffff"); // no vMix Server
      break;
    case '4':
      response_message += F("#ffffff"); // no vMix Server
      break;
    default:
      response_message += F("#ffffff"); //default off
  }
  response_message += F("'>");
  switch (currentState2)
  {
    case '0':
      response_message += F("OFF"); //off
      break;
    case '1':
      response_message += F("PROGRAM"); //prg
      break;
    case '2':
      response_message += F("PREVIEW"); //prv
      break;
    case '3':
      response_message += F("vMix Server not found!"); // no vMix Server
      break;
    case '4':
      response_message += F("connected to vMix Server, waiting for data."); // no vMix Server
      break;
    default:
      response_message += F("OFF"); //default off
  }
  response_message += F("</th>");

  response_message += F("<th style='text-align:center;background-color:rgb(");
  switch (currentState3)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");color:");
  switch (currentState3)
  {
    case '0':
      response_message += F("#ffffff"); //off
      break;
    case '1':
      response_message += F("#ffffff"); //prg
      break;
    case '2':
      response_message += F("#ffffff"); //prv
      break;
    case '3':
      response_message += F("#ffffff"); // no vMix Server
      break;
    case '4':
      response_message += F("#ffffff"); // no vMix Server
      break;
    default:
      response_message += F("#ffffff"); //default off
  }
  response_message += F("'>");
  switch (currentState3)
  {
    case '0':
      response_message += F("OFF"); //off
      break;
    case '1':
      response_message += F("PROGRAM"); //prg
      break;
    case '2':
      response_message += F("PREVIEW"); //prv
      break;
    case '3':
      response_message += F("vMix Server not found!"); // no vMix Server
      break;
    case '4':
      response_message += F("connected to vMix Server, waiting for data."); // no vMix Server
      break;
    default:
      response_message += F("OFF"); //default off
  }
  response_message += F("</th>");

  response_message += F("<th style='text-align:center;background-color:rgb(");
  switch (currentState4)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");color:");
  switch (currentState4)
  {
    case '0':
      response_message += F("#ffffff"); //off
      break;
    case '1':
      response_message += F("#ffffff"); //prg
      break;
    case '2':
      response_message += F("#ffffff"); //prv
      break;
    case '3':
      response_message += F("#ffffff"); // no vMix Server
      break;
    case '4':
      response_message += F("#ffffff"); // no vMix Server
      break;
    default:
      response_message += F("#ffffff"); //default off
  }
  response_message += F("'>");
  switch (currentState4)
  {
    case '0':
      response_message += F("OFF"); //off
      break;
    case '1':
      response_message += F("PROGRAM"); //prg
      break;
    case '2':
      response_message += F("PREVIEW"); //prv
      break;
    case '3':
      response_message += F("vMix Server not found!"); // no vMix Server
      break;
    case '4':
      response_message += F("connected to vMix Server, waiting for data."); // no vMix Server
      break;
    default:
      response_message += F("OFF"); //default off
  }
  response_message += F("</th></tr>");



  response_message += F("<tr><th style='background-color:rgb(");
  switch (currentState1)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th>");
  response_message += F("<th style='background-color:rgb(");
  switch (currentState2)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th>");

  response_message += F("<th style='background-color:rgb(");
  switch (currentState3)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th>");
  response_message += F("<th style='background-color:rgb(");
  switch (currentState4)
  {
    case '0':
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //off
      break;
    case '1':
      response_message += String(settings.prgred);
      response_message += F(",");
      response_message += String(settings.prggreen);
      response_message += F(",");
      response_message += String(settings.prgblue); //prg
      break;
    case '2':
      response_message += String(settings.prvred);
      response_message += F(",");
      response_message += String(settings.prvgreen);
      response_message += F(",");
      response_message += String(settings.prvblue); //prv
      break;
    case '3':
      response_message += F("255,0,0"); // no vMix Server
      break;
    case '4':
      response_message += F("255,255,0"); // no vMix Server
      break;
    default:
      response_message += String(settings.offred);
      response_message += F(",");
      response_message += String(settings.offgreen);
      response_message += F(",");
      response_message += String(settings.offblue); //default off
  }
  response_message += F(");'>&nbsp;</th></tr>");

  response_message += F("</tbody></table>");
  response_message += F("</body></html>");

  httpServer.sendHeader("Connection", "close");
  httpServer.send(200, "text/html", String(response_message));

  //Serial.print(F("| FreeHeap: "));
  //Serial.println(ESP.getFreeHeap(),DEC);
}

// Handle http server root request
void rootPageHandler() {
  String response_message;

  httpServer.setContentLength(CONTENT_LENGTH_UNKNOWN);

  response_message = F("<!DOCTYPE html><html lang='en'><head><title>vTally-Multi by VID-PRO - ");
  response_message += String(deviceName);
  response_message += F("</title><meta name='viewport' content='width=device-width, initial-scale=1, shrink-to-fit=no'><meta charset='utf-8'>");
  response_message += F("<link rel='icon' type='image/x-icon' href='favicon.ico'>");
  response_message += F("<link rel='stylesheet' href='styles.css'>");
  response_message += F("<script src='jquery.slim.min.js'></script>");
  response_message += F("<script type='text/javascript'>");
  response_message += F("var ESPurl = 'http://");
  response_message += WiFi.localIP().toString();
  response_message += F("/zend';");
  response_message += F("if(typeof(EventSource) !== 'undefined') {");
  response_message += F("  var source = new EventSource(ESPurl);");
  response_message += F("  source.onmessage = function(event) {");
  response_message += F("    if(event.data == 'refresh0' || event.data == 'refresh1') {");
  response_message += F("      $('#tally').html(\"<object type='text/html' data='/tally' style='height:100%;width:100%;'></oject>\");");
  response_message += F("      if(event.data == 'refresh1') {");
  response_message += F("        $('#vmixstatus').html(\"Connected\");");
  response_message += F("        $('#vmixstatus').css(\"background-color\",\"green\");");
  response_message += F("      }");
  response_message += F("      if(event.data == 'refresh0') {");
  response_message += F("        $('#vmixstatus').html(\"Disconnected\");");
  response_message += F("        $('#vmixstatus').css(\"background-color\",\"red\");");
  response_message += F("      }");
  response_message += F("    }");
  response_message += F("  }");
  response_message += F("} else {");
  response_message += F("  document.getElementById('tally').innerHTML('Your browser does not support EventSource!')");
  response_message += F("}");

  response_message += F("</script>");
  response_message += F("<style>body {width: 100%;height: 100%;padding: 25px;}</style>");
  response_message += F("</head>");

  httpServer.send(200, "text/html", response_message);

  response_message = F("<body class='bg-light'>");

  response_message += F("<h1 style='border-radius: 10px 10px 0px 0px;background-color:#0066ff;color:#FFFFFF;text-align:center;padding:5px;margin-bottom:0px;'><span style='vertical-align:text-top;'>vTally-Multi by </span><a href=https://www.vid-pro.de target=_new><img src='logo.png' style='vertical-align:bottom;'></a></h1>");

  response_message += F("<h1 style='border-radius:0px 0px 10px 10px;background-color:#c6cdd2;text-align:center;'>vTally ID: ");
  response_message += String(settings.tallyNumber);
  response_message += F(", ");
  response_message += String(settings.tallyNumber+1);
  response_message += F(", ");
  response_message += String(settings.tallyNumber+2);
  response_message += F(", ");
  response_message += String(settings.tallyNumber+3);
  response_message += F("</h1>");

  response_message += F("<form action='/save' method='post' enctype='multipart/form-data' data-ajax='false'>");

  response_message += F("<div data-role='content' class='row' style='margin-bottom:.5rem;'>");

  response_message += F("<div class='col-md-6'>");
  response_message += F("<h2 style='border-radius: 10px 10px 0px 0px;margin-bottom:0px;background-color:#99c2ff;padding:2px;'>&nbsp; Network/vMix/VISCA</h2>");

  response_message += F("<table class='table' style='border-radius: 0px 0px 10px 10px;'><tbody  style='border-radius: 0px 0px 10px 10px;background-color:#d5dadd;'><tr style='border-radius: 0px 0px 10px 10px;'><td style='border-radius: 0px 0px 10px 10px;'>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='ssid' class='col-sm-4 col-form-label'>WiFi SSID</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='ssid' class='form-control' type='text' size='64' maxlength='32' name='ssid' value='");
  response_message += String(settings.ssid);
  response_message += F("'>");
  response_message += F("</div></div>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='ssidpass' class='col-sm-4 col-form-label'>WiFi Passphrase</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='ssidpass' class='form-control' type='password' autocomplete='current-password' size='64' maxlength='32' name='ssidpass' value='");
  response_message += String(settings.pass);
  response_message += F("'>");
  response_message += F("</div></div>");

  response_message += F("<hr>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='hostname' class='col-sm-4 col-form-label'>vMix Server Hostname/IP</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='hostname' class='form-control' type='text' size='64' maxlength='32' name='hostname' value='");
  response_message += String(settings.hostName);
  response_message += F("'>");
  response_message += F("</div></div>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='inputnumber' class='col-sm-4 col-form-label'>Tally number (1-");
  response_message += String(TallyNumberMaxValue);
  response_message += F(") +1/+2/+3</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='inputnumber' class='form-control' type='number' size='64' min='0' max='");
  response_message += String(TallyNumberMaxValue);
  response_message += F("' name='inputnumber' value='");
  response_message += String(settings.tallyNumber);
  response_message += F("'>");
  response_message += F("</div></div>");

  response_message += F("<hr>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='inputnumber' class='col-sm-4 col-form-label'>VISCA Baudrate (9600)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<select id='viscabaud' name='viscabaud' class='form-control'>");
  response_message += F("<option value='4800' ");
  (String(settings.viscabaud) == "4800") ? response_message += "selected" : response_message += "";
  response_message += F(">4800</option>");
  response_message += F("<option value='9600' ");
  (String(settings.viscabaud) == "9600") ? response_message += "selected" : response_message += "";
  response_message += F(">9600</option>");
  response_message += F("<option value='14400' ");
  (String(settings.viscabaud) == "14400") ? response_message += "selected" : response_message += "";
  response_message += F(">14400</option>");
  response_message += F("<option value='19200' ");
  (String(settings.viscabaud) == "19200") ? response_message += "selected" : response_message += "";
  response_message += F(">19200</option>");
  response_message += F("<option value='57600' ");
  (String(settings.viscabaud) == "57600") ? response_message += "selected" : response_message += "";
  response_message += F(">57600</option>");
  response_message += F("<option value='115200' ");
  (String(settings.viscabaud) == "115200") ? response_message += "selected" : response_message += "";
  response_message += F(">115200</option>");
  response_message += F("</select>");
  //  response_message += F("<input id='viscabaud' class='form-control' type='number' size='64' min='2400' max='115200' name='viscabaud' value='") + String(settings.viscabaud) + F("'>");
  response_message += F("</div></div>");

  httpServer.sendContent(response_message);

  response_message = F("<div class='form-group row'>");
  response_message += F("<label for='viscaport' class='col-sm-4 col-form-label'>VISCA UDP Port (52381)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='viscaport' class='form-control' type='number' size='64' min='1024' max='65554' name='viscaport' value='");
  response_message += String(settings.viscaport);
  response_message += F("'>");
  response_message += F("</div></div>");

  response_message += F("<div class='form-group row' style='border-radius:0px 0px 0px 10px;background-color:c6cdd2;'>");
  response_message += F("<label for='save' class='col-sm-4 col-form-label'> </label>");
  response_message += F("<div class='col-sm-8' style='border-radius:0px 0px 10px 0px;text-align:center;'>");
  response_message += F("<input type='submit' value='SAVE' class='btn btn-primary'>");
  response_message += F("</div></div>");

  response_message += F("</td></tr></tbody></table>");

  response_message += F("</div>");

  response_message += F("<div class='col-md-6'>");
  response_message += F("<h2 style='border-radius: 10px 10px 0px 0px;margin-bottom:0px;background-color:#99c2ff;padding:2px;'>&nbsp;   vMix Tally</h2>");

  response_message += F("<table class='table' style='height:90%;border-radius: 0px 0px 10px 10px;'><tbody  style='border-radius: 0px 0px 10px 10px;background-color:#d5dadd;'><tr style='border-radius: 0px 0px 10px 10px;'><td style='border-radius: 0px 0px 10px 10px;'>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='prg' class='col-sm-4 col-form-label'>Program Color (0-254)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<div class='form-group row' style='margin-bottom:0px;'>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;R&nbsp;</div><div>");
  response_message += F("<input id='prgred' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='prgred' value='");
  response_message += String(settings.prgred);
  response_message += F("' onchange=\"document.getElementById('pprg').style.backgroundColor = 'rgb('+document.getElementById('prgred').value+','+document.getElementById('prggreen').value+','+document.getElementById('prgblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;G&nbsp;</div><div>");
  response_message += F("<input id='prggreen' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='prggreen' value='");
  response_message += String(settings.prggreen);
  response_message += F("'onchange=\"document.getElementById('pprg').style.backgroundColor = 'rgb('+document.getElementById('prgred').value+','+document.getElementById('prggreen').value+','+document.getElementById('prgblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;B&nbsp;</div><div>");
  response_message += F("<input id='prgblue' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='prgblue' value='");
  response_message += String(settings.prgblue);
  response_message += F("'onchange=\"document.getElementById('pprg').style.backgroundColor = 'rgb('+document.getElementById('prgred').value+','+document.getElementById('prggreen').value+','+document.getElementById('prgblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div>&nbsp;&nbsp;</div>");
  response_message += F("<div id='pprg' style='display: -webkit-flex;display: flex;align-items: center;color:#ffffff;background-color:rgb(");
  response_message += String(settings.prgred);
  response_message += F(",");
  response_message += String(settings.prggreen);
  response_message += F(",");
  response_message += String(settings.prgblue);
  response_message += F(")'>&nbsp;Program&nbsp;</div>");
  response_message += F("</div></div></div>");

  httpServer.sendContent(response_message);

  response_message = F("<div class='form-group row'>");
  response_message += F("<label for='prv' class='col-sm-4 col-form-label'>Preview Color (0-254)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<div class='form-group row' style='margin-bottom:0px;'>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;R&nbsp;</div><div>");
  response_message += F("<input id='prvred' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='prvred' value='");
  response_message += String(settings.prvred);
  response_message += F("' onchange=\"document.getElementById('pprv').style.backgroundColor = 'rgb('+document.getElementById('prvred').value+','+document.getElementById('prvgreen').value+','+document.getElementById('prvblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;G&nbsp;</div><div>");
  response_message += F("<input id='prvgreen' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='prvgreen' value='");
  response_message += String(settings.prvgreen);
  response_message += F("' onchange=\"document.getElementById('pprv').style.backgroundColor = 'rgb('+document.getElementById('prvred').value+','+document.getElementById('prvgreen').value+','+document.getElementById('prvblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;B&nbsp;</div><div>");
  response_message += F("<input id='prvblue' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='prvblue' value='");
  response_message += String(settings.prvblue);
  response_message += F("' onchange=\"document.getElementById('pprv').style.backgroundColor = 'rgb('+document.getElementById('prvred').value+','+document.getElementById('prvgreen').value+','+document.getElementById('prvblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div>&nbsp;&nbsp;</div>");
  response_message += F("<div id='pprv' style='display: -webkit-flex;display: flex;align-items: center;color:#ffffff;background-color:rgb(");
  response_message += String(settings.prvred);
  response_message += F(",");
  response_message += String(settings.prvgreen);
  response_message += F(",");
  response_message += String(settings.prvblue);
  response_message += F(")'>&nbsp;&nbsp;Preview&nbsp;&nbsp;</div>");
  response_message += F("</div></div></div>");
  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='off' class='col-sm-4 col-form-label'>Off Color (0-254)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<div class='form-group row' style='margin-bottom:0px;'>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;R&nbsp;</div><div>");
  response_message += F("<input id='offred' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='offred' value='");
  response_message += String(settings.offred);
  response_message += F("' onchange=\"document.getElementById('poff').style.backgroundColor = 'rgb('+document.getElementById('offred').value+','+document.getElementById('offgreen').value+','+document.getElementById('offblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;G&nbsp;</div><div>");
  response_message += F("<input id='offgreen' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='offgreen' value='");
  response_message += String(settings.offgreen);
  response_message += F("' onchange=\"document.getElementById('poff').style.backgroundColor = 'rgb('+document.getElementById('offred').value+','+document.getElementById('offgreen').value+','+document.getElementById('offblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div style='display: -webkit-flex;display: flex;align-items: center'>&nbsp;B&nbsp;</div><div>");
  response_message += F("<input id='offblue' class='form-control' type='number' size='21' min='0' max='254' style='width:5em;' name='offblue' value='");
  response_message += String(settings.offblue);
  response_message += F("' onchange=\"document.getElementById('poff').style.backgroundColor = 'rgb('+document.getElementById('offred').value+','+document.getElementById('offgreen').value+','+document.getElementById('offblue').value+')'\">");
  response_message += F("</div>");
  response_message += F("<div>&nbsp;&nbsp;</div>");
  response_message += F("<div id='poff' style='display: -webkit-flex;display: flex;align-items: center;color:#ffffff;background-color:rgb(");
  response_message += String(settings.offred);
  response_message += F(",");
  response_message += String(settings.offgreen);
  response_message += F(",");
  response_message += String(settings.offblue);
  response_message += F(")'>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Off&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</div>");
  response_message += F("</div></div></div>");

  httpServer.sendContent(response_message);

  response_message = F("<hr>");

  response_message += F("<div class='form-group row' style='background-color=#dcdcdc'>");
  response_message += F("<label for='intensFull' class='col-sm-4 col-form-label'>Brightness Program/Preview (0-254)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='intensFull' class='form-control' type='number' style='width:5em;' size='8' min='0' max='254' name='intensFull' value='");
  response_message += String(settings.intensFull);
  response_message += F("'>");
  response_message += F("</div></div>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label for='intensDim' class='col-sm-4 col-form-label'>Brightness Off (0-254)</label>");
  response_message += F("<div class='col-sm-8'>");
  response_message += F("<input id='intensDim' class='form-control' type='number' style='width:5em;' size='8' min='0' max='254' name='intensDim' value='");
  response_message += String(settings.intensDim);
  response_message += F("'>");
  response_message += F("</div></div>");

  httpServer.sendContent(response_message);

  response_message = F("<hr>");

  response_message += F("<div class='form-group row'>");
  response_message += F("<label class='col-sm-4 col-form-label'>&nbsp;</label>");
  response_message += F("<div class='col-sm-8'>&nbsp;</div></div>");

  response_message += F("<div class='form-group row' style='background-color:c6cdd2;'>");
  response_message += F("<label for='save' class='col-sm-4 col-form-label'> </label>");
  response_message += F("<div class='col-sm-8' style='text-align:center;'>");
  response_message += F("<input type='submit' value='SAVE' class='btn btn-primary'>");
  response_message += F("</div></div>");

  response_message += F("</td></tr></tbody></table>");

  response_message += F("</div></div></div>");
  response_message += F("</form>");

  httpServer.sendContent(response_message);

  response_message = F("<div data-role='content' class='row'>");
  response_message += F("&nbsp;</div>");

  response_message += F("<div data-role='content' class='row'>");

  response_message += F("<div class='col-md-6'>");
  response_message += F("<h2 style='border-radius: 10px 10px 0px 0px;margin-bottom:0px;background-color:#99c2ff;padding:2px;'>&nbsp; Information</h2>");
  response_message += F("<table class='table'><tbody  style='border-radius: 0px 0px 10px 10px;background-color:#d5dadd;'>");

  char ip[13];
  sprintf(ip, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  response_message += F("<tr><th>IP</th><td>");
  response_message += String(ip);
  response_message += F("</td><th>WiFi Signal Strength</th><td style='background-color:");

  String color = "#ff0000";

  if (WiFi.RSSI() > -80) {
    color = "#ff0000";
  }
  if (WiFi.RSSI() > -67) {
    color = "#ffff00";
  }
  if (WiFi.RSSI() > -50) {
    color = "#00ff00";
  }

  response_message += color;

  response_message += F(";'>");
  response_message +=String(WiFi.RSSI());
  response_message +=F(" dBm</td></tr>");

  response_message += F("<tr><th>MAC</th><td>");
  response_message += String(WiFi.macAddress());
  response_message += F("</td>");

  response_message += F("<th>WiFi AP</th>");
  if (apEnabled)
  {
    sprintf(ip, "%d.%d.%d.%d/%d/%d/%d", WiFi.softAPIP()[0], WiFi.softAPIP()[1], WiFi.softAPIP()[2], WiFi.softAPIP()[3],( WiFi.softAPIP()[3]+2),( WiFi.softAPIP()[3]+2),( WiFi.softAPIP()[3]+3));
    response_message += F("<td style='background-color:green;color:white;'>Active (");
    response_message += String(ip);
    response_message += F(")");
  }
  else
  {
    response_message += F("<td style='background-color:yellow;'>Inactive");
  }
  response_message += F("</td>");

  response_message += F("</tr>");

  response_message += F("<tr><th>Device Name</th><td>");
  response_message += String(deviceName);
  response_message += F("</td>");
  response_message += F("<th>vMix Status</th><td id='vmixstatus'");
  if (vmixcon == 1)
  {
    response_message += F("style='background-color:green;color:white;'>Connected");
  }
  else
  {
    response_message += F("style='background-color:red;color:white;'>Disconnected");
  }
  response_message += F("</td>");

  response_message += F("</tr>");

  response_message += F("<tr><th style='border-radius: 0px 0px 0px 10px;'>&nbsp;</th><td>&nbsp;</td>");
  response_message += F("<th>VISCA IP2SERIAL Status</th><td id='viscastatus'");
  if (udpstate > 0)
  {
    response_message += F("style='border-radius: 0px 0px 10px 0px;background-color:green;color:white;'>Running");
  }
  else
  {
    response_message += F("style='border-radius: 0px 0px 10px 0px;background-color:red;color:white;'>Not Running");
  }
  response_message += F("</td>");

  response_message += F("</tr>");

  response_message += F("</tbody></table>");
  response_message += F("</div>");

  response_message += F("<div class='col-md-6'>");
  response_message += F("<h2 style='border-radius: 10px 10px 0px 0px;margin-bottom:0px;background-color:#99c2ff;padding:2px;'>&nbsp; vMix Tally</h2>");

  response_message += F("<div id='tally' style='height:90%;'></div>");
  response_message += F("<script>document.getElementById('tally').innerHTML=\"<object type='text/html' data='/tally' style='height:100%;width:100%;'></object>\";</script>");

  response_message += F("</div>");

  response_message += F("</div>");

  response_message += F("<h4 style='border-radius: 10px 10px 10px 10px;background-color:#c8c8c8;text-align:center;margin-bottom:0px;'>vTally v");
  response_message += String(vers);
  response_message += F(" &nbsp;&nbsp;&nbsp; &copy; 2024 by <a href=https://www.vid-pro.de target=_new>VID-PRO</a></h4>");

  response_message += F("</body>");
  response_message += F("</html>");

  httpServer.sendContent(response_message);
  //httpServer.sendHeader("Connection", "close");
  //httpServer.send(200, "text/html", response_message.c_str());

  //Serial.print(F("| FreeHeap: "));
  //Serial.println(ESP.getFreeHeap(),DEC);
}

// Settings POST handler
void handleSave()
{
  bool doRestart = false;

  httpServer.sendHeader("Location", String("/"), true);
  httpServer.send(302, "text/plain", "Redirected to: /");

  if (httpServer.hasArg("ssid"))
  {
    if (httpServer.arg("ssid").length() <= SsidMaxLength)
    {
      httpServer.arg("ssid").toCharArray(settings.ssid, SsidMaxLength);
      doRestart = true;
    }
  }

  if (httpServer.hasArg("ssidpass"))
  {
    if (httpServer.arg("ssidpass").length() <= PassMaxLength)
    {
      httpServer.arg("ssidpass").toCharArray(settings.pass, PassMaxLength);
      doRestart = true;
    }
  }

  if (httpServer.hasArg("hostname"))
  {
    if (httpServer.arg("hostname").length() <= HostNameMaxLength)
    {
      httpServer.arg("hostname").toCharArray(settings.hostName, HostNameMaxLength);
      doRestart = true;
    }
  }

  if (httpServer.hasArg("inputnumber"))
  {
    if (httpServer.arg("inputnumber").toInt() > 0 and httpServer.arg("inputnumber").toInt() <= TallyNumberMaxValue)
    {
      settings.tallyNumber = httpServer.arg("inputnumber").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("intensFull"))
  {
    if (httpServer.arg("intensFull").toInt() >= 0 and httpServer.arg("intensFull").toInt() < 255)
    {
      settings.intensFull = httpServer.arg("intensFull").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("intensDim"))
  {
    if (httpServer.arg("intensDim").toInt() >= 0 and httpServer.arg("intensDim").toInt() < 255)
    {
      settings.intensDim = httpServer.arg("intensDim").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("prgred"))
  {
    if (httpServer.arg("prgred").toInt() >= 0 and httpServer.arg("prgred").toInt() < 255)
    {
      settings.prgred = httpServer.arg("prgred").toInt();
      doRestart = true;
    }
  }
  if (httpServer.hasArg("prggreen"))
  {
    if (httpServer.arg("prggreen").toInt() >= 0 and httpServer.arg("prggreen").toInt() < 255)
    {
      settings.prggreen = httpServer.arg("prggreen").toInt();
      doRestart = true;
    }
  }
  if (httpServer.hasArg("prgblue"))
  {
    if (httpServer.arg("prgblue").toInt() >= 0 and httpServer.arg("prgblue").toInt() < 255)
    {
      settings.prgblue = httpServer.arg("prgblue").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("prvred"))
  {
    if (httpServer.arg("prvred").toInt() >= 0 and httpServer.arg("prvred").toInt() < 255)
    {
      settings.prvred = httpServer.arg("prvred").toInt();
      doRestart = true;
    }
  }
  if (httpServer.hasArg("prvgreen"))
  {
    if (httpServer.arg("prvgreen").toInt() >= 0 and httpServer.arg("prvgreen").toInt() < 255)
    {
      settings.prvgreen = httpServer.arg("prvgreen").toInt();
      doRestart = true;
    }
  }
  if (httpServer.hasArg("prvblue"))
  {
    if (httpServer.arg("prvblue").toInt() >= 0 and httpServer.arg("prvblue").toInt() < 255)
    {
      settings.prvblue = httpServer.arg("prvblue").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("offred"))
  {
    if (httpServer.arg("offred").toInt() >= 0 and httpServer.arg("offred").toInt() < 255)
    {
      settings.offred = httpServer.arg("offred").toInt();
      doRestart = true;
    }
  }
  if (httpServer.hasArg("offgreen"))
  {
    if (httpServer.arg("offgreen").toInt() >= 0 and httpServer.arg("offgreen").toInt() < 255)
    {
      settings.offgreen = httpServer.arg("offgreen").toInt();
      doRestart = true;
    }
  }
  if (httpServer.hasArg("offblue"))
  {
    if (httpServer.arg("offblue").toInt() >= 0 and httpServer.arg("offblue").toInt() < 255)
    {
      settings.offblue = httpServer.arg("offblue").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("viscabaud"))
  {
    if (httpServer.arg("viscabaud").toInt() >= 2400 and httpServer.arg("viscabaud").toInt() <= 115200)
    {
      settings.viscabaud = httpServer.arg("viscabaud").toInt();
      doRestart = true;
    }
  }

  if (httpServer.hasArg("viscaport"))
  {
    if (httpServer.arg("viscaport").toInt() >= 1024 and httpServer.arg("viscaport").toInt() <= 65554)
    {
      settings.viscaport = httpServer.arg("viscaport").toInt();
      doRestart = true;
    }
  }

  if (doRestart == true)
  {
    restart();
  }
}

// Connect to WiFi
void connectToWifi()
{
  Serial.println(F("+--------------------+"));
  Serial.println(F("| Connecting to WiFi |"));
  Serial.println(F("+--------------------+"));
  Serial.print(F("| SSID            : "));
  Serial.println(settings.ssid);
  Serial.print(F("| Passphrase      : "));
  Serial.println(settings.pass);
  Serial.println(F("|"));

  int timeout = 15;

  WiFi.mode(WIFI_STA);
  WiFi.hostname(deviceName);
  WiFi.begin(settings.ssid, settings.pass);

  Serial.print(F("| Waiting for connection ."));
  while (WiFi.status() != WL_CONNECTED and timeout > 0)
  {
    delay(1000);
    timeout--;
    Serial.print(F("."));
  }

  Serial.println(F(""));

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println(F("| WiFi connected"));
    Serial.println(F("|"));
    Serial.print(F("| IP address      : "));
    Serial.print(WiFi.localIP());
    Serial.print("/");
    Serial.print(WiFi.localIP()[3]+1);
    Serial.print("/");
    Serial.print(WiFi.localIP()[3]+2);
    Serial.print("/");
    Serial.println(WiFi.localIP()[3]+3);
    Serial.print(F("| Device name     : "));
    Serial.println(deviceName);
    Serial.println(F("+---------------------"));
    Serial.println(F(""));
  }
  else
  {
    if (WiFi.status() == WL_IDLE_STATUS)
      Serial.println(F("| Idle"));
    else if (WiFi.status() == WL_NO_SSID_AVAIL)
      Serial.println(F("| No SSID Available"));
    else if (WiFi.status() == WL_SCAN_COMPLETED)
      Serial.println(F("| Scan Completed"));
    else if (WiFi.status() == WL_CONNECT_FAILED)
      Serial.println(F("| Connection Failed"));
    else if (WiFi.status() == WL_CONNECTION_LOST)
      Serial.println(F("| Connection Lost"));
    else if (WiFi.status() == WL_DISCONNECTED)
      Serial.println(F("| Disconnected"));
    else
      Serial.println(F("| Unknown Failure"));

    Serial.println(F("+---------------------"));
    apStart();
  }
}

// Connect to vMix instance
void connectTovMix()
{
  Serial.println(F("+--------------------+"));
  Serial.println(F("| Connecting to vMix |"));
  Serial.println(F("+--------------------+"));
  Serial.print(F("| Connecting to "));
  Serial.println(settings.hostName);

  if (client.connect(settings.hostName, port))
  {
    Serial.println(F("| Connected  to vMix"));

    Serial.println(F("+---------------------"));
    Serial.println(F(""));

    Serial.println(F("+--------------------+"));
    Serial.println(F("|  vMix Message Log  |"));
    Serial.println(F("+--------------------+"));

    tallySetOff(1);
    tallySetOff(2);
    tallySetOff(3);
    tallySetOff(4);

    // Subscribe to the tally events
    client.println(F("SUBSCRIBE TALLY"));
  }
  else
  {
    vmixcon = 0;
    currentState1 = '3';
    currentState2 = '3';
    currentState3 = '3';
    currentState4 = '3';
    Serial.println(F("| vMix Server not found!"));
    Serial.println(F("+---------------------"));
    Serial.println(F(""));
  }
}

void banner()
{
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("+---------------------+"));
  Serial.print(F("|   vTally    v"));
  Serial.print(String(vers));
  Serial.println(F("   |"));
  Serial.println(F("| (c)2024 by  VID-PRO |"));
  Serial.println(F("+---------------------+"));
  Serial.println(F(""));
}

void start()
{
  tallySetConnecting();

  loadSettings();

  Serial.println(F(""));
  sprintf(deviceName, "vTally_%d", settings.tallyNumber);
  sprintf(apPass, "%s%s", deviceName, "_pwd");

  connectToWifi();

  if (WiFi.status() == WL_CONNECTED)
  {
    viscaSerial1.begin(settings.viscabaud, SWSERIAL_8N1, rxpin1, txpin1, false, 200);
    viscaSerial2.begin(settings.viscabaud, SWSERIAL_8N1, rxpin2, txpin2, false, 200);
    viscaSerial3.begin(settings.viscabaud, SWSERIAL_8N1, rxpin3, txpin3, false, 200);
    viscaSerial4.begin(settings.viscabaud, SWSERIAL_8N1, rxpin4, txpin4, false, 200);
    start_visca();
    visca_power(true);
    connectTovMix();
  }
}

double zoomcurve(int v)
{
  return ZOOMMULT * pow(v, ZOOMEXP);
}

double ptzcurve(int v)
{
  return PTZMULT * pow(v, PTZEXP);
}

void debug(char c)
{
  Serial.print(c);
}

void debug(int n, int base)
{
  Serial.print(n, base);
  Serial.print(F(""));
}

void debug(uint8_t *buf, int len)
{
  for (uint8_t i = 0; i < len; i++)
  {
    uint8_t elem = buf[i];
    debug(elem, HEX);
  }
}

void send_bytes1(uint8_t *b, int len)
{
  for (int i = 0; i < len; i++)
  {
    uint8_t elem = b[i];
    viscaSerial1.println(elem);
  }
}

void send_bytes2(uint8_t *b, int len)
{
  for (int i = 0; i < len; i++)
  {
    uint8_t elem = b[i];
    viscaSerial2.println(elem);
  }
}

void send_bytes3(uint8_t *b, int len)
{
  for (int i = 0; i < len; i++)
  {
    uint8_t elem = b[i];
    viscaSerial3.println(elem);
  }
}

void send_bytes4(uint8_t *b, int len)
{
  for (int i = 0; i < len; i++)
  {
    uint8_t elem = b[i];
    viscaSerial4.println(elem);
  }
}

void visca_power(bool turnon)
{
  if (turnon)
  {
    Serial.println(F("| VISCA Power On"));
    send_visca1(addr_set);
    send_visca2(addr_set);
    send_visca3(addr_set);
    send_visca4(addr_set);
    delay(500);
    send_visca1(pwr_on);
    send_visca2(pwr_on);
    send_visca3(pwr_on);
    send_visca4(pwr_on);
    delay(2000);
    send_visca1(if_clear);
    send_visca2(if_clear);
    send_visca3(if_clear);
    send_visca4(if_clear);
    delay(500);
    send_visca1(irOff);
    send_visca2(irOff);
    send_visca3(irOff);
    send_visca4(irOff);
  }
  else
  {
    Serial.println(F("| VISCA Power Off"));
    send_visca1(if_clear);
    send_visca2(if_clear);
    send_visca3(if_clear);
    send_visca4(if_clear);
    delay(2000);
    send_visca1(pwr_off);
    send_visca2(pwr_off);
    send_visca3(pwr_off);
    send_visca4(pwr_off);
  }
  pwr_is_on = turnon;
}

void handle_visca(int port, uint8_t *buf, size_t len)
{
  uint8_t modified[18];
  size_t lastelement = 0;

  if ((buf[0] == 0x01 && buf[1] == 0x00 && buf[2] == 0x00)) { // && buf[3] == 0x09) || (buf[0] == 0x01 && buf[1] == 0x00 && buf[2] == 0x00 && buf[3] == 0x06)) {
    int j = 0;
    for (int i = 8; (i < len && i < 18); i++)
    {
      modified[j] = buf[i];
      lastelement = j;
      j++;
    }
    //modified[0] = 0x01;
  } else {
    for (int i = 0; (i < len && i < 18); i++)
    {
      modified[i] = buf[i];
      lastelement = i;
    }
  }

  // is this a PTZ?
  if (modified[1] == 0x01 && modified[2] == 0x06 && modified[3] == 0x01)
  {
    Serial.println(F("| PTZ CONTROL DETECTED"));
    //modified[4] = (int)ptzcurve(modified[4]);
    //modified[5] = (int)ptzcurve(modified[5]);
  }

  // is this ZOOM?
  if (modified[1] == 0x01 && modified[2] == 0x04 && modified[3] == 0x07)
  {
    Serial.println(F("| ZOOM CONTROL DETECTED"));
    //int zoomspeed = modified[4] & 0b00001111;
    //zoomspeed = (int)zoomcurve(zoomspeed);
    //int zoomval = (modified[4] & 0b11110000) + zoomspeed;
    //modified[4] = zoomval;
  }

  if (modified[1] == 0x01 && modified[2] == 0x04 && modified[3] == 0x08)
  {
    Serial.println(F("| FOCUS CONTROL DETECTED"));
  }

  if (port == 1)
  {
    viscaSerial1.write(modified, lastelement + 1);
    //Serial.println(F("| VISCA IP: Send ACK"));
    udp.writeTo(modified, lastelement + 1, lastclientip, lastclientport);
  }
  else if (port == 2)
  {
    viscaSerial2.write(modified, lastelement + 1);
    //Serial.println(F("| VISCA IP: Send ACK"));
    udp.writeTo(modified, lastelement + 1, lastclientip, lastclientport);
  } if (port == 3)
  {
    viscaSerial3.write(modified, lastelement + 1);
    //Serial.println(F("| VISCA IP: Send ACK"));
    udp.writeTo(modified, lastelement + 1, lastclientip, lastclientport);
  } if (port == 4)
  {
    viscaSerial4.write(modified, lastelement + 1);
    //Serial.println(F("| VISCA IP: Send ACK"));
    udp.writeTo(modified, lastelement + 1, lastclientip, lastclientport);
  }

  /*
    if (data_state) {
      digitalWrite(LED_DATA, data_state);
      data_state = false;
    } else {
      digitalWrite(LED_DATA, data_state);
      data_state = true;
    }
  */
}

void start_visca()
{
  Serial.println(F("+--------------------+"));
  Serial.println(F("|    VISCA server    |"));
  Serial.println(F("+--------------------+"));
  Serial.print(F("| starting 4 Servers on UDP port "));
  Serial.println(settings.viscaport);
  //udp.close(); // will close only if needed
  
  if (udp.listenMulticast(WiFi.localIP(), settings.viscaport))
  {
    udpstate = 1;
    Serial.println(F("| Server 1 is Running!"));
  }
  if (udp.listenMulticast(IPAddress(WiFi.localIP()[0],WiFi.localIP()[1],WiFi.localIP()[2],(WiFi.localIP()[3]+1)), settings.viscaport))
  {
    udpstate++;
    Serial.println(F("| Server 2 is Running!"));
  }
  if (udp.listenMulticast(IPAddress(WiFi.localIP()[0],WiFi.localIP()[1],WiFi.localIP()[2],(WiFi.localIP()[3]+2)), settings.viscaport))
  {
    udpstate++;
    Serial.println(F("| Server 3 is Running!"));
  }
  if (udp.listenMulticast(IPAddress(WiFi.localIP()[0],WiFi.localIP()[1],WiFi.localIP()[2],(WiFi.localIP()[3]+3)), settings.viscaport))
  {
    udpstate++;
    Serial.println(F("| Server 4 is Running!"));
  }
  if (udpstate > 0)
  {
    udp.onPacket([](AsyncUDPPacket packet) {
      // debug(packet);
      lastclientip = packet.remoteIP();
      lastclientport = packet.remotePort();

      Serial.print(F("| Type of UDP datagram: "));
      Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                   : "Unicast");
      Serial.print(F(", Sender: "));
      Serial.print(lastclientip);
      Serial.print(F(":"));
      Serial.print(lastclientport);
      Serial.print(F(", Receiver: "));
      Serial.print(packet.localIP());
      Serial.print(F(":"));
      Serial.print(packet.localPort());
      Serial.print(F(", Message length: "));
      Serial.print(packet.length());
      Serial.print(F(" Payload (hex):"));
      debug(packet.data(), packet.length());
      Serial.println(F(""));

      if (packet.isMulticast() & packet.localIP() == WiFi.localIP()) {
        handle_visca(1, packet.data(), packet.length());
      } else if (packet.isMulticast() & packet.localIP() == IPAddress(WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] + 1)) {
        handle_visca(2, packet.data(), packet.length());
      } else if (packet.isMulticast() & packet.localIP() == IPAddress(WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] + 21)) {
        handle_visca(3, packet.data(), packet.length());
      } else if (packet.isMulticast() & packet.localIP() == IPAddress(WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3] + 3)) {
        handle_visca(4, packet.data(), packet.length());
      }
    });
    Serial.print(F("| "));
    Serial.print(udpstate);
    Serial.println(F("/4 Servers started"));
  }
  else
  {
    Serial.print(F("| "));
    Serial.print(4 - udpstate);
    Serial.println(F("/4 Servers failed to start"));
    udpstate = 0;
  }
  Serial.println(F("+---------------------"));
  Serial.println(F(""));
}

void check_serial()
{
  int available1 = viscaSerial1.available();
  while (available1 > 0)
  {
    Serial.println(F("| VISCA SER1 -> IP"));
    int actual = viscaSerial1.readBytesUntil(0xff, lastser_in, available1); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);

    available1 = viscaSerial1.available();
  }
  int available2 = viscaSerial2.available();
  while (available2 > 0)
  {
    Serial.println(F("| VISCA SER2 -> IP"));
    int actual = viscaSerial2.readBytesUntil(0xff, lastser_in, available2); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);

    available2 = viscaSerial2.available();
  }
  int available3 = viscaSerial3.available();
  while (available3 > 0)
  {
    Serial.println(F("| VISCA SER3 -> IP"));
    int actual = viscaSerial3.readBytesUntil(0xff, lastser_in, available3); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);

    int available3 = viscaSerial3.available();
  }
  int available4 = viscaSerial4.available();
  while (available4 > 0)
  {
    Serial.println(F("| VISCA SE4R -> IP"));
    int actual = viscaSerial4.readBytesUntil(0xff, lastser_in, available4); // does not include the terminator char
    if (actual < 16)
    {
      lastser_in[actual] = 0xff;
      actual++;
    }
    debug(lastser_in, actual);
    if (lastclientport > 0)
      udp.writeTo(lastser_in, actual, lastclientip, lastclientport);

    available4 = viscaSerial4.available();
  }
}

void setup()
{
  Serial.begin(9600);
  EEPROM.begin(512);
  SPIFFS.begin();
  leds.begin();
  leds.setBrightness(50);
  leds.show();

  viscaSerial1.begin(9600, SWSERIAL_8N1, rxpin1, txpin1, false);
  viscaSerial2.begin(9600, SWSERIAL_8N1, rxpin2, txpin2, false);
  viscaSerial3.begin(9600, SWSERIAL_8N1, rxpin3, txpin3, false);
  viscaSerial4.begin(9600, SWSERIAL_8N1, rxpin4, txpin4, false);

  //pinMode(LED_DATA, OUTPUT);

  httpServer.on("/", HTTP_GET, rootPageHandler);
  httpServer.on("/save", HTTP_POST, handleSave);
  httpServer.on("/tally", HTTP_GET, tallyPageHandler);
  httpServer.on("/zend", []() {
    //httpServer.sendHeader("Connection", "close");
    httpServer.sendHeader("Cache-Control", "no-cache");
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");

    if (currentState1 != oldState1 || currentState2 != oldState2 || currentState3 != oldState3 || currentState4 != oldState4 ) {
      httpServer.send(200, "text/event-stream", "event: message\ndata: refresh" + String(vmixcon) + "\nretry: 500\n\n");
      oldState1 = currentState1;
      oldState2 = currentState2;
      oldState3 = currentState3;
      oldState4 = currentState4;
    } else {
      httpServer.send(200, "text/event-stream", "event: message\ndata: norefresh\nretry: 500\n\n");
    }

    //Serial.print(F("| FreeHeap: "));
    //Serial.println(ESP.getFreeHeap(),DEC);
  });
  httpServer.serveStatic("/", SPIFFS, "/", "max-age=315360000");
  httpServer.begin();

  banner();

  start();
}

void loop()
{
  httpServer.handleClient();

  while (client.available())
  {
    String data = client.readStringUntil('\r'); //\r\n
  }
  if (!client.connected() && !apEnabled && millis() > lastCheck + interval)
  {
    tallySetConnecting();

    client.stop();

    connectTovMix();
    lastCheck = millis();
  }

  check_serial();
}
