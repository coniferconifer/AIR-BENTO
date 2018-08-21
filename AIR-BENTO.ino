/**
   MQTT transmitter from MH-Z14A CO2 sensor , GP2Y1014AU0F/GP2Y1010AU0F dust sensor and
   pressure sensor BMP180 for ESP32
   . connects ESP32 to NodeRED or Thingsboard via WiFi / MQTT server at 1883 port
   or broadcast co2 and dust level as BLE name (option)


   Author: coniferconifer
   License: Apache License v2
   Aug 21,2018
       disabeABCcommand() is commented out and enableABCcommand() is ON.
   Aug 4,2018
       watch dog timer for loop monitoring
   May 14,2018
      I2C SHT21 humidity sensor is supported by #define HUMIDITY
   May 13,2018
      ESP32 Deep sleep mode is implemented , but comment outed. try #define DEEPSLEEP
      ESP32 sleeps 30sec after data display is done once.
   May 12,2018
      intermittent WiFi connection
   May 10,2018
      uniq clientId for MQTT is supported
      http://public.dhe.ibm.com/software/dw/webservices/ws-mqtt/mqtt-v3r1.html
      "The first UTF-encoded string. The Client Identifier (Client ID) is between 1 and 23 characters long,
      and uniquely identifies the client to the server. It must be unique across all clients connecting to
      a single server, and is the key in handling Message IDs messages with QoS levels 1 and 2. If the
      Client ID contains more than 23 characters, the server responds to the CONNECT message with a
      CONNACK return code 2: Identifier Rejected."
      "If a client with the same Client ID is already connected to the server, the "older" client
      must be disconnected by the server before completing the CONNECT flow of the new client."
   May 4,2018
     1) MQTT post begins after WARMUPTIME msec elapsed to avoid unusual data
       from MH-Z14A, mostly 400 or 410ppm after power on.
     2) averaged pressure is used to send it to MQTT server
     3) AIR BENTO dose not publish data to MQTT server if data integrity is lost.
       such as CO2 is higher than 5000 or less than 300, etc.

   May 3,2018
     If there is a communication error for MH-Z14A , then read out remaining data from MH-Z14A.
   May 2,2018
     BLE is optional due to compiled flash memory size exceeds 100%
   April 30,2018
     BMP180 air pressure sensor is supported.

   1. supports multiple WiFi access points and multiple MQTT servers for each access point.
   2. BLE advertizing
      when there is no WiFi access points , then simple BLE is used to advertize sensor values.
      by using BLE name.(option)
   3. supports mono OLED display SSD1306

   note: I have no idea to run WiFi and BLE simultaneously, so if WiFi is not accessible then BLE is ON
   at boot time.
*/
// References
// CO2 sensor:
// http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf
// http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z14%20CO2%20V2.4.pdf
// https://revspace.nl/MHZ19
//
// SSD1306 library from https://github.com/LilyGO/ESP32-OLED0.96-ssd1306
// http://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/spi_master.html
//
// PubSubClient from https://github.com/knolleary/pubsubclient
// MQTT server with visiualization tools https://thingsboard.io/
// public NTP server https://developers.google.com/time/
//
// Dust sensor:
// https://media.digikey.com/pdf/Data%20Sheets/Sharp%20PDFs/GP2Y1014AU0F_Spec_2-6-15.pdf
// technical detail of GP2Y10' analog front end  http://www.ti.com/lit/ug/tidub65c/tidub65c.pdf
//
// BMP180 air pressure sensor with temperature
// https://github.com/adafruit/Adafruit-BMP085-Library
//
// trouble shooting:
// Why GPIO33 is used for ADC
// referer to "ADC2 Channel cannot be used when WiFi is in use #440"
// https://github.com/espressif/arduino-esp32/issues/440
#define VERSION "20180821"
#include <WiFi.h>
#include <PubSubClient.h>

#define VERBOSE //print messages to console
#define OLED  //Use SPI SSD1603 display
#define BMP180
#define HUMIDITY //SHT21

//#define BLE   // by using the latest Arduino core for ESP32 with BLE ,
//               the compiled flash memory reached to 104%.
//               if #define BLE is not used , it is 45%.
//               Use "Partition Scheme" in "Tools" menu in Arduino IDE and select "No OTA(large APP)" then this
//               program will fit in the space for #define BLE
// hardware configuration
// SDA ---GPIO21 , SCK ---GPIO22 for BMP180
// CO2 sensor TX(19) --- GPIO 16 (RX) , RX --- GPIO 17(TX)
// SSD1603 RES --- GPIO0,DC/MISO --- GPIO2, CS --- GPIO36, D1/MOSI --- GPIO23, D0(SCK) --- GPIO18
//
//
HardwareSerial Co2Sensor(2);
//HardwareSerial Serial1(1); //RX as GPIO 9 TX as GPIO 10
#ifdef OLED
#include "SSD1306Spi.h"
#endif
// Initialize the OLED display using SPI
#ifdef HUMIDITY
#include <SHT21.h>
SHT21 SHT21;
#endif

#ifdef OLED
#ifdef BMP180
SSD1306Spi        display(0, 2, 36); //GPIO0(RES),GPIO2(DC/MISO),GPIO36(CS), GPIO23(D1(MOSI)),GPIO18(D0(SCK))
#define GP2YLED 39 //GPIO22 is used to drive LED pin of GP2Y by NPN transister 2N5551
#else
SSD1306Spi        display(0, 2, 21); //GPIO0(RES),GPIO2(DC/MISO),GPI21(CS), GPIO23(D1(MOSI)),GPIO18(D0(SCK))
#endif
#endif
#define GP2YLED 22 //GPIO22 is used to drive LED pin of GP2Y by NPN transister 2N5551
#define GPIO33 33 // Analog digital converter works when WiFi is in use
// do not use ADC2* pins with WiFi ON
// referer to "ADC2 Channel cannot be used when WiFi is in use #440"
// https://github.com/espressif/arduino-esp32/issues/440
//#define WIFI_POWERSAVE // intermittent WiFi connection reduces power consumpution from 0.18A to 0.08 A on average
// as of June 3,2018 , recompiled version with WIFI_POWERSAVE does not send data to MQTT server
//-------- Customise these values -----------
#include "credentials.h"
// credentials.h should include #define WIFI_SSID XXXXXX
//                        #define WIFI_PASS YYYYYY
//                       home , office
//#define BLETEST
#ifdef BLETEST
#define WIFI_SSID3 "testSSID" //non existing SSID to turn on BLE mode
char* ssidArray[] = { WIFI_SSID3, WIFI_SSID3 , WIFI_SSID3};
#else
char* ssidArray[] = { WIFI_SSID , WIFI_SSID1, WIFI_SSID2};
#endif

char* passwordArray[] = {WIFI_PASS, WIFI_PASS1, WIFI_PASS2};
char* tokenArray[] = { TOKEN , TOKEN1, TOKEN2};
char* serverArray[] = {SERVER, SERVER1, SERVER2};
#define MQTTRETRY 1
#define DEVICE_TYPE "ESP32" // 
String clientId = DEVICE_TYPE ; //uniq clientID will be generated from MAC
char topic[] = "v1/devices/me/telemetry"; //for Thingsboard
#define MQTTPORT 2883 //for Thingsboard or MQTT server
#define WARMUPTIME 90000 // 60sec -> 90sec
#define TIMEZONE 9 //in Japan
#define NTP1 "time.google.com"
#define NTP2 "ntp.nict.jp"
#define NTP3 "ntp.jst.mfeed.ad.jp"

//https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Timer/WatchdogTimer/WatchdogTimer.ino#include "esp_system.h"

const int wdtTimeout = 180000; //time in ms to trigger the watchdog
hw_timer_t *timer = NULL;

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart_noos();
}

/* https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/DeepSleep/ExternalWakeUp/ExternalWakeUp.ino
  Method to print the reason by which ESP32
  has been awaken from sleep
*/
void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case 1  : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case 2  : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case 3  : Serial.println("Wakeup caused by timer"); break;
    case 4  : Serial.println("Wakeup caused by touchpad"); break;
    case 5  : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
  }
}


#define CELSIUS
#ifdef BLE
#include "SimpleBLE.h"
SimpleBLE ble;
#endif

//long int etime = 0;

float mhtempC = 0.0; //temperature in C reported from MH-Z14A
float mhtempF = 0.0; //temperature in F reported from MH-Z14A
////#include "esp_deep_sleep.h"


WiFiClient wifiClient;
PubSubClient client(serverArray[0], MQTTPORT, wifiClient);
// index of WiFi access point
int AP = -1; // access point is not yet found , -1 means not WiFi , but Simple BLE mode

#ifdef BMP180
#include <Adafruit_BMP085.h>
#include <Wire.h>
Adafruit_BMP085 pressure;
#endif

//#define DEEPSLEEP  // experimental deep sleep while waiting

#ifdef  DEEPSLEEP
#define WAITLOOP 1
#define SLEEPSEC 30
#else
#define WAITLOOP 5  // loop times x (co2,temp,pressure,dust)x DISPLAYINTERVAL(in msec) for each OLED display
#endif

void setup() {
  pinMode(GP2YLED, OUTPUT); // do not use board LED for NodeMCU ESP32 board
  digitalWrite(GP2YLED, LOW);
  Co2Sensor.begin(9600); // communication with MH-Z14A

  Serial.begin(115200);
  print_wakeup_reason();

#ifdef VERBOSE
  Serial.println();
  Serial.print("AIR BENTO "); Serial.println( VERSION);
  Serial.println("ESP32-MH-Z14A-GP2Y10-BMP180-SHT21-WiFi-MQTT or BLE");
#ifdef BLE
  Serial.println("supports multiple WiFi access points or BLE");
#else
  Serial.println("supports multiple WiFi access points");
#endif
#endif
#ifdef OLED
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
#ifndef DEEPSLEEP
  display.drawString(0, 0, "AIR BENTO");
  display.drawString(0, 32, VERSION);
  display.display();
  delay(1000);

  drawProgressBarMain(); // just for demonstration
  delay(1000);
  display.clear();
  display.display();
#endif
  // generate uniq clientId
  uint64_t chipid;
  chipid = ESP.getEfuseMac();
  clientId += "-";
  //  clientId += DEVICE_ID;
  clientId += String((uint32_t)chipid, HEX);
  Serial.println("clientId :" + clientId);

#endif
  AP = initWiFi();
  if ( AP != -1) {  // found  WiFi AP
    client.setClient(wifiClient);
    client.setServer(serverArray[AP], MQTTPORT); // MQTT server for NodeRED or MQTT by Thingsboarxd
  }


#ifdef PREHEATING
  int i;
  for (i = 0; i < 180; i++) {
#ifdef VERBOSE
    Serial.print(".");
#endif
    delay(1000);
  }
#endif

  /*
    #ifdef DEEPSLEEP
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_OFF);
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_MAX, ESP_PD_OPTION_OFF);
    #endif
  */
  //  disableABCcommand(); //I'm not sure this works for MH-Z14A
  //  resetCO2(); // if co2 sensor should be calibrated, then leave AIR BENTO outdoor for at least 30min
                  // and run resetCO2(); once
  enableABCcommand();
  initDust();

#ifdef BMP180
  if (pressure.begin()) {
    Serial.println("BMP180 init success");
    float P;
    P = pressure.readPressure();
    Serial.println(P / 100);
  } else {
    Serial.println("BMP180 init fail\n\n");
  }
#endif
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer);                          //enable interrupt
}
boolean mqttflag = false;

void loop() {

#define DISPLAYINTERVAL 2000
  timerWrite(timer, 0); //reset timer (feed watchdog)
  long int co2; long int d; float P; float T; float H;
  d = 0; co2 = 0; P = 0.0; T = 0.0;
  long int co2_ave = 0; long int d_ave = 0; float P_ave = 0.0; float T_ave = 0.0; float H_ave = 0.0;
  int i;

  boolean dataIntegrity;
  dataIntegrity = HIGH;
#define MAXCO2 5000

#ifdef DEEPSLEEP
#define MINCO2 430
#else
#define MINCO2 390
#endif
  for (i = 0; i < WAITLOOP; i++) {
    co2 = getCO2();
    //    co2 = 9999; // test data for dataIntegrity=LOW;
    //    co2 = 9; // test data for dataIntegrity=LOW;
    if ((co2 > MAXCO2) || (co2 < MINCO2)) dataIntegrity = LOW;
    co2_ave = co2_ave + co2;

#ifdef OLED
    displayCO2(co2);
#endif
    delay(DISPLAYINTERVAL);
    d = getDust(); d_ave = d_ave + d;
#ifdef OLED
    displayDust(d);
#endif
    delay(DISPLAYINTERVAL);
    T = getTemperature(); T_ave = T_ave + T;
#ifdef CELSIUS
    if ( (T > 50.0) ) dataIntegrity = LOW; // it is too hot , indicating something wrong in reading the sensor
#else                                     // dataIntegrity flag is set to LOW , will prevent MQTT server to 
    // get unreliable data
    // 122F == 50C
    if ( (T > 122.0) ) dataIntegrity = LOW;
#endif
#ifdef OLED
    displayTemp(T);
#endif
    delay(DISPLAYINTERVAL);
#ifdef HUMIDITY
    H = GetHumidity(); H_ave = H_ave + H;
    if ( (H < 0.0) || (H > 100.0) ) dataIntegrity = LOW;
#ifdef OLED
    displayHumidity(H);
#endif
    delay(DISPLAYINTERVAL);
#endif

#ifdef BMP180
    P = pressure.readPressure() / 100.0;
    //    P =1200;// test data for dataIntegrity=LOW;
    //    P = 0;// test data for dataIntegrity=LOW;
    if ( (P < 500.0) || (P > 1100.0) ) dataIntegrity = LOW;
    P_ave = P_ave + P;
#ifdef OLED
    displayPressure(P);
#endif
    delay(DISPLAYINTERVAL);
#else
    P = 1000.0; P_ave = P_ave + P;
#endif
  }
  co2 = co2_ave / WAITLOOP;
  d = d_ave / WAITLOOP;
  T = T_ave / WAITLOOP;
  P = P_ave / WAITLOOP;
  H = H_ave / WAITLOOP;
#ifdef OLED
  displayCO2(co2);
#endif

  if ( AP == -1 ) {
#ifdef WIFI_POWERSAVE
    Serial.println("WiFi turn off");
    WiFi.mode(WIFI_OFF);
#endif
#ifdef OLED
#ifdef BLE
    displayBLE();
#endif
#endif
#ifdef BLE
    simpleBLE(co2, d); //annouce CO2 as BLE name and wait for WAITLOOP
#endif

  } else {
#ifdef WIFI_POWERSAVE
    initWiFi_retry();
#endif
    if (dataIntegrity == HIGH) {
      publishToMQTT(co2, P, T, H, d );
    } else {
      Serial.println("Data Integrity Error occurred. MQTT publish is not done.");
    }
  } // end of WiFi mode

}


void publishToMQTT(int co2, float P, float T, float H, float d) {
  // WiFi mode
  String payload = "{";
  payload += "\"temperature\":"; payload += String( T , 1); payload += ",";
#ifdef HUMIDITY
  payload += "\"humidity\":"; payload += String(H, 1) ; payload += ",";
#endif
  payload += "\"co2\":"; payload += co2; payload += ",";
#ifdef BMP180
  payload += "\"press\":"; payload += String(P, 2); payload += ",";
#endif
  payload += "\"dust\":"; payload += String(d, 0); payload += ",";
  payload += "\"rssi\":"; payload += WiFi.RSSI();
  //payload += ",";
  //  payload += "\"date\":"; payload += s;
  payload += "}";

  // dont send too long string to MQTT server
  // max 128byte
#ifndef DEEPSLEEP
#ifdef OLED
  displayWiFiAP(AP, WiFi.RSSI());
  displayWiFi();
  delay(DISPLAYINTERVAL);
  displayServer(AP);
  delay(DISPLAYINTERVAL);
#endif
#endif
#ifdef VERBOSE
  Serial.print("Sending payload: "); Serial.println(payload);
  Serial.print("AP = "); Serial.println(AP);
#endif

  //    if (!client.connected()) {
  //      initWiFi_retry();
  //    }
#ifdef VERBOSE
  Serial.print("Reconnecting client to "); Serial.println(serverArray[AP]);
#endif
  int mqttloop = 0;
  while (1) { // for thingsboard MQTT server
    mqttflag = client.connect(clientId.c_str(),  tokenArray[AP], NULL);
    if (mqttflag == true) break;
    Serial.print("-"); delay(500);
    mqttloop++;
    if (mqttloop > MQTTRETRY) { //there may be something wrong
      mqttflag = false;
      initWiFi_retry();
      // ESP.restart();
      break;
    }
  }

  if (mqttflag == true) {
#ifndef DEEPSLEEP
    if (millis() > WARMUPTIME) { // check if CO2 sensor get enough warm up time
#else
    if (true) { //in case of DEEP SLEEP , MQTT publish any time
#endif
      if (client.publish(topic, (char*) payload.c_str())) {
#ifdef VERBOSE
        Serial.println("Publish ok");
#endif
      } else {
#ifdef VERBOSE
        Serial.println("Publish failed");
#endif
      }
    } // WARMUP decision end
  } else {
#ifdef VERBOSE
    Serial.println("unable to connect to MQTT server");
#endif
  }


#ifdef WIFI_POWERSAVE
#ifdef VERBOSE
  Serial.println("WiFi turn off");
#endif
  WiFi.mode(WIFI_OFF);
#endif

#ifdef DEEPSLEEP
  Serial.print("Deep Sleep: "); Serial.println(SLEEPSEC);
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEPSEC * (uint64_t)uS_TO_S_FACTOR);
  // esp_deep_sleep_enable_timer_wakeup(SLEEPSEC * 1000 * 1000); // wakeup(restart) after WAITLOOP msecs
  //  esp_deep_sleep_start();
#else
  //  int i;
  //  for (i = 0; i < WAITLOOP; i++) {
  //    delay(1000);
  //  }
#endif
}

#ifdef OLED
void displayTime() {
  struct tm timeInfo;
  if ( AP == -1) return; // BLE mode , do nothing
  getLocalTime(&timeInfo);
  Serial.printf("Date: %04d/%02d/%02d %02d:%02d:%02d , ",
                timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
                timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
  char s[20];
  sprintf(s, "%04d/%02d/%02d %02d:%02d:%02d",
          timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
          timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);

  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, s );
  display.display();

}
#ifdef BLE
void displayBLE() {
  //  display.clear();
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(127, 0, "BLE mode" );
  display.display();
}
#endif
void displayServer(int i) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "MQTT server at" );
  display.drawString(0, 20, (String)serverArray[i] );
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(127, 40, "port=" + (String)MQTTPORT);
  display.display();
}
void displayWiFiAP(int i, int rssi) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "...connected to" );
  display.drawString(0, 20, (String)ssidArray[i] );
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(127, 40, (String)rssi + "dBm");
  display.display();
}
void displayWiFi() {
  //  display.clear();
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(127, 0, "WiFi" );
  display.display();
}
void displayWiFioff() {
  //  display.clear();
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(127, 0, "WiFi off" );
  display.display();
}
void displayWiFiSearch() {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(127, 0, "WiFi AP Search" );
  display.display();
}
void drawProgressBarMain() {
  int counter;
  for (counter = 0; counter < 100; counter++) {
    drawProgressBar(counter) ;
    delay(10);
  }
}
void drawProgressBar(int progress) {
  //  int progress = counter % 100;
  // draw the progress bar
  display.setFont(ArialMT_Plain_24);
  display.clear(); display.display();
  display.drawProgressBar(0, 50, 120, 10, progress);
  // draw the percentage as String
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(64, 15, String(progress) + "%");
  display.display();

}
void displayDust(int d) {

  display.clear(); display.display();
  displayTime();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  if (d >= 100) {
    display.drawString(0, 10, "Dust: hell!" );
  } else {
    if (d >= 50) {
      display.drawString(0, 10, "Dust: bad!" );
    } else {
      if (d > 20) {
        display.drawString(0, 10, "Dust: bad" );
      } else {
        display.drawString(0, 10, "Dust: good" );
      }
    }
  }

  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  display.drawString(127, 36, "   " + (String)d + "ug/m3");

  display.display();

#ifdef VERBOSE
  Serial.print("averaged dust(ug/m3) = "); Serial.println(d);
#endif
}
void displayCO2(int co2) {

  display.clear(); display.display();
  displayTime();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);
  if (co2 >= 1500) {
    display.drawString(0, 8, "CO2: bad!" );
  } else {
    if (co2 >= 1000) {
      display.drawString(0, 10, "CO2: vent" );
    } else {
      if (co2 > 390) {
        display.drawString(0, 10, "CO2: good" );
      } else {
        display.drawString(0, 10, "CO2:     " );
      }
    }
  }
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  if (co2 > 350 ) {
    display.drawString(127, 36,  (String)co2 + "ppm");
  } else {
    display.drawString(127, 36, "warming up");
  }
  display.display();
  //    Serial.print("CO2 level(ppm): " );
  Serial.println(co2);
}

void displayTemp(float temp) {

  display.clear(); display.display();
  displayTime();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);

  display.drawString(0, 8, "Temp.:" );
  display.setTextAlignment(TEXT_ALIGN_RIGHT);
  int itemp;
  itemp = (int)temp;
#ifdef CELSIUS
  display.drawString(127, 36,  (String)itemp + "C");
#else
  display.drawString(127, 36,  (String)itemp + "F");
#endif

  display.display();
  Serial.println(itemp);
}
#ifdef BMP180
void displayPressure(float P) {

  display.clear(); display.display();
  displayTime();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);

  display.drawString(0, 8, "Press.:" );
  display.setTextAlignment(TEXT_ALIGN_RIGHT);

  display.drawString(127, 36,  (String)P + "hPa");
  display.display();
  Serial.println(P);
}
#ifdef HUMIDITY
void displayHumidity(float H) {

  display.clear(); display.display();
  displayTime();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_24);

  display.drawString(0, 8, "Humid.:" );
  display.setTextAlignment(TEXT_ALIGN_RIGHT);

  display.drawString(127, 36,  (String)H + "%RH");
  display.display();
  Serial.println(H);
}
#endif

#endif

#endif

#ifdef BLE
void simpleBLE(int co2, int d) {
#ifdef VERBOSE
  Serial.println("Simple BLE mode");
#endif
  String out = "CO2(ppm) ";
  out += (String)co2;
  out += ",dust ";
  out += (String)d;
  out += "(ug/m3)";
  ble.begin(out);
  delay(WAITLOOP * 1000);

}
#endif

#define MAX_TRY 15
int initWiFi() {
  int i ;
  int numaccesspt = (sizeof(ssidArray) / sizeof((ssidArray)[0]));
#ifdef OLED
  displayWiFiSearch() ;
#endif
#ifdef VERBOSE
  Serial.print("Number of Access Point = "); Serial.println(numaccesspt);
#endif
  for (i = 0;  i < numaccesspt; i++) {
#ifdef VERBOSE
    Serial.print("WiFi connecting to "); Serial.println(ssidArray[i]);
#endif
    WiFi.mode(WIFI_OFF);
    WiFi.begin(ssidArray[i], passwordArray[i]);

    int j;
    for (j = 0; j < MAX_TRY; j++) {
      if (WiFi.status() == WL_CONNECTED) {

        int rssi = WiFi.RSSI();
        Serial.printf("RSSI= %d\n", rssi);
        //        displayWiFiAP(i,  rssi);
        //        delay(3000);
        configTime(TIMEZONE * 3600L, 0,  NTP1, NTP2, NTP3);
#ifdef VERBOSE
        Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
#endif
#ifdef OLED
        display.clear();
        display.display();
#endif
        return (i);
      }
#ifdef OLED
      //     drawProgressBar((j * 100) / MAX_TRY); //this prevents WiFi connection due to electromagnetic noise ?
#endif
      delay(500);
#ifdef VERBOSE
      Serial.print(".");
#endif

    }
#ifdef VERBOSE
    Serial.println(" can not connect to WiFi AP");
#endif

  }
  return (-1);
}



int initWiFi_retry() {
#ifdef VERBOSE
  Serial.print("initWiFi_retry() WiFi connecting to "); Serial.println(ssidArray[AP]);
#endif
  //  Serial.print(" "); Serial.print(passwordArray[AP]);
  WiFi.mode(WIFI_OFF);
  WiFi.begin(ssidArray[AP], passwordArray[AP]);

  int j;
  for (j = 0; j < MAX_TRY; j++) {
    if (WiFi.status() == WL_CONNECTED) {
#ifdef VERBOSE
      Serial.print("WiFi connected, IP address: "); Serial.println(WiFi.localIP());
#endif
#ifdef OLDE
      display.clear();
      display.display();
#endif

      configTime(TIMEZONE * 3600L, 0,  "time.google.com", "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");
      return (AP);
    }
    delay(500);
#ifdef VERBOSE
    Serial.print(".");
#endif
  }
#ifdef VERBOSE
  Serial.println(" can not connect to WiFi AP");
#endif
#ifdef OLED
  displayWiFioff();
#endif
  return (-1);
}



// get CPU core temperature and translate into estimated ambient temperature
extern "C" {
  uint8_t temprature_sens_read();
}

float getTemperature()
{
  float temp;
  //  Serial.printf("Temperature(temperature_sens_read()): % d\n", temprature_sens_read());
#ifdef CELSIUS
  //  temp = temperatureRead() - 26.0; //get ESP32 core temperature in Celsius and convert it to ambient temperature
  //modify offset to adopt for your ambient temperature
#ifdef BMP180
  temp = pressure.readTemperature() - 3.0 ; //need calibration for your BMP180
#else
  temp = mhtempC; // MH-Z14A temperature
#endif

#else  //F
  //   temp = temperature_sens_read();
#ifdef BMP180
  float t;
  t = pressure.readTemperature();
  temp = t * 1.8 + 32;
#else
  temp = mhtempF; // MH-Z14A temperature
#endif
#endif

#ifdef VERBOSE
  //  Serial.printf("Temperature: % 2.1f\n", temp);
#endif
  return (temp);
}

float GetHumidity()
{
#ifdef HUMIDITY
  float H = SHT21.getHumidity();
  //  Serial.println(H);
  return (H);
#else
  return (33.0 + (float)random(1, 4)); //dummy
#endif
}

uint8_t command[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
uint8_t reset[] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
//http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z19b-co2-ver1_0.pdf
// i'm not sure following ABC(automatic base correction) command for MH-Z19B can work for MH-Z14A
uint8_t disableABC[] = {0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86};
uint8_t enableABC[] = {0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6};

void initCO2() {
  uint8_t c;
  while (Co2Sensor.available()) {
    c = Co2Sensor.read();
  }

}
// assume MH-Z14A has the same ABC enable/disable function as MH-Z19
void disableABCcommand() {
  initCO2();
  Co2Sensor.write(disableABC, sizeof(disableABC));
}
void enableABCcommand() {
  initCO2();
  Co2Sensor.write(enableABC, sizeof(enableABC));
}
void resetCO2() {
  Co2Sensor.write(reset, sizeof(reset));
}

int getCO2() {
  int returnnum = 0;
  uint16_t co2 = 0;
  uint8_t readBuffer[9];
  int i; uint8_t checksum = 0;

  initCO2();
  Co2Sensor.write(command, sizeof(command));
  delay(100);
  Co2Sensor.setTimeout(300);//set 300msec timeout
  returnnum = Co2Sensor.readBytes(readBuffer, sizeof(readBuffer));
  //  if ( (readBuffer[0] == 0xFF) && (readBuffer[1] == 0x86)) {
  if ( (readBuffer[0] == 0xFF) ) {
    for ( i = 1; i < 8; i++)
    {
      checksum += readBuffer[i];
    }
    checksum = 0xff - checksum;
    checksum += 1;
#define DEBUG
#ifdef DEBUG
    for (i = 0; i < sizeof(readBuffer); i++) {
      Serial.print(readBuffer[i], HEX);
      Serial.print(" ");
    }  Serial.print(":"); Serial.println(checksum, HEX);
#endif
    if (readBuffer[8] != checksum) {
      Serial.println("MH-Z14A communication check sum error");
      initCO2(); // in case something wrong happens, then clear data from MH-Z14A
      return (9999);
    }
    switch ( readBuffer[1]) {
      case 0x86:
        co2 = (uint16_t)readBuffer[2] << 8;
        co2 += readBuffer[3];

        mhtempF = (float)readBuffer[4]; // MH-Z14A internal temperature (not documented in specification)
        mhtempC = ( mhtempF - 32.0) / 1.8 + 4; // 4 is a offset for my sensor , needs calibration
        return (co2);
      case 0x79:
        Serial.println("ABC disable command is issued.");
        initCO2(); //  clear data from MH-Z14A
        return (returnnum);
      default:
        initCO2(); //  clear data from MH-Z14A
        return (returnnum);
    }

  }
  initCO2(); //  clear data from MH-Z14A
  return (returnnum);
}
int dustbase = 0;

void initDust() {
  pinMode(GP2YLED, OUTPUT);
  dustbase = getDustbase();
  if ( dustbase > 0) dustbase = 0;
#ifdef VERBOSE
  Serial.print("dustbase = "); Serial.println(dustbase);
#endif
}
#define LEARNING 100
int getDustbase() {
  int i;
  long int d = 0;
  for (i = 0; i < LEARNING; i++) {
    d = d + getDustSub();
  }
  return (d / LEARNING);

}
#define COUNTER 10
int getDust() {
  int i;
  long int d = 0;
  for (i = 0; i < COUNTER; i++) {
    d = d + getDustSub();
  }
  if (d < 0) d = 0;

  return (d / COUNTER);

}
//this function assumes LED pin of GP2Y1014AU0F is driven by NPN transister such as 2N5551
//, asserting digitalWrite(GP2YLED, HIGH); first
int getDustSub() { // ug/m3
#define NPNTRANSITER
#ifdef NPNTRANSITER
  digitalWrite(GP2YLED, HIGH);
  delayMicroseconds(280);
  int v = analogRead(GPIO33);
  delayMicroseconds(40);
  digitalWrite(GP2YLED, LOW);
#else
  digitalWrite(GP2YLED, LOW);
  delayMicroseconds(280);
  int v = analogRead(GPIO33);
  delayMicroseconds(40);
  digitalWrite(GP2YLED, HIGH);
#endif
  delayMicroseconds(10000 - 280 - 40); // see specsheet of GP2Y10
  float vf; vf =  ((float)v * 3.3) / 4096.0; // ESP32 has 4096 resolution for 3.3V input.
  float dust;
  float Vs = 0.5; // Vs depend on your GP2Y10 device
  dust = (vf / 6.0 - Vs / 6.0) * 1000 - dustbase;
#ifdef VERBOSE
  //  Serial.print("Raw: "); Serial.print(v);
  //  Serial.print(" V: "); Serial.print(vf);
  //  Serial.print(" Dust: (ug / m3)"); Serial.println(dust);
#endif

  return ((int)dust );

}
