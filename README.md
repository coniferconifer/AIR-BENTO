## AIR BENTO CO2 and dust monitor by ESP32
## Improve Decision-Making Performance by AIR BENTO
   
### watch CO2 and dust levels at home/car/workplace , vent breathless air
- Winsen MH-Z14A CO2 sensor
- SHARP GP2Y1014AU0F dust/smoke sensor
- Bosch BMP180 pressure sensor GY-68
- Sensirion SHT21 humidity sensor GY-21  
- runs on ESP32 NodeMCU board
- 128x64 mono color OLED display is supported
- works as a MQTT client over WiFi or a BLE device
are packed in BENTO lunch box.

## what's new
- By using the latest Arduino core for ESP32 with BLE , the compiled flash memory reached 
  to 104%, so that #define BLE  at AIR-BENTO is changed to an option by default.<br>
  Update Arduino core for ESP32 WiFi chip , then use "Partition Scheme" in "Tools" menu in Arduino IDE and select "No OTA(large APP)" , 
  this program will fit in the space in case of #define BLE
  
- SHT21 humidity sensor is supported in May/14/2018 by #define HUMIDITY
- GY-21 board with SHT21 has 5V SDA/SCL pins when VIN is connected to 5V , I'm using FET 3.3V-5V level converter to connect to ESP32 I2C bus. (see trouble shooting below)
- wirings are changed to use BMP180 as of April/30/2018
- #define DEEPSLEEP for ESP32 deep sleep is supported in May 13,2018

![Fig.1 AIR BENTO](https://github.com/coniferconifer/AIR-BENTO/blob/master/AIR%20BENTO.jpg)

## CO2 sensor Winsen MH-Z14A
- via 9600bps serial port

## Dust sensor SHARP GP2Y1014AU0F or maybe GP2Y1010AU0F
- analog voltage output 0-3.7V 
- Vo output is connected to ESP32 ADC33 port
- internal LED is driven by NPN transister 2N5551 from GPIO39 (Fig.2)
- max Vo output goes up to 3.7V and exceeds ESP32 max GPIO input voltage spec 3.6V , so Vo is clipped by shotkey diode 1N5819 to protect GPIO33.
  (Fig.2) , but this may be harmful to the last stage amplifier of GP2Y1016AU0F.
  <br>Voltage divider by resisters or another operational amplifier that has 0.8x gain may be better solution for product level quality.
  
  

## GY-68 BMP180 pressure sensor (Apr/30/2018)
- connected via I2C (GPIO21 as SDA , GPIO22 as SCL)
- GY-68 BMP180 has 3.3V linear regulator on board and SDA,SCL are pulled up to 3.3V,
  so, I supply 5V to VCC pin. (depending on your hardware) 

## AIR BENTO posts sensor values to MQTT server via WiFi
- ThingsBoard is recommended to visualize data by its dashboard.
  I'm running ThingsBoard on Raspberry pi.

## AIR BENTO broadcasts sensor values as bluetooth device name
- in case there is no WiFi AP available,
Mobile phone users near AIR BENTO can read out sensor values without application.


## License: Apache License v2

## References

### CO2 sensor:

- [MH-Z14A  User's Manual V1.01](http://www.winsen-sensor.com/d/files/infrared-gas-sensor/mh-z14a_co2-manual-v1_01.pdf)

- [MH-Z14A User's ManualV2.4](http://www.winsen-sensor.com/d/files/PDF/Infrared%20Gas%20Sensor/NDIR%20CO2%20SENSOR/MH-Z14%20CO2%20V2.4.pdf)

- [https://revspace.nl/MHZ19](https://revspace.nl/MHZ19)

### SSD1306 OLED display library from 
- [SSD1306 https://github.com/LilyGO/ESP32-OLED0.96-ssd1306](https://github.com/LilyGO/ESP32-OLED0.96-ssd1306)

- [http://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/spi_master.html](http://esp-idf.readthedocs.io/en/latest/api-reference/peripherals/spi_master.html)

### GY-68 BMP180 pressure sensor
- [GY-68 BMP180 http://artofcircuits.com/product/bmp180-digital-barometric-sensor-module-model-gy-68](http://artofcircuits.com/product/bmp180-digital-barometric-sensor-module-model-gy-68)

### BMP180 library
- [https://github.com/adafruit/Adafruit-BMP085-Library](https://github.com/adafruit/Adafruit-BMP085-Library)

### SHT21 library 
- [https://github.com/e-radionicacom/SHT21-Arduino-Library](https://github.com/e-radionicacom/SHT21-Arduino-Library)


### PubSubClient
- [https://github.com/knolleary/pubsubclient](https://github.com/knolleary/pubsubclient)

### MQTT server with visiualization tools 
- [https://thingsboard.io/] (https://thingsboard.io/)

### Time display
- public NTP server  [https://developers.google.com/time/](https://developers.google.com/time/)

### Dust sensor:
- [https://media.digikey.com/pdf/Data%20Sheets/Sharp%20PDFs/GP2Y1014AU0F_Spec_2-6-15.pdf](https://media.digikey.com/pdf/Data%20Sheets/Sharp%20PDFs/GP2Y1014AU0F_Spec_2-6-15.pdf)

- technical detail of GP2Y10's analog front end  
[http://www.ti.com/lit/ug/tidub65c/tidub65c.pdf](http://www.ti.com/lit/ug/tidub65c/tidub65c.pdf)
- wiring<br>
![Fig.2 dust sensor and ESP32](https://github.com/coniferconifer/AIR-BENTO/blob/master/AIRBENTOcircuit.jpg)

### trouble shooting:
- Why GPIO33 is used for ADC <br>
  refer to "ADC2 Channel cannot be used when WiFi is in use #440"<br>
  [https://github.com/espressif/arduino-esp32/issues/440](https://github.com/espressif/arduino-esp32/issues/440)
- about [W] [esp32-hal-i2c.c:334] i2cRead() Ack Error! Addr:40 <br><br>
  GY-21 has 3.3V LDO(low dropout regulator connected to VIN) on board , 3.3V from ESP32 was connected to VIN.
  [schematics of GY-21](https://www.ebay.com/itm/New-HTU21D-GY-21-HTU-Temperature-Humidity-Sensor-Breakout-Board-Module-Hot-/271731131299)
  <br>(pull up resisters are 10KOhm for GY-21 I got)<br>
  External SDA and SCL pins are confirmed that they are pulled up to 3.3V (thanks to Low dropOut regulator 662K )
  but got [W] [esp32-hal-i2c.c:334] i2cRead() Ack Error! Addr:40 for access to SHT21(GY-21 board).
  Whats wrong ?? <br> I have no oscilloscope to check the waveform of I2C bus!<br><br>
  Another try was to connect 5V to VIN of GY-21 and added 5V - 3.3V FET gate level converter between 
  GY-21 and ESP32 I2C bus, but still got the same error.<br>
  I have added 4.7kOhm pullup resisters to SDA and SCL lines from GY21 and pulled up to 5V. <br>
  This fixed the i2cRead() Ack error. The waveform I2C may be shaped by lower pullup resisters, but could not figure out exact reason.

### remaining issues
- ~~WiFi is always ON , not good for longer battery operation.~~

### "Elevated Indoor Carbon Dioxide Impairs Decision-Making Performance"
- [http://newscenter.lbl.gov/2012/10/17/elevated-indoor-carbon-dioxide-impairs-decision-making-performance/](http://newscenter.lbl.gov/2012/10/17/elevated-indoor-carbon-dioxide-impairs-decision-making-performance/)

### pulseview by sigrok project is quite useful 
- SHT21 I2C bus observed by CY7C68013A-56 EZ-USB FX2LP USB board<br>
  [https://sigrok.org/wiki/Lcsoft_Mini_Board](https://sigrok.org/wiki/Lcsoft_Mini_Board)
- software logic analyzer<br>
  [https://sigrok.org/wiki/PulseView](https://sigrok.org/wiki/PulseView)<br>
  SHT21 command write<br>
  ![Fig.3 write](https://github.com/coniferconifer/AIR-BENTO/blob/master/SHT21commandwrite.png)<br>
  SHT21 command read<br>
  ![Fig.4 read](https://github.com/coniferconifer/AIR-BENTO/blob/master/SHT21commandread.png)<br>

### CO2 measureing may be a brain activity monitor<br>
![Fig.5 CO2 graph](https://github.com/coniferconifer/AIR-BENTO/blob/master/CO2graph.png)<br>

### dust sensor detects when I'm home
![Fig.6 I'm home at around 13:23, humidity , dust and co2 level jumped!](https://github.com/coniferconifer/AIR-BENTO/blob/master/whenIamhome.png)<br>