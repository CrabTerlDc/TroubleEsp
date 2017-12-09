/*
  Trouble Software for ESP8266, ESP32 and Co

  inspired :
    ESP8266 Blink by Simon Peter
  Doc :
    http://wiki.3615senor.org/doku.php?id=commenailles-3
    Fichier|preference|parametres| gestionnaire de cartes supp|http://arduino.esp8266.com/stable/package_esp8266com_index.json

  size ESP8266:
   1074464/81920 at max
    179840/40804 on 20170404
    274395/40656 on 20170306
    274439/40088 on 20170201
    263155/38500 on 20170122
    237650/33832 on 20160727
    222878/35008 on 20160529
    221022/34624 on 20160101
    220786/34660 on 20160204

  TODO_LIST
  - DC Dir+Pwm (MD50) or LEft+Right (or stepper?) selectable
  - DC & step drive per speed,
  - DC&Step, stoppers, homing, autotune, followers
  - RS485 on phantom power
  - standard arduino w/wo ethernet shield w/wo rs485
  - screen
  - off led after reboot, 00 as statug orange at boot, green on wifi connected
  - group connnected friends on main page (osc info collect)
  - main selector (BEN_) in config
  - 'I' 'i' input to return to sender
  - security S1 == S2 in eeprom add and check signature for 1char Crc on osc and admin/pwd on html xxxxxnU
  - websocket support https://fr.wikipedia.org/wiki/WebSocket
  - wifiMulti dynamic alloc to reinit without hard reboot
  ... memory not so secure, power down before rx tx plugs or config is forgotten, think about trying "esp filesystem" rumors
  CRAB_20171111
  - 4 parameters for speed and pos X and Y mstly for apps
  - stepper and stopper works together
  - Merged adler corrections
  - DC, step not so different... Servo Pwm not so different, motor type are now soft parameter
  CRAB_20170504
  - servo back on stage
  - correction for 'o'
  CRAB_20170417
  - additionnal list of wifi points switcheable
  CRAB_20170411
  - Timed Resend to patch udp 15p100 loss, add retries( 5x 200 ms)
  - wifi as client runs only if CliSSID'D' is not empty, don't disturb with connects attempts if not asked to
  CRAB_20170408
  - corrections wifi et switch
  CRAB_20170404
  premiere version pour boites Gui
  - Xxxx in command replaced live by Adc
  - Adc 0..9999
  CRAB_20170325
  - function launcher 'F' Fct(DbgNum, DbgStr)
  - correction for osc on all wifi segments
  - multi motors
  CRAB_20170304
  - gray + DC == lego motor ^^ MODULE_MWS
  - pinout in defines, multi config
  - OSC udp relay on local wifi and preferred net star-ring (multi wifi connect)
  - servo
  - man in the middle stepper driver, get pulse with interrupt
  - drive a not so little stepper motor
  - read gray code
  - RGB Leds, wait 'W' and output 'O'
  - OSC client-server support
  - dynamic osc command in flash
  - store ssid and pwd in flash
  CRAB_20170201
  - title in html identify device
  - multi now has html config page too
  CRAB_20170129
  - connect another network ssid/pwd
  - autorefresh status page
  CRAB_20170121
  Change play method to avoid stutter
  CRAB_20170120
  micro http:80 to conf like an IoT,
    player emits ITS_<node number>/192.168.4.1 to configure
    passenger emits BEN_<node number>/192.168.4.1 to configure
  CRAB_20160803 (and ..02)
  B switch for the B guy
  play 'till the end of the track
  CRAB_20160228 (and ..27)
  soft wifi power adjust w..z reserved (ignored) - lots of books were set to 'x' meaningless
  support the '0' book
  better levelling, corrected
  CRAB_20160717
  securize parameters by 3 copies
  corrections in head remove detect
  shut after x sec without pressed
  don't play if no passenger strong enough, i.e. too far
  CRAB_20160528
  default power to 128-(MAX_TPW/2) (minimal electrics)
  restart button for ROLE_MULTI (to test)
  wifi detect leverage (to test)
  restart if head removed more tan 2 seconds
  corrected glitch auto-play
  CRAB_20160204
  TODO_LATER : http://playground.arduino.cc/Learning/Memory prefix necessary strigs with F("")
  led strip Ko , solved by arduino environment reinstalled
  CRAB_20150102 :
  power emission tunable
  CRAB_20151229 :
  first ESP revision for Gui & Ben

  Player full avec 1 seule batterie 2500ma ... 2h15?...
  Passenger 2500ma ... 24H

  example pastille 070V 00R 87P 100m
*/

#define HARDWARE_NAME "CRAB_20171204"

// a way to not have my house AP-Pwd in the build
// if none just comment the include or create an Mines.h with ESP8266_MULTIPERSO full of wifiMulti.addAP
#include "Mines.h" 

// global module kind
//#define MODULE_LAAPIN
//#define MODULE_STEPPER
//#define MODULE_MWS // drive NXT motor
//#define MODULE_TROUBLE
#ifdef ESP32
  #define MODULE_DEV32
#else
  #define MODULE_DEV
#endif

// constants
#define ROLE_PASSENGER 1 // annonce son Id wifi
#define ROLE_PLAYER 0    // joue la piste en fonction de son numero et du passager le plus proche
#define ROLE_SOLO 2      // joue la piste en fonction de son numero
#define ROLE_MULTI 3     // joue les pistes 1..MAX a chaque appui on change pour la piste suivante
#define ROLE_TEST 4      // joue en continu + wifi, pour test d'autonomie
#define ROLE_IOTM 5      // Iot Master (set wifi node)
#define ROLE_IOTS 6      // Iot Subrogated autoconnect (TODO_HERE)

// Hardware configuration
#if defined( MODULE_LAAPIN) // ---------------
  #define WITH_WIFI
  #define WITH_WIFICLIENT // connect other network
  #define WITH_IOTMUTUAL // web server
  #define WITH_OSC

  #define WITH_WS2812 0 // D3
  #define WITH_SWITCH 14
  //#define WITH_STEPPER_IN { 0, 4}
  //#define WITH_STOPPER
  #define WITH_SERVO  { 2} // rem : tried {12} alone, need {12, 2} to move TODO_LATER : wtf?
#define WITH_ADC      A0

// #define WITH_NOKIA5110
// #define WITH_NOKIA5110_ADAFRUIT
// #define WITH_OLED

#elif defined (MODULE_STEPPER) // ---------------
  #define WITH_WIFI
  #define WITH_WIFICLIENT // connect other network
  #define WITH_IOTMUTUAL // web server
  #define WITH_OSC

  #define WITH_WS2812   2 // D4
  #define WITH_SWITCH  14 //D5
  #define WITH_STEPPER { 0, 4} // D3, D2
  #define TIMER_SLICE_SU_DEF 71
  #define WITH_ADC      A0
  #define WITH_STEPPER_IN { 12, 13} // D6 D7
  //#define WITH_STOPPER
  //#define WITH_GRAYCODE_IN { 12, 13}
#elif defined (MODULE_MWS) // ---------------
// http://www.philohome.com/nxtmotor/nxtmotor.htm
  #define WITH_WIFI
  #define WITH_WIFICLIENT // connect other network
  #define WITH_IOTMUTUAL // web server
  #define WITH_OSC
  
  #define WITH_DCMOTOR { D2, D4}
  #define WITH_GRAYCODE_IN { D0, D1}
  #define WITH_STEPPER_IN { D6, D7}
  #define WITH_STOPPER { D8, D5} // min max
  #define WITH_WS2812  D3 // 0
  //#define WITH_SERVO  { D4} //2
#elif defined( MODULE_TROUBLE) // ---------------
#define WITH_WIFI
#define WITH_IOTMUTUAL // web server
#define WITH_OSC
// #define WITH_LED_SHOW_WIFI
#define ROLE_DEFAULT ROLE_PASSENGER) // we will produce more passengers than players
// #define ROLE_DEFAULT ROLE_PLAYER; // we will produce more players than passengers


#define WITH_WS2812    2
#define WITH_DFPLAYER {12, 13} // { Rx, Tx}
#define WITH_SWITCH   14
//#define WITH_AUTODETECT
#elif defined( MODULE_DEV) // ---------------
  #define WITH_WIFI
  #define WITH_WIFICLIENT // connect other network
  #define WITH_IOTMUTUAL // web server
  #define WITH_OSC

  // #define WITH_WS2812    0
  #define WITH_ADC      A0
  //#define WITH_STEPPER_IN { 12, 13} // D6 D7
  #define WITH_STEPPER {  D2, D3, D7, D6} // DIR1 PULS1
  //#define WITH_WS2812    D4
  //#define WITH_RELAY D5

  #define WITH_STOPPER { D5, D4} // min max

#elif defined( MODULE_DEV32) // --------------- dev ESP32 Yeah

  #define WITH_WIFI
  #define WITH_WIFICLIENT // connect other network
  //#define WITH_IOTMUTUAL // web server
  //#define WITH_OSC
  #define WITH_SCREEN_SSD1306 { 0x3c, 5, 4}

#endif


#define ACT_UNKN 0
#define ACT_ININIT 1
#define ACT_READY 2
#define ACT_PLAY 3
#define ACT_DONTSHOW 4 // off to spare power

// default soft configuration
// 0..99, but for buttons in blind mode we will constraint node id
#define MAX_STORY 20
#define MAX_TRACK 20

#ifndef BEN_TAG
  #define BEN_TAG "BEN_" // 4 chars expected
#endif

#ifndef BEN_PWD
  #define BEN_PWD "MyAdmin"
#endif /* BEN_PWD */

#ifndef ROLE_DEFAULT
  #define ROLE_DEFAULT ROLE_IOTS
#endif
// -- Lib includes and const

// http://esp8266.github.io/Arduino/versions/2.3.0/doc/reference.html
// http://www.esp8266.com/wiki/doku.php?id=esp8266_gpio_pin_allocations
// so ... nodemcu (lolin, wemos) renamed pins, is it funny for someone?
#define NODEMCU_D0 16 // pulled down, all other can pull up
#define NODEMCU_D1  5
#define NODEMCU_D2  4
#define NODEMCU_D3  0 // can be out, beware as input for prog, (analog 0 in is A0?)
#define NODEMCU_D4  2 // float 3.3v +-0.2 during prog +  // TODO_LATER see 0-3.3v frames servo dislike
#define NODEMCU_D5 14
#define NODEMCU_D6 12
#define NODEMCU_D7 13 // rem: blink on prog
#define NODEMCU_D8 15
#define NODEMCU_D9  3 // Rx ?
#define NODEMCU_D10 1 // Tx ?

#ifdef ESP32
// TODO_HERE
// https://esp-idf.readthedocs.io/en/v2.0/api/peripherals/adc.html

  #define D1 1
  #define D2 2
  #define D3 3
  #define D4 4
  #define D5 5
  #define D6 6
  #define D7 7
  #define D8 8
  #define D9 9
  #define D10 10

  #define A0 36

  // TODO_HERE
  #define analogWrite( x, y)

#endif /* ESP32 */

// inject int into C++ string
#define CSTRI( Stuff) String( (int) Stuff)
// int to formated string n-len for java-like
#define CSTRIn( n, Val)  (((n>7) &&( Val < 10000000)) ? String("0") : String("")) \
                       + (((n>6) &&( Val < 1000000 )) ? String("0") : String("")) \
                       + (((n>5) &&( Val < 100000  )) ? String("0") : String("")) \
                       + (((n>4) &&( Val < 10000   )) ? String("0") : String("")) \
                       + (((n>3) &&( Val < 1000    )) ? String("0") : String("")) \
                       + (((n>2) &&( Val < 100     )) ? String("0") : String("")) \
                       + (((n>1) &&( Val <   10)) ? String("0") : String("")) \
                       + CSTRI( Val)


typedef uint32_t pr_uint32_t[3];
typedef int32_t pr_int32_t[3];

#ifdef WITH_WIFI
  #ifdef ESP32
    #include <WiFi.h>
    #include <ESPmDNS.h>
    #include <WiFiClient.h>
  #endif /* ESP32 */
  #ifdef ESP8266
    // ESP8266 activated by selecting menu|tool|board like "NodeMCU 1.0 (ESP 12E)"

    // see http://wiki.3615senor.org/doku.php?id=esp8266
    // minimal ESP8266 8 pattes
    // Alim 3.3V -> ESP +3.3V
    // Alim Gnd  -> ESP Gnd
    // Alim 3.3V -> ESP CH_PD
    //
    // minimal ESP12:
    // Alim 3.3V -> ESP +3.3V
    // Alim 3.3V -> ESP EN
    // Alim Gnd  -> ESP Gnd
    // Alim Gnd  -> ESP GPIO15
    //
    // Dev
    // ftdi TXD -> ESP RXD
    // ftdi RXD -> ESP TXD
    // ftdi Gnd -> ESP Gnd
    //      Gnd -> ESP GPIO0 // prog

    // rem : seems put sources in C:\Users\utilisateur\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\libraries\ ESP8266 xxx
    #include <ESP8266WiFi.h>

    #define MAX_TPW 82

    // about ICACHE_RAM_ATTR see https://github.com/esp8266/Arduino/issues/2680

  #else
    //TODO_LATER : check with std arduino and a wifi shield, wifi parts of this code probably derivate a lot to ESP spacific only...
    #include <WiFi.h>
  #endif /* ESP8266 */

  #ifdef WITH_OSC
      // failed try, not building to add lib to arduino ide ArdOSC https://github.com/Chris--A/ArdOSC, #include <ArdOSC.h>

      // C:\Users\utilisateur\AppData\Local\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\libraries\ESP8266WiFi\src\WiFiUdp.h
      // C:\Users\utilisateur\AppData\Roaming\Arduino15\packages\esp8266\hardware\esp8266\2.3.0\libraries\ESP8266WiFi\src\WiFiUdp.h
      // C:\Users\utilisateur\Documents\Arduino\libraries\WiFi\src\WiFiUdp.h
      #include <WiFiUdp.h>

      // https://github.com/CNMAT/OSC
      // C:\Users\utilisateur\Documents\Arduino\libraries\OSC-master
      #include <OSCMessage.h> // this include seems particularly order dependent ESP8266WiFi.h, WiFiUdp.h, OSCMessage.h
      #include <OSCBundle.h>
      #include <OSCData.h>
      #define OSCMESSAGE OSCMessage
  #else
      // seems OSCMessage declared weird and bypass ifdef's
      #define OSCMESSAGE char*
  #endif /* WITH_OSC */

  #define WIFI_NONBLOCKING
  int WifiInit( void);
  int WifiScan( void);
#else
  #define WifiInit()
  #define WifiScan()
#endif /* WITH_WIFI */
void TimeRunningDump( String& LocStr);

#ifdef WITH_NOKIA5110_ADAFRUIT
  #include <SPI.h>
  // manage libraries | add | Adafruit GFX
  #include <Adafruit_GFX.h>
  // manage libraries | add | Adafruit PCD8544 Nokia 5110
  // C:\Users\utilisateur\Documents\Arduino\libraries\Adafruit_PCD8544_Nokia_5110_LCD_library
  // TODO_LATER : reqs mods in .h and portregs and ... not build
  // http://www.mon-club-elec.fr/pmwiki_reference_arduino/pmwiki.php?n=Main.PROGMEM

  // grmph, no more with  https://github.com/bbx10/Adafruit-PCD8544-Nokia-5110-LCD-library
  // remarks : this ide try to build includes even if behind ifdef
  //#include <Adafruit_PCD8544.h>

  #define WITH_SCREEN
#endif

#ifdef WITH_SCREEN_SSD1306 
  #include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
  #include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`
  #define WITH_SCREEN
#endif

#ifdef WITH_WIFICLIENT
  #ifdef ESP32
    #include <WiFiMulti.h>
  #endif /* ESP32 */
  #ifdef ESP8266
    #include <ESP8266WiFiMulti.h>
    #endif /* ESP8266 */
    #endif /* WITH_WIFICLIENT */

    #ifdef WITH_NOKIA5110
      // PCD8544 by carlos
      // in C:\Users\utilisateur\Documents\Arduino\libraries\PCD8544 change <avr/progblabla> by <progblabla>
      #include <PCD8544.h>
      // TODO_LATER ... it builds but not yet displays
      #define WITH_SCREEN
    #endif

#ifdef WITH_NOKIA5110_ADAFRUIT
  // SCREEN 7 - Serial clock out (SCLK)
  // SCREEN 6 - Serial data out (DIN)
  // SCREEN 5 - Data/Command select (D/C)
  // SCREEN 4 - LCD chip select (CS)
  // SCREEN 3 - LCD reset (RST)
  // int8_t SCLK, int8_t DIN, int8_t DC, int8_t RST
  Adafruit_PCD8544 display = Adafruit_PCD8544( 14, 0, 4, 5, 3);
  // lolin 5-D1, 4-D2, 0-D3, 2-D4, 14-D5
#endif
#ifdef WITH_NOKIA5110
  // SCREEN 7 - Serial clock out (SCLK)
  // SCREEN 6 - Serial data out (DIN)
  // SCREEN 5 - Data/Command select (D/C)
  // SCREEN 4 - LCD chip select (CS)
  // SCREEN 3 - LCD reset (RST)
  // int8_t SCLK, int8_t DIN, int8_t DC, int8_t RST
  PCD8544 display = PCD8544( 14, 0, 4, 5);
  // lolin 5-D1, 4-D2, 0-D3, 2-D4, 14-D5
#endif
#ifdef WITH_OLED
  // Add lib sparkfun micro oled breakout (64x48)

  #include <Wire.h>  // Include Wire if you're using I2C
  #include <SPI.h>  // Include SPI if you're using SPI
  //#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

  #define WITH_SCREEN
#endif /* WITH_OLED */
#ifdef WITH_SCREEN_SSD1306 
  SSD1306 display (0x3c, 5, 4);
  int32_t ScreenLastMs;
#endif


#ifdef WITH_IOTMUTUAL
  #include <WiFiClient.h>
  #ifdef ESP32
    //#include <ESPWebServer.h> thru wifi.h WiFiServer
    #include <ESPmDNS.h>
  #endif /* ESP32 */
  #ifdef ESP8266
    #include <ESP8266WebServer.h>
    #include <ESP8266mDNS.h>
  #endif /* ESP8266 */
#endif /* WITH_IOTMUTUAL */

#ifdef WITH_STEPPER
  // So ... where do you want to wire the stepper driver
  #define WITH_MOTOR
  #ifndef ESP32
    #define WITH_TIMER
  #endif /* ESP32 */
#endif /* WITH_STEPPER */

#ifdef WITH_DCMOTOR
#define WITH_MOTOR
#define WITH_TIMER // TODO_LATER : might not be necessary if pwm pin correctly drived
#endif /* WITH_DCMOTOR */

#ifdef WITH_SERVO
  #define SERVO_ROOT
  // https://github.com/fablabnbg/EspWebServo/blob/master/EspWebServo.ino
  // update servo libs ?

  #ifdef SERVO_ROOT
  //  #include <Servo.h>
  #else
  //  #include <SoftwareServo.h>
  #endif

  #define WITH_MOTOR
  //#define WITH_TIMER
#endif /* WITH_SERVO */

#ifndef ESP32 // TODO_LATER : ESP32
  #include <SoftwareSerial.h>
#endif /* ESP32 */

#include <EEPROM.h>

#define MAX_LTR 24

int PinModes[20];

//#ifdef ESP8266
//  #define RunOtherTasks( T) delay( (0 == T)? 1 : T)
//#else /* WITH_ESP8266 */
#define RunOtherTasks( T) delay(T)
//#endif /* WITH_ESP8266 */

#ifdef WITH_DFPLAYER
// DFPlayer from www.dfrobot.com/index.php?route=product/product&product_id=1121&search=DFPlayer&description=true
// DFPlayer
// VCC 5V         - Rouge -> DFPlayer 1 (haut gauche)
// Esp 12(DFP_RX) - vert  -> DFPlayer 2 (haut gauche sous VCC)
// Esp 13(DFP_TX) - Bleu  -> DFPlayer 3
//                           DFPlayer 6       -> Spk marron L
// Gnd noir               -> DFPlayer 7 (ou en face, capot SD)
//                           DFPlayer 8       -> Spk marron R
//                           DFPlayer 4 DAC_R -> ampli R
//                           DFPlayer 5 DAC_L -> ampli L
uint8_t DfpPins[] = WITH_DFPLAYER;
// #define DFP_RX DfpPins[0]
// #define DFP_TX DfpPins[1]
#define DFP_BUFFZSIZE 30
#endif /* WITH_DFPLAYER */

#ifdef WITH_WS2812
// kind of 1 wire milticolor leds
//
// see http://wiki.3615senor.org/doku.php?id=matos_ws2812_ledsmulticolores
// add lib to arduino ide|croquis|include library|manage libraries|adafruit neopixel 1.0.4
// 5v (or 3.3V)    -> Strip 5v
// 2  (NEOPIX_PIN) -> Strip Din
// Gnd             -> Strip Gnd
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif
// how many leds on the strip
#endif

#define PLAYERS_NB  8

#define WIFI_IDLEN 32
#define WIFI_PWDLEN 64

#ifdef WITH_WIFI
char SrvSSID[WIFI_IDLEN];// up to 32
char SrvPWD[WIFI_PWDLEN];// theorically up to 63 but...
#endif /* WITH_WIFI */

#ifdef WITH_SWITCH
  #define SWITCH_LINE WITH_SWITCH
  // Esp12 SWITCH_LINE -> start switch 1 (normally open)
  // Gnd -> start switch 2

  // time the push must stay pressed to consider it's not a wrong signal, usual 500
  #ifdef WITH_DFPLAYER
    #define SWITCH_GLITCH 500
    #define SWITCH_GLITCH_SHUT 1000
  #else
    #define SWITCH_GLITCH 5
    #define SWITCH_GLITCH_SHUT 5
  #endif

#endif /* WITH_SWITCH */

// restart pin reset the solo mode to track 1, solo mode runx between 1 and Max
// #define RESTART_PIN 4
#ifdef RESTART_PIN
// Esp12 RESTART_PIN -> restart switch 1 (normally open)
// Gnd -> restart switch 4
long RestartSwitchState;
long RestartSwitchTimemillis;
#endif /* RESTART_PIN */

// restart pin B reset the solo mode to track Max Track, so solo mode runs between Max+1 and 2xMax
// #define RESTART_B_PIN 5
#ifdef RESTART_B_PIN
// Esp12 RESTART_B_PIN -> restart switch 1 (normally open)
// Gnd -> restart switch 5
long RestartBSwitchState;
long RestartBSwitchTimemillis;
int RestartedGuy = 0;
#endif /* RESTART_PIN */

#ifdef WITH_ADC
uint16_t AdcVal = 0;  // 0..9999
#define ADC_SMOOTH 2 // smoothing v+x/x+1
uint32_t AdcLastReadMs = 0;
uint32_t AdcLastSent = 0;
#define OSC_DEFAULT_CMD3 "S"
#endif /* WITH_ADC */

// static config

#ifdef WITH_IOTMUTUAL
  #ifdef ESP32
    WiFiServer server(80);
  #endif
  #ifdef ESP8266
    ESP8266WebServer server(80);
  #endif
#endif /* WITH_IOTMUTUAL */

#ifdef WITH_WIFICLIENT

  typedef enum WifiState_e {
      WifiState_OFF
    , WifiState_STABLE
    , WifiState_TOCONNECT
    , WifiState_JUSTCONNECTED
  } WifiState_t;

  #ifdef ESP32
    WiFiMulti wifiMulti;
  #endif /* ESP32 */
  #ifdef ESP8266
    ESP8266WiFiMulti wifiMulti;
  #endif /* ESP8266 */
  WiFiClient CliWF;
  WifiState_t WifiCliConnectState = WifiState_OFF; // client connection state 1-no connection but required, 2- just connected, 0 - stable
  char CliSSID[WIFI_IDLEN];// up to 32 or 31+z
  char CliPWD[WIFI_PWDLEN];// theorically up to 63+z
#endif /* WITH_WIFICLIENT */

#ifdef WITH_OSC
  #ifdef WITH_WIFICLIENT
    WiFiUDP OscUdpCli;
    uint8_t OscWifiCliState = 0;
    #endif /* WITH_WIFICLIENT */

  WiFiUDP OscUdpSrv;
  uint8_t OscWifiSrvState = 0;

  int OscLastCmd;
  int OscLastCount = 0;
  int OscLastMs = 0;
  int OscLastSendMs = 0;
  uint32_t OscCmdSequence = 0;
  uint8_t OscReply = 0;

#define OSC_DEFAULT_ADDR "/"BEN_TAG
#define OSC_DEFAULT_CMD1 "h"
#define OSC_DEFAULT_CMD2 "H"
#define OSC_PORT 8000
OSCErrorCode OscError;
#define OSC_CMD_LEN 30
char OscAddr[OSC_CMD_LEN];
#ifdef WITH_SWITCH
char OscCmd1[OSC_CMD_LEN]; // pushed
char OscCmd2[OSC_CMD_LEN]; // released
#endif /* WITH_SWITCH */
#ifdef WITH_ADC
char OscCmd3[OSC_CMD_LEN]; // ADC
#endif /* WITH_ADC */
char OscAddrMine[OSC_CMD_LEN];
#endif /* WITH_OSC */


extern "C" {
#ifndef ESP32
  // TODO_HERE : ESP32
  // http://espressif.com/sites/default/files/documentation/2c-esp8266_non_os_sdk_api_reference_en.pdf
  #include "user_interface.h" // Access system_phy_set_max_tpw()
#endif /* ESP32 */
}

#ifdef WITH_TIMER
  // must be after motor declarations, they often needs timers
  // TODO_LATER : seems related to esp8266 somehow

  // http://www.switchdoc.com/2015/10/iot-esp8266-timer-tutorial-arduino-ide/
  os_timer_t Timer;
  uint8_t TimerActivated = 0;

  void TimerStart();
  void TimerStop();
#else
  #define TimerSetup()
  #define TimerStart()
  #define TimerStop()
#endif /* WITH_TIMER */


#ifdef WITH_STEPPER_IN
  uint8_t StepperInPins[] = WITH_STEPPER_IN;
  #define STEPPER_IN_DIR_PIN StepperInPins[0]
  #define STEPPER_IN_PULSE_PIN StepperInPins[1]

  uint32_t StepperInVal = 0;
  uint32_t StepperInUs = 0;
  uint32_t StepperInPos = 0;
  pr_uint32_t pr_StepperInPos;

  // interrupt proved to be unstable if called a lot
  // wait for corrections in ESP?..
  // # define STEPPER_IN_PULSE_PIN_INTERRUPT
#endif /* WITH_STEPPER_IN */

#ifdef WITH_STOPPER
  uint8_t StopperPins[] = WITH_STOPPER;

  #define STOPPER_MIN_PIN StopperPins[0]
  #define STOPPER_MAX_PIN StopperPins[1]

  int32_t StopperMax = 0;
  int32_t StopperMin = 0;
#endif /* WITH_STOPPER */


#ifdef WITH_SERVO
  uint8_t ServoPins[] = WITH_SERVO;
  #define ServoNb (sizeof( ServoPins) / sizeof(uint8_t))
  #define MOTOR_SERVO_IDX 0

  #define SERVO_ROOT
  // https://github.com/fablabnbg/EspWebServo/blob/master/EspWebServo.ino

  #ifdef SERVO_ROOT
  //  Servo ServoList[ServoNb];//TODO_LATER : bug around?
  #else
  //  SoftwareServo ServoList[ServoNb];
  #endif
#else /* WITH_SERVO */
  #define ServoNb 0
#endif /* WITH_SERVO */


#ifdef WITH_STEPPER
  uint8_t StepperPins[] = WITH_STEPPER;
  #define StepperNb (sizeof( StepperPins) / (2*sizeof(uint8_t)))
  #define MOTOR_STEPPER_IDX ServoNb
#else /* WITH_STEPPER */
  #define StepperNb 0;
#endif /* WITH_STEPPER */

#ifdef WITH_DCMOTOR
  uint8_t dcMotorPins[] = WITH_DCMOTOR;
  #define dcMotorNb (sizeof( StepperPins) / (2*sizeof(uint8_t)))
  #define MOTOR_DC_IDX = (ServoNb + StepperNb)
  
  #define pMotor->Pin1 dcMotorPins[0]
  #define pMotor->Pin2 dcMotorPins[1]
#else /* WITH_DCMOTOR */
  #define dcMotorNb 0
#endif /* WITH_DCMOTOR */

#ifdef WITH_MOTOR

  #define MOTOR_NB (ServoNb + StepperNb + dcMotorNb)
  
  typedef enum MotorMode_e {
    // motor Mode
    MOTOR_UNDEF = 0,
    MODOR_DC_LR   , //left right, grayIn
    //MOTOR_DC_DP, //Dir Pwm, NoGray
    MOTOR_PWM,   // Pwm 0..9999, NoGray
    MOTOR_PWM2,   // Pwm 0..9999, NoGray, no slice
    MOTOR_STEPPER,
    #ifdef WITH_SERVO
      MOTOR_SERVO,
    #endif
    MOTOR_MAXDEFS
  } MotorMode_t;

  typedef struct {
    uint32_t P1; // segment drive P1->P2 if drived per position
    uint32_t T1ms;
    uint32_t P2;
    uint32_t T2ms;

    int8_t Order;
    pr_int32_t WishP2; // the app part (out of the interruption) asks for a pos
    pr_uint32_t WishDTms;
    
    int16_t WishDec; // TODO_LATER probab pr_ too
    uint8_t HomingOn;
    uint8_t DriverMode; // MotorMode_t MODOR_DC_LR & co.

    uint8_t Pin1;
    uint8_t Pin2;

    pr_uint32_t Pos; // the position of the motor for the external world

    uint32_t LastPulseUs; // last time we set a value to the motor

    // to compute inside each kind of motor
    uint32_t LastPos; 
    uint32_t LastSlice;

    // 200 min microsec up (or down) for a pulse, 2us for a https://www.pololu.com/file/0J590/drv8825.pdf for example
    // min microsec up (or down) for a pulse, >=2us for a https://www.pololu.com/file/0J590/drv8825.pdf for example
    uint32_t MinCommandUs; // minimum time between two commands to the motor in microseconds

    uint16_t InstVal; // las val set in power pin(pwm)

    #ifdef WITH_STEPPER
      uint8_t StepperD; // current dir
      uint8_t StepperP; // current Pulse (up or down)
    #endif /* WITH_STEPPER */

  } Motor_t;

  Motor_t MotorArray[MOTOR_NB];
#endif /* WITH_MOTOR */

#ifdef WITH_GRAYCODE_IN
uint8_t GrayPins[] = WITH_GRAYCODE_IN;
#define GRAYCODE_IN_A_PIN GrayPins[0]
#define GRAYCODE_IN_B_PIN GrayPins[1]

uint32_t GrayPos = 0;
pr_uint32_t PrGrayPos;
uint8_t GrayA = 0;
uint8_t GrayB = 0;
#endif /* WITH_GRAYCODE_IN */

#ifdef WITH_WS2812
#define PIX_NB (1+PLAYERS_NB)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel( PIX_NB, WITH_WS2812, NEO_GRB + NEO_KHZ800);
uint8_t PixView;
#endif /*WITH_WS2812 */

#ifdef WITH_LED_SHOW_WIFI
uint8_t PixState;
#endif /* WITH_LED_SHOW_WIFI */

#ifdef WITH_OLED
  MicroOLED display(D5, D1, D2);

  //SSD1306  display(0x3c, D1, D2);
#endif /* WITH_OLED */

// Acquired by power
uint8_t PlayersId[PLAYERS_NB];
uint32_t PlayersMac[PLAYERS_NB];
int16_t PlayersRssi[PLAYERS_NB];
// Last for levelling
uint8_t PlayersId2[PLAYERS_NB];
uint32_t PlayersMac2[PLAYERS_NB];
int16_t PlayersRssi2[PLAYERS_NB];

#define RSSI_MIN (-200)
int RssiTmp;
int RssiContrib;

#ifdef WITH_DFPLAYER
SoftwareSerial DFPSerial( DfpPins[0] , DfpPins[1] ); // RX, TX
// test frame to inject data
uint8_t DFPMockFrame[10];
uint8_t DFPMockFrameCnt = 0;
#endif /* WITH_DFPLAYER */
int PlayerRunning = 0;
char PlayerHealth = 0;

#ifdef WITH_SWITCH
long SwitchTimePushmillis = 0;
long SwitchTimeRelmillis = 0;
long SwitchTimeTestmillis = 0;
bool SwitchState = 0;
#endif /* WITH_SWITCH */

// BUILTIN_LED is the pin with the internal LED (which is also the TXD pin; so we cannot use Serial.print() at the same time)
//#define ACTIVITY_LED    BUILTIN_LED
#define ACTIVITY_LED (-1)

// -- variables declaration

// 0 - rien du tout sauf hello pwr on
// 1 - boot et infos systeme
// 2 - interessant pour le client (un script externe, par exemple liste des passagers)
// 3 - Dev
int Verbose = 2;
uint32_t DbgNum = 0; // 0..999999999
int8_t DbgSign = 1;
#define DbgStr_NBMAX 32
uint8_t DbgStr[DbgStr_NBMAX];
uint8_t DbgStrIdx = 0;
uint8_t DbgStrMode = 0;

void CmdLineParse( unsigned char Ch);

int g_iTmp;

void Activity( int NewState);
unsigned long DbgLastActivityMillis = 0;
uint32_t TimeShowMs = 0;
int State = 0;
uint32_t LoopLastUs;
uint32_t LoopUs;

// TODO_LATER : should be in default package...
#define min( a, b) (a<b ? (a) : (b))

void dbgprintf( int Lev, const char *fmt, ... ) {
  char buf[128]; // resulting string limited to 128 chars

  if ( Lev <= Verbose) {
    va_list args;
    va_start (args, fmt );
    vsnprintf(buf, 128, fmt, args);
    va_end (args);
    Serial.print(buf);
  }
}

void dbgprintch(unsigned char Ch) {
  Serial.write((byte)Ch);
}

// cause time cycle regularly
// GL_TYPE DiffTime( GL_TYPE T0, GL_TYPE T1) ... .ino is not .c ...
uint32_t ICACHE_RAM_ATTR DiffTime( uint32_t T0, uint32_t T1) {
  // TODO_HERE : time cycle, test...
  //if ( T0 > T1) {
  //  return( 0xFFFFFFFF - T0 + T1 + 1);
  //}
  return ( abs((int32_t)(T1 - T0)));
}


// attempt to pass data between threads
void ICACHE_RAM_ATTR pr_uint32_write( pr_uint32_t& Id, uint32_t Val) {
  Id[0] = Val;
  Id[1] = Val;
  Id[2] = Val;
}

uint32_t pr_uint32_read (pr_uint32_t& Id) {
  uint32_t V1, V2, V3, Val = 0;

  V3 = Id[2];
  V2 = Id[1];
  V1 = Id[0];
  if (V1 == V2) {
    Val = V1;
  } else if (V2 == V3) {
    Val = V2;
  } else if (V3 == V1) {
    Val = V3;
  } else {
    Val = V1; // supposed currently writing V2, V1 is the new val 
    //  (V3/3+V2/3+V1/3);
  }

  return ( Val);
}

void ICACHE_RAM_ATTR pr_int32_write( pr_int32_t& Id, int32_t Val) {
  Id[0] = Val;
  Id[1] = Val;
  Id[2] = Val;
}

int32_t pr_int32_read (pr_int32_t& Id) {
  uint32_t V1, V2, V3, Val = 0;

  V3 = Id[2];
  V2 = Id[1];
  V1 = Id[0];
  if (V1 == V2) {
    Val = V1;
  } else if (V2 == V3) {
    Val = V2;
  } else if (V3 == V1) {
    Val = V3;
  } else {
    Val = V1; // supposed currently writing V2, V1 is the new val 
    //  (V3/3+V2/3+V1/3);
  }

  return ( Val);
}

#define dbgReadCh Serial.read

// node number (story for passenger, track for player
#define NodeGet() Epr3Get(0)
#define NodeSet( Id) Epr3Set(0, Id)

// volume for mp3 player, 0..99
#define GetVolume() Epr3Get(1)
#define SetVolume( Vol) Epr3Set(1, Vol)

// Role, 0:player, 1:user
#define RoleGet() Epr3Get(2)
#define RoleSet( R) Epr3Set(2, R)

#ifdef WITH_AUTODETECT
// TODO_LATER :  Db will start to play without contact (0 no autostart)
#define DetectLevGet() Epr3Get(3)
#define DetectLevSet( Val) Epr3Set(3, Val)
#endif /* WITH_AUTODETECT */

#define MaxGet() Epr3Get(4)
#define MaxSet( Val) Epr3Set(4, Val)

#define VerboseGet() Epr3Get(5)
#define VerboseSet( Val) Epr3Set(5, Val)

#define RssiCorrectionGet() Epr3Get(6)
#define RssiCorrectionSet( Val) Epr3Set(6, Val)

#define RssiMinimalGet() Epr3Get(7)
#define RssiMinimalSet( Val) Epr3Set(7, Val)

#define EprReqGet() Epr3Get(9)
#define EprReqSet( Val) Epr3Set(9, Val)

#define EPROM_POS_009 10

#ifdef WITH_WIFI
//#define WIFI_SRV_SSID_POS
//#define EPROM_POS EPROM_POS + WIFI_IDLEN

//#define WIFI_SRV_PWD_POS EPROM_POS
//#define EPROM_POS EPROM_POS + WIFI_PWDLEN

#define EPROM_POS_010 EPROM_POS_009+WIFI_IDLEN+WIFI_PWDLEN
#else
#define EPROM_POS_010 EPROM_POS_009
#endif /* WITH_WIFI */

#ifdef WITH_WIFICLIENT
#define WiCliSSIDGet() Epr3StrGet( EPROM_POS_010, CliSSID, WIFI_IDLEN)
#define WiCliSSIDSet() Epr3StrSet( EPROM_POS_010, CliSSID, WIFI_IDLEN)
#define EPROM_POS_010a EPROM_POS_010+WIFI_IDLEN

#define WiCliPwdGet() Epr3StrGet( EPROM_POS_010a, CliPWD, WIFI_PWDLEN)
#define WiCliPwdSet() Epr3StrSet( EPROM_POS_010a, CliPWD, WIFI_PWDLEN)

#define EPROM_POS_011 EPROM_POS_010+WIFI_IDLEN+WIFI_PWDLEN
#else
#define EPROM_POS_011 EPROM_POS_010
#endif /* WIFI_CLIENT */

#ifdef WITH_OSC
#define OscAddrGet() Epr3StrGet( EPROM_POS_011                , OscAddr, OSC_CMD_LEN)
#define OscAddrSet() Epr3StrSet( EPROM_POS_011                , OscAddr, OSC_CMD_LEN)

// TODO_LATER : spare memory if need some here on not defined functionnalities
#ifdef WITH_SWITCH
#define OscCmd1Get() Epr3StrGet( EPROM_POS_011+OSC_CMD_LEN  , OscCmd1, OSC_CMD_LEN)
#define OscCmd1Set() Epr3StrSet( EPROM_POS_011+OSC_CMD_LEN  , OscCmd1, OSC_CMD_LEN)

#define OscCmd2Get() Epr3StrGet( EPROM_POS_011+2*OSC_CMD_LEN, OscCmd2, OSC_CMD_LEN)
#define OscCmd2Set() Epr3StrSet( EPROM_POS_011+2*OSC_CMD_LEN, OscCmd2, OSC_CMD_LEN)
#endif /* WITH_SWITCH */

#ifdef WITH_ADC
#define OscCmd3Get() Epr3StrGet( EPROM_POS_011+3*OSC_CMD_LEN, OscCmd3, OSC_CMD_LEN)
#define OscCmd3Set() Epr3StrSet( EPROM_POS_011+3*OSC_CMD_LEN, OscCmd3, OSC_CMD_LEN)
#endif /* WITH_ADC */

#define EPROM_POS_012 EPROM_POS_011+4*OSC_CMD_LEN
#else
#define EPROM_POS_012 EPROM_POS_011
#endif /* WITH_OSC */

#define EprMotor0ModeGet() Epr3Get(EPROM_POS_012)
#define EprMotor0ModeSet( Val) Epr3Set(EPROM_POS_012, Val)

#define EprMotor1ModeGet() Epr3Get(EPROM_POS_012+1)
#define EprMotor1ModeSet( Val) Epr3Set(EPROM_POS_012+1, Val)

#define EprMotor2ModeGet() Epr3Get(EPROM_POS_012+2)
#define EprMotor2ModeSet( Val) Epr3Set(EPROM_POS_012+2, Val)

#define WifiAddsGet() Epr3Get(EPROM_POS_012+3)
#define WifiAddsSet( Val) Epr3Set(EPROM_POS_012+3, Val)

#define EPROM_POS_013 (EPROM_POS_012+4)

uint32_t MaxX;
#define MaxXGet() Epr3StrGet(EPROM_POS_013, (char*)&MaxX, 4)
#define MaxXSet() Epr3StrSet(EPROM_POS_013, (char*)&MaxX, 4)

uint32_t SpeedX; 
#define SpeedXGet() Epr3StrGet(EPROM_POS_013+4, (char*)&SpeedX, 4)
#define SpeedXSet() Epr3StrSet(EPROM_POS_013+4, (char*)&SpeedX, 4)
int32_t VectX = 77;
int32_t SensX = 1;
int32_t MyX = 0;

uint32_t MaxY; 
#define MaxYGet() Epr3StrGet(EPROM_POS_013+8, (char*)&MaxY, 4)
#define MaxYSet() Epr3StrSet(EPROM_POS_013+8, (char*)&MaxY, 4)

uint32_t SpeedY; 
#define SpeedYGet() Epr3StrGet(EPROM_POS_013+12, (char*)&SpeedY, 4)
#define SpeedYSet() Epr3StrSet(EPROM_POS_013+12, (char*)&SpeedY, 4)
int32_t VectY = 88;
int32_t SensY = 1;
int32_t MyY = 0;

#define EPROM_REQ_SIZE (EPROM_POS_013+16)

char Epr3Get( int Pos) {
  char V1;
  V1 = EEPROM.read( Pos);
  return ( V1);
}

void Epr3Set ( int Pos, char Val) {
  EEPROM.write( Pos, Val);
  // no more enough ressources to duplicate EEPROM.write( Pos+256, Val);
}

char Epr3StrGet( int Pos, char* Str, int Len) {
  int Idx = 0;
  while (Idx < Len) {
    Str[Idx] = Epr3Get( Pos + Idx);
    Idx++;
  }
  Str[Len - 1] = 0;
}

void Epr3StrSet ( int Pos, char* Str, int Len) {
  int Idx = 0;
  while (Idx < Len) {
    Epr3Set( Pos + Idx, Str[Idx]);
    Idx++;
  }
}


// check stored parameters are in correct ranges
void EpromSanity() {

  // DFPlayer volume for adjust
  if (GetVolume() > 99) {
    SetVolume( 90);
  }

  // Role check
  if ( RoleGet() > 9) {
    RoleSet( ROLE_DEFAULT);
  }

  // max number of tracks for SOLO mode
  if (ROLE_MULTI ==  RoleGet()) {
    if (MaxGet() > MAX_TRACK) {
      MaxSet( MAX_TRACK);
    }
  }

  if (NodeGet() > 99) {
    NodeSet( 1);
  }

#ifdef WITH_AUTODETECT
  if (DetectLevGet() > 99) {
    DetectLevSet( 0);
  }
#endif /* WITH_AUTODETECT */

  // debug level, see Verbose variable for definition
  if ((VerboseGet() > 9) && (VerboseGet() > Verbose)) {
    VerboseSet( Verbose);
  }
  Verbose = VerboseGet();

  g_iTmp = RssiCorrectionGet();

#ifdef WITH_WIFI
#ifdef ESP8266
  // 82 by Pwr, 64*2 by letters
  if ((g_iTmp < 128 - (MAX_TPW / 2) - MAX_LTR) || (g_iTmp > 128 + (MAX_TPW / 2) + MAX_LTR)) {
    RssiCorrectionSet( 128 - (MAX_TPW / 2));
  }
#else /* ESP8266 */
  if ((g_iTmp < 128 - MAX_LTR) || (g_iTmp > 128 + MAX_LTR)) {
    RssiCorrectionSet( 128);
  }
#endif /* ESP8266 */

  if ((0 > RssiMinimalGet()) || (RssiMinimalGet() > (-RSSI_MIN))) {
    //RssiMinimalSet( -RSSI_MIN);
    RssiMinimalSet( 100);
  }
#endif /* WITH_WIFI */


  if (EprReqGet() != (EPROM_REQ_SIZE) % 0x100) {
#ifdef WITH_WIFICLIENT
    CliSSID[0] = 0;
    WiCliSSIDSet();
    CliPWD[0] = 0;
    WiCliPwdSet();
#endif /* WITH_WIFICLIENT */

#ifdef WITH_OSC
    strcpy( OscAddr, OSC_DEFAULT_ADDR);
    OscAddrSet();
#ifdef WITH_SWITCH
    strcpy( OscCmd1, OSC_DEFAULT_CMD1);
    OscCmd1Set();
    strcpy( OscCmd2, OSC_DEFAULT_CMD2);
    OscCmd2Set();
#endif /* WITH_SWITCH */
#ifdef WITH_ADC
    strcpy( OscCmd3, OSC_DEFAULT_CMD3);
    OscCmd3Set();
#endif /* WITH_ADC */
#endif /* WITH_OSC */

#ifdef WITH_MOTOR
    EprMotor0ModeSet( MOTOR_UNDEF);
    EprMotor1ModeSet( MOTOR_UNDEF);
    EprMotor2ModeSet( MOTOR_UNDEF);
#endif /* WITH_MOTOR */

    WifiAddsSet( 1);

    EprReqSet( (EPROM_REQ_SIZE) % 0x100);

    MaxX=1000;
    MaxXSet();
    SpeedX=70;
    SpeedXSet();
    MaxY=2400;
    MaxYSet();
    SpeedY=80;
    SpeedYSet();

  }

#ifdef WITH_WIFICLIENT
  WiCliSSIDGet();
  WiCliPwdGet();
#endif /* WITH_WIFICLIENT */

#ifdef WITH_OSC
  OscAddrGet();
#ifdef WITH_SWITCH
  OscCmd1Get();
  OscCmd2Get();
#endif /* WITH_SWITCH */
#ifdef WITH_ADC
  OscCmd3Get();
#endif /* WITH_ADC */
#endif /* WITH_OSC */
}

// show eprom content on debug line
void EpromDumpBld( String& LocStr) {

  int Idx;
  int iTmp;

  LocStr += "\n";
  LocStr +=  BEN_TAG "Soft (c)ben\n";
#if defined( MODULE_LAAPIN)
  LocStr +=  ", HDW:Lapin";
#elif defined( MODULE_STEPPER)
  LocStr +=  ", HDW:Stepper";
#elif defined( MODULE_MWS)
  LocStr +=  ", HDW:MWS";
#elif defined( MODULE_TROUBLE)
  LocStr +=  ", HDW:Trouble";
#endif

  LocStr +=  ", rev. " HARDWARE_NAME;
  LocStr +=  " , nl&cr endline recommended\n";
  LocStr +=  "\n";

  LocStr  +=  "*" + CSTRI( NodeGet()) + "N :  xxN Node\n";

#ifdef WITH_DFPLAYER
  LocStr +=  "*" + CSTRI( GetVolume()) + "V :  xxV Volume\n";
  if (ROLE_MULTI ==  RoleGet()) {
    LocStr +=  "*" + String( (int)MaxGet()) + "M :  xxM Max stories\n";
  }
#endif /* WITH_DFPLAYER */

  LocStr +=  "*" + String( (int)RoleGet()) + "R :  xR Role ";
  switch ( RoleGet()) {
    //#ifdef WITH_DFPLAYER
    case ROLE_PASSENGER: LocStr += "passenger"; break;
    case ROLE_PLAYER:    LocStr += "player"; break;
    case ROLE_SOLO:      LocStr += "solo"; break;
    case ROLE_MULTI:     LocStr += "multi"; break;
    //#endif /* WITH_DFPLAYER */
    case ROLE_TEST:      LocStr += "test"; break;
    case ROLE_IOTM:      LocStr += "IotM"; break;
    case ROLE_IOTS:      LocStr += "IotS"; break;
    default:             LocStr += "undef"; break;
  }
  LocStr +=  "\n";
  LocStr +=  "     ( ";
#ifdef WITH_DFPLAYER
  LocStr +=    " " + CSTRI(  ROLE_PASSENGER)  + "  Passenger";
  LocStr +=  " , " + String( (int)ROLE_PLAYER) + "  Player";
  LocStr +=  " , " + String( (int)ROLE_SOLO)  + "  Solo";
  LocStr +=  " , " + String( (int)ROLE_MULTI) + "  Multi";
#endif /* WITH_DFPLAYER */
  LocStr +=  " , " + String( (int)ROLE_TEST)  + "  Test";
  LocStr +=  " , " + String( (int)ROLE_IOTM)  + "  IotM";
  LocStr +=  " , " + String( (int)ROLE_IOTS)  + "  IotS";
  LocStr +=  " )\n";

#ifdef WITH_AUTODETECT
  LocStr +=  "*" + CSTRI(DetectLevGet()) + "G :xxG Autodetect range\r\n";
#endif /* WITH_AUTODETECT */

  LocStr +=  "*" + CSTRI(Verbose) + "v :   xv verbose";
  LocStr +=  " (2 recommended)\r\n", Verbose;

#ifdef WITH_WIFICLIENT
  LocStr  +=  "*\"" ;
  LocStr  +=  (char*)CliSSID;
  LocStr  +=  "\"D : \"x" + CSTRI(WIFI_IDLEN - 1) + "x\"D SSID Wifi as client\n";
  LocStr  +=  "*\"";
  LocStr  +=  (char*)CliPWD; // ha ha won't tell you
  if (0 != CliPWD[0]) {
    //  LocStr += "*******************************";
  }
  LocStr  +=  "\"d : \"x" + CSTRI(WIFI_PWDLEN - 1) + "x\"d PWD Wifi as client\n"; // E

  #ifdef ESP8266_MULTIPERSO
  if (1 == WifiAddsGet()) {
    LocStr  +=  "*01 00 000(1)f Additionnal list activated\n";
  } else {
    LocStr  +=  "*01 00 000(0)f Additionnal list not activated\n";
  }
  #endif /* ESP8266_MULTIPERSO */
#endif /* WITH_WIFICLIENT */
#ifdef WITH_OSC
  LocStr  +=  "=== OSC ===\n" ;
  LocStr  +=  "*\"" ;
  LocStr  +=  (char*)OscAddr;
  LocStr  +=  "\"J : \"..." + CSTRI(OSC_CMD_LEN - 1) + "chars...\"J osc destination addr\n";

  #ifdef WITH_SWITCH
    LocStr  +=  "*\"" ;
    LocStr  +=  (char*)OscCmd1;
    LocStr  +=  "\"01K : \"..." + CSTRI(OSC_CMD_LEN - 1) + "chars...\"yyK osc sent on sw push\n";
    LocStr  +=  "*\"" ;
    LocStr  +=  (char*)OscCmd2;
    LocStr  +=  "\"02K : \"..." + CSTRI(OSC_CMD_LEN - 1) + "chars...\"yyK osc sent on sw released\n";
  #endif /* WITH_SWITCH */

  #ifdef WITH_ADC
    LocStr  +=  "*\"" ;
    if (0 == OscCmd3[0]) {
      LocStr  +=  "zzzz";
    } else {
      LocStr  +=  (char*)OscCmd3;
    }
    LocStr  +=  "\"03K : \"x" + CSTRI(OSC_CMD_LEN - 1) + "x\"yyK osc sent on ADC change (example \"_000XxxxS\"03K or \"_XxXxXx01L\"03K)\n";
  #endif /* WITH_ADC */
    LocStr  +=  "\"_16i\"T : send command to OSC\n";
  #endif /* WITH_OSC */

  // 00100200304L

  #ifdef WITH_WIFI
    LocStr  +=  "=== WiFi ===\n";
    LocStr += "*" + CSTRI(RssiCorrectionGet()) + "P : xxxP Rssi Correction";
    #ifdef ESP8266
      //LocStr +=  ", -%i.", (RssiCorrectionGet()-128)/4;
      //LocStr +=  ".%2i dB by pwr\r\n", (RssiCorrectionGet()-128)%4*25;
      LocStr +=  " (" + CSTRI(128 - (MAX_TPW / 2) - MAX_LTR) + " min";
      LocStr +=      ", [" + CSTRI(128 - (MAX_TPW / 2)) + "] min pwr";
      LocStr +=      ", " + CSTRI(128 + (MAX_TPW / 2)) + " max pwr";
      LocStr +=      ", " + CSTRI(128 + (MAX_TPW / 2 + MAX_LTR)) + " max";
      LocStr +=      ")\r\n";
    #else /* ESP8266 */
      LocStr +=  " (" + CSTRI(RssiCorrectionGet() - 128) + " dB)\r\n";
      LocStr +=  "      (127 is -1 adjust, 128 is 0 adjust, 130 is +2 adjust, etc)\n";
    #endif /* ESP8266 */
  #else /* WITH_WIFI */
    LocStr +=  "NO Wifi support\n"; //... without wifi this soft isn't that usefull... anyway it's your stuff ...
  #endif /* WITH_WIFI */

  LocStr +=  "*" + CSTRI(RssiMinimalGet()) + "m : xxxm -dB Minimal Rssi (nearer 0 is biger circle, 100 recommended (84 is 4 meters?))\n";

  LocStr +=  "*xxxxF : Custom function\n"; // example 01F open garage door
  LocStr +=  "       01F : Garage door\n";
  LocStr +=  "   xxxx02F : full led white\n";
  LocStr +=  "*ccnnxxxxf : set internal parameter class(c) Id(n) value(x)\n"; // example DC motor mode PWM "00 00 0002 f"

#ifdef WITH_DFPLAYER
  LocStr  +=  "xxyyT : play track\n";
#endif /* WITH_DFPLAYER */

#ifdef WITH_MOTOR
  for (Idx = 0; Idx < MOTOR_NB; Idx++) {
    Motor_t* pMotor = &(MotorArray[Idx]);
    iTmp = pr_int32_read( pMotor->WishP2);
    LocStr  +=  "*" + CSTRIn( 2, Idx) + CSTRIn( 4, iTmp) + "S : yyxxxxS motor pos for the next " + CSTRI( pr_uint32_read( pMotor->WishDTms)) + " ms\n";
    // LocStr  +=  "*" +CSTRI( StepperSpeedPerMilSec)+"s : xxxxx-s motor speed step per 1000s\n";
    iTmp = pr_uint32_read( pMotor->WishDTms);
    LocStr  +=  "*" + CSTRIn( 2, Idx) + CSTRIn( 4, iTmp ) + "l : yyxxxxl motor ms to reach the next 'S'\n";
  }
#endif /* WITH_MOTOR */

  MaxXGet();
  LocStr += "xxxxxxx01Q : MaxX "+ CSTRIn( 7, MaxX) + ", vertical or cam, typic pan 1000, typ rail 1000 \n";
  SpeedXGet();
  LocStr += "xxxxxxx02Q : SpeedX "+ CSTRIn( 7, SpeedX) + ", typic pan 70, typ rail 0(linked to Y) \n";
  MaxYGet();
  LocStr += "xxxxxxx03Q : MaxY "+ CSTRIn( 7, MaxY) + ", horiz or rail, typic pan 2400, typic rail 200000\n";
  SpeedYGet();
  LocStr += "xxxxxxx04Q : SpeedY "+ CSTRIn( 7, SpeedY) + " typin 80\n";

#ifdef WITH_WS2812
  LocStr  +=  "*rrggbbnnL : RGB for neopix n\n";
#endif /* WITH_WS2812 */

  LocStr  +=  "xxxxW : wait\n";
  LocStr  +=  "nnI : digital input 0 or 1\n";
  LocStr  +=  "nni : analog input\n";
  LocStr  +=  "xnnO : digital output 0 or 1\n";
  LocStr  +=  "xxxxnno : analog output x as 0000..9999\n";

  LocStr +=  "\n";

}

// move from position 1 to position 0xFFFFFFFF is 2 in distance and -1 in direction
int32_t Vectorize( uint32_t CurrentPos, uint32_t ExpectedPos) {
  int Dir;
  uint32_t Dist;

  ExpectedPos -= CurrentPos;
  if ( ExpectedPos > 0) {
    Dir = 1;
    Dist = ExpectedPos;
  } else {
    Dir = -1;
    Dist = -ExpectedPos;
  }

  if (Dist > 0x7FFFFFFF) {
    Dist = - Dist;
    Dir = Dir * -1;
  }

  return (Dir * Dist);
}
void DumpIp ( String& LocStr, uint32_t IpNum) {
  LocStr += (IpNum) % 0x100;
  LocStr += ".";
  LocStr += (IpNum / 0x100) % 0x100;
  LocStr += ".";
  LocStr += (IpNum / 0x10000) % 0x100;
  LocStr += ".";
  LocStr += (IpNum / 0x1000000) % 0x100;
}

#ifdef WITH_WIFICLIENT
int WifiCliConnected() {
  if (WifiState_TOCONNECT == WifiCliConnectState) {
    return false;
  } else {
#ifdef ESP8266
    return (wifiMulti.run() == WL_CONNECTED);
#else
    return ( WiFi.status() == WL_CONNECTED);
#endif /* */
  }
}

void WifiCliLoop() {
int SthToConnect = 0;
#ifdef WITH_WIFI
  if ((WifiState_JUSTCONNECTED == WifiCliConnectState) && WifiCliConnected()) {
    // detect lan conflict, not perfect but it do the tricks
    uint32_t IpNumSrv = 0;
    uint32_t IpNumCli = 0;

    IpNumSrv = WiFi.softAPIP();
    IpNumCli = WiFi.localIP();
    if ( (IpNumSrv & 0xFFFFFF) == (IpNumCli & 0xFFFFFF) ) {
      // try to act clean
#ifdef WITH_OSC
      if (1 == OscWifiSrvState) {
        // TODO_LATER : soupcons que ca cupe debug OscUdpSrv.stop();
        OscWifiSrvState = 0;
      }
#endif /* WITH_OSC */
      dbgprintf( 2, " Warn - solve lan conflict", IpNumSrv);
      dbgprintf( 2, " Srv 0x%x ", IpNumSrv);
      dbgprintf( 2, " Cli 0x%x\n", IpNumCli);
      // softAPConfig  ( local_ip  , gateway   , subnet    );
      WiFi.softAPConfig( 0x0105A8C0, 0x0105A8C0, 0x00FFFFFF);
    }
    WifiCliConnectState = WifiState_STABLE;
    dbgprintf( 2, " Wifi connected as client is stable\n");
  }
#endif /* WITH_WIFI */

  if (WifiState_TOCONNECT == WifiCliConnectState) {
    if (0 != CliSSID[0]) {
      #ifdef ESP8266
        // wifiMulti.APlistClean();
        wifiMulti.addAP( (char*)CliSSID, (char*)CliPWD);
      #else
        WiFi.begin( (char*)CliSSID, (char*)CliPWD);
      #endif
      SthToConnect = 1;
    }
    #ifdef ESP8266_MULTIPERSO
      if (1 == WifiAddsGet()) {
        ESP8266_MULTIPERSO
        // in mines.h, half dozen of wifiMulti.addAP( "myap", "mypwd");
        SthToConnect = 1;
      }
    #endif
    if (1 == SthToConnect) {
      WifiCliConnectState = WifiState_JUSTCONNECTED; // give a loop run chance for other tasks
    }
  }
}
#endif /* WITH_WIFICLIENT */

#ifdef WITH_WIFI
int WifiSrvConnected() {
  return (0 != WiFi.softAPIP());
}
#endif /* WITH_WIFI */

// show eprom content on debug line
void StateDumpBld_1( String& LocStr) {
  int Idx;

#ifdef WITH_DFPLAYER
  LocStr += "-Player:";
  LocStr += (PlayerHealth > 0) ? "Yes" : "Ko";
  LocStr += "\n";
#endif /* WITH_DFPLAYER */

#ifdef SWITCH_LINE
  LocStr += "-Switch ";
  LocStr +=  CSTRI(SwitchState) + " ";
  LocStr +=  CSTRI(digitalRead( SWITCH_LINE)) + " readed";
  LocStr += "\n";
#endif /* SWITCH_LINE */

#ifdef RESTART_PIN
  LocStr += "-RestartA ";
  LocStr +=  CSTRI(RestartSwitchState) + " ";
  LocStr +=  CSTRI(digitalRead( RESTART_PIN)) + " readed";
  LocStr += "\n";
#endif /* RESTART_PIN */

#ifdef RESTART_B_PIN
  LocStr += "-RestartB ";
  LocStr +=  CSTRI(RestartBSwitchState) + " ";
  LocStr +=  CSTRI(digitalRead( RESTART_B_PIN)) + " readed,";
  LocStr +=  CSTRI(RestartedGuy) + " Restarted guy";
  LocStr += "\n";
#endif /* RESTART_B_PIN */

#ifdef WITH_WIFI
  LocStr += "-Wifi as server:";
  if (0 != SrvSSID[0]) {
    if (WifiSrvConnected()) {
      LocStr += " acivated ";
      DumpIp (LocStr, WiFi.softAPIP());
      LocStr += "\n";
    } else {
      LocStr += " not activated ";
      LocStr += WiFi.status();
      LocStr += "\n";

      if ( ROLE_PLAYER == RoleGet()) {
        LocStr += "  Player means wifi scan ... do not like client connector\n";
      }
    }
    LocStr  +=  "  \"" ;
    LocStr  +=  (char*)SrvSSID;
    LocStr  +=  "\" :  SSID\n"; // D
    LocStr  +=  "  \"";
    LocStr  +=  (char*)SrvPWD; // ha ha won't tell you
    if (0 != SrvPWD[0]) {
      //  LocStr += "*******************************";
    }
    LocStr  +=  "\" :  PWD\n"; // E
  } else {
    LocStr += "  Wifi as server no SSID\n"; // check IOTMUTUAL
  }
#endif /* WITH_WIFI */

#ifdef WITH_WIFICLIENT
  LocStr += "-Wifi as client:";
  if ( WifiCliConnected()) {
    LocStr += " connected ";
    DumpIp (LocStr, WiFi.localIP());
    LocStr += " st:" + CSTRI( WifiCliConnectState);
    LocStr += "\n";
  } else {
    LocStr += " not connected ";
    LocStr += WiFi.status();
    LocStr += " st:" + CSTRI( WifiCliConnectState);
    LocStr += "\n";

    if ( ROLE_PLAYER == RoleGet()) {
      LocStr += "  Player means wifi scan ... do not like client connector\n";
    }
  }
#endif /* WITH_WIFICLIENT */

#ifdef WITH_OSC
  LocStr += "-Osc";

#ifdef WITH_WIFICLIENT
  LocStr += ", CliState:";
  LocStr += CSTRI( OscWifiCliState);
#endif /* WITH_WIFICLIENT */
  LocStr += ", SrvState:";
  LocStr += CSTRI( OscWifiSrvState);

  LocStr += ", MyId:";
  LocStr += OscAddrMine;
  LocStr += "\n";
  LocStr += "rem : received command starting with '_' will be sent to the command line\n";
#endif /* WITH_OSC */

  LocStr += "-Eprom reqs:";
  LocStr += EPROM_REQ_SIZE;
  LocStr += "\n";
}

void StateDumpBld_2( String& LocStr) {
  int Idx;

#ifdef WITH_MOTOR

  for (Idx = 0; Idx < MOTOR_NB; Idx++) {
    Motor_t* pMotor = &(MotorArray[Idx]);
    LocStr  +=  "-Motor:" + CSTRI(Idx) + ", pos:" + CSTRI( pr_uint32_read(pMotor->Pos))+ "\n";
    LocStr  +=  "  , CommandUs:" + CSTRI( pMotor->MinCommandUs) + "us\n";
    LocStr  +=  "  , Mode:" + CSTRI( pMotor->DriverMode) ;
      switch( pMotor->DriverMode) {
        case MOTOR_UNDEF : LocStr  +=  " , undefined";break;
        case MODOR_DC_LR : LocStr += " , DC_LR, A:" + CSTRI( pMotor->Pin1) + ", B:" + CSTRI( pMotor->Pin2); break;
        case MOTOR_PWM   : LocStr  +=   " , PWM" ; break;
        case MOTOR_PWM2  : LocStr  +=   " , PWM2"; break;
        case MOTOR_STEPPER :   
           LocStr += " , Stepper PulsP:" + CSTRI( pMotor->Pin2);
           LocStr += ", DirP:" + CSTRI( pMotor->Pin1);

        break;
        #ifdef WITH_SERVO
          case MOTOR_SERVO : LocStr +=  " SERVO"; break;
        #endif /* WITH_SERVO */
        default : LocStr  +=  "undef"; break;
      }
      LocStr += "\n";
  }

  LocStr  +=  " change mode '00 xx 000yf', xx is motor num\n";
  for (int i =0; i< MOTOR_MAXDEFS; i++) {
      LocStr  += "    y is ";
      switch( i) {
        case MOTOR_UNDEF : LocStr  +=  CSTRI(i) + " default\n";break;
        case MODOR_DC_LR : LocStr  +=  CSTRI(i) + " DC_LR\n"  ; break;
        case MOTOR_PWM   : LocStr  +=  CSTRI(i) + " PWM\n" ; break;
        case MOTOR_PWM2  : LocStr  +=  CSTRI(i) + " PWM2\n"; break;
        case MOTOR_STEPPER : LocStr  +=  CSTRI(i) + " STEPPER\n"; break;
        #ifdef WITH_SERVO
          case MOTOR_SERVO : LocStr += CSTRI(i) + " SERVO\n"; break;
        #endif /* WITH_SERVO */
        default : LocStr  +=  CSTRI(i) + "undef\n"; break;
      }
  }
#endif /* WITH_MOTOR */

#ifdef WITH_DCMOTOR
  
  LocStr += "\n";
#endif /* WITH_DCMOTOR */

#ifdef WITH_SERVO
  LocStr += "-Servo pins:{ ";
  for ( uint8_t i = 0; i < ServoNb; i++) {
    LocStr += CSTRI(ServoPins[i]) + ", ";
  }
  LocStr += "}\n";
#endif /* WITH_STEPPER */

#ifdef WITH_STEPPER_IN
  LocStr  +=  "-StepperIn pos:" + CSTRI( pr_uint32_read( pr_StepperInPos));
#ifdef STEPPER_IN_DIR_PIN
  LocStr += ", D:" + CSTRI( STEPPER_IN_DIR_PIN);
#endif
#ifdef STEPPER_IN_PULSE_PIN
  LocStr += ", P:" + CSTRI( STEPPER_IN_PULSE_PIN);
#endif
  //LocStr += ", p:" + CSTRI( StepperInPulse);
  LocStr += "\n";
#endif /* WITH_STEPPER_IN */

#ifdef WITH_GRAYCODE_IN
  LocStr  +=  "-Graycode pos:" + CSTRI( pr_uint32_read( PrGrayPos));
#ifdef GRAYCODE_IN_A_PIN
  LocStr += ", A:" + CSTRI( GRAYCODE_IN_A_PIN);
#endif
#ifdef GRAYCODE_IN_B_PIN
  LocStr += ", B:" + CSTRI( GRAYCODE_IN_B_PIN);
#endif
  LocStr += ", GL:" + CSTRI( GrayA) + CSTRI( GrayB);
  LocStr += "\n";
#endif /* WITH_GRAYCODE_IN */

#ifdef WITH_ADC
  LocStr  +=  "-Adc:" + CSTRI( AdcVal) + ", pin:" + CSTRI(A0);
#ifndef WITH_WIFI
  LocStr  +=  " Raw:" + CSTRI( analogRead(WITH_ADC)); // suspect ADC and http weirdness
#endif /* WITH_WIFI */
  LocStr  +=  "\n";
#endif /* WITH_ADC */

#ifdef STOPPER_MIN_PIN
  LocStr  +=  "-StopMin :" + CSTRI( digitalRead( STOPPER_MIN_PIN)) + ", pin:" + CSTRI(STOPPER_MIN_PIN) +"\n";
#endif /* STOPPER_MIN_PIN */
#ifdef STOPPER_MAX_PIN
  LocStr  +=  "-StopMax :" + CSTRI( digitalRead( STOPPER_MAX_PIN)) + ", pin:" + CSTRI(STOPPER_MAX_PIN) +"\n";
#endif /* STOPPER_MAX_PIN */

  TimeRunningDump( LocStr);
  
}

void EpromDump() {
  String LocStr;

  EpromDumpBld( LocStr);
  StateDumpBld_1( LocStr);
  StateDumpBld_2( LocStr);
  Serial.print( LocStr);
  RunOtherTasks( 0); // lot of print, release a little bit
}

void SetNodeNum( int Num) {
  Num = Num % 100;
  NodeSet(Num);

  if (ROLE_PASSENGER == RoleGet()) {
    if (NodeGet() > MAX_STORY) {
      NodeSet( MAX_STORY);
    }
  } else if (ROLE_PLAYER == RoleGet()) {
    if (NodeGet() > MAX_TRACK) {
      NodeSet( MAX_TRACK);
    }
  }

#ifdef WITH_WIFI
  if (ROLE_PASSENGER == RoleGet()) { // user change id
    WifiInit();
  }
#endif /* WITH_WIFI */
}

void MyPinmode( uint8_t Pin, int Mode) {
#ifdef ESP8266
  if ((16 == Pin) && (INPUT_PULLUP == Mode)) {
    // rem : if RESTART_PIN is 16 on ESP8266 a physical pullup (10kOhm?) is required
    // rem : seems no interrupt either
    Mode = INPUT;
    dbgprintf( 1, "Warn, Restart SwitchInit %i specific (pulldown)\n", Pin);
  }
#endif /* #ifdef ESP8266 */

  if ((Pin < 20) && (PinModes[Pin] != Mode)) {
    pinMode( Pin, Mode);
    PinModes[Pin] = Mode;
  } else {
    dbgprintf( 1, "Warn, duplicated init %i \n", Pin);
    pinMode( Pin, Mode);
  }
}

#ifdef WITH_WS2812 // -------------------------------

void PixelSet99( uint8_t R99, uint8_t G99, uint8_t B99, uint8_t Pos) {
  if ( Pos < PIX_NB) {
    pixels.setPixelColor( Pos, pixels.Color( map( R99, 0, 99, 0, 255)
                          , map( G99, 0, 99, 0, 255)
                          , map( B99, 0, 99, 0, 255)));
  }
}

// Description : init devices (setup only)
int PixelInit( void) {
  int i;

  MyPinmode( WITH_WS2812, OUTPUT);
  digitalWrite( WITH_WS2812, LOW);
  pixels.begin();
  for (i = 1; i< PIX_NB; i++) {
    // PixelSet99( 99, 99, 99, i);
    PixelSet99( 0, 0, 1, i);
  }
  PixelSet99( 1, 1, 1, 0);
  pixels.show();
  return ( 0);
}

// display stte to leds
// WithStatus 0- don't show, 1- show, 3- was las time to show or not ...
#define RSMIN -60
#define RSMAX -7

#ifdef WITH_LED_SHOW_WIFI
int PixelShow( int WithStatus) {
  int i = 0;
  for (i = 0; i < (1 + PLAYERS_NB); i++)
  {
    int R = 0;
    int G = 0;
    int B = 0;

    if (0 == i) { // status
      switch ( PixState) {
        case ACT_DONTSHOW :
          R =  0; G =  0; B =   0; break; // Blue
        case ACT_ININIT:
          R = 128; G = 128; B =   0; break; // orange
        case ACT_PLAY :
          R =  0; G = 255; B =   0; break; // Green
        case ACT_READY :
          R = 255; G = 255; B = 255; break; // White
        default :
          R =  0; G =  0; B = 255; break; // Blue
      }
      if (3 != WithStatus) {
        PixView = WithStatus;
      }
      if (0 == PixView) {
        R =  0; G =  0; B =   0;
      }
      RssiTmp = RSMIN + 5;
    } else {
      switch (PlayersId[i - 1]) {
        case 0 : R =  0; G =  0; B =   0; break;
        case 1 : R = 255; G =  0; B =   0; break; // Red
        case 2 : R =  0; G = 255; B =   0; break; // Green
        case 3 : R =  0; G =  0; B = 255; break; // Blue

        case 4 : R = 128; G = 128; B =   0; break; // orange
        case 5 : R = 128; G =  0; B = 128; break;
        case 6 : R =  0; G = 128; B = 128; break;

        case  7 : R = 170; G =  85; B =  0; break;
        case  8 : R = 170; G =   0; B = 85; break;
        case  9 : R = 85; G = 170; B = 0; break;
        case 10 : R =  0; G = 170; B = 85; break;
        case 11 : R = 85; G =  0; B = 170; break;
        case 12 : R =  0; G = 85; B = 170; break;

        default : R = 255; G = 255; B = 255; break; // White
      }
      RssiTmp = constrain( PlayersRssi[i - 1], RSMIN + 1, RSMAX);
      //dbgprintf( 3, "Id %i\r\n", PlayersId[i]);
    }
    R = map( RssiTmp, RSMIN, RSMAX, 0, R);
    G = map( RssiTmp, RSMIN, RSMAX, 0, G);
    B = map( RssiTmp, RSMIN, RSMAX, 0, B);

    if ((0 != RssiMinimalGet()) && (PlayersRssi[i - 1] < (-RssiMinimalGet()))) {
      R = G = B = 0;
    }
    //dbgprintf( 2, "PixShow in %i", i);
    //dbgprintf( 2, " R%i", R);
    //dbgprintf( 2, " G%i", G);
    //dbgprintf( 2, " B%i\r\n", B);

    pixels.setPixelColor( i, pixels.Color( R, G, B));
  }
  pixels.show();
  //dbgprintf( 2, "PixShow val1 %i\r\n", pixels.getPixelColor(1));
  //dbgprintf( 2, "PixShow end %i\r\n", i);

  return ( 0);
}
#endif /* WITH_LED_SHOW_WIFI */

void PixelShow99( uint8_t R99, uint8_t G99, uint8_t B99, uint8_t Pos) {
  PixelSet99( R99, G99, B99, Pos);
  if ( Pos < PIX_NB) {
    pixels.show();
  }
}

#endif /* WITH_WS2812 ------------------------*/

#ifdef WITH_DFPLAYER // ------------------------pixel

// wait response during maximum TimeWait, read during TimeOff
// TimeWait - wait for a frame to start
// TimeOff  - wait for the frame to complete
int DFPlayerWaitBlocking( uint8_t* RecvBuf, int Max, uint16_t TimeWait, uint16_t TimOff) {
  uint32_t TimeSt;
  uint32_t TimeEndCh;
  uint8_t Received = 0;

  dbgprintf( 3, "DFPlayerWaitBlocking", TimeWait);
  dbgprintf( 3, " %i", TimeWait);
  dbgprintf( 3, " %i", TimOff);
  dbgprintf( 3, "\r\n");

  // wait for TimeWait at max
  TimeSt = millis();
  while ((DFPSerial.available() <= 0) && (abs(millis() - TimeSt) < TimeWait)) {
    RunOtherTasks( 1);
  }
  // dbgprintf( 3, "PlayerFrom");
  // dbgprintf( 3, " %ims", millis()-TimeSt);

  // get 'til inactiv for at least TimOff
  TimeEndCh = millis();
  do {
    RunOtherTasks( 1);
    if (DFPMockFrameCnt > 0) {
      (RecvBuf + Received)[0] = DFPMockFrame[DFPMockFrameCnt - 1];
      Received += 1;
      DFPMockFrameCnt--;
    } else if (DFPSerial.available() > 0) {
      g_iTmp = min( DFPSerial.available(), Max - Received);
      Received += DFPSerial.readBytes( (char*)RecvBuf + Received, g_iTmp);
      TimeEndCh = millis();
    }
  } while ( (DiffTime( millis(), TimeEndCh) < TimOff) && (Received < Max));
  // dbgprintf( 3, "-%ims ", millis()-TimeSt);

  return ( Received);
}

int DFPlayerMockAddCh( uint8_t Ch) {
  DFPMockFrame[DFPMockFrameCnt] = Ch;
  DFPMockFrameCnt++;
}

// wait response and extract datas from frame if CRC is ok
// during maximum TimeWait, read during TimeOff plus treatment for extract
int DFPlayerWaitResult( uint8_t* pCmd, uint8_t* pFbk, uint8_t* pP1, uint8_t* pP2, uint16_t TimeWait = 400, uint16_t TimeStop = 100) {

  int Res = 0;

  uint8_t RecvBuf[DFP_BUFFZSIZE];
  uint8_t Count = 0;
  uint8_t Len = 0;
  uint16_t Crc = 0; // TODO_LATER : hummm seems ESP8266/arduino mess btw 16 and 32 ...
  uint16_t CrcR = 0;

  // wait response during maximum TimeWait, read during TimeOff
  // TimeOff 9B -> 100b 9600p.s-1 -> 11ms, give 50ms-> time for 48B
  Len = DFPlayerWaitBlocking( RecvBuf, DFP_BUFFZSIZE, TimeWait, 50);

  if (Len > 0) {
    dbgprintf( 3, "Player  recv %i ", Len);
    if (0x7E != RecvBuf[Count++]) {
      //dbgprintf( 3,"bad start0\n");
      Res = -1;
    }
    Crc += RecvBuf[Count];
    Count++;// VersionLow?
    //dbgprintf( 3,"ver 0x%2.2X ", RecvBuf[Count]);
    Crc += RecvBuf[Count];
    Count++;// who cares version?
    //dbgprintf( 3,"cmd 0x%02X ", RecvBuf[Count]);
    Crc += RecvBuf[Count];
    *pCmd = RecvBuf[Count];
    Count++; // Cmd
    //dbgprintf( 3,"fbk 0x%02X ", RecvBuf[Count]);
    Crc += RecvBuf[Count];
    *pFbk = RecvBuf[Count];
    Count++; // Feedback
    //dbgprintf( 3,"P1 0x%02X P2 0x%02X ", RecvBuf[Count], RecvBuf[Count+1]);
    Crc += RecvBuf[Count];
    *pP1 = RecvBuf[Count];
    Count++; // P1
    Crc += RecvBuf[Count];
    *pP2 = RecvBuf[Count];
    Count++; // P2
    CrcR = RecvBuf[Count++] * 0x100;
    CrcR += RecvBuf[Count++];
    //dbgprintf( 3,"CrcR 0x%04X exp 0x%04X ", CrcR, (-Crc)&0xFFFF);
    if (CrcR != ((-Crc) & 0xFFFF)) {
      dbgprintf( 3, "-bad Crc-");
      Res = -2;
    }
    //dbgprintf( 3,"End 0x%2.2X\r\n ", RecvBuf[Count]);
    if (0xEF != RecvBuf[Count++]) {
      //dbgprintf( 3,"bad End\n");
      Res = -3;
    }
    for (int i = 0; i < Count; i++) {
      dbgprintf( 3, "-%02X", RecvBuf[i]);
    }
    if (0 == Res) {
      dbgprintf( 3, " Ok\r\n");
    } else {
      dbgprintf( 3, " ERR %i\r\n", Res);
    }
  } else { // no recv
    Res = -1;
    dbgprintf( 3, "Player no recv\r\n");
  }
  return ( Res);
}

int DFPlayerSendCmd( uint8_t Cmd, uint8_t Fbk, uint8_t P1, uint8_t P2) {
  uint8_t SndBuf[10];
  int Count = 0;
  uint16_t Crc = 0;


  dbgprintf( 3, "Player To");
  dbgprintf( 3, " Cmd  0x%02X", Cmd);
  SndBuf[Count] = 0x7E;
  //dbgprintf( 3, "Cmd  0x%02X", SndBuf[Count]);
  Count++;
  SndBuf[Count] = 0xFF; // Ver
  Crc += SndBuf[Count++];
  SndBuf[Count] = 0x06;
  Crc += SndBuf[Count++];
  SndBuf[Count] = Cmd;
  Crc += SndBuf[Count++];
  SndBuf[Count] = Fbk;
  Crc += SndBuf[Count++];
  SndBuf[Count] = P1;
  Crc += SndBuf[Count++];
  SndBuf[Count] = P2;
  Crc += SndBuf[Count++];
  Crc = -Crc;
  SndBuf[Count] = Crc / 256;
  Count++;
  SndBuf[Count] = Crc % 256;
  Count++;
  SndBuf[Count] = 0xEF;
  Count++;

  for (int i = 0; i < Count; i++) {
    dbgprintf( 3, "-%02X", SndBuf[i]);
  }
  dbgprintf( 3, "\r\n");

  DFPSerial.write( SndBuf, Count);

  return (0);
}

// send a command and retry until succeeded, >=0 on success. MsWaitPerLoop - time to wait for response. Tries - number of times to try at max, >=1
int DFPlayerCmdInsist( uint8_t CmdIn, uint8_t P1In, uint8_t P2In, int MsWaitPerLoop, int Tries) {
  int Res = -1;
#ifdef WITH_DFPLAYER
  uint8_t Cmd;
  uint8_t Fbk;
  uint8_t P1;
  uint8_t P2;

  while ((Res < 0) && (Tries > 0)) {
    DFPlayerSendCmd( CmdIn, 1, P1In, P2In);
    Res = DFPlayerWaitResult( &Cmd, &Fbk, &P1, &P2, MsWaitPerLoop);// typical respond in 100ms
    if (Res >= 0)
      switch (Cmd) {
        //case 0x43: // 67 Get volume
        // break; // TODO_LATER : some commands
        case 0x42: // 66 status
          if (0x42 != Cmd) {
            Res = -1;
          }
          Res = P2;

          break;
        case 0x46: // 70 version
          if (CmdIn != Cmd) {
            Res = -1;
          } else {
            // dbgprintf( 3,"DFPlayer V%i.%i\r\n", P1, P2);
          }
          break;
        default : // most respond 0x41 0x00 0x00
          // case 0x06: // Volume
          // case 0x0F: // play by folder /P1/P2x.mp3
          if ( (0x41 != Cmd) || (0x00 != P1) || (0x00 != P2)) {
            Res = -1;
          }
          break;
      }
    Tries--;
  }
#endif /* WITH_DFPLAYER */
  return ( Res);
}

#endif /* WITH_DFPLATER ---------------------------------- */

// -1 no resp after 2 tries, 0 not play, >0 play
int TrackIsRunning() {
  int Res = -1;

#ifdef WITH_DFPLAYER
  Res = DFPlayerCmdInsist( 0x42, 0, 0, 100, 2);
#endif /* WITH_DFPLAYER */
  return ( Res);
}

/*send the track to play to DFP and check it runs */
int PlayTrack( int TrackToPlay) {
  int Res = -1;
  uint8_t Cmd;
  uint8_t Fbk;
  uint8_t P1;
  uint8_t P2;
  int Tries = 4;

#ifdef WITH_DFPLAYER
  // decimal xxyy play folder /xx/0yy.mp3
  dbgprintf( 2, "DFP play track %i\r\n", TrackToPlay);


  // Tries x ( send play + IntlTries x (check play))
  while ((Res < 0) && (Tries > 0)) {

    //DFPlayerMockAddCh( 0xca); // <- inject an error to test resilience
    // send a command to play, >=0 on success. 100 - time to wait for response. 1 - number of times to try at max, >=1
    if (DFPlayerCmdInsist( 0x0F, (TrackToPlay / 100) % 100, TrackToPlay % 100, 0, 1) >= 0) {
      Res = 1;
      RunOtherTasks( 100); // resp but consider 100ms for track to really start or next Track check might be irrevealeant
      return (Res);
    }
    //DFPlayerMockAddCh( 0xca); // <- inject an error to test resilience
    RunOtherTasks( 100); // no resp but consider 100ms for track to eventually start

    if (Res < 0) {
      Res = TrackIsRunning(); // -1 no resp, 0 not play, >0 play
      dbgprintf( 2, " Track Reply %i - ", Res);
      if ( Res > 0) {
        dbgprintf( 2, "playing despite direct reply fail\n");
        Res = 1;
        return (Res);
      } else if (0 == Res) {
        // responded not playing, need retry play
        dbgprintf( 2, "respond not playing\n");
        Res = -1;
      } else { // not responded
        dbgprintf( 2, "maybe not playing\n");
      }
    }

    Tries --;
  }
  dbgprintf( 2, "mostly not playing\n"); // except if go line is ok and reply line cut ...

#else
  dbgprintf( 3, "\r\n- no hardware to play %i \r\n", TrackToPlay);
#endif

  return ( Res);
}

// set volume 0..99
void PlayerVolumeSet( int Volume) {
  int Vl;
  Vl =   Volume * 30 / 99; // 0.99 --> 0..100
#ifdef WITH_DFPLAYER
  DFPlayerCmdInsist( 0x06, 0x00, Vl, 200, 5); // 1 sec
#endif /* WITH_DFPLAYER */
}

void PlayerInit() {
  int Res;
#ifdef WITH_DFPLAYER

  MyPinmode( DfpPins[1], OUTPUT); // Tx
  digitalWrite( DfpPins[1], LOW);
  MyPinmode( DfpPins[0], INPUT); // Rx
  //digitalWrite( DfpPins[0], LOW);
  DFPSerial.begin(9600);
  DFPSerial.setTimeout(0); // ms

  if ((ROLE_PLAYER == RoleGet()) || (ROLE_TEST == RoleGet()) || (ROLE_SOLO == RoleGet()) || (ROLE_MULTI == RoleGet())) {
    Res = DFPlayerCmdInsist( 0x46, 0, 0, 300, 4) ; // Version 8 sec, between 2 and 5 sec typically
  } else {
    Res = DFPlayerCmdInsist( 0x46, 0, 0, 100, 1) ; // not player, not insist
  }
  PlayerVolumeSet( GetVolume()); // 0x06
  if (Res < 0) {
    PlayerHealth = 0;
    dbgprintf( 3, "NO DFPlayer\n");
  } else {
    dbgprintf( 3, "DFPlayer respond\r\n");
    PlayerHealth = 1;
  }

#endif
}

void TrackRun() {
#ifdef WITH_DFPLAYER
  int Track;
  if (ROLE_PASSENGER == RoleGet()) {
    Track = 9900 + NodeGet() % 100;
    // unexpected but might be fun
    // dbgprintf( 3,"\r\n - unexpected for a user %i \r\n", PlayersId[0]);
    PlayTrack( Track);
    // Play the expected track
  } else if (ROLE_SOLO == RoleGet()) {
    Track = NodeGet();
    PlayTrack( Track);
  } else if (ROLE_MULTI == RoleGet()) {
    // passenger moves from player to player
    Track = (100 * (PlayersId[0] % 100))
#ifdef RESTART_B_PIN
            + RestartedGuy * MaxGet()
#endif
            + NodeGet() % 100;
    PlayTrack( Track);
  } else {
    // passenger moves from player to player
    Track = (100 * (PlayersId[0] % 100)) + NodeGet() % 100;
    PlayTrack( Track);
  }
#endif /* WITH_DFPLAYER */
}

void TrackStop() {
  int Res;

  // ROLE_PLAYER
#ifdef WITH_DFPLAYER
  Res = DFPlayerCmdInsist( 0x16, 0, 0, 2, 2);
#endif /* WITH_DFPLAYER */
}

// #endif /* WITH_DFPLAYER */

#ifdef WITH_OSC

// received commands from OSC to process
void OscMine( OSCMESSAGE &Msg) {
  //get 1st argument(int32)
  char Command[OSC_CMD_LEN];

  Msg.getString( 0, Command, OSC_CMD_LEN - 1);
  Command[OSC_CMD_LEN - 1] = 0;
  dbgprintf( 3, " OscMine %8ims - %s\n", DiffTime( OscLastSendMs, millis()), Command);
  if ( !OscMsgExec( Command)) {
    // HI_GIRL : here your specific commands from OSC
    dbgprintf( 2, " Ignored specific not '_' -%s-\n", Command);
  }
}

// build and send frame
void OscSendCmdAct( int CmdNum) {
  OSCMessage Msg( OscAddr);
  uint8_t ReadyToSend = 0;
  int Res;
  char Cmd[40];
  int Pos = 0;
  int Chf = 0;
  int SeqChf = 0;
  int pattern_offset;
  int address_offset;
  //String LocStr;

  OscLastSendMs = millis();
  switch (CmdNum) {
#ifdef WITH_SWITCH
    case 1 :
      if (0 != OscCmd1[0]) {
        strncpy( Cmd, OscCmd1, OSC_CMD_LEN);
        ReadyToSend = 1;
      }
      break;
    case 2 :
      if (0 != OscCmd2[0]) {
        strncpy( Cmd, OscCmd2, OSC_CMD_LEN);
        ReadyToSend = 1;
      }
      break;
#endif /* WITH_SWITCH */
#ifdef WITH_ADC
    case 3 :
      if (0 != OscCmd3[0]) {
        strncpy( Cmd, OscCmd3, OSC_CMD_LEN);
        ReadyToSend = 1;
      }
      break;
#endif /* WITH_ADC */
    case 4:
      strncpy( Cmd, (char*)DbgStr, OSC_CMD_LEN);
      ReadyToSend = 1;
      break;
  }
  if (ReadyToSend) {
    Chf = 0;
    SeqChf = 0;
    for (Pos = 0; Pos < strlen( Cmd); Pos++) {
      if ( 'X' == Cmd[Pos]) {
        Cmd[Pos] = 'x'; Chf = 0;
      }
      if ( 'x' == Cmd[Pos]) {
        switch ( Chf) {
#ifdef WITH_ADC
          case 0: Cmd[Pos] = '0' + ((AdcVal / 1000) % 10); Chf++; break;
          case 1: Cmd[Pos] = '0' + ((AdcVal / 100 ) % 10); Chf++; break;
          case 2: Cmd[Pos] = '0' + ((AdcVal / 10  ) % 10); Chf++; break;
          case 3: Cmd[Pos] = '0' + ((AdcVal     ) % 10); Chf++; break;
#endif /* WITH_ADC */
          default: Cmd[Pos] = '0'                  ; Chf++; break;
        }
      }
      if ('n' == Cmd[Pos]) {
        switch (SeqChf) {
          case 1: Cmd[Pos] = '0' + ((OscCmdSequence     ) % 10); SeqChf++; break;
          case 2: Cmd[Pos] = '0' + ((OscCmdSequence / 10  ) % 10); SeqChf++; break;
          case 3: Cmd[Pos] = '0' + ((OscCmdSequence / 100 ) % 10); SeqChf++; break;
          case 4: Cmd[Pos] = '0' + ((OscCmdSequence / 1000) % 10); SeqChf++; break;
          default: Cmd[Pos] = '0'                  ; SeqChf++; break;
        }
      }
    }
    dbgprintf( 3, "OscSendCmd %s \n", Cmd);
    Msg.add( Cmd);
    // to the wifi we connected to
    if (0 != OscWifiCliState) {
      #ifdef ESP8266
        Res = OscUdpCli.beginPacketMulticast( WiFi.localIP() | 0xFF000000, OSC_PORT, WiFi.localIP());// 0xFF000000 beware of the endians
      #else
        Res = OscUdpCli.beginPacket( WiFi.localIP() | 0xFF000000, OSC_PORT);// TODO_HERE : test
      #endif
      Msg.send( OscUdpCli);
      Res = OscUdpCli.endPacket();// Finish off this packet and send it
    }
    // to me
    Msg.dispatch( OscAddrMine, OscMine);
    // to the generated wifi
    if (0 != OscWifiSrvState) {
      #ifdef ESP8266
        Res = OscUdpSrv.beginPacketMulticast( WiFi.softAPIP() | 0xFF000000, OSC_PORT, WiFi.softAPIP());// 0xFF000000 beware of the endians
      #else
        Res = OscUdpCli.beginPacket( WiFi.softAPIP() | 0xFF000000, OSC_PORT);
      #endif
      Msg.send( OscUdpSrv);
      Res = OscUdpSrv.endPacket();// Finish off this packet and send it
    }

    Msg.empty();
  }
}

// send in the first place, resend is in OscLoop
void OscSendCmd( int CmdNum) {
  OscLastCount = 5;
  OscLastCmd = CmdNum;
  OscLastMs = millis();
  OscCmdSequence ++;

  OscSendCmdAct( OscLastCmd);
}

// execute the received command
int OscMsgExec( char* Command) {
  int Pos;
  int Res = 0;

  if ('_' == Command[0]) {

    Pos = 1;
    while (0 != Command[Pos]) {
      CmdLineParse( Command[Pos]);
      Pos++;
    }
    
    dbgprintf( 3, " Processed rp%i, -%s-\n", OscReply, DbgStr);
    if(OscReply && (0 != DbgStr[0])) {
      OscSendCmdAct( 4); // send DbgStr
      OscReply = 0;
    }

    Res = 1;
  }

  return (Res);
}

void OscPlaytrack( OSCMESSAGE &Msg, int Offset) {
  uint32_t Arg1;

  // get 1st argument(int32)
  Arg1 = Msg.getInt(0);
  dbgprintf( 2, "OscUdpCli parsed %i\n", Arg1);
  PlayTrack( Arg1);
  PlayerRunning = 1;
}

void OscInit() {
}

void OscLoop() {
  OSCMESSAGE bundle;
  int size;
  int Res;
  uint8_t incomingByte;

  // resend to patch udp reliability
  if ((OscLastCount > 0) && (DiffTime( OscLastMs, millis()) > 200)) {
    OscLastCount --;
    OscLastMs = millis();
    OscSendCmdAct( OscLastCmd);
  }

  // catch orders
#ifdef WITH_WIFICLIENT
  if ((0 == OscWifiCliState) && WifiCliConnected()) {
    Res = OscUdpCli.begin(OSC_PORT);
    if (1 == Res) { // 1 if successful
      OscWifiCliState = 1;
    } else if (0 == Res) {
      dbgprintf( 2, "no sock availlable %i\n", Res);
      OscWifiCliState = 1;// but seems works anyway
    } else {
      dbgprintf( 2, "OscUdpCli err %i\n", Res);
    }
  } else if ((1 == OscWifiCliState) && !WifiCliConnected()) {
    // TODO_LATER : soupcons que ca coupe debug  OscUdpCli.stop();
    OscWifiCliState = 0;
  }

  if (0 != OscWifiCliState) {
    size = OscUdpCli.parsePacket();
    if (size > 0) {
      // dbgprintf( 2,"OscUdpCli got %i-", size);
      while (size--) {
        incomingByte = OscUdpCli.read();
        //dbgprintf( 2,"%c", incomingByte);
        bundle.fill( incomingByte);
      }
      //dbgprintf( 2,"\n");
      if (!bundle.hasError()) {
        //dbgprintf( 2,"OscUdpCli bundle rdy");
        //bundle.route("/*", OscPlaytrack);
        bundle.dispatch( OscAddrMine, OscMine);
      } else {
        OscError = bundle.getError();
        dbgprintf( 2, "OscUdpCli err %i", OscError);
      }
    }
  }
#endif /* WITH_WIFICLIENT */

#ifdef WITH_WIFI
  if ((0 == OscWifiSrvState) && ( WifiSrvConnected())) {
    Res = OscUdpSrv.begin(OSC_PORT);
    // dbgprintf( 2,"OscUdpSrv begin %i\n", Res);
    if (1 == Res) { // 1 if successful
      OscWifiSrvState = 1;
    }
  } else if ((1 == OscWifiSrvState) && !WifiSrvConnected()) {
    // TODO_LATER : soupcons que ca cupe debug OscUdpSrv.stop();
    OscWifiSrvState = 0;
  }

  if (0 != OscWifiSrvState) {
    size = OscUdpSrv.parsePacket();
    if (size > 0) {
      // dbgprintf( 2,"OscUdpSrv got %i-", size);
      while (size--) {
        incomingByte = OscUdpSrv.read();
        //dbgprintf( 2,"%c", incomingByte);
        bundle.fill( incomingByte);
      }
      //dbgprintf( 2,"\n");
      if (!bundle.hasError()) {
        //dbgprintf( 2,"OscSrvCli bundle rdy");
        //bundle.route("/*", OscPlaytrack);
        bundle.dispatch( OscAddrMine, OscMine);
      } else {
        OscError = bundle.getError();
        dbgprintf( 2, "OscSrvCli err %i", OscError);
      }
    }
  }
#endif /* WITH_WIFI */

  // TODO_LATER : who knows ... RS-485
}

#endif /* WITH_OSC */

void GarageDoorOpen() {
  MyPinmode( 5, OUTPUT);
  digitalWrite( 5, 1);
  delay(500);
  digitalWrite( 5, 0);
}

void FctRun( uint8_t* Str, uint32_t Num) {

  uint8_t FctId;
  int Val;
  int Idx;

  FctId = Num % 100;
  Num = Num / 100;

  switch ( FctId) {
    case 1 :
      dbgprintf( 2, " FctRun01\n");
      GarageDoorOpen();
      break;
    #ifdef WITH_WS2812
      case 2 :
        Val = map( Num % 10000, 0 , 9999, 0, 99);
        for (Idx = 0; Idx < PIX_NB; Idx++) {
          PixelSet99( Val, Val, Val, Idx);
        }
        pixels.show();
      break;
    #endif /* WITH_WS2812 */
    // HI_GIRLS : here everything specific to your project
    default :
      dbgprintf( 2, "FctRun : none for %i\n", FctId);
      break;
  }
}

// just give the 4 last digits and follow the rest of the digits, like a gear doing more than one turn
uint32_t PartialAdd4( uint32_t Orig, int32_t Delta) {
  // Delta     0000     9999
  // Orig  xxxx0000 xxxx9999
  int32_t My4;
  uint32_t Rest;
  uint32_t Result;

  My4 = Orig % 10000;
  Rest = Orig - My4;

  Result = Rest + Delta;

  if (Orig >= 5000) {
    if (abs(My4 - Delta)< 5000) {
      Result = Rest + Delta;
    } else if (Delta < 5000) {
      Result =  Rest + Delta + 10000;
    } else { 
      Result =  Rest + Delta - 10000;
    }
  }

  //dbgprintf( 2, "PartialAdd4 : Orig %i Delta %i Result %i\n", Orig, Delta, Result);
  // TODO_LATER : what about > 4M, cyclic
 
  return( Result);
}

// command (debug) enter here
void CmdLineParse( unsigned char Ch) {
  char EprSet = 0;
  uint8_t uITmp1;
  uint8_t uITmp;
#ifdef WITH_MOTOR
  Motor_t* pMotor;
#endif /* WITH_MOTOR */

  if (1 == DbgStrMode) {
    if ('"' == Ch) {
      DbgStr[DbgStrIdx] = 0;
      DbgStrMode = 0;
    } else  if (DbgStrIdx < DbgStr_NBMAX) {
      DbgStr[DbgStrIdx] = Ch;
      DbgStrIdx ++;
    } else {
      // oversize
    }
  } else {
    switch (Ch) {
      case '"':
        if (1 == DbgStrMode) {
          DbgStrMode = 0;
        } else {
          DbgStrMode = 1;
          DbgStrIdx = 0;
        }
        break;

#ifdef WITH_WIFICLIENT
      case 'D' : // Cli SSID
        memcpy( CliSSID, DbgStr, WIFI_IDLEN);
        WiCliSSIDSet();
        WifiCliConnectState = WifiState_TOCONNECT;
        EprSet = 1;
        break;
      case 'd' : // Cli PWD
        memcpy( CliPWD, DbgStr, WIFI_PWDLEN);
        WiCliPwdSet();
        WifiCliConnectState = WifiState_TOCONNECT;
        EprSet = 1;
        break;
#endif /* WITH_WIFICLIENT */
      case 'F': // multi funct
        FctRun( DbgStr, DbgNum);
        break;
      case 'f': // parameters | Class 2, Id 2, Val 4
        uITmp1 = (DbgNum / 1000000) % 100; // Class
        uITmp = (DbgNum / 10000) % 100;    // Id
        switch (uITmp1) { // Class
#ifdef WITH_MOTOR
          case 0: // 00<2 motor><4 Val>
            if ((uITmp >= 0) && (uITmp < MOTOR_NB)) {
              pMotor = &(MotorArray[uITmp]);
              pMotor->DriverMode = (MotorMode_t) DbgNum % 10000; // Val
              dbgprintf( 2, "Set Motor %i Driver Mode %i\n", uITmp, pMotor->DriverMode);
              switch (uITmp) {
                case 0 : EprMotor0ModeSet( pMotor->DriverMode); EprSet = 1; break;
                case 1 : EprMotor1ModeSet( pMotor->DriverMode); EprSet = 1; break;
                case 2 : EprMotor2ModeSet( pMotor->DriverMode); EprSet = 1; break;
                default : dbgprintf( 2, "Warn - Driver Mode not saved\n", uITmp); break;
              }
            }
            else {
              dbgprintf( 2, "Warn - No Such Motor %i\n", uITmp);
            }
            break;
#endif /* WITH_MOTOR */
          case 1 : //0100xxxxf
            if (0 == uITmp) {
              WifiAddsSet( DbgNum % 10000);
              EprSet = 1;
            }
          break;
          default :
            dbgprintf( 2, "Warn - No Such parameter class %i\n", uITmp1);
            break;
        }
        break;
#ifdef WITH_AUTODETECT
      case 'G' : // 00..99 Autodetect
        DetectLevSet(DbgNum % 100);
        EprSet = 1;
        break;
#endif /* WITH_AUTODETECT */
      case 'h' : // help
      case 'H' :
        EprSet = 1;
        break;

      case 'I': // digital input 0 or 1
        uITmp = DbgNum % 100;
        MyPinmode( uITmp, INPUT_PULLUP);
        uITmp1 = digitalRead( uITmp);// TODO_HERE
        dbgprintf( 2, "InD %i %i %i\n", uITmp, (DbgNum / 100) % 10, uITmp1);
        //snprintf( (char*)DbgStr, DbgStr_NBMAX, "\"%s%3.3i\" %4.4it", BEN_TAG, CSTRIn( 3, NodeGet()).getString(), CSTRIn( 4, uITmp1).getString());
        snprintf( (char*)DbgStr, DbgStr_NBMAX, "\"%s%3i\" %4it", BEN_TAG, NodeGet(), uITmp1);
        dbgprintf( 2, "%s\n", DbgStr);
        #ifdef WITH_OSC
          OscReply ++;
        #endif /* WITH_OSC */
        break;
      case 'i': // analog output
        uITmp = DbgNum % 100;
        MyPinmode( uITmp, INPUT_PULLUP);
        uITmp1 = analogRead( uITmp);// TODO_HERE
        dbgprintf( 2, "InA %i %i %i\n", uITmp, (DbgNum / 100) % 10, uITmp1);
        //snprintf( (char*)DbgStr, DbgStr_NBMAX, "\"%s%s\" %st", BEN_TAG, CSTRIn( 3, NodeGet()).getString(), CSTRIn( 4, uITmp1).getString());
        snprintf( (char*)DbgStr, DbgStr_NBMAX, "\"%s%3i\" %4it", BEN_TAG, NodeGet(), uITmp1);
        dbgprintf( 2, "%s\n", DbgStr);
        #ifdef WITH_OSC
          OscReply ++;
        #endif /* WITH_OSC */
        break;

    #ifdef WITH_OSC
      case 'J':
        if ('/' == DbgStr[0]) {
          memcpy( OscAddr, DbgStr, min( OSC_CMD_LEN, WIFI_IDLEN));
        } else {
          OscAddr[0] = '/';
          memcpy( OscAddr + 1, DbgStr, min( OSC_CMD_LEN - 1, WIFI_IDLEN));
        }
        OscAddr[OSC_CMD_LEN - 1] = 0;
        OscAddrSet();
        EprSet = 1;
        break;
      case 'K':
        switch ( DbgNum % 100) {
          #ifdef WITH_SWITCH
          case 1:
            memcpy( OscCmd1, DbgStr, OSC_CMD_LEN);
            OscCmd1Set();
            EprSet = 1;
            break;
          case 2:
            memcpy( OscCmd2, DbgStr, OSC_CMD_LEN);
            OscCmd2Set();
            EprSet = 1;
            break;
          #endif /* WITH_SWITCH */
          #ifdef WITH_ADC
          case 3:
            memcpy( OscCmd3, DbgStr, OSC_CMD_LEN);
            OscCmd3Set();
            EprSet = 1;
            break;
          #endif /* WITH_ADC */
        }
        break;
      case 'T':
        OscSendCmdAct( 4);
        break;
      case 't':
        // 't' is just a response from osc, put here in this "case" to conflict if reused
        break;
    #endif /* WITH_OSC */
#ifdef WITH_WS2812
      case 'L':
        PixelShow99( (DbgNum / 1000000) % 100 , (DbgNum / 10000) % 100 , (DbgNum / 100) % 100 , DbgNum % 100);
        break;
#endif /*  */
#ifdef WITH_MOTOR
      case 'l' :
        uITmp = (DbgNum / 10000) % 10;
        if (uITmp < MOTOR_NB) {
          Motor_t* pMotor = &(MotorArray[uITmp]);
          pr_uint32_write( pMotor->WishDTms, DbgNum % 10000);
        }
        break;
#endif /* WITH_MOTOR */
      case 'M' : // set max stories
        MaxSet(DbgNum % 100); EprSet = 1;
        break;
      case 'm':
        RssiMinimalSet( DbgNum % 1000); EprSet = 1;
        break;
      case 'N' : // player number/passenger number
        SetNodeNum(DbgNum % 100); EprSet = 1;
        break;

      case 'O': // digital output 0 or 1
        uITmp = DbgNum % 100;
        MyPinmode( uITmp, OUTPUT);
        digitalWrite( uITmp, (DbgNum / 100) % 10);
        dbgprintf( 2, "Outd %i %i\n", uITmp, (DbgNum / 100) % 10);
        break;
      case 'o': // analog output
        uITmp = DbgNum % 100;
        MyPinmode( uITmp, OUTPUT);
        analogWrite( uITmp, (DbgNum / 100) % 1000);
        dbgprintf( 2, "Outa %i %i\n", uITmp, (DbgNum / 100) % 1000);
        break;

      case 'P' : // Rssi correction
        RssiCorrectionSet( DbgNum % 1000);
        WifiInit();
        EprSet = 1;
        break;
#ifdef WITH_DFPLAYER
      case 'p' : // DFPlayer test
        uint8_t Cmd;
        uint8_t Fbk;
        uint8_t P1;
        uint8_t P2;
        int Res;
        dbgprintf( 3, "%lup\r\n", DbgNum % 100000000);
        // f P1 P2 CMD
        DFPlayerSendCmd( DbgNum % 1000, (DbgNum / 10000000) % 10, (DbgNum / 100000) % 100, (DbgNum / 1000) % 100); //x,x,x,0..255
        Res = DFPlayerWaitResult( &Cmd, &Fbk, &P1, &P2, 5000, 100);
        break;
#endif /* WITH_DFPLAYER */
      case 'Q' :
         uITmp = DbgNum % 100;
         switch(uITmp){
          case 1:
            MaxX = (DbgNum/100)%10000000;
            MaxXSet();
            EprSet = 1;
            break;
          case 2:
            SpeedX = (DbgNum/100)%10000000;
            SpeedXSet();
            if (VectX >= 0) {
              VectX = SpeedX;
            } else {
              VectX = -SpeedX;
            }
            EprSet = 1;
            break;
          case 3:
            MaxY = (DbgNum/100)%10000000;
            MaxYSet();
            EprSet = 1;
            break;
          case 4:
            SpeedY = (DbgNum/100)%10000000;
            SpeedYSet();
            if (VectY >= 0) {
              VectY = SpeedY;
            } else {
              VectY = -SpeedY;
            }
            EprSet = 1;
            break;
          default:
            dbgprintf( 2, "Warn - No Such Parameter %i\n", uITmp);
            break;
         }
        break;
      case 'R' : // 0 - ROLE_PLAYER, 1 - PASSENGER, 2 - ROLE_SOLO
        RoleSet(DbgNum % 10);
        WifiInit();
        EprSet = 1;
        break;
#ifdef WITH_MOTOR
      case 'S':
        uITmp = (DbgNum / 10000) % 10;
        if ((uITmp >= 0) && (uITmp < MOTOR_NB)) {
          Motor_t* pMotor = &(MotorArray[uITmp]);

          switch( pMotor->DriverMode) {
            default :
            case MOTOR_PWM :
            case MOTOR_PWM2 :
            #ifdef WITH_SERVO
              case MOTOR_SERVO :
            #endif /* WITH_SERVO */
              pr_int32_write( pMotor->WishP2, DbgSign*(DbgNum % 10000));
            break;
            case MOTOR_STEPPER:
            case MODOR_DC_LR:
              if( 9999 == DbgNum % 10000) {
                // TODO_HERE : Homing
                pMotor->HomingOn = !pMotor->HomingOn;
                dbgprintf( 2, "Warn - TODO_HERE Homing\n");
              } else {
                if (pMotor->Order < 0) { // don't know where we are after boot
                  pMotor->WishDec = DbgSign*(DbgNum % 10000);
                  pMotor->Order = 0;
                }
                pr_int32_write( pMotor->WishP2, DbgSign*(DbgNum % 10000));
                // TODO_HERE mostly ready pr_uint32_write( pMotor->WishP2, PartialAdd4( pr_uint32_read(pMotor->WishP2), DbgNum % 10000));
              }
              break;
          }
          if(pMotor->Order < 3) { // suppose ++ atomic and no more than 2 collisions
            pMotor->Order++;
          }
        }
        else {
          dbgprintf( 2, "Warn - No Such Motor %i\n", uITmp);
        }
        break;
        //case 's' :
        //  StepperSpeedPerMilSec = DbgSign*DbgNum%100000;
        //  break;
#endif /* WITH_MOTOR */
#ifdef WITH_DFPLAYER
      case 'T':
        PlayTrack( DbgNum % 10000);
        PlayerRunning = 1;
        break;
#endif /* WITH_DFPLAYER */
      case 'U': // reserved security
        break;
#ifdef WITH_DFPLAYER
      case 'V' :  // 00..99 Volume
        SetVolume(DbgNum % 100);
        PlayerVolumeSet( GetVolume());
        EprSet = 1;
        break;
#endif /* WITH_DFPLAYER */
      case 'v' : // Debug verbose level
        VerboseSet( DbgNum % 10);
        EprSet = 1;
        break;
      case 'W': // wait 9999
        RunOtherTasks(DbgNum % 10000); // TODO_LATER : block only cmdparse
        break;
      case '-':
        DbgSign = -1;
        break;
      default :
        if ((Ch >= '0') && (Ch <= '9')) {
          DbgNum = DbgNum % 100000000; // 400000000 --> 8 digits 0..9
          DbgNum = DbgNum * 10 + Ch - '0';
          DbgSign = 1;
        }
        break;
    }
  }
  if (1 == EprSet) {
#ifdef ESP8266
    EEPROM.commit();
#endif
    EpromSanity();
    EpromDump();
    EprSet = 0;
  }

}

// Rssi

void RssiDump( int Lev, int Num) {
  int i = 0;

  switch (RoleGet()) {
    case ROLE_PLAYER :
      dbgprintf( Lev, BEN_TAG "List%i\r\n", Num);
      for (i = 0; i < PLAYERS_NB; i++) {
        dbgprintf( Lev, "%03i", PlayersId[i]);
        dbgprintf( Lev, ", 0x%08x", PlayersMac[i]);
        dbgprintf( Lev, ", %04i \r\n", PlayersRssi[i]);
      }
      dbgprintf( Lev, BEN_TAG "End%i\r\n", Num);
      break;
    case ROLE_PASSENGER :
    case ROLE_SOLO :
      // no need, time running display is enough
      break;
    default :
      i = 0;
      break;
  }
}

#ifdef WITH_LED_SHOW_WIFI
// first reserved to a status
int PixelStatus( int State) {
  PixState = State;
}
#endif /* WITH_LED_SHOW_WIFI */

// search if known, returns a bad Idx if not found
// Arr : array 1 or array 2
int RssiSearch( int Arr, uint8_t Id, uint32_t Mac) {
  int CntWorst;
  int Cnt;

  // dbgprintf( 3, "RssiSearch(... %i ...)\n", Id);
  if (1 == Arr) {
    for (Cnt = 0; Cnt < PLAYERS_NB; Cnt++) {
      if (Mac == PlayersMac[Cnt]) {
        // dbgprintf( 3, "RssiSearch 1 found %i\n", Cnt);
        return ( Cnt);
      }
    }
  } else {
    for (Cnt = 0; Cnt < PLAYERS_NB; Cnt++) {
      if (Mac == PlayersMac2[Cnt]) {
        // dbgprintf( 3, "RssiSearch 2 found %i\n", Cnt);
        return ( Cnt);
      }
    }
  }
  return ( -1);
}

// insert RSSI in shorted candidate list, bettter first
int RssiAdd( uint8_t Id, uint32_t Mac, int16_t Rssi) {
  int CntWorst;
  int Cnt;

  // dbgprintf( 3, "RssiAdd(... %i ...) \n", Id);
  Cnt = RssiSearch( 1, Id, Mac);
  if (Cnt >= 0) { // remove if already in
    // dbgprintf( 3, "RssiAdd fnd, move down \n");
    for (CntWorst = Cnt; CntWorst < PLAYERS_NB - 1; CntWorst++) {
      PlayersId[Cnt]   = PlayersId[Cnt + 1];
      PlayersMac[Cnt]  = PlayersMac[Cnt + 1];
      PlayersRssi[Cnt] = PlayersRssi[Cnt + 1];
    }
    // dbgprintf( 3, "RssiAdd fnd, empty last \n");
    Cnt = PLAYERS_NB - 1;
    PlayersId[Cnt]   = 0;
    PlayersMac[Cnt]  = 0;
    PlayersRssi[Cnt] = RSSI_MIN;
  }

  for (CntWorst = 0; CntWorst < PLAYERS_NB; CntWorst++) {
    // dbgprintf( 3, "where %i \n", CntWorst);
    if (PlayersRssi[CntWorst] < Rssi) { // our candidate is bettter
      // dbgprintf( 3, "here %i \n", CntWorst);
      for (Cnt = PLAYERS_NB - 1; Cnt > CntWorst; Cnt--) { // move others up
        PlayersId[Cnt]   = PlayersId[Cnt - 1];
        PlayersMac[Cnt]  = PlayersMac[Cnt - 1];
        PlayersRssi[Cnt] = PlayersRssi[Cnt - 1];
      }
      PlayersId[CntWorst]   = Id;
      PlayersMac[CntWorst]  = Mac;
      PlayersRssi[CntWorst] = Rssi;
      return (0);
    }
  }

  dbgprintf( 3, "our %i is pitty \n", Rssi);
  return (0);
}

// Description : clean arrays
int RssiCls() {
  int i;

#ifdef WITH_LED_SHOW_WIFI
  PixState = 0;
#endif /* WITH_LED_SHOW_WIFI */

  for (i = 0; i < PLAYERS_NB; i++) {
    PlayersId[i]   = 0;
    PlayersMac[i]  = 0;
    PlayersRssi[i] = RSSI_MIN;

    PlayersId2[i]   = 0;
    PlayersMac2[i]  = 0;
    PlayersRssi2[i] = RSSI_MIN;
  }
  return ( 0);
}

#ifdef WITH_IOTMUTUAL

/* --------------------  IOT  ---------------------------------*/

void StrHeader( String& MyStr) {
  MyStr += BEN_TAG;
  MyStr += " ";
  MyStr += SrvSSID;
  MyStr += " ";
  if (ROLE_MULTI != RoleGet()) {
    MyStr += CSTRI( NodeGet());
  } else {
    MyStr += WiFi.macAddress();
  }
}

void handleRoot() {
  String MyStr;

  MyStr = "";
  MyStr +=
    "<html>\
  <head>";
  MyStr += "<title>";
  StrHeader(MyStr);
  MyStr += "</title>";
  MyStr += "<style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>";
  StrHeader( MyStr);
  MyStr += "</h1>\
    <br>\
    <pre>";

  EpromDumpBld( MyStr);
  StateDumpBld_1( MyStr);
  StateDumpBld_2( MyStr);
  MyStr += "\
    </pre>\
    <br>\
    <form action='command.php'>\
      Command <input type='text' name='Command'<br>\
      <input type='submit' value='Submit'>\
    </form><br>";
  // TODO_HERE Player show friends
  MyStr += "\
    <br>\
    <br>\
    <form action='globalstatus'>\
      <input type='submit' value='Status'>\
    </form><br>";
  MyStr += "\
  </body>\
</html>";


  #ifndef ESP32 // TODO_HERE
    server.send( 200, "text/html", MyStr );
  #endif
}

void IotCommandResp() {
  String MyStr;

  MyStr = "";
  MyStr += "<html> <head>";
  MyStr +=    "<meta http-equiv=\"refresh\" content=\"1;/\" />";
  MyStr += "<title>";
  StrHeader(MyStr);
  MyStr += "</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>\
    <h1>Done ";
  MyStr += "\
  </body>\
</html>";
  #ifndef ESP32 // TODO_HERE
    server.send ( 200, "text/html", MyStr );
  #endif
}

void IotGlobalStatusHandle() {
  String MyStr;

  MyStr = "";
  MyStr += "<html> <head>";
  MyStr +=    "<meta http-equiv=\"refresh\" content=\"1\" />";
  MyStr += "<title>";
  StrHeader(MyStr);
  MyStr += "</title>\
    <style>\
      body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }\
    </style>\
  </head>\
  <body>";
  StrHeader( MyStr);
  MyStr += "<pre>";
  StateDumpBld_1( MyStr);
  StateDumpBld_2( MyStr);
  MyStr += "</pre>\
    <br>\
    <form action='/'>\
      <input type='submit' value='stop'>\
    </form><br>";
  MyStr += "\
  </body>\
</html>";
  //dbgprintf( 2, "PosC0\n");  // TODO_LATER : maybe udp during send locks for 9 sec
  #ifndef ESP32 // TODO_HERE
    server.send ( 200, "text/html", MyStr );
  #endif
  //dbgprintf( 2, "PosD0\n");
}

void handleNotFound() {
  // digitalWrite ( led, 1 );
  String message = "File Not Found\n\n";
  message += "URI: ";
  #ifndef ESP32 // TODO_HERE
    message += server.uri();
  #endif
  message += "\nMethod: ";
  #ifndef ESP32 // TODO_HERE
    message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  #endif
  message += "\nArguments: ";
  #ifndef ESP32 // TODO_HERE
    message += server.args();
  #endif
  message += "\n";

  #ifndef ESP32 // TODO_HERE
  for ( uint8_t i = 0; i < server.args(); i++ ) {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }
  #endif

  #ifndef ESP32 // TODO_HERE
    server.send ( 404, "text/plain", message );
  #endif
  //digitalWrite ( led, 0 );
}

void IotCommand(void) {

  char Uri[200];
  int Pos;

  #ifndef ESP32 // TODO_HERE
  if (server.args() > 0 ) {
    for ( uint8_t i = 0; i < server.args(); i++ ) {
      server.arg(i).toCharArray(Uri, 200);
      if (server.argName(i) == "Command") {
        dbgprintf( 3, "Internet injects %s\r\n", Uri);
        Pos = 0;
        while (0 != Uri[Pos]) {
          CmdLineParse( Uri[Pos]);
          Pos++;
        }
      }
    }
  }
  #endif
  //- See more at: http://www.esp8266.com/viewtopic.php?f=8&t=4345#sthash.fEzVLQQQ.dpuf

  IotCommandResp();
}


void IotMutualSetup ( void ) {
  dbgprintf( 3, "IotInit\r\n");
  if ( MDNS.begin ( "esp8266" ) ) {
    dbgprintf( 3,  "MDNS responder started" );
  }

  #ifndef ESP32 // TODO_HERE
  server.on( "/", handleRoot);
  server.on( "/command.php", IotCommand);
  server.on( "/globalstatus", IotGlobalStatusHandle);
  server.onNotFound ( handleNotFound );
  server.begin();
  dbgprintf( 3, "HTTP server started\n" );
  #endif
}

#endif /* WITH_IOTMUTUAL */

#ifdef WITH_STEPPER

void StepperSetup( ) {

  Motor_t* pMotor;
  int Idx;
  int PinDir;
  int PinPulse;

  for( Idx = 0; Idx < StepperNb; Idx++){
    
    pMotor = &(MotorArray[Idx+MOTOR_STEPPER_IDX]);
    PinDir = StepperPins[0+2*Idx];
    PinPulse = StepperPins[1+2*Idx];

    pMotor->StepperD = 0;
    pMotor->StepperP = 0;
    pMotor->Pin1 = PinDir;
    pMotor->Pin2 = PinPulse;
    
    MyPinmode( PinDir, OUTPUT);
    digitalWrite( PinDir, pMotor->StepperD);
    MyPinmode( PinPulse, OUTPUT);
    digitalWrite( PinPulse, pMotor->StepperP);

    // 200 min microsec up (or down) for a pulse, theoric 2us for a https://www.pololu.com/file/0J590/drv8825.pdf for example
    // 150 good for small motors, too short for Big without microstep
    //pMotor->MinCommandUs = 500;
    pMotor->MinCommandUs = 150;

    if (MOTOR_UNDEF == pMotor->DriverMode) {
      pMotor->DriverMode = MOTOR_STEPPER;
    }

    #ifdef STOPPER_MIN_PIN
       MyPinmode( STOPPER_MIN_PIN, INPUT_PULLUP);
    #endif /* STOPPER_MIN_PIN */
    #ifdef STOPPER_MAX_PIN
      MyPinmode( STOPPER_MAX_PIN, INPUT_PULLUP);
    #endif /* STOPPER_MAX_PIN */

  }
}

uint32_t StepperLoop( Motor_t* pMotor, uint32_t ExpectedPos) {
  long CurTime;
  int32_t Dist;
  uint8_t DirPin = pMotor->Pin1;
  uint8_t PulsPin = pMotor->Pin2;
  uint32_t StepperPos = pMotor->LastPos;

  CurTime = millis();

  Dist = Vectorize( StepperPos, ExpectedPos);
  // dbgprintf( 2, " Pos:%5u, Expect:%5u,", StepperPos, ExpectedPos);
  // dbgprintf( 2, " D:%5i\n", Dist);

#ifdef STOPPER_MIN_PIN
      if ((Dist < 0) && (HIGH != digitalRead( STOPPER_MIN_PIN))) {
        // nop
        //dbgprintf(2, "Min nop\n");
        return ( StepperPos);
      }
#endif /* STOPPER_MIN_PIN */
#ifdef STOPPER_MAX_PIN
      if ((Dist > 0) && (HIGH != digitalRead( STOPPER_MAX_PIN))) {
        // nop
        //dbgprintf(2, "Max nop\n");
        return ( StepperPos);
      }
#endif /* STOPPER_MAX_PIN */

  if (Dist < 0) {
    if (1 == pMotor->StepperD) {
      pMotor->StepperD = 0;
      digitalWrite( DirPin, pMotor->StepperD);
      // dir <- StepperD
    } else {
      StepperPos --;
      pMotor->StepperP = !pMotor->StepperP;
      // puls <- StepperP
      digitalWrite( PulsPin, pMotor->StepperP);
    }
  } else if (Dist > 0) {
    if (0 == pMotor->StepperD) {
      pMotor->StepperD = 1;
      // dir <- StepperD
      digitalWrite( DirPin, pMotor->StepperD);
    } else { // 0 == dist
        StepperPos ++;
        pMotor->StepperP = !pMotor->StepperP;
      // puls <- StepperP
      digitalWrite( PulsPin, pMotor->StepperP);
    }
  } else {  // expected pos
/*#ifdef STOPPER_MIN_PIN
    // commented because not decided yet if it's about app specific or this generic to re-align
    if (LOW == digitalRead( STOPPER_MIN_PIN)) {
      StepperPos = 0;
    }
#endif*/ /* STOPPER_MIN_PIN */

  }
  pMotor->LastPos = StepperPos;

  return ( StepperPos);
}

#endif /* WITH_STEPPER */

#ifdef STEPPER_IN_PULSE_PIN

void ICACHE_RAM_ATTR StepperInPosLoop () {

  uint32_t CurrMicros = micros();
  uint32_t CurrVal = digitalRead(STEPPER_IN_PULSE_PIN);

  //dbgprintf( 2, "I %i\n", StepperInPos);

  if ((DiffTime(CurrMicros, StepperInUs) < 4) || (StepperInVal == CurrVal)) {
    return;
  }

  if (digitalRead(STEPPER_IN_DIR_PIN)) {
    StepperInPos++;
  } else {
    StepperInPos--;
  }
  pr_uint32_write( pr_StepperInPos, StepperInPos);

  StepperInVal = CurrVal;
  StepperInUs = CurrMicros;


  //dbgprintf( 2, "O %i\n", StepperInPos);
}

void StepperInSetup() {
  MyPinmode( STEPPER_IN_DIR_PIN, INPUT_PULLUP);
  MyPinmode( STEPPER_IN_PULSE_PIN, INPUT_PULLUP);
  StepperInPosLoop();
  StepperInPos = 0;
  pr_uint32_write( pr_StepperInPos, StepperInPos);
  #ifdef STEPPER_IN_PULSE_PIN_INTERRUPT
    attachInterrupt( digitalPinToInterrupt(STEPPER_IN_PULSE_PIN), StepperInPosLoop, CHANGE);
  #endif /* STEPPER_IN_PULSE_PIN_INTERRUPT */
}

#endif /* STEPPER_IN_PULSE_PIN */


#ifdef WITH_DCMOTOR
uint32_t DcPosLoop(  Motor_t* pMotor, uint32_t ExpectedPos) {
  int32_t Dist;
  uint32_t CurrentPos = pMotor->LastPos;
  uint8_t MotA = pMotor->Pin1;
  uint8_t MotB = pMotor->Pin2;

  Dist = Vectorize( CurrentPos, ExpectedPos);

  // dbgprintf( 2, " Pos%5u, Expect%5u,", CurrentPos, ExpectedPos);
  // dbgprintf( 2, " D%5i", Dist);

  if (abs( Dist) < 2) { // 1 stops 'tic tic tic', 4 is not smooth
    //dbgprintf( 2, "I\n");
    digitalWrite( MotA, LOW);
    digitalWrite( MotB, LOW);
  } else if (Dist > 0) {
    //dbgprintf( 2, "A\n");
    digitalWrite( MotB, LOW);
    digitalWrite( MotA, HIGH);
    CurrentPos--;
  } else {
    //dbgprintf( 2, "B\n");
    digitalWrite( MotA, LOW);
    digitalWrite( MotB, HIGH);
    CurrentPos++;
  }
  //dbgprintf( 2, " \n");

  pMotor->LastPos = CurrentPos;
  return (CurrentPos);

}

void DcMotorSetup() {
  Motor_t* pMotor;
  int Idx;

  for( Idx = 0; Idx < dcMotorNb; Idx++){
    
    pMotor = &(MotorArray[Idx+MOTOR_DC_IDX]);

    pMotor->Pin1 = dcMotorPins[Idx*2];
    pMotor->Pin2 = dcMotorPins[1+Idx*2];
    pMotor->MinCommandUs = 4;

    MyPinmode( pMotor->Pin1, OUTPUT);
    MyPinmode( pMotor->Pin2, OUTPUT);

    if (MOTOR_UNDEF == pMotor->DriverMode) {
      pMotor->DriverMode = MODOR_DC_LR;
    }
  }
}
#endif /* WITH_DCMOTOR */

#ifdef WITH_SERVO
void ServoInit( int Idx, uint8_t MotA) {
  Motor_t* pMotor;
  uint32_t Pos = 0;

  pMotor = &(MotorArray[MOTOR_SERVO_IDX+Idx]);
  pMotor->Pin1 = MotA;
  digitalWrite( pMotor->Pin1, LOW);
  MyPinmode( pMotor->Pin1, OUTPUT);
  
 # if 0
    
  pMotor->DriverMode = MOTOR_SERVO;
  ServoList[Idx].attach( MotA);
#ifdef SERVO_ROOT
    ServoList[Idx].writeMicroseconds( 2500);
#else
    ServoList[Idx].setMaximumPulse(2200);
#endif
  ServoList[Idx].write( Pos);
#endif

  pMotor->LastPos = Pos;

  // Oscillo said servo pulse is 10 ms, under 50 is toublesome
  // 15 ms for servo to act anyway
  pMotor->MinCommandUs = 0;//30*1000; 

  if (MOTOR_UNDEF == pMotor->DriverMode) {
    pMotor->DriverMode = MOTOR_SERVO;
  }
   
}

// servo pulse from 500us to 2400us
// rem : servo direct on the VIN and a 3.3v usually mess with ADC
void ServoPosLoop( uint8_t Idx, uint32_t ExpectedPos) {
#if 0
  uint32_t Pos;
  uint32_t Min = 0;//30;
  uint32_t Max = 180;//170;
  Motor_t* pMotor;
   
  pMotor = &(MotorArray[Idx]);

  if (ExpectedPos <= 0) {
    Pos = Min;
  } if (ExpectedPos >= 9999) {
    Pos = Max;
  } else {
    Pos = map ( ExpectedPos , 0, 9999, Min, Max);
  }

  dbgprintf( 2, "ServoPosLoop %i %i %4i)\n", Idx, Pos, ExpectedPos);

  if ( DiffTime( Pos, pMotor->LastPos) > 1 ) { 
    // ExpectedPos 0..180
    ServoList[Idx].write( Pos);
    pMotor->LastPos = Pos;
    dbgprintf( 2, "ServoPosLoop +( %3i, %4i)\n", Pos, ExpectedPos);
  } else {
    // dbgprintf( 2, "ServoPosLoop -( %3i, %4i)\n", Pos, ExpectedPos);
  }
  #else

  uint32_t Pos;
  uint32_t Min = 500;
  uint32_t Max = 2400;//170;
  uint32_t CurMicros;
  uint32_t PulseAlreadyDone;
  uint32_t Remaining;
  Motor_t* pMotor;
   
  pMotor = &(MotorArray[Idx]);

  if (ExpectedPos <= 0) {
    Pos = Min;
  } if (ExpectedPos >= 9999) {
    Pos = Max;
  } else {
    Pos = map ( ExpectedPos , 0, 9999, Min, Max);
  }

dbgprintf( 2, "ServoPosLoop %i %i %i\n", pMotor->Pin1, Pos, ExpectedPos);

  if (pMotor-> LastPos <= 0) { // out of a pulse, let's start a new pulse
    pMotor->MinCommandUs = 0; // call each turn
    pMotor->LastPos = 1;
    digitalWrite( pMotor->Pin1, HIGH);
    pMotor->LastSlice = micros();
  }

  PulseAlreadyDone = DiffTime( pMotor->LastSlice, micros());

  if (PulseAlreadyDone >= Pos) { // stop pulse
    pMotor->MinCommandUs = 20*1000-Pos; // next call in 20ms
    pMotor->LastPos = 0;
    digitalWrite( pMotor->Pin1, LOW);
    return;
  }

  Remaining = DiffTime( PulseAlreadyDone, Pos);

  if (Remaining > 100) { // more than a loop
    return;
  }

  CurMicros = micros();
  while ( DiffTime( CurMicros, micros()) < Remaining) {
    // eat time
  }

  //delayMicroseconds(ExpectedPos);
  pMotor->MinCommandUs = 20*1000-Pos; // next call in 20ms
  pMotor->LastPos = 0;
  digitalWrite( pMotor->Pin1, LOW);

  #endif
}
#endif /* WITH_SERVO */

#ifdef WITH_DCMOTOR
// pwm from frequency
void MotorDcPwm( uint8_t Idx, uint32_t ExpectedPos) {

  #if 0
    // ... not reactive ...

   analogWriteFreq( 200);// change the frequency for MotorDcPwm, https://github.com/esp8266/Arduino/issues/2592.
   if (0 == ExpectedPos){
     analogWrite( pMotor->Pin1, 0);
     //digitalWrite( pMotor->Pin1, LOW);
   } else if (9999 == ExpectedPos) {
     analogWrite( pMotor->Pin1, PWMRANGE);
     //digitalWrite( pMotor->Pin1, HIGH);
   } else {
     analogWrite( pMotor->Pin1, map( ExpectedPos, 0, 9999, 0, PWMRANGE));
   }
  #else
   uint32_t CurrMicros;
   uint32_t DeltaMicros;
   #define SliceUs 5000
   Motor_t* pMotor;
   // LastPos as slice switch indication
   
   pMotor = &(MotorArray[Idx]);

   CurrMicros = micros();
   DeltaMicros = DiffTime( CurrMicros, pMotor->LastSlice);

   if (0 == ExpectedPos) {
     if (DeltaMicros >= SliceUs) {
       pMotor->LastPos = 0;
       pMotor->LastSlice = CurrMicros;
       digitalWrite( pMotor->Pin1, LOW);
     }
   } else if (9999 == ExpectedPos) {
     if (DeltaMicros >= SliceUs) {
       pMotor->LastPos = 1;
       pMotor->LastSlice = CurrMicros;
       digitalWrite( pMotor->Pin1, HIGH);
     }
   } else {
     if (1 == pMotor->LastPos) {
       if (DeltaMicros >= map( ExpectedPos, 0, 9999, 0, SliceUs)) {
         pMotor->LastPos = 0;
         pMotor->LastSlice = CurrMicros;
         digitalWrite( pMotor->Pin1, LOW);
       }
     } else {
       if (DeltaMicros >= (SliceUs - map( ExpectedPos, 0, 9999, 0, SliceUs))) {
         pMotor->LastPos = 1;
         pMotor->LastSlice = CurrMicros;
         digitalWrite( pMotor->Pin1, HIGH);
       }
     }
   }
   // dbgprintf( 2, "MotorDcPwm0 %i %i\n", pMotor->LastPos, ExpectedPos);
   
  #endif
 
}

// Adaptative pwm ... not sliced
void MotorDcPwm3( uint8_t Idx, uint32_t ExpectedPos) {
  Motor_t* pMotor;
  uint32_t CurrMicros;
  uint32_t Dt;
  #define U 2
  
  CurrMicros = micros();
  pMotor = &(MotorArray[Idx]);
  Dt = DiffTime( CurrMicros, pMotor->LastPulseUs);

  pMotor->LastPos = ( pMotor->LastSlice*pMotor->LastPos + Dt*(pMotor->InstVal) )/( pMotor->LastSlice+Dt);

  if (ExpectedPos > pMotor->LastPos) {
    digitalWrite( pMotor->Pin1, HIGH);
    pMotor->InstVal = 9999;
  } else {
    digitalWrite( pMotor->Pin1, LOW);
    pMotor->InstVal = 0;
  }

  pMotor->LastSlice = (pMotor->LastSlice*U+Dt)/(U+1);
  
}

void MotorDcPwm2( uint8_t Idx, uint32_t ExpectedPos) {
  Motor_t* pMotor;
  uint32_t Rand;
  uint32_t Dt;
  
  pMotor = &(MotorArray[Idx]);

  Rand = random(9999);
  
  if (0 == ExpectedPos) {
    pMotor->InstVal = 0;
  } else if (9999 == ExpectedPos) {
    pMotor->InstVal = 9999;
  } else {
    if (ExpectedPos < Rand) {
      pMotor->InstVal = 0;
    } else {
      pMotor->InstVal = 9999;
    }
  }

  if (pMotor->LastPos != pMotor->InstVal) {
    if (pMotor->InstVal > 0){
      digitalWrite( pMotor->Pin1, HIGH);
    } else {
      digitalWrite( pMotor->Pin1, LOW);
    }
    pMotor->LastPos = pMotor->InstVal;
  }

  //dbgprintf( 2, "MotorDcPwm2 %i %i\n", ExpectedPos, pMotor->InstVal);
  //dbgprintf( 2, "%i\n", pMotor->InstVal);
  
}
#endif /* WITH_DCMOTOR */

#ifdef WITH_GRAYCODE_IN
// 00 -> 01 -> 11 -> 10 -> 00

void ICACHE_RAM_ATTR GraycodeInAInt () {
  if (digitalRead(GRAYCODE_IN_A_PIN) != GrayA)
  {
    if (GrayA) {
      if (GrayB) { // 11 -< 01
        GrayPos--;
      } else { // 01 -> 11
        GrayPos++;
      }
    } else {
      if (GrayB) { // 10 -> 00
        GrayPos++;
      } else { // 00 -< 10
        GrayPos--;
      }
    }
    GrayA = !GrayA;
    // dbgprintf( 2, "graycode A int %i\n", GrayA);
    pr_uint32_write( PrGrayPos, GrayPos);
  }
}

void GraycodeInASetup() {
  MyPinmode( GRAYCODE_IN_A_PIN, INPUT); // lego nxt dislike pullup
  GraycodeInAInt ();
  // dbgprintf( 2, "graycode A init %i\n", GRAYCODE_IN_A_PIN);
  attachInterrupt( digitalPinToInterrupt(GRAYCODE_IN_A_PIN), GraycodeInAInt, CHANGE);
}

// 00 -> 01 -> 11 -> 10 -> 00

void ICACHE_RAM_ATTR GraycodeInBInt () {
  if (digitalRead(GRAYCODE_IN_B_PIN) != GrayB)
  {
    if (GrayA) {
      if (GrayB) { // 11 -> 10
        GrayPos++;
      } else { // 01 -< 00
        GrayPos--;
      }
    } else {
      if (GrayB) { // 10 -< 11
        GrayPos--;
      } else { // 00 -> 01
        GrayPos++;
      }
    }
    GrayB = !GrayB;
    // dbgprintf( 2, "graycode B int %i\n", GrayB);
    pr_uint32_write( PrGrayPos, GrayPos);
  }
}

void GraycodeInBSetup() {
  MyPinmode( GRAYCODE_IN_B_PIN, INPUT); // lego nxt dislike pullup
  GraycodeInBInt ();
  attachInterrupt( digitalPinToInterrupt(GRAYCODE_IN_B_PIN), GraycodeInBInt, CHANGE);
}

void GraycodeInSetup() {
  GraycodeInASetup();
  GraycodeInBSetup();

  pr_uint32_write( PrGrayPos, 0);
}
#endif /* WITH_GRAYCODE_IN */

#ifdef WITH_WIFI // ------------------------------------

int WifiInit( void) {
  int tpw;
  int iTmp;
  int n;

  int NodeNum = NodeGet();

  strcpy( SrvPWD, BEN_PWD);
  tpw = RssiCorrectionGet();
#ifdef ESP8266
  // TODO_HERE : ESP32
  if ((tpw >= 128 - MAX_TPW / 2) && (tpw <= 128 + MAX_TPW / 2)) {
    iTmp = map(RssiCorrectionGet(), 128 - MAX_TPW / 2, 128 + MAX_TPW / 2, 0, 82);
    tpw = 128;
  } else if (tpw > 128) {
    iTmp = MAX_TPW;
    tpw -= MAX_TPW / 2 - 1;
  } else {
    iTmp = 0;
    tpw += MAX_TPW / 2 + 1;
  }
  system_phy_set_max_tpw( iTmp); // uint8 max_tpw 0..82
  dbgprintf( 3, "tpw set %i pwr", iTmp);
  dbgprintf( 3, ", %i by pwr\r\n", iTmp - MAX_TPW / 2);
  #ifdef WITH_WIFI
    n = WiFi.scanNetworks( true);
    dbgprintf( 2, "WifiInit Net1 %i\r\n", n);
  #endif
#endif /* ESP8266 */

  RssiCls();

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_AP_STA);

  SrvSSID[0] = 0;
  switch (RoleGet()) {
    case ROLE_TEST :
    case ROLE_PLAYER :
#ifdef WITH_IOTMUTUAL
      NodeNum = NodeGet();
      memcpy( SrvSSID, "ITS_", 4);
      SrvSSID[4] = '0' + (NodeNum / 100) % 10;
      SrvSSID[5] = '0' + (NodeNum / 10) % 10;
      SrvSSID[6] = '0' + (NodeNum    ) % 10;
      SrvSSID[7] = 0;
#endif /* WITH_IOTMUTUAL */
      break;
    case ROLE_SOLO :
    case ROLE_MULTI :
#ifdef WITH_IOTMUTUAL
      memcpy( SrvSSID, "ITS_", 5);
      NodeNum = WiFi.macAddress()[16] * 0x10 + WiFi.macAddress()[17]; // try to build something different from a device to another
      SrvSSID[4] = '0' + (NodeNum / 100) % 10;
      SrvSSID[5] = '0' + (NodeNum / 10) % 10;
      SrvSSID[6] = '0' + (NodeNum    ) % 10;
      SrvSSID[7] = 0;
#endif /* WITH_IOTMUTUAL */
      // no wifi, just button and sound
      break;
    case ROLE_IOTS :
    case ROLE_IOTM :
#ifdef WITH_IOTMUTUAL
      NodeNum = NodeGet();
      memcpy( SrvSSID, BEN_TAG, 4);
      SrvSSID[4] = '0' + (NodeNum / 100) % 10;
      SrvSSID[5] = '0' + (NodeNum / 10) % 10;
      SrvSSID[6] = '0' + (NodeNum    ) % 10;
      SrvSSID[7] = 0;
#endif /* WITH_IOTMUTUAL */
      break;
    case ROLE_PASSENGER :
      NodeNum = NodeGet();

      memcpy( SrvSSID, BEN_TAG, 4);
      SrvSSID[4] = '0' + (NodeNum / 100) % 10;
      SrvSSID[5] = '0' + (NodeNum / 10) % 10;
      SrvSSID[6] = '0' + (NodeNum    ) % 10;
      SrvSSID[7] = 0;
      // RssiCorrectionGet -> a..z A..Z
      if ((tpw > 128) && (tpw < 128 + MAX_LTR)) {
        SrvSSID[7] = 'A' + (tpw - 128);
        SrvSSID[8] = 0;
      }
      if ((tpw > 128 - MAX_LTR) && (tpw < 128)) {
        SrvSSID[7] = 'a' + (128 - tpw);
        SrvSSID[8] = 0;
      }
      dbgprintf( 3, "tpw set %i by letter\r\n", tpw - 128);
      break;
  }
  if (0 != SrvSSID[0]) {
#ifdef WITH_WIFICLIENT
    if (0 == memcmp( BEN_TAG, CliSSID, 4)) {
      dbgprintf( 2, "Connection between relatives detected, try to serve 192.168.5.1\n", SrvSSID); // instead of 192.168.5.1
      WiFi.softAPConfig( 0x0105A8C0, 0x0105A8C0, 0x00FFFFFF);
    } else {
      dbgprintf( 2, "Try to serve 192.168.4.1\n", SrvSSID); // default
    }
#else
    dbgprintf( 2, "Try to serve 192.168.4.1\n", SrvSSID); // default
#endif /* WITH_WIFICLIENT */

    WiFi.softAP( SrvSSID, SrvPWD);
    dbgprintf( 3, "Wifi AP %s started\n", SrvSSID);
  }

#ifdef WITH_OSC
  memset( OscAddrMine, 0, OSC_CMD_LEN);
  OscAddrMine[0] = '/';
  memcpy( OscAddrMine + 1, SrvSSID, min( OSC_CMD_LEN - 1, WIFI_IDLEN));
  OscAddrMine[ OSC_CMD_LEN - 1] = 0;
#endif /* WITH_OSC */

  return ( 0);
}

int WifiScan( void) {
  int n;
  int i;
  int vLev = 2;
  char szId[WIFI_IDLEN];

  //dbgprintf( 2,"WifiScan()\r\n");
#ifdef WITH_WIFI
#ifdef WIFI_NONBLOCKING
  n = WiFi.scanComplete();
  if ( WIFI_SCAN_RUNNING == n) {
    //dbgprintf( 2, "WifiScan Running %i\r\n", n);
    return (0);
  } else if (n < 0) {
    dbgprintf( 2, "WifiScan error milord %i\r\n", n);
    n = WiFi.scanNetworks( true);
    //dbgprintf( 2, "WifiScan NetC %i\r\n", n);
    return (0);
  }
  //dbgprintf( 2, "WifiScan completed %i\r\n", n);
  // https://github.com/esp8266/Arduino/blob/d108a6ec30193a8e44eb9c72efc7a55853b54f09/hardware/esp8266com/esp8266/libraries/ESP8266WiFi/src/ESP8266WiFiMulti.cpp#L74
#else
  n = WiFi.scanNetworks(); // sync, cost +- 4 seconds
  // dbgprintf( 2, "WifiScan Blocking %i\r\n", n);
#endif /* WIFI_NONBLOCKING */


  if (n < 0) { // WIFI_SCAN_RUNNING -1? WIFI_SCAN_FAILED -2?
    //dbgprintf( vLev+1, "WIFI_SCAN_FAILED %i\r\n", WIFI_SCAN_FAILED);
    //dbgprintf( vLev+1, "Scan %i\r\n", n);
    return ( 0);
  }
  dbgprintf( vLev + 1, "Got Scan %i\r\n", n);
#else
  n = 0;
  dbgprintf( vLev + 1, "ERR- no hardware, no scan");
#endif

  if (n == 0) {
    dbgprintf( vLev + 1, "no networks found\r\n");
  } else {
    int Cnt;

    if (n > 12) {
      dbgprintf( vLev + 1, "%i networks found\r\n", n);
    }

    // init array (old are in backups)
    for (Cnt = 0; Cnt < PLAYERS_NB; Cnt++) {
      PlayersId[Cnt]   = 0;
      PlayersMac[Cnt]  = 0;
      PlayersRssi[Cnt] = RSSI_MIN;
    }

    // fill array with what we found
    for (int i = 0; i < n; ++i)
    {
      byte* bssid; // [6] ?
      uint32_t Mac;

      bssid = WiFi.BSSID( i);
      Mac = bssid[5] * 0x100 + bssid[4]  +  bssid[3] * 0x1000000 + bssid[2] * 0x10000 + bssid[1] * 0x100 + bssid[0]; // TODO_LATER : map 6 on 4
      RssiTmp = WiFi.RSSI(i);

      memset( szId, 0, WIFI_IDLEN);
      WiFi.SSID(i).toCharArray( szId, WIFI_IDLEN); // len contains ending 0

      // Print SSID and RSSI for each network found
      dbgprintf( vLev + 2, "Net:%i:", i + 1);
      dbgprintf( vLev + 2, " 0x%08x", Mac);
      dbgprintf( vLev + 2, ":%12s", szId);
      //Serial.print(WiFi.SSID(i));
      dbgprintf( vLev + 2, " Pwr:%i", RssiTmp);
  #ifndef ESP32 // TODO_HERE
      if (ENC_TYPE_NONE == WiFi.encryptionType(i)) {
        dbgprintf( vLev + 2, " *");
      }
  #endif
      dbgprintf( vLev + 2, "\r\n");

      RunOtherTasks( 0);
      // select only the candidates, the passengers
      if (0 == memcmp( BEN_TAG, szId, 4)) {
        // get id
        g_iTmp  = 0;
        g_iTmp  = szId[4] - '0';
        g_iTmp  = g_iTmp * 10;
        g_iTmp += szId[5] - '0';
        g_iTmp  = g_iTmp * 10;
        g_iTmp += szId[6] - '0';

        // RssiCorrectionGet
        if (0 != szId[7]) {
          int Adj = 0;
          Adj = szId[7];
          if ((Adj >= 'a') && (Adj <= 'v')) { // w..z reserved ... lots of books set 'x' meaningless
            RssiTmp -= Adj - 'a';
          }
          if ((Adj >= 'A') && (Adj <= 'Z')) {
            RssiTmp += Adj - 'A';
          }
        }

        Cnt = RssiSearch( 2, g_iTmp, Mac);
        RssiContrib = RSSI_MIN;
        if (Cnt >= 0) {
          RssiContrib = PlayersRssi2[Cnt];
          PlayersId2[Cnt] = 0;
          PlayersMac2[Cnt] = 0;
          PlayersRssi2[Cnt] = 0;
        }
        // dbgprintf( vLev+2, " Rssi Contrib:%i", RssiContrib);
        RssiTmp = (RssiTmp * 2 + RssiContrib) / 3; // here is the leveling
        // dbgprintf( vLev+2, " Rssi aftr:%i", RssiTmp);
        RssiAdd( g_iTmp, Mac, RssiTmp);
        // dbgprintf( vLev+2, "\n");
      }
    } // end net loop

    // faint the missing ones
    for (Cnt = 0; Cnt < PLAYERS_NB; Cnt++) {
      if (0 != PlayersMac2[Cnt]) {
        RssiAdd( PlayersId2[Cnt], PlayersMac2[Cnt], ((PlayersRssi2[Cnt] * 2 + RSSI_MIN) / 3));
      }
    }
    // check coherency
    for (Cnt = 0; Cnt < PLAYERS_NB; Cnt++) {
      if (0 == PlayersMac[Cnt]) {
        PlayersId[Cnt]   = 0;
        PlayersMac[Cnt]  = 0;
        PlayersRssi[Cnt] = RSSI_MIN;
      }
    }
    // copy for memory on next scan
    for (Cnt = 0; Cnt < PLAYERS_NB; Cnt++) {
      PlayersId2[Cnt]   = PlayersId[Cnt];
      PlayersMac2[Cnt]  = PlayersMac[Cnt];
      PlayersRssi2[Cnt] = PlayersRssi[Cnt];
    }
    // dbgprintf( 3, "After full parse\r\n");
    //RssiDump( 2, 0);

  }
#ifdef WITH_WIFI
#ifdef WIFI_NONBLOCKING
  n = WiFi.scanNetworks( true);
  //dbgprintf( 2, "WifiScan Net %i\r\n", n);
#endif
#endif


  //  best is PlayersId[0] PlayersRssi[0]
  dbgprintf( 3, "Winner %i", PlayersId[0]);
  dbgprintf( 3, " %i \n", PlayersRssi[0]);

  RssiDump( 3, 0);
#ifdef WITH_LED_SHOW_WIFI
  PixelShow( 3);
#endif /* WITH_LED_SHOW_WIFI */


  return ( 0);
}

#endif /* WITH_WIFI */

#ifdef WITH_SWITCH // -----------------------
int SwitchInit( void) {
#if (16 == SWITCH_LINE)
  // rem : if SWITCH_LINE is 16 on ESP8266 a physical pullup (10kOhm?) is required
  dbgprintf( 2, "SwitchInit 16\r\n");
  MyPinmode( SWITCH_LINE, INPUT);
#else
  MyPinmode( SWITCH_LINE, INPUT_PULLUP);
#endif
  SwitchState = HIGH;
  SwitchTimePushmillis = millis();
  SwitchTimeRelmillis  = millis();
  SwitchTimeTestmillis = millis();
  return (0);
}

bool SwitchIsPressed( void) {
  // dbgprintf( 2,"SW check\r\n");
  bool ReadedPhy;
  long CurrTimeMillis = millis();
  int SwShut;

  if (ROLE_TEST == RoleGet()) { // auto click all night long
    SwitchState = HIGH;
    if (abs(CurrTimeMillis - SwitchTimeTestmillis) > 25000) {
      SwitchState = LOW; // simul pressed
      dbgprintf( 2, "SW test autoplay %i\r\n", SwitchState);
      if (abs(millis() - SwitchTimeTestmillis) > 26000) {
        SwitchTimeTestmillis = millis();
      }
    }
    return (HIGH != SwitchState); // switch activated (connected to gnd)
  } // end test mode

  // time one must remove it's head before we stop playing or restart, usual 6000
  //SwShut = 6000;
  SwShut = SWITCH_GLITCH_SHUT;

  if (ROLE_MULTI ==  RoleGet()) { // don't bother the guy learning how to play
    SwShut = SWITCH_GLITCH_SHUT;
  }


  ReadedPhy = digitalRead( SWITCH_LINE); // HIGH(1) released, LOW(0) pushed (connected to ground)
  if ( HIGH == SwitchState) { // Supposed released
    if (HIGH == ReadedPhy) { // phy released
      SwitchTimeRelmillis = CurrTimeMillis;
    }
    if (abs(CurrTimeMillis - SwitchTimeRelmillis) > SWITCH_GLITCH) { // not released more than SWITCH_GLITCH/1000 sec
      SwitchState = LOW; // not released for a while means firmly pushed
      SwitchTimePushmillis = CurrTimeMillis - SwShut - 100;
#ifdef WITH_OSC
      OscSendCmd( 1);
#endif /* WITH_OSC */
    }
    if (abs(CurrTimeMillis - SwitchTimeRelmillis) > 2 * SWITCH_GLITCH) {
      // TODO_LATER : don't work SwitchTimeRelmillis = CurrTimeMillis - 2*SWITCH_GLITCH; // never let a chance to cycle
    }
  } else { // supposed pushed
    if (LOW == ReadedPhy) { // phy pushed
      SwitchTimePushmillis = CurrTimeMillis;
    }
    if (abs(CurrTimeMillis - SwitchTimePushmillis) > SwShut) { // not pushed more than SW_SHUT/1000 sec
      SwitchState = HIGH; // not pushed for a while means released
      SwitchTimeRelmillis = CurrTimeMillis - SWITCH_GLITCH - 100;
#ifdef WITH_OSC
      OscSendCmd( 2);
#endif /* WITH_OSC */
    }
    if (abs(CurrTimeMillis - SwitchTimePushmillis) > 2 * SwShut) {
      // TODO_LATER : don't work SwitchTimePushmillis = CurrTimeMillis - 2*SwShut; // never let a chance to cycle
    }
  }

  /* this debugs the switch
    dbgprintf( 3,"SW phy %s", (HIGH == ReadedPhy) ? "rel" : "psh");
    dbgprintf( 3,"(%i) ", ReadedPhy);
    dbgprintf( 3,", Stat %s", (HIGH == SwitchState) ? "rel" : "psh");
    dbgprintf( 3,"(%i) ", SwitchState);
    dbgprintf( 3,",tr %08i ",SwitchTimeRelmillis);
    dbgprintf( 3,",tp %08i ",SwitchTimePushmillis);
    dbgprintf( 3,",tc %08i ",CurrTimeMillis);
    dbgprintf( 3,",rel %08i ",abs(CurrTimeMillis - SwitchTimeRelmillis));
    dbgprintf( 3,",pus %08i ", abs(CurrTimeMillis - SwitchTimePushmillis));
    dbgprintf( 3,"\r\n", SwitchState);
  */

  return (HIGH != SwitchState); // switch activated (connected to gnd)
}

void SwitchParse( ) {

  if (SwitchIsPressed()) {
    if ((0 == PlayerRunning)
        && (  (ROLE_MULTI ==  RoleGet())
              || (0 == RssiMinimalGet())
              || (PlayersRssi[0] > (-RssiMinimalGet())))
       ) {
      dbgprintf( 3, "SW pressed\r\n");
      if (ROLE_MULTI == RoleGet()) {// solo plays track per track
        if (NodeGet() >= MaxGet()) {
          NodeSet( 1);
        } else {
          NodeSet( NodeGet() + 1);
        }
      } // end ROLE_MULTI
#ifdef WITH_DFPLAYER
      TrackRun();
#else
      dbgprintf( 3, "Can't play %i %i due to no DFP\r\n", NodeGet(), PlayersId[0]);
#endif
      Activity( ACT_PLAY);
      PlayerRunning = 1;
    }
  } else {
    //TODO_HERE : store, timer, + off time + etc....

    if (1 == PlayerRunning) {

      /*
        dbgprintf( 3,"SW released\r\n");
        #ifdef WITH_DFPLAYER
        TrackStop();
        #else
        dbgprintf( 3, "Can't stop play due to no DFP\r\n", NodeGet(), PlayersId[0]);
        #endif
        Activity( ACT_READY);
        PlayerRunning = 0;
      */

      if (TrackIsRunning() <= 0) { // -1 no resp, 0 not play, >0 play
        Activity( ACT_READY);
        PlayerRunning = 0;
      }
    }
  }
}

#endif /* WITH_SWITCH ----------------- */

#ifdef RESTART_PIN //  ----------------- 

int RestartSwitchInit( void) {
  MyPinmode( RESTART_PIN, INPUT_PULLUP);
  RestartSwitchState = HIGH;
  RestartSwitchTimemillis = millis();
  return (0);
}

bool RestartSwitchIsPressed( void) {
  // dbgprintf( 2,"Restart SW check\r\n");

  if (abs(millis() - RestartSwitchTimemillis) > 500) { // anti rebond
    RestartSwitchState = digitalRead( RESTART_PIN);
    RestartSwitchTimemillis = millis();
  }
  return (HIGH != RestartSwitchState); // switch activated (connected to gnd)
}

void RestartSwitchParse( ) {
  if (RestartSwitchIsPressed()) {
    dbgprintf( 2, "Restart SW pressed\r\n");
    if (ROLE_MULTI == RoleGet()) {// solo plays track per track
      NodeSet( MaxGet()); // will go 1 next head push
#ifdef RESTART_B_PIN
      RestartedGuy = 0;
#endif
    } // end ROLE_MULTI
  } else {
    //dbgprintf( 3,"SW released\r\n");
  }
}

#endif /* RESTART_PIN ----------------- */

#ifdef RESTART_B_PIN //  ----------------- 

int RestartBSwitchInit( void) {
  MyPinmode( RESTART_B_PIN, INPUT);
  RestartBSwitchState = HIGH;
  RestartBSwitchTimemillis = millis();
  return (0);
}

bool RestartBSwitchIsPressed( void) {
  // dbgprintf( 2,"Restart SW check\r\n");

  if (abs(millis() - RestartBSwitchTimemillis) > 500) { // anti rebond
    RestartBSwitchState = digitalRead( RESTART_B_PIN);
    RestartBSwitchTimemillis = millis();
  }
  return (HIGH != RestartBSwitchState); // switch activated (connected to gnd)
}

void RestartBSwitchParse( ) {
  if (RestartBSwitchIsPressed()) {
    //dbgprintf( 2,"Restart B SW pressed\r\n");
    if (ROLE_MULTI == RoleGet()) {// solo plays track per track
      NodeSet( MaxGet()); // will go 1 next head push
      RestartedGuy = 1;
    } // end ROLE_MULTI
  } else {
    //dbgprintf( 3,"SW B released\r\n");
  }
}

#endif /* RESTART_B_PIN ----------------- */

#ifdef WITH_ADC
// read and Act ... if Act not 0
void AdcProcess( uint8_t Act) {
  uint32_t CurTime;
  uint32_t RootVal;
  uint32_t ReadVal;

  CurTime = millis();
  if (DiffTime(AdcLastReadMs, CurTime) > 25) { // don't read too often or wifi can't connect ... who know why
    ReadVal = analogRead(WITH_ADC);
    if (ReadVal < 10) {
      RootVal = 0;
      AdcVal = 0;
    } else if (ReadVal < 950) {
      RootVal = map(ReadVal, 10, 950, 0, 9999);
      AdcVal = (RootVal + AdcVal * ADC_SMOOTH) / (ADC_SMOOTH + 1);
    } else {
      RootVal = 9999;
      AdcVal = 9999;
    }
    AdcLastReadMs = CurTime;
  }
  if (   Act
         && (   (DiffTime( AdcLastSent, AdcVal) >= 35)
                || ((AdcLastSent != 0) && (AdcVal == 0))
            )  ) {
    // dbgprintf( 1, "Adc %5i\n", AdcVal);
#ifdef WITH_OSC
    // send to osc, osc can drive motors, including this one if set some "*" in the destination
    OscSendCmd( 3);
#else
#if (MOTOR_NB > 0)
    // send direct to motor
    for (Idx = 0; Idx < MOTOR_NB; Idx++) {
      Motor_t* pMotor = &(MotorArray[Idx]);
      pr_uint32_write( pMotor->WishP2, AdcVal);
      if(pMotor->Order < 3) {
        pMotor->Order++;
      }
    }
#endif /* WITH_MOTOR */
#endif
    AdcLastSent = AdcVal;
  }
}
#endif /* WITH_ADC */

#ifdef WITH_SCREEN
void ScreenSetup() {

  display.init();

  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  ScreenLastMs = millis();

}

void ScreenLoop() {
  String LocStr;
  uint32_t TimeRunningMs;

  if ( DiffTime(ScreenLastMs, millis())< 232) {
    return;
  }
  ScreenLastMs = millis();
  
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  //TimeRunningDump( LocStr);
  //display.drawString(0, 0, LocStr);

  TimeRunningMs = millis();
  LocStr += "-TimeRunning ";
  LocStr += TimeRunningMs;
  display.drawString(0, 0, LocStr);

   LocStr = "loop ";
  LocStr += LoopUs;
  LocStr += "us\n";
  display.drawString(0, 20, LocStr);
  
  display.display();
}
#endif /* WITH_SCREEN */

void TimeRunningDump( String& LocStr) {
  uint32_t TimeRunningMs; // temp de fonctionnement, affiche sur la sortie debug, utile pour les tests de duree de fonctionnement
  int iTmp;
  int Count =3;
  int St = 0;

  TimeRunningMs = millis();
  LocStr += "-TimeRunning ";
  iTmp = (TimeRunningMs/(24*60*60*(uint32_t)1000)); // jours
  LocStr += iTmp;
  LocStr += "D";
  iTmp = (TimeRunningMs/(60*60*(uint32_t)1000)%24); // heures
  LocStr += CSTRIn( 2, iTmp);
  LocStr += "H";
  iTmp = (TimeRunningMs/(60*(uint32_t)1000))%60;// minutes
  LocStr += CSTRIn( 2, iTmp);
  LocStr += "m";
  iTmp = (TimeRunningMs/(uint32_t)1000)%60;  // secondes
  LocStr += CSTRIn( 2, iTmp);
  LocStr += "s, loop ";
  LocStr += LoopUs;
  LocStr += "us\n";
}

// Description : just show alive (if NewState is 0) or set a new state
// the caller is responsible to call frequently enough but not that munch :)
void Activity( int NewState) {
  String LocStr;
  TimeRunningDump( LocStr);
  Serial.print( LocStr);
  
  if (0 != NewState) {
    State = NewState;
  }

#ifdef TO_RIP
  if (ACTIVITY_LED > 0) {
    digitalWrite(ACTIVITY_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
    delay(400);                      // Wait for a second
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
    delay(100);                      // Wait for two seconds (to demonstrate the active low LED)
  }
#endif /* TO_RIP */

#ifdef WITH_LED_SHOW_WIFI
  if (ACT_READY == State) {
    PixelStatus( State);
#ifdef WITH_LED_SHOW_WIFI
    PixelShow( 1);
#endif /* WITH_LED_SHOW_WIFI */
    delay(10);
#ifdef WITH_LED_SHOW_WIFI
    PixelShow( 0); // spare power
#endif /* WITH_LED_SHOW_WIFI */
  } else {
    if (0 != NewState) {
      PixelStatus( State);
#ifdef WITH_LED_SHOW_WIFI
      PixelShow( 1);
#endif /* WITH_LED_SHOW_WIFI */
    }
  }
#endif /*WITH_LED_SHOW_WIFI */
}

// WITH_MOTOR ----------------------------------------------------------
#ifdef WITH_MOTOR

void MotorSetup() {
  int Idx;
  int UserMode;
  Motor_t* pMotor;

  memset( &(MotorArray[0]), 0, sizeof(MotorArray));

  for (Idx = 0; Idx < MOTOR_NB; Idx++) {
    pMotor = &(MotorArray[Idx]);
    pr_uint32_write( pMotor->WishDTms, 100);
    pMotor->Order = -1;
    switch (Idx) {
      case 0 : UserMode = EprMotor0ModeGet(); break;
      case 1 : UserMode = EprMotor1ModeGet(); break;
      case 2 : UserMode = EprMotor2ModeGet(); break;
      default : UserMode = 0;                 break;
    }
    if (0 != UserMode) {
      pMotor->DriverMode = UserMode;
    }
  }

  pMotor->LastPulseUs = micros();

  #ifdef WITH_STEPPER
    StepperSetup( );
  #endif /* WITH_STEPPER */

  #ifdef WITH_DCMOTOR
    DcMotorSetup();
  #endif /* WITH_DCMOTOR */

  #ifdef WITH_SERVO
    for ( uint8_t i = 0; i < ServoNb; i++) {
      ServoInit( i, ServoPins[i]);
    }
  #endif /* WITH_SERVO */

  #ifdef STEPPER_IN_PULSE_PIN
    StepperInSetup();
  #endif /* STEPPER_IN_PULSE_PIN */

  #ifdef WITH_RELAY
    MyPinmode( WITH_RELAY, OUTPUT);
    digitalWrite( WITH_RELAY, LOW);
  #endif /* WITH_RELAY */

  MaxXGet();
  SpeedXGet();
  VectX = SpeedX;
  MaxYGet();
  SpeedYGet();
  VectY = SpeedY;

}

void MotorLoop( int FromInterrupt) {
  int32_t k; // 0..KFACT
  uint32_t CurrPos;
  uint32_t ExpectedPos;
  uint32_t CurTime;
  uint32_t NextTime;
  uint32_t CurMicros;
  int Idx;
  Motor_t* pMotor;
#define KFACT 1000

  // T1M = k T1T2
  // P1M = k P1P2

  CurTime = millis();
  CurMicros = micros();

  for (Idx = 0; Idx < MOTOR_NB; Idx++) {
    pMotor = &(MotorArray[Idx]);

    CurrPos = pr_uint32_read( pMotor->Pos);
    // dbgprintf( 2, " CurrPos %u\n", CurrPos);

    switch( pMotor->DriverMode) {
      default : 
        break;
      case MODOR_DC_LR :
        #ifdef WITH_GRAYCODE_IN
          CurrPos = pr_uint32_read( PrGrayPos);
        #endif /* WITH_GRAYCODE_IN */
        break;
    }

    // return for the unsuitables from inside interrupts
    switch( pMotor->DriverMode) {
      default :
      case MOTOR_PWM2:
        // no use to cut the flow
        break;
      case MOTOR_PWM: // full of analog write probably interrupt driven ... unsuitable inside interrupts itself
      #ifdef WITH_SERVO
        case MOTOR_SERVO: // servo interrupt driven ... unsuitable to set inside interrupts itself
      #endif /* WITH_SERVO */
        if (FromInterrupt) {
          return;
        }
        break;
    }

    // returns if inside moratory time window
    if ( DiffTime( CurMicros, pMotor->LastPulseUs) < pMotor->MinCommandUs) {
      return;
    }

    //dbgprintf( 2, " Order:%5i\n", pMotor->Order);
    // transfert external orders in the vector
    if ( pMotor->Order == 5) { // order 5 set everything
      pMotor->LastPos = pMotor->P2 = pMotor->P1 = pr_int32_read( pMotor->WishP2) - pMotor->WishDec;
      pMotor->Order = 0;
    } else if ( pMotor->Order > 0) {
      pMotor->P1 = CurrPos;
      //dbgprintf( 2, " trorder P1 CP %u %u\n", pMotor->P1, CurrPos);
      pMotor->T1ms = CurTime;
      k = 50;
      while( (k > 0) && (pMotor->Order > 0)) { // some friend might flow with orders
        pMotor->Order = 0;
        k--;
        pMotor->P2 = pr_int32_read( pMotor->WishP2) - pMotor->WishDec;
        pMotor->T2ms = CurTime + pr_uint32_read( pMotor->WishDTms);
      }
      // dbgprintf( 2, " transfert order %i %u %u\n", pMotor->Order, pMotor->P1, pMotor->P2);
      //dbgprintf( 2, " transfert order %i %u\n", pMotor->Order, pMotor->P1);
    }

    // compute expected pos
    if ( CurTime > pMotor->T2ms) {
      //k = KFACT;
      k = KFACT + 1;
      ExpectedPos = pMotor->P2;
    } else if (CurTime < pMotor->T1ms) {
      //k = 0;
      k = -1;
      ExpectedPos = pMotor->P1;
    } else {
      int32_t DiffT;
      int32_t DiffK;
      DiffT = Vectorize( pMotor->T1ms, pMotor->T2ms);
      // dbgprintf( 2, " DiffT %i", DiffT);
      if (0 == DiffT) {
        k = KFACT;
      } else {
        DiffK = Vectorize( pMotor->T1ms, CurTime);
        //dbgprintf( 2, " DiffK %i", DiffK);
        k = (KFACT * DiffK) / DiffT;
      }
      //if (pMotor->P2 > pMotor->P1) {
      ExpectedPos = pMotor->P1 + k * Vectorize( pMotor->P1, pMotor->P2) / KFACT;
      //} else {
      //  ExpectedPos = pMotor->P1 - k*(pMotor->P1 - pMotor->P2)/KFACT;
      //}
    }
    // dbgprintf( 2, " k %i", k);
    // RunOtherTasks( 200); // for Dev

    // TODO_LATER : timed component and immediate component does not mix as expected, some work to do around
    if (0 == Idx) {
#ifdef WITH_STEPPER_IN
      ExpectedPos += pr_uint32_read( pr_StepperInPos);
#endif /* WITH_STEPPER_IN */
    }

    CurrPos = ExpectedPos;
    switch( pMotor->DriverMode) {
      default : 
      case MOTOR_UNDEF:
        // hoops
        // dbgprintf( 2, "MotorLoop Hoops %i undef %i\n", Idx, pMotor->DriverMode); // miss in code, must init somewhere
        break;
      #ifdef WITH_STEPPER
        case MOTOR_STEPPER:
          CurrPos = StepperLoop( pMotor, ExpectedPos);
          break;
      #endif /* WITH_STEPPER */
      #ifdef WITH_DCMOTOR
        case MODOR_DC_LR :
          CurrPos = DcPosLoop( pMotor, CurrPos);
          break;
        case MOTOR_PWM:
          MotorDcPwm( Idx, ExpectedPos);
          break;
        case MOTOR_PWM2:
          MotorDcPwm2( Idx, ExpectedPos);
          break;
      #endif /* WITH_DCMOTOR */
      #ifdef WITH_SERVO
        case MOTOR_SERVO:
          ServoPosLoop( Idx, ExpectedPos);
          break;
      #endif /* WITH_SERVO */
    }
    

    pMotor->LastPulseUs = CurMicros;
    pr_uint32_write( pMotor->Pos, CurrPos);
  } // end for each motor
}
#endif /* WITH_MOTOR */

// WITH_TIMER ----------------------------------------------------------

#ifdef WITH_TIMER

void TimerStart();
void TimerStop();
void TimedInt( void* pArg);

void TimerSetup() {
  os_timer_setfn( &Timer, TimedInt, NULL);
}

void TimerStart() {
  if (!TimerActivated) {
    //os_timer_arm( &Timer, 1, true);
    //os_timer_arm_us( &Timer, 100, true);
    // to_test
    ets_timer_arm_new( &Timer
#ifdef TIMER_SLICE_SU_DEF
                       , TIMER_SLICE_SU_DEF
#else
                       , 500
#endif
                       , 1, 0); // See more at: http://www.esp8266.com/viewtopic.php?p=14853#sthash.mHhnPhpF.dpuf
    TimerActivated = 1;
  }
}

void TimerStop() {
  if (TimerActivated) {
    os_timer_disarm( &Timer);
    TimerActivated = 0;
  }
}
#endif /* WITH_TIMER */

void TimedInt( void* pArg) {
#ifdef WITH_TIMER
  TimerStop();
#endif /* WITH_TIMER */
  // TODO_HERE   StepperSpeedPerMilSec ; // ticks per hour if drived per speed
#ifdef WITH_MOTOR
  MotorLoop( 1);
#endif /* WITH_MOTOR */
#ifdef WITH_TIMER
  TimerStart();
#endif /* WITH_TIMER */
}

void AutoActivated() {
// this really is app specific

#ifndef NO_APP
  static int LastMs = 0;
  // orig rail 1000
  int CurMs = millis();
  #ifdef WITH_MOTOR
    Motor_t* pMotor;
  #endif /* WITH_MOTOR */
  int K;
    uint32_t Rand;

  
  if ( DiffTime(CurMs, LastMs) > 500) {
    LastMs = CurMs;
    
    MyY += VectY;
    if (0 == SpeedX) { // X linked to Y
      MyX = map(MyY, 0, MaxY, 0, MaxX);
      MyX = constrain( MyX, 0, MaxX);
    } else{
      MyX += VectX;
    }
  //dbgprintf( 2, "MyX %i, MyY %i", MyX, MyY);
  //dbgprintf( 2, "VectX %i, VectY %i\n", VectX, VectY);
    
#ifdef WITH_MOTOR
    pMotor = &(MotorArray[0]);
    pr_uint32_write( pMotor->WishDTms, 5000);
    pr_int32_write( pMotor->WishP2, MyX);
    if(pMotor->Order < 3) {
      pMotor->Order++;
    }

    pMotor = &(MotorArray[1]);
    pr_uint32_write( pMotor->WishDTms, 5000);
    pr_int32_write( pMotor->WishP2, MyY);
    if(pMotor->Order < 3) {
      pMotor->Order++;
    }

  #ifdef WITH_STOPPER
    if(HIGH != digitalRead( StopperPins[0])) {
      dbgprintf( 2, "touch min\n");
      if (SensY < 0) {
        dbgprintf( 2, "set min\n");
        //SensY = 1;
        MyY = -1;
        pMotor = &(MotorArray[1]);
        pr_int32_write( pMotor->WishP2, MyY);
        pMotor->Order=5;
        
      }
    }

    if(HIGH != digitalRead( StopperPins[1])) {
      dbgprintf( 2, "touch max\n");
      if (SensY > 0) {
        dbgprintf( 2, "set max\n");
        MyY = MaxY+1;
        //SensY = -1;
        pMotor = &(MotorArray[1]);
        pr_int32_write( pMotor->WishP2, MyY);
        pMotor->Order=5;
      }
    }
    //dbgprintf( 2, "MyY %i ", MyY);
    //dbgprintf( 2, "LastPosY %i \n", MotorArray[1].LastPos);
    /*
    if(HIGH != digitalRead( StopperPins[1])) {
      dbgprintf( 2, "touch max\n");
      StopperMax = (StopperMax*2+1000)/3;
    } else {
      StopperMax = (StopperMax*2+0)/3;
      
    }
    //dbgprintf( 2, "max %i\n", StopperMax);
    if (StopperMax > 500) {
      dbgprintf( 2, "real max\n");
            MyY = MaxY+1;
    }
    */
  #endif /* WITH_STOPPER */
#endif /* WITH_MOTOR */

    if ((MyX > MaxX) && (SensX > 0)) {
      VectX = -SpeedX;
      SensX = -1;
      MyX -= 1;
    }
    if ((MyX < 0) && (SensX < 0)) {
      VectX = SpeedX;
      SensX = 1;
      MyX = 1;
    }
    if ((MyY > MaxY) && (SensY > 0)) {
      VectY = - SpeedY;
      SensY = -1;
      MyY -= 1;
    }
    if ((MyY < 0) && (SensY < 0)) {
      VectY = SpeedY;
      SensY = 1;
      MyY = 1;
    }
  }
  //dbgprintf( 2, "VectX %i\n", VectX);


// marianne eyes blink
  //K = map( MyX, 0, MaxX, 0, 99);
  //K = constrain(K, 0, 99); 
  //PixelShow99( K , K/2 , 0 , 0);
  //PixelShow99( K , K/2 , 0 , 1);

#ifdef WITH_RELAY
// marianne light blink
  Rand = random(9999);

  K = map( MyX, 0, MaxX, 0, 1000);
  K = constrain(K, 0, 1000); 
  dbgprintf( 3, "Relay %i\n", K);
  if (K < 100) {
    digitalWrite( WITH_RELAY, LOW);
  } else if (K < 400) {
    K = map( K, 100, 400, 0, 10000);
    if (K>Rand) {
      digitalWrite( WITH_RELAY, LOW);
    } else {
      digitalWrite( WITH_RELAY, HIGH);
    }
  } else if (K < 900) {
    K = map( K, 400, 900, 0, 10000);
    if (K>Rand) {
      digitalWrite( WITH_RELAY, LOW);
    } else {
      digitalWrite( WITH_RELAY, HIGH);
    }
  } else {
    digitalWrite( WITH_RELAY, HIGH);
  }
  #endif /* WITH_RELAY */

#endif /* NO_APP */
    
}

int Automation( void) {
  int Result = 0;
  unsigned char Ch;
  int ActivityMs = 100000; // 60000 is 60 sec to see in a debug window, each message blinks blue led and cost energy
  int iTmp;
  uint32_t CurrMicros;

  if ( DiffTime( millis(), TimeShowMs) > ActivityMs) { // display activity
    TimeShowMs = millis();
    Activity( 0);
  }

  // LoopLastUs = micros(); // time for perf check during dev

  // about command line
  iTmp = 20; // maybe we were waiting ( wifi, deep sleep ...), treat full command in one loop
  do {
    Ch = dbgReadCh();
    if ((0 != Ch) && (255 != Ch)) {
      DbgLastActivityMillis = millis();
      CmdLineParse( Ch);
      RunOtherTasks( 20); // or get bad chars
      iTmp--;
    } else {
      iTmp = 0;
    }
  } while (iTmp > 0);

  switch ( RoleGet()) {
    case ROLE_PLAYER:
      int Res;

      if (1 != PlayerHealth) {
        PlayerInit();
      }
      WifiScan();// blocking... not blocking
      RunOtherTasks( 100);

      break;
    case ROLE_TEST:
      if (1 != PlayerHealth) {
        PlayerInit();
      }
      WifiScan();// blocking... not blocking
      RunOtherTasks( 100);

      break;
    case ROLE_SOLO:
    case ROLE_MULTI:
      if (1 != PlayerHealth) {
        PlayerInit();
      }
      RunOtherTasks( 100);
      break;
    case ROLE_PASSENGER:
      if (abs(millis() - DbgLastActivityMillis) > 30000) {
        // TODO_LATER : sleep to spare power
        // ESP.deepSleep( 1*000*000, WAKE_RF_DEFAULT); // microseconds
      }
      RunOtherTasks( 1000);
      break;
  }


  CurrMicros = micros();
  LoopUs = (LoopUs + 5 * DiffTime( LoopLastUs, CurrMicros)) / 6;
  LoopLastUs = CurrMicros;

#ifndef ESP32
  AutoActivated();
#endif /* 0 debug */

  //RunOtherTasks(100);
  RunOtherTasks(0);

  #ifdef WITH_SWITCH
    SwitchParse();
  #endif /* WITH_SWITCH */

  #ifdef RESTART_PIN
    RestartSwitchParse();
  #endif /* RESTART_PIN */

  #ifdef RESTART_B_PIN
    RestartBSwitchParse();
  #endif /* RESTART_PIN */

  #ifdef WITH_ADC
    AdcProcess( 1);
  #endif /* WITH_ADC */

  #ifdef STEPPER_IN_PULSE_PIN
    #ifndef STEPPER_IN_PULSE_PIN_INTERRUPT
      StepperInPosLoop ();
    #endif /* STEPPER_IN_PULSE_PIN_INTERRUPT */
  #endif

  #ifdef WITH_IOTMUTUAL
    // TODO_LATER : sometimes lock below
    //dbgprintf( 2, "PosB\n");
    //  TimerStop();
    #ifndef ESP32 // TODO_HERE
      server.handleClient();
    #endif
    //  TimerStart();
    //dbgprintf( 2, "PosC\n");
  #endif /* WITH_IOTMUTUAL */

  #ifdef WITH_WIFICLIENT
    WifiCliLoop();
  #endif /* WITH_WIFICLIENT */

  #ifdef WITH_OSC
    OscLoop();
  #endif /* WITH_OSC */

  #ifdef WITH_SCREEN
    ScreenLoop();
  #endif /* WITH_SCREEN */

  //RunOtherTasks(100);
  RunOtherTasks(0);

  TimerStop();
  #ifdef WITH_MOTOR
    MotorLoop( 0);
  #endif /* WITH_MOTOR */
  #ifdef WITH_GRAYCODE_IN
    // dbgprintf( 2, "graycode %i %i\n", digitalRead(GRAYCODE_IN_A_PIN), digitalRead(GRAYCODE_IN_B_PIN));
    GraycodeInAInt();
    GraycodeInBInt();
  #endif /* WITH_GRAYCODE_IN */
  TimerStart();

  // LoopLastUs = micros(); // 6us out of loop
  return ( 0);
}

void setup() {
 LoopLastUs = micros();

  Serial.begin(115200); // for debug
  Serial.setTimeout( 0);// ms, default 1000
  dbgprintf( 0, "Hello");

  #ifdef ESP32
    EEPROM.begin(512);// ESP : Size can be anywhere between 4 and TODO_HERE
  #endif
#ifdef ESP8266
  EEPROM.begin(512);// ESP : Size can be anywhere between 4 and 4096 bytes
#endif

  memset( PinModes, 0xca, sizeof(PinModes));

  EpromSanity();
  EpromDump();

  if (ACTIVITY_LED > 0) {
    MyPinmode( ACTIVITY_LED, OUTPUT); // Activity
  }

  #ifdef WITH_STOPPER
    MyPinmode( StopperPins[0], INPUT_PULLUP);
    MyPinmode( StopperPins[1], INPUT_PULLUP);
  #endif /* WITH_STOPPER */

  #ifdef WITH_WS2812
    PixelInit();
  #endif /*WITH_WS2812 */

  Activity( ACT_ININIT);

  #ifdef WITH_WIFI
    WifiInit();
  #endif /* WITH_WIFI */

  #ifdef WITH_WIFICLIENT
    WifiCliConnectState = WifiState_TOCONNECT;
  #endif /* WITH_WIFICLIENT */

  #ifdef WITH_DFPLAYER
    PlayerInit();
  #endif /* WITH_DFPLAYER */

  #ifdef WITH_ADC
    // init
    AdcVal = map(analogRead(WITH_ADC), 0, 1024, 0, 9999);
    AdcProcess( 0);
  #endif /* WITH_ADC */

  #ifdef WITH_MOTOR
    MotorSetup();
  #endif /* WITH_MOTOR */

  #ifdef WITH_GRAYCODE_IN
    GraycodeInSetup();
  #endif /* WITH_GRAYCODE_IN */

  TimerSetup();

#ifdef WITH_SWITCH
  SwitchInit();
#endif /* WITH_SWITCH */

#ifdef RESTART_PIN
  RestartSwitchInit();
#endif /* RESTART_PIN */

#ifdef WITH_ADC
  pinMode( WITH_ADC, INPUT);
#endif /* WITH_ADC */

#ifdef RESTART_B_PIN
  RestartBSwitchInit();
#endif /* RESTART_PIN */

  Activity( ACT_READY);
  EpromDump();

#ifdef WITH_IOTMUTUAL
  IotMutualSetup();
#endif /* WITH_IOTMUTUAL */

#ifdef WITH_OSC
  OscInit();
#endif /* WITH_OSC */

#ifdef WITH_SCREEN
  ScreenSetup();
#endif /* WITH_SCREEN */

  DbgLastActivityMillis = millis();
  LoopUs = DiffTime( LoopLastUs, micros()); // init

  TimerStart();
}

// the loop function runs over and over again forever
void loop() {
  int Res;

  Res = Automation();
}

