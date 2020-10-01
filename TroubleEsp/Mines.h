
// add below the wifi ssid and password to try to connect to
#define ESP8266_MULTIPERSO \
  wifiMulti.addAP( "your_ap1", "ap1_pwd");\
  wifiMulti.addAP( "your_ap2", "ap2_pwd");\

// generated wifi ap password
#define BEN_PWD "your_pwd"


#define MODULE_TASS
#if defined( MODULE_TASS)
  #define WITH_MODULE "Tass"

  #define WITH_WIFI
   #undef WITH_WIFISRV    // do not generate wifi access point
  #define WITH_WIFICLIENT // connect other network
  #define WITH_IOTMUTUAL  // web server
  #define WITH_OSC
  #undef ESP8266_MULTIPERSO // kind of trouble on a couple of esp... on example WifiMulti too

  // app in core sources 
    #define WITH_STEPPER {  D2, D3, D7, D6} // DIR1 PULS1
    #define WITH_INVERT1 // motor 0 clockwise, motor 1 anticlock
    // D4 not happy #define WITH_STOPPER {  D4, D1, D5, D8} // min max
    #define WITH_STOPPER {  D1, -1, D5, -1} // min0 max0 min1 max1
    // #define WITH_ADC      A0

    // 049N"SpePwd"d"SpeAp"D
#endif /* MODULE_TASS */
