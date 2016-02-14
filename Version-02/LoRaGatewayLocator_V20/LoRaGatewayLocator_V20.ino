/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello, world!", that
 * will be processed by The Things Network server.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in g1, 
*  0.1% in g2). 
 *
 * Change DEVADDR to a unique address! 
 * See http://thethingsnetwork.org/wiki/AddressSpace
 *
 * Do not forget to define the radio type correctly in config.h, default is:
 *   #define CFG_sx1272_radio 1
 * for SX1272 and RFM92, but change to:
 *   #define CFG_sx1276_radio 1
 * for SX1276 and RFM95.
 *
 * ----------------------------------------------------------------------------
 * ----------------------------------------------------------------------------
 * LoRa Gateway Locator by Urs Marti / 2016
 * this node gets it's position from a GPS
 * it runs unattended and sends its data in selectable intervals (1', 5', 10')
 * or attended on a press of a button
 * data will be displayed on an OLED display (128x64)
 * 
 * PLEASE CHANGE mydata & DEVADDR
 * adjust the size of the databuffer, depending on your Geolocation
 * each node needs a unique DEVADDR
 * 
 * OLD: this version runs with attached, but not INITIALIZED SDcard ---> see Teensy32-XBee-GPS-Board_spi_SDcard
 * NEW: Instead of waiting for the SPI "corrected" I'll use an OpenLog module
 *      much more expensive, but it seems to be the only feasible way
 *  --> It will only compiled if the define "isOpenLog" is enabled
 *  
 *  Tested with: Arduino 1.6.7 & Teensyduino under Win10 
 *  in your final compillation you should disable DEBUG1 so the code will be much smaller
 *  
 *  Hardware:  
 *  - Teensy 3.2
 *  - RFM95W  from Hoperf.nl  (compatible with SX1272)
 *  - u-blox PAM-Q  sponsored by u-blox
 *  - OLED (blue or white) with connectors: GND - VCC - SCL - SDA  !! solder or cut the jumpers
 *         I2C 128X64 OLED Display from Aliexpress
 *  - Antenna: 868Mhz antenna module aerial 2dbi Omni direction SMA male from Aliexpress
 *  
 *  KNOWN ISSUES to be SOLVED:
 *  - menu does not go back to a defined status.
 *    reinitialize when leaving a choosen RoD
 *  - During "get Fix" it's very busy, so actions on the switches did not respond immediately   
 *  
 *
 *******************************************************************************/

#include <lmic.h>                     // https://github.com/tftelkamp/arduino-lmic-v1.5
#include <hal/hal.h>                  //
#include <SPI.h>                      //
#include <Adafruit_GPS.h>             //
#include <SoftwareSerial.h>           //
#include <Adafruit_ssd1306syp.h>      //
#include <MenuSystem.h>               // http://blog.humblecoder.com/arduino-menu-system-library/
                                      // https://github.com/jonblack/arduino-menusystem

									  
// debugging with serial monitor , comment out in final version
 #define DEBUG1

// if a OpenLog module is installed uncomment next line
// #define isOpenLog 


// Adafruit GPS or u-blox
// use Hardware RX1/TX1 on Teensy3.2
Adafruit_GPS GPS(&Serial1);

// define OLED
Adafruit_ssd1306syp display(SDA,SCL);    // für Teensy3.2  ********

#ifdef isOpenLog
  #define OpenLog Serial3                  // using HWserial on port 3 (7/8)
#endif

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true


// Define RGB LED pins 
const int green  = 16;
const int red    = 15;
const int blue   = 17;

uint32_t timer = millis(); 
boolean HighPowerTx = false;     // set if SF12 choosen below
boolean TXcomplete  = false;     // Transaction Competed (Sent)
boolean initFix     = false;     // Initial Fix completed
boolean noFix       = true;      // no Fix yet
boolean ManButFire  = false;     // Manual Button Fire when no GPS-Fix available
boolean doRequest   = false;     // Button Request in Run on Request WITH GPS
boolean showOnce    = true;      // display menu selection once
boolean immedAction = true;      // at first selection, send immediate with no delay
char lat_buf[10];
char long_buf[10];
String DateTime;
String StatusStr;
int ManFireCntr = 0;
String outbuf_s  = "";

// how long do we wait between measurements ?
long delayTime0    =  30000;          // 30 seconds only for debugging
long delayTime1    =  60000;          // one minute
long delayTime2    = 300000;          // five minutes
long delayTime3    = 600000;          // ten minutes
long delayTime     =  60000;          // default
long saveDelayTime = 0;               // holding the desired delay time during immediate action

// definitions for the menu and the navigation with the buttons
int KeyUp  = A9;                 // pad 23
int KeySel = A8;                 // pad 22
int KeyDwn = A7;                 // pad 21
int buttonState_Up;              // the current reading from the input pin
int lastButtonState_Up  = HIGH;  // the previous reading from the input pin
int buttonState_Sel;             // the current reading from the input pin
int lastButtonState_Sel = HIGH;  // the previous reading from the input pin
int buttonState_Dwn;             // the current reading from the input pin
int lastButtonState_Dwn = HIGH;  // the previous reading from the input pin
long debounceDelay = 50;         // the debounce time; increase if the output flickers
long lastDebounceTime_Up  = 0;   // the last time the output pin was toggled
long lastDebounceTime_Sel = 0;   // the last time the output pin was toggled
long lastDebounceTime_Dwn = 0;   // the last time the output pin was toggled
boolean MenuLevel = true;        // is true if on menu, not on sub-menu level
int MenuSelection = 0;           // 

// Menu variables
MenuSystem ms;
Menu mm("");  
MenuItem mi_SDP("Show Date/Position ");  
Menu mu_RoR("Run on Request     ");
MenuItem mi_RoR_up(" up");
MenuItem mi_RoR_Fix("RoR with Fix");
MenuItem mi_RoR_Cnt("RoR with Cnt");
Menu mu_RoDel("Run on Delay");  
MenuItem mi_Delay_up(" up");  
MenuItem mi_Delay_01(" 1 minute ");  
MenuItem mi_Delay_05(" 5 minutes");
MenuItem mi_Delay_10("10 minutes");

// Geocoordinates were shown form zero-meridian (London)/Equator = N0° 0.0 E0° 0.0 somewhere south of Ghana
// to N+90°/-90° to E+180°/-180° so the maximum rnge of mydata must be adjusted to your location
// with 14bytes it's adjusted to Europe
// Switzerland N 45..47  E 005...10
// ----------------------------1---------2---------3---------4------
//                  1234567890123456789012345678901234567890
uint8_t mydata[] = "              ";   // length see above
static osjob_t sendjob;

// Pin mapping
lmic_pinmap pins = {
  .nss = 10,                                     // Nsel/nSS  (10)
                                                 // MOSI(11), MISO(12), SCK(13) as standard
  .rxtx = 20,   // Not connected on RFM92/RFM95  // (7)  --> in case of using OpenLog canged to 20
  .rst  = 9,    // Needed on RFM92/RFM95         // (9) Reset/POR    CHANGE for XBee-Module to "8"
  .dio  = {2, 5, 6},                             // input, interrupt (2,5,6)
                                                 // 2=DIO0 , 5=DIO3 , 6=DIO4
                                                 // 3=DIO1 , 4=DIO2 = not used
};

// OpenLog definitions
int resetOpenLog           = 4;                  //This pin resets OpenLog. Connect pin 4 to pin GRN on OpenLog.
String OpenLogFileName     = "";                 // 8.3  will be crated when a GPSfix is ok
boolean OpenLogFileCreated = false;


// LoRaWAN Application identifier (AppEUI)
// Not used in this example
static const u1_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// LoRaWAN DevEUI, unique device ID (LSBF)
// Not used in this example
static const u1_t DEVEUI[8]  = { 0x42, 0x42, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// LoRaWAN NwkSKey, network session key 
// Use this key for The Things Network
static const u1_t DEVKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN AppSKey, application session key
// Use this key to get your data decrypted by The Things Network
static const u1_t ARTKEY[16] = { 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C };

// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
static const u4_t DEVADDR = 0x5A4801AA ;                        // <-- Change this address for every node!


//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    memcpy(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    memcpy(buf, DEVEUI, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    memcpy(buf, DEVKEY, 16);
}

//---------------------------------------------------------------------
// Menu callback function
void on_menu_set_SDP(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> Show actual Date/Pos");
  #endif
  MenuSelection = 1;
  MenuLevel = true;
  showDisplay_SDP();
}
void on_menu_set_RoR(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> Sel Fix or Cnt");
  #endif
  MenuLevel = true;
}
void on_menu_set_RoR_up(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> leave RoR submenu");
  #endif
  MenuSelection = 0;
  MenuLevel = true;
  displayMenu();
}
void on_menu_set_RoR_Fix(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> Run on Request with Fix");
  #endif
  MenuSelection = 2;
  MenuLevel = false;
  showDisplay_RoR_Fix();
}
void on_menu_set_RoR_Cnt(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> Run on Request with Counter");
  #endif
  MenuSelection = 2;
  ManButFire = true;
  MenuLevel  = false;
  showDisplay_RoR_MFB();
}
void on_menu_set_RoDel(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> Select Delay");
  #endif
  MenuLevel = true;
}
void on_menu_set_DelayUp(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println("--> leave RoD submenu");
  #endif
  MenuSelection = 0;
  MenuLevel = true;
  displayMenu();
}
void on_menu_set_Delay01(MenuItem* pMenuItem)  {
  MenuSelection = 3;
  if ( !HighPowerTx ) {
     delayTime = delayTime1;
     #ifdef DEBUG1
       Serial.print(" Delay: ");
	   Serial.println(delayTime);
     #endif
  } else {
     delayTime = delayTime2;
     #ifdef DEBUG1
       Serial.println(" High Power Tx  set, so Delay: 5 minutes");
     #endif
  }  
  MenuLevel = false;
  showDisplay_RonDel();
}
void on_menu_set_Delay05(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println(" Delay: 5 minutes");
  #endif
  MenuSelection = 3;
  delayTime = delayTime2;
  MenuLevel = false;
  showDisplay_RonDel();
}
void on_menu_set_Delay10(MenuItem* pMenuItem)  {
  #ifdef DEBUG1
    Serial.println(" Delay: 10 minutes");
  #endif
  MenuSelection = 3;
  delayTime = delayTime3;
  MenuLevel = false;
  showDisplay_RonDel();
}

// ===================================================================================================
void setup() {

// initialize button pins (internal pullup, switch takes it LOW)
  pinMode(KeyUp,  INPUT_PULLUP);
  pinMode(KeySel, INPUT_PULLUP);
  pinMode(KeyDwn, INPUT_PULLUP);
  
// setup the LED pins as output
  pinMode(red,   OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(blue,  OUTPUT);
  // blink red/blue to show that initialization begins
  blink(red);     
  blink(blue);  
  
// initialize OLED  128 wide , 64 high
  display.initialize();
  display.clear();
  display.drawLine(0,0, 127,63, WHITE);     // x,x top left y,y bottom right
  display.drawLine(127,0, 0,63, WHITE);     // x,x top left y,y bottom right
  display.update();
  delay(1000);
  display.clear();
  display.setTextSize(2);                   // Textsize 2: 10 chars per line
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("INITIALIZE");
   display.setTextSize(1);                  // Textsize 1:
   display.setCursor(0,30);
   display.print("Dev-Addr: "); 
   display.print(DEVADDR, HEX);
  display.update();

  #ifdef DEBUG1
    Serial.begin(57600);                         // fast read for GPS echo
    delay(5000);
    Serial.println("Starting");
  #endif
  
// GPS initialize
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's - some use 4800
   GPS.begin(9600);
   delay(100);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate


  // Menu setup
  mm.add_item(&mi_SDP, &on_menu_set_SDP);             // show actual date / Position
  mm.add_menu(&mu_RoR);                               // run on request (press Select)
  mu_RoR.add_item(&mi_RoR_up,  &on_menu_set_RoR_up);      //    up one level
  mu_RoR.add_item(&mi_RoR_Fix, &on_menu_set_RoR_Fix);     //    with GPS Fix
  mu_RoR.add_item(&mi_RoR_Cnt, &on_menu_set_RoR_Cnt);     //    with Counter
  mm.add_menu(&mu_RoDel);
  mu_RoDel.add_item(&mi_Delay_up, &on_menu_set_DelayUp);  //    up one level
  mu_RoDel.add_item(&mi_Delay_01, &on_menu_set_Delay01);
  mu_RoDel.add_item(&mi_Delay_05, &on_menu_set_Delay05);
  mu_RoDel.add_item(&mi_Delay_10, &on_menu_set_Delay10);
  ms.set_root_menu(&mm);


// LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // Set static session parameters. Instead of dynamically establishing a session 
  // by joining the network, precomputed session parameters are be provided.
  LMIC_setSession (0x1, DEVADDR, (uint8_t*)DEVKEY, (uint8_t*)ARTKEY);
  // Disable data rate adaptation
  LMIC_setAdrMode(0);
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  // Disable beacon tracking
  LMIC_disableTracking ();
  // Stop listening for downstream data (periodical reception)
  LMIC_stopPingable();
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF7,14);
  // LMIC_setDrTxpow(DR_SF12,14);
  // HighPowerTx = true;             // disable 1 minute RoD !!
                                     // make true if DR_SF12 is choosen.
  
  // restrict to channel 0 * aus einer mail von Thomas  (keep secret!)
    #ifdef DEBUG1
      LMIC_disableChannel(1);
      LMIC_disableChannel(2);
      LMIC_disableChannel(3);
      LMIC_disableChannel(4);
      LMIC_disableChannel(5);
      LMIC_disableChannel(6);
      LMIC_disableChannel(7);
      LMIC_disableChannel(8);
      //display.clear();
      display.setTextSize(1);
      display.setCursor(0,20);
      display.print("debug mode | CH-0");
      display.setCursor(0,30);
      if ( !HighPowerTx ) 
         display.print("normal Pwr Mode");
      else  
         display.print("HIGH Pwr Mode");
      display.update();
      Serial.println("---> ONLY channel 0 is active");
    #endif
    #ifdef DEBUG1
      delayTime = delayTime0;                   // 
      Serial.print("Device-Addr: ");
      Serial.print(DEVADDR, HEX);
      Serial.println();
    #endif 

  #ifdef isOpenLog
     setupOpenLog();        //Resets logger and waits for the '<' I'm alive character
     display.setCursor(0,48);
     display.print("OpenLog ready");
     display.update();
     #ifdef DEBUG1
       Serial.println("OpenLog ready ...");
       gotoCommandMode();               //Puts OpenLog in command mode
       readDisk();
     #endif
   #endif
     
// do not uncomment - is for normal SDcard usage   (not implemented) 
//   http://forum.arduino.cc/index.php?topic=27986.msg207074#msg207074
// digitalWrite(SS,HIGH);      //disable device
// digitalWrite(10, HIGH);     //physical SS pin high before setting SPCR  

  //
  Serial.flush();
  // blink green/blue to show that initialization finished
  blink(green);     
  blink(blue);  

  timer = millis();
  
}
// =========================== setup finished ========================================================


// here following some subroutines -------------------------------------------------------------------
// handle LMIC ---------------------------------------------------------------------------------------
void onEvent (ev_t ev) {
    //debug_event(ev);

    switch(ev) {
      // scheduled data sent (optionally data received)
      // note: this includes the receive window!
      case EV_TXCOMPLETE:
          // use this event to keep track of actual transmissions
          #ifdef DEBUG1
            Serial.print("Event EV_TXCOMPLETE, time: ");
            Serial.println(millis() / 1000);
          #endif
          // StatusStr = "TX completed";
          StatusStr = "           done";
          showStatus();         
          TXcomplete = true;
          blink(green);
          if(LMIC.dataLen) { // data received in rx slot after tx
              //debug_buf(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
              #ifdef DEBUG1
                Serial.println("Data Received!");
              #endif
              blink(green); blink(blue); blink(green);
          }
          break;
      default:
          break;
    }
}

void do_send(osjob_t* j){
      #ifdef DEBUG1
        Serial.print("Time: ");
        Serial.println(millis() / 1000);
        // Show TX channel (channel numbers are local to LMIC)
        Serial.print("Send, txCnhl: ");
        Serial.println(LMIC.txChnl);
        Serial.print("Opmode check: ");
      #endif

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & (1 << 7)) {
      #ifdef DEBUG1
        Serial.println("OP_TXRXPEND, not sending");
      #endif
      blink(red);
        StatusStr = "TX NOT sending";
        showStatus();         
      } else {
      #ifdef DEBUG1
        Serial.println("ok");
      #endif
        StatusStr = "TX sending";
        showStatus();         
      // Prepare upstream data transmission at the next possible time.
      TXcomplete = false;
      LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
    }
    // Schedule a timed job to run at the given timestamp (absolute system time)
    // os_setTimedCallback(j, os_getTime()+sec2osticks(120), do_send);
         
}

// blink with the debug RGB led --------------------------------------------------------
void blink(int pin) {
  digitalWrite(pin, HIGH);
  delay(200);
  digitalWrite(pin, LOW);
}

// show status on OLED row 8 -------------------------------------------------------------------------
void showStatus() {
  display.setTextSize(1);
  display.setCursor(0,56);
  display.print("                    ");
  display.update();
  display.setCursor(0,56);
  display.print(StatusStr);
  display.update();
}

// handle menu --------------------------------------------------------------------------------------
void displayMenu() {
  String lineX = "";  

  display.clear();
  display.setCursor(0,0);
  display.print("Choose Menue");
  Menu const* cp_menu = ms.get_current_menu();
  MenuComponent const* cp_menu_sel = cp_menu->get_selected();
  for (int i = 0; i < cp_menu->get_num_menu_components(); ++i)
  {
    MenuComponent const* cp_m_comp = cp_menu->get_menu_component(i);
    lineX = cp_m_comp->get_name();
    if (cp_menu_sel == cp_m_comp){
      if ( lineX == " up")  MenuLevel = false;   // was " 1 minute "
      else MenuLevel = true; 
      lineX += " <";
      display.setCursor(0,i*8+8);
      display.print(lineX);
    } else { 
      display.setCursor(0,i*8+8);
      display.print(lineX);
    }
    // Serial.println();    //  (MenuLevel)
  }
  display.setCursor(0,56);
  display.print(" <-UP | SEL | DOWN->  ");
  display.update();

}

// handle additional keypress for run on request ----------------------------------------------------
void getKeypress() {
  doRequest   = false;
  int reading_Up = digitalRead(KeyUp);
  if (reading_Up != lastButtonState_Up) {
    lastDebounceTime_Up = millis();
  }
  if ((millis() - lastDebounceTime_Up) > debounceDelay) {
    if (reading_Up != buttonState_Up) {
      buttonState_Up = reading_Up;
      if (buttonState_Up == LOW) {
       MenuSelection = 0;
       ManButFire = false;
      }
    }
  }
        lastButtonState_Up = reading_Up;
  
  int reading_Sel = digitalRead(KeySel);
  if (reading_Sel != lastButtonState_Sel) {
    lastDebounceTime_Sel = millis();
  }
  if ((millis() - lastDebounceTime_Sel) > debounceDelay) {
    if (reading_Sel != buttonState_Sel) {
      buttonState_Sel = reading_Sel;
      if (buttonState_Sel == LOW) {
       doRequest = true;
      }
    }
  }
         lastButtonState_Sel = reading_Sel;
  
  int reading_Dwn = digitalRead(KeyDwn);
  if (reading_Dwn != lastButtonState_Dwn) {
    lastDebounceTime_Dwn = millis();
  }
  if ((millis() - lastDebounceTime_Dwn) > debounceDelay) {
    if (reading_Dwn != buttonState_Dwn) {
      buttonState_Dwn = reading_Dwn;
      if (buttonState_Dwn == LOW) {
       MenuSelection = 0;
       ManButFire = false;
      }
    }
  }
        lastButtonState_Dwn = reading_Dwn;
}

// handle buttons for menu selection ----------------------------------------------------------------
void Button()  {
  int reading_Up = digitalRead(KeyUp);
  if (reading_Up != lastButtonState_Up) {
    lastDebounceTime_Up = millis();
  }
  if ((millis() - lastDebounceTime_Up) > debounceDelay) {
    if (reading_Up != buttonState_Up) {
      buttonState_Up = reading_Up;
      if (buttonState_Up == LOW) {
      // MENU_ACTION
      if (MenuLevel == false)  ms.back();   // if in sub-menu you've to go up to the menu
        else ms.prev();                     // on menu level go easy one up
        blink(blue);                        // indicate button pressed
        #ifdef DEBUG1 
          Serial.println("------------> button_UP pressed");  
        #endif
        displayMenu();
      }
    }
  }
        lastButtonState_Up = reading_Up;
  
  int reading_Sel = digitalRead(KeySel);
  if (reading_Sel != lastButtonState_Sel) {
    lastDebounceTime_Sel = millis();
  }
  if ((millis() - lastDebounceTime_Sel) > debounceDelay) {
    if (reading_Sel != buttonState_Sel) {
      buttonState_Sel = reading_Sel;
      if (buttonState_Sel == LOW) {
      // MENU_ACTION
         ms.select();                // returns from sub-menu, but "stays" in selected part (not nice)
         blink(blue);                // indicate button pressed
         showOnce = true;            // shows menu selection once if on a top one
         immedAction = true;         // next send should be immediately		 
         #ifdef DEBUG1 
           Serial.println("------------> button_SEL pressed");  
         #endif
         // displayMenu();
      }
    }
  }
         lastButtonState_Sel = reading_Sel;
  
  int reading_Dwn = digitalRead(KeyDwn);
  if (reading_Dwn != lastButtonState_Dwn) {
    lastDebounceTime_Dwn = millis();
  }
  if ((millis() - lastDebounceTime_Dwn) > debounceDelay) {
    if (reading_Dwn != buttonState_Dwn) {
      buttonState_Dwn = reading_Dwn;
      if (buttonState_Dwn == LOW) {
      // MENU_ACTION
         ms.next();
         blink(blue);                     // indicate button pressed
         #ifdef DEBUG1 
           Serial.println("------------> button_DWN pressed");  
         #endif
         displayMenu();
      }
    }
  }
        lastButtonState_Dwn = reading_Dwn;
}

//---------------------------------------------------------------------------------------------------
// on different actions show the corresponding basic display ----------------------------------------
void showDisplay_SDP() {                  // show Date/Time & Position, updated each xx seconds
  display.clear();
  display.setTextSize(2);                 // Textsize 2: 10 chars per line
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print(" Date/Pos");
  display.setTextSize(1);                 // Textsize 1:
  // other lines to be handeled by the corresponding function
  display.setCursor(0,48);
  display.print("  any key breaks");
  display.update();
}

void showDisplay_RoR_Fix() {              // Run on Request with GPS Fix
  display.clear();
  display.setTextSize(2);                 // Textsize 2: 10 chars per line
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Run On Req");
  display.setTextSize(1);                 // Textsize 1: 20 chars per line (default)
  // other lines to be handeled by the corresponding function
  display.setCursor(0,48);
  display.print("SEL:Send/other breaks");
  display.update();
}

void showDisplay_RoR_MFB()  {             // show Manual Fire Button Title & Counter
  display.clear();
  display.setTextSize(2);                 // in big letters
  display.setCursor(0,0);
  display.print("ManualFire");
  display.setTextSize(1);                 // in small letters
  display.setCursor(0,22);
  display.print("Counter: ");
  display.print(ManFireCntr, 6);
  if ( HighPowerTx ) {
    display.setCursor(0,32);
    display.print("HighPwrTx - delay 3'");
  }
  display.setCursor(0,48);
  display.print("SEL:Send/other breaks");
  display.update();
}

void showDisplay_RonDel() {               // 
  display.clear();
  display.setTextSize(2);                 // Textsize 1: 10 chars per line
  // display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.print("Delay ");
  if (delayTime == delayTime1) display.print("01'");
  if (delayTime == delayTime2) display.print("05'");
  if (delayTime == delayTime3) display.print("10'");
  display.setTextSize(1);                 // Textsize 1:
  // other lines to be handeled by the corresponding function
  display.setCursor(0,48);
  display.print("  any key breaks");
  display.update();
}

//some routines for the OpenLog ---------------------------------------------------------------------
#ifdef isOpenLog
void setupOpenLog(void) {
  pinMode(resetOpenLog, OUTPUT);
  OpenLog.begin(9600);                   // using HWserial Serial3
    
  //Reset OpenLog
  digitalWrite(resetOpenLog, LOW);
  delay(100);
  digitalWrite(resetOpenLog, HIGH);

  #ifdef DEBUG1
     Serial.println("resetting OpenLog. waiting for prompt");
  #endif   
  //Wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
  while(1) {                              // this is crutial and could result in hanging
    if(OpenLog.available())
      if(OpenLog.read() == '<') break;
  }
  #ifdef DEBUG1
     Serial.println("OpenLog alive ...");
  #endif       
}

//This function pushes OpenLog into command mode
void gotoCommandMode(void) {
  //Send three control z to enter OpenLog command mode
  //Works with Arduino v1.0
  OpenLog.write(26);                    // #1A  Ctrl-Z  (SUB)
  OpenLog.write(26);
  OpenLog.write(26);

  //Wait for OpenLog to respond with '>' to indicate we are in command mode
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '>') break;
  }
  #ifdef DEBUG1
     Serial.println("OpenLog, leave command prompt ...");
  #endif       
}

//This function creates a given file and then opens it in append mode (ready to record characters to the file)
//Then returns to listening mode
// void createFile(char *fileName) {   // OpenLogFileName from GPSfix
void createFile(String fileName) {     // OpenLogFileName from GPSfix
  
  //Old way                            // using this for compatibility
  OpenLog.print("new ");
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r

  //New way
  //OpenLog.print("new ");
  //OpenLog.println(filename);   //regular println works with OpenLog v2.51 and above

  //Wait for OpenLog to return to waiting for a command
  #ifdef DEBUG1
     Serial.print("Filename: ");
     Serial.print(fileName);
     Serial.println();
     Serial.println("OpenLogFile created. waiting for prompt ...");
  #endif   
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '>') break;
  }
  #ifdef DEBUG1
     Serial.println("OpenLog prompted ");
  #endif   

  OpenLog.print("append ");
  OpenLog.print(fileName);
  OpenLog.write(13); //This is \r
  
  // #ifdef DEBUG1
     // Serial.println("OpenLogFile append. waiting for prompt ...");
  // #endif   
  //Wait for OpenLog to indicate file is open and ready for writing
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '<') break;
  }
  #ifdef DEBUG1
     Serial.println("OpenLogFile append & waiting for input");
  #endif   
  OpenLogFileCreated == true;
  OpenLog.println("new file created ...");
  
  //OpenLog is now waiting for characters and will record them to the new file  
}

//Check the stats of the SD card via 'disk' command
//This function assumes the OpenLog is in command mode
void readDisk() {

  //Old way
  OpenLog.print("disk");
  OpenLog.write(13); //This is \r

  //New way
  //OpenLog.print("read ");
  //OpenLog.println(filename); //regular println works with OpenLog v2.51 and above

  //The OpenLog echos the commands we send it by default so we have 'disk\r' sitting 
  //in the RX buffer. Let's try to not print this.
  while(1) {
    if(OpenLog.available())
      if(OpenLog.read() == '\r') break;
  }  

  //This will listen for characters coming from OpenLog and print them to the terminal
  //This relies heavily on the SoftSerial buffer not overrunning. This will probably not work
  //above 38400bps.
  //This loop will stop listening after 1 second of no characters received
  for(int timeOut = 0 ; timeOut < 1000 ; timeOut++) {
    while(OpenLog.available()) {
      char tempString[100];
      
      int spot = 0;
      while(OpenLog.available()) {
        tempString[spot++] = OpenLog.read();
        if(spot > 98) break;
      }
      tempString[spot] = '\0';
      Serial.write(tempString); //Take the string from OpenLog and push it to the Arduino terminal
      timeOut = 0;
    }

    delay(1);
  }
}
#endif       // isOpenLog installed

//---------------------------------------------------------------------------------------------------
void showPosition() {
       if (MenuSelection == 1) showDisplay_SDP();
       if (MenuSelection == 2) showDisplay_RoR_Fix();
       if (MenuSelection == 3) showDisplay_RonDel();
         display.setCursor(0,20);
         display.print(DateTime);
         display.setCursor(0,30);
         display.print("Lan (N):"); display.println(lat_buf);
         display.setCursor(0,40);
         display.print("Long(E):"); display.print(long_buf);
         display.update();
}

// ---------------------------------------------------------------------------------------------------
String zweistellig(String input) {  // to enhance date/time string from single to double character
  String result = "";
  if (input.length() == 1)  {
    result += "0";
    result += input;
  }
  else result = input;

  return result;
}

// loop until the GPS has a fix ---------------------------------------------------------------------
void readUntilFix() {
  boolean leave_rUF = false;
  while ( !leave_rUF ) {

     char c = GPS.read();
      // if you want to debug, this is a good time to do it!
      if ((c) && (GPSECHO))
        Serial.write(c); 
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
  // OK we have good data, but also a fix?
      if (GPS.fix) {
        #ifdef DEBUG1
           Serial.println("FIXed");
        #endif
        display.setCursor(0,40);
        display.print("      FIXed      ");
        display.update();  
        leave_rUF   = true;
        initFix     = true;
        #ifdef isOpenLog                   // if an OpenLog module is installed
          #ifdef DEBUG1
            Serial.println("makeing filename for OpenLog");
          #endif
          getPosition();                   // get date time for a new filename
          gotoCommandMode();               //Puts OpenLog in command mode
          createFile(OpenLogFileName);     //Creates a new file called mmddHHMM
        #endif
      }
  }
}  

// ---------------------------------------------------------------------------------------------------
void getPosition()  {
  boolean leave = false;
  while ( !leave ) {

     char c = GPS.read();
     // if you want to debug, this is a good time to do it!
      if ((c) && (GPSECHO))
        Serial.write(c); 
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))      // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
    timer = millis(); // reset the timer
    
  // OK we have good data, but also a fix?
      if (GPS.fix) {                                 //   Serial.println("!");
         leave = true; noFix = false; String TempStr = "";
         TempStr = String(GPS.year);
         String year_s   = zweistellig(TempStr);     // Serial.print(year_s);
         TempStr = String(GPS.month);
         String month_s  = zweistellig(TempStr);     // Serial.print(month_s);
         TempStr = String(GPS.day);
         String day_s    = zweistellig(TempStr);     // Serial.print(day_s);      
         TempStr = String(GPS.hour);
         String hour_s   = zweistellig(TempStr);     // Serial.print(hour_s);  // UTC! 
         TempStr = String(GPS.minute);
         String minute_s = zweistellig(TempStr);     // Serial.print(minute_s); 
                                                     // Serial.println();

         DateTime = "";
         DateTime = "20" + year_s + "/" + month_s + "/" + day_s + " " + hour_s + ":" + minute_s;
         #ifdef isOpenLog
            if ( !OpenLogFileCreated ) {                     // if OpenLogfile not created
              OpenLogFileName = "";
              OpenLogFileName = month_s + day_s + hour_s + minute_s + ".log";   // for OpenLogFilename
            }
         #endif   
         // dtostrf(FLOAT,WIDTH,PRECSISION,BUFFER);   ....4725.778
         dtostrf(GPS.latitude,9,3,lat_buf);
         dtostrf(GPS.longitude,9,3,long_buf);
         #ifdef DEBUG1
           Serial.print("Date/Time    :");
           Serial.println(DateTime);
           Serial.print("Latitude  (N):");
           Serial.println(lat_buf);
            // int qla = sizeof(lat_buf);         // uncomment if you would see HEX values
            // for (int x=0; x!=qla; x++) {
            //   Serial.print(lat_buf[x], HEX);
            // }
            // Serial.println();  
           Serial.print("Longitude (E):");
           Serial.println(long_buf);
            // int qlo = sizeof(long_buf);        // uncomment if you would see HEX values
            // for (int x=0; x!=qlo; x++) {
            //   Serial.print(long_buf[x], HEX);
            // }
            // Serial.println();  
         #endif
         #ifdef isOpenLog                  // write timestamp and position to OpenLog
           OpenLog.print("Date/Time: ");
           OpenLog.print(DateTime);
           OpenLog.print("  Latitude(N): ");
           OpenLog.print(lat_buf);
           OpenLog.print("  Longitude (E): ");
           OpenLog.println(long_buf);
         #endif  

         int q = 0;
         // move latitude into buffer  4725.771 -> 4725771
         for (int i=0; i<9; i++) {
           if ((lat_buf[i] != 0x20) && (lat_buf[i] != 0x2E)) {
             mydata[q] = lat_buf[i];
             q++;
           }
         }   
         // move latitude into buffer   832.123 -> 832123
         for (int i=0; i<9; i++) {
           if ((long_buf[i] != 0x20) && (long_buf[i] != 0x2E)) {
             mydata[q] = long_buf[i];
             q++;
           }
         } 
         #ifdef DEBUG1
           // int q = sizeof(mydata);           // uncomment if you would see HEX values
           // for (int x=0; x!=q; x++) {
           //   Serial.print(mydata[x], HEX);
           // }
           // Serial.println();  
           Serial.print("Length: ");
           Serial.print(sizeof(mydata));
           Serial.print("  >");
           for (int q=0; q<sizeof(mydata); q++)  {
             Serial.print(char(mydata[q]));              //
           }
           Serial.print("<");
           Serial.println();
         #endif 
       }  // if fix
   }  // while
}     // getPosition
  

// ===================================================================================================
// ===================================================================================================
void loop() {

   char c = GPS.read();                  // the GPS must be read continiously
    if (GPS.newNMEAreceived()) {         // otherwise the data will not be updated properly
       if (!GPS.parse(GPS.lastNMEA()))   // recognized with date/time
        return;  
    }

  if (( !initFix ) && ( !ManButFire )) {
     // #ifdef DEBUG1
     //   Serial.println("warte auf GPS-Fix oder Button");
     // #endif
     display.setCursor(0,40);
     display.print("warte auf GPS-Fix");
     display.update();  
     readUntilFix();
	 getPosition();
	 showPosition();
  }
       if ( showOnce ) {
         Serial.print("Menu Selection: ");
         Serial.print(MenuSelection);
         Serial.println();
         showOnce = false;
       }  

  if (MenuSelection != 2)    // on request do getKeypress
     Button();               // otherwise any button pressed?
  
  switch (MenuSelection) {
    case 0:
    // nothing selected or actual action breaked
       displayMenu();
       break;
	   
    case 1:
    // show actual position
       if ( immedAction ) 
		   delayTime = 100;                    // if the function is called the first time, do it immediately
	   else
           delayTime = delayTime1;     	     // set delayTime to level1 = 1 Minute	   
       if (timer > millis())  timer = millis();
       if (millis() - timer > delayTime)  {                 //  && (TXcomplete == true)   not needed
          timer = millis(); // reset the timer
         #ifdef DEBUG1
            Serial.println();
			if ( immedAction ) {
			  Serial.print("Immediate run");
			  Serial.println();
			}
			else {
              Serial.print("case 1, show actual position each ");
              Serial.print(delayTime);
              Serial.print(" msec");
              Serial.println();
			}
          #endif
          immedAction = false;
          getPosition();
          showPosition();
       }
       break;
	   
    case 2:
    // run on request
       getKeypress();
       if (doRequest == true) {
          if ( ManButFire ) {
            // uint8_t mydata[] = "Counter:     ";
            ManFireCntr++;
            char  rangeTestNumber_b[4];
            dtostrf(ManFireCntr, 3, 0, rangeTestNumber_b);
            outbuf_s = rangeTestNumber_b;
            // copy rangeTestNumber into sendbuffer
            for (int r=0; r!=outbuf_s.length(); r++)  {
              mydata[r] = outbuf_s[r];
            }
            showDisplay_RoR_MFB();
            #ifdef DEBUG1
              Serial.print("Manual;  Counter: ");
              Serial.print(ManFireCntr, 4);   
              Serial.println();
            #endif  
          }
          else {
           getPosition();
            #ifdef DEBUG1
              Serial.print("RoR with Fix");
              Serial.println();
            #endif  
          } 
          if ( !ManButFire )  showPosition();
           do_send(&sendjob);  
           // os_runloop_once();
       }  
       break;

    case 3:
    // run with selected delay
       if ( immedAction ) {
		   saveDelayTime  = delayTime;
		   delayTime      = 100;                    // if the function is called the first time, do it immediately
		   TXcomplete     = true;                   // needed to enter the loop the first time
	   }
       if (timer > millis())  timer = millis();
       if ((millis() - timer > delayTime) && (TXcomplete == true)){
          timer = millis(); // reset the timer
          immedAction = false;
		  delayTime = saveDelayTime;
          #ifdef DEBUG1
            Serial.println();
            Serial.print("case 3, run on selected delay: ");
            Serial.print(delayTime);
            Serial.print(" msec");
            Serial.println();
			if ( immedAction ) Serial.println("immedAction true");
          #endif
          getPosition();
          showPosition();
          if (noFix == false) do_send(&sendjob);
        }
       break;
	   
    default:
    // do nothing
       break;
  }

     os_runloop_once();  
}

