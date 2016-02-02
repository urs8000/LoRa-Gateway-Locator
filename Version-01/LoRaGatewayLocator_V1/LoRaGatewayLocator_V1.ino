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
 * LoRa Gateway Locator by Urs Marti / 2016
 * this node gets it's position from a GPS
 * it runs unattended and sends its data in selectable intervals (1', 5', 10')
 * or attended on a press of a button
 * data will be displayed on an OLED display (128x64)
 *
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
 *  
 *  Tested with: Arduino 1.6.7 & Teensyduino under Win10 
 *  in your final compillation you should disable DEBUG1 so the code will be a little bit smaller
 *  used compiling parameters under Tools:
 *    - Board: Teensy 3.2/3.1
 *    - USB Type: Serial
 *    - Keyboard: German Swiss
 *    - CPU Speed: 48MHz optimized
 *    - Port: depending on your installation
 *    with DEBUG1
 *      Sketch uses 72,080 bytes (27%) of program storage space. Maximum is 262,144 bytes.
 *      Global variables use 6,284 bytes (9%) of dynamic memory, leaving 59,252 bytes for local variables. Maximum is 65,536 bytes.
 *    without DEBUG1
 *      Sketch uses 70,712 bytes (26%) of program storage space. Maximum is 262,144 bytes.
 *      Global variables use 6,268 bytes (9%) of dynamic memory, leaving 59,268 bytes for local variables. Maximum is 65,536 bytes.
 *
 *  
 *  Hardware:  
 *  - Teensy 3.2
 *  - RFM95W  from Hoperf.nl  (compatible with SX1272)
 *  - u-blox PAM-Q  sponsored by u-blox
 *  - OLED (blue or white) with connectors: GND - Vcc - SCL - SDA  !! solder or cut the jumpers
 *    I2C 128X64 OLED Display from Aliexpress
 *  - Antenna: 868Mhz antenna module aerial 2dbi Omni direction SMA male from Aliexpress
 *  
 *  KNOWN ISSUES to be SOLVED:
 *  - running on delay it waits the selected delay. nice: run immediately
 *  - menu does not go back to a defined status.
 *    reinitialize when leaving a choosen RoD
 *  -  when running directly on delay; that does not work.
 *     it always need a Run on Request
 *  - During "get Fix" it's very busy, so actions on the switches did not respond immediately   
 *  
 *
 *******************************************************************************/

 
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_ssd1306syp.h>
#include <MenuSystem.h>


// Adafruit GPS or u-blox
// use RX1/TX1 on Teensy3.2
Adafruit_GPS GPS(&Serial1);

// define OLED
Adafruit_ssd1306syp display(SDA,SCL);          // für Teensy3.2  ********

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  false

// debugging with serial monitor , comment out in final version
// #define DEBUG1

// Define RGB LED pins 
const int green  = 16;
const int red    = 15;
const int blue   = 17;

uint32_t timer = millis(); 
boolean HighPowerTx = false;     // set if SF12 choosen below
boolean TXcomplete  = true;      // Transaction Competed (Sent)
boolean initFix     = false;     // Initial Fix completed
boolean noFix       = true;      // no Fix yet
boolean ManButFire  = false;     // Manual Button Fire when no GPS-Fix available
boolean doRequest   = false;     // Button Request in Run on Request WITH GPS
boolean showOnce    = true;      // display menu selection once
char lat_buf[10];
char long_buf[10];
String DateTime;
String StatusStr;
int ManFireCntr;
String outbuf_s  = "";

// how long do we wait between measurements ?
long delayTime0 =  30000;                // 30 seconds only for debugging
long delayTime1 =  60000;                // one minute
long delayTime2 = 300000;                // five minutes
long delayTime3 = 600000;                // ten minutes
long delayTime  =  60000;                // default

// definitions for the menu and the navigation with the buttons
int KeyUp  = A9;                 // pad 23
int KeySel = A8;                 // pad 22
int KeyDwn = A7;                 // pad 21
int buttonState_Up;              // the current reading from the input pin
int lastButtonState_Up  = LOW;   // the previous reading from the input pin
int buttonState_Sel;             // the current reading from the input pin
int lastButtonState_Sel = LOW;   // the previous reading from the input pin
int buttonState_Dwn;             // the current reading from the input pin
int lastButtonState_Dwn = LOW;   // the previous reading from the input pin
long debounceDelay = 30;         // the debounce time; increase if the output flickers <---
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


// ----------------------------1---------2---------3---------4------
//                    1234567890123456789012345678901234567890
uint8_t mydata[14] = "";
static osjob_t sendjob;

// Pin mapping
lmic_pinmap pins = {
  .nss = 10,                                     // Nsel/nSS  (10)
                                                 // MOSI(11), MISO(12), SCK(13) as standard
  .rxtx = 7,    // Not connected on RFM92/RFM95  // (7)
  .rst  = 9,    // Needed on RFM92/RFM95         // (9) Reset/POR    CHANGE for XBee-Module to "8"
  .dio  = {2, 5, 6},                             // input, interrupt (2,5,6)
                                                 // 2=DIO0 , 5=DIO3 , 6=DIO4
                                                 // 3=DIO1 , 4=DIO2 = not used
};

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
static const u4_t DEVADDR = 0xXXXXXXXX ; // <-- Change this address for every node!


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
  Serial.println("--> Show actual Date/Pos");
  MenuSelection = 1;
  MenuLevel = true;
  showDisplay_SDP();
}
void on_menu_set_RoR(MenuItem* pMenuItem)  {
  Serial.println("--> Sel Fix or Cnt");
  MenuLevel = true;
}
void on_menu_set_RoR_up(MenuItem* pMenuItem)  {
  Serial.println("--> leave RoR submenu");
  MenuSelection = 0;
  MenuLevel = true;
  displayMenu();
}
void on_menu_set_RoR_Fix(MenuItem* pMenuItem)  {
  Serial.println("--> Run on Request with Fix");
  MenuSelection = 2;
  MenuLevel = false;
  showDisplay_RoR_Fix();
}
void on_menu_set_RoR_Cnt(MenuItem* pMenuItem)  {
  Serial.println("--> Run on Request with Counter");
  MenuSelection = 2;
  ManButFire = true;
  MenuLevel = false;
  showDisplay_RoR_MFB();
}
void on_menu_set_RoDel(MenuItem* pMenuItem)  {
  Serial.println("--> Select Delay");
  MenuLevel = true;
}
void on_menu_set_DelayUp(MenuItem* pMenuItem)  {
  Serial.println("--> leave RoD submenu");
  MenuSelection = 0;
  MenuLevel = true;
  displayMenu();
}
void on_menu_set_Delay01(MenuItem* pMenuItem)  {
  MenuSelection = 3;
  if ( !HighPowerTx ) {
     delayTime = delayTime1;
     Serial.println(" Delay: 1 minute");
  } else {
     delayTime = delayTime2;
     Serial.println(" High Power Tx  set, so Delay: 5 minutes");
  }  
  MenuLevel = false;
  showDisplay_RonDel();
}
void on_menu_set_Delay05(MenuItem* pMenuItem)  {
  Serial.println(" Delay: 5 minutes");
  MenuSelection = 3;
  delayTime = delayTime2;
  MenuLevel = false;
  showDisplay_RonDel();
}
void on_menu_set_Delay10(MenuItem* pMenuItem)  {
  Serial.println(" Delay: 10 minutes");
  MenuSelection = 3;
  delayTime = delayTime3;
  MenuLevel = false;
  showDisplay_RonDel();
}

// ---------------------------------------------------------------------------------------------------
void setup() {

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

// GPS initialize
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
   GPS.begin(9600);
   delay(100);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  #ifdef DEBUG1
    Serial.begin(57600);                         // fast read for GPS echo
    delay(5000);
    Serial.println("Starting");
  #endif

// initialize button pins (input via 10k to ground, switch to Vcc
  pinMode(KeyUp,  INPUT);
  pinMode(KeySel, INPUT);
  pinMode(KeyDwn, INPUT);
  
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
  
    #ifdef DEBUG1
      display.setTextSize(1);
      display.setCursor(0,20);
      display.println("debug mode");
      display.update();
    #endif
    #ifdef DEBUG1
      delayTime = delayTime0;                   // 
      Serial.print("Device-Addr: ");
      Serial.print(DEVADDR, HEX);
      Serial.println();
    #endif 

  //
  Serial.flush();
  // blink green/blue to show that initialization finished
  blink(green);     
  blink(blue);  

  timer = millis();
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
          #endif
          // StatusStr = "TX completed";
          StatusStr = "           done";
          showStatus();         
          TXcomplete = true;
          blink(green);
          Serial.println(millis() / 1000);
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

// -------------------------------------------------------------------------------------
void blink(int pin) {
  digitalWrite(pin, HIGH);
  delay(200);
  digitalWrite(pin, LOW);
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
      if (buttonState_Up == HIGH) {
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
      if (buttonState_Sel == HIGH) {
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
      if (buttonState_Dwn == HIGH) {
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
      if (buttonState_Up == HIGH) {
      // MENU_ACTION
      if (MenuLevel == false)  ms.back();   // if in sub-menu you've to go up to the menu
        else ms.prev();                     // on menu level go easy one up
        blink(blue);      // indicate button pressed
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
      if (buttonState_Sel == HIGH) {
      // MENU_ACTION
         ms.select();       // returns from sub-menu, but "stays" in selected part (not nice)
         blink(blue);       // indicate button pressed
         showOnce =true;    // shows menu selection once if on a top one
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
      if (buttonState_Dwn == HIGH) {
      // MENU_ACTION
         ms.next();
         blink(blue);      // indicate button pressed
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
        leave_rUF   = true;
        initFix     = true;
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
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
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

}
  


// ---------------------------------------------------------------------------------------------------
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
  }
       #ifdef DEBUG1
         if ( showOnce ) {
           Serial.print("Menu Selection: ");
           Serial.print(MenuSelection);
           Serial.println();
           showOnce = false;
         }  
       #endif  

  if (MenuSelection != 2)    // on request do getKeypress
     Button();               // otherwise any button pressed?
  
  switch (MenuSelection) {
    case 0:
    // nothing selected or actual action breaked
       displayMenu();
       break;
    case 1:
    // show actual position
       if (timer > millis())  timer = millis();
       if ((millis() - timer > delayTime) && (TXcomplete == true)){
          timer = millis(); // reset the timer
          #ifdef DEBUG1
            Serial.println();
            Serial.print("case 1, show actual position each ");
            Serial.print(delayTime);
            Serial.print(" msec");
            Serial.println();
          #endif
          getPosition();
          showPosition();
       }
       break;
    case 2:
    // run on request
       getKeypress();
       if (doRequest == true) {
          if (ManButFire) {
            //uint8_t mydata[] = "Manual; Cnt: ";
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
          } 
          if ( !ManButFire )  showPosition();
           do_send(&sendjob);  
           // os_runloop_once();
       }  
       #ifdef DEBUG
         Serial.println("RoR, break arrived");
       #endif  
       break;

    case 3:
    // run with selected delay
       if (timer > millis())  timer = millis();
       if ((millis() - timer > delayTime) && (TXcomplete == true)){
          timer = millis(); // reset the timer
          #ifdef DEBUG1
            Serial.println();
            Serial.print("case 3, run on selected delay: ");
            Serial.print(delayTime);
            Serial.print(" msec");
            Serial.println();
          #endif
          getPosition();
          showPosition();
          if (noFix == false) do_send(&sendjob);
        }
/*       
       #ifdef DEBUG
         Serial.println("RoDel, os_runloop_once arrived");
       #endif 

       os_runloop_once();
              
       #ifdef DEBUG
         Serial.println("RoDel, break arrived");
       #endif 
*/         
       break;
    default:
    // do nothing
       break;
  }

     os_runloop_once();  
}

