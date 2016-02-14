Based on Version 01
- Teensy 3.2
- GPS u-blox PAM-7Q with battary backup
- RFM95W (LoRa Radio 868MHz)
- OLED display (128x64) look careful when ordering. some have Vcc ans GND switched!
- navigation switches

this Version (02) added the following features
- debug/status LED (Common Anode Piranha)
- PPS LED  (pulse per second)
- board mounted switches (wires could be connected for box mounted ones instead of them)
- second antenna connector (u.fl)
- only one type of battery (CR1220)
- OpenLog SDcard based logger, connected via Serial3
  all sent positions will be logged, so it could be compared from where no connection to a given gateway could be established.

some explanations to the power supply:
getting the most out of 3 x 1.5V AA batteries and having stable 5V therefor I've added a xV to 5V step-up boost converter
I know, that all components would work with 3V3, but for that all of them had to be modified.
So the simpler way is a stable 5V supply and they convert it by themselfe.

If you plan to use a 5V power bank you just can bridge Vin and Vout and connect your supply instead of batteries to JPOWER
  
  
  THIS HAS ONLY BE TESTED ON A BREADBOARD (2016.02.14)
  ***** ALL REPRODUCTION ON YOUR OWN RISK *****

