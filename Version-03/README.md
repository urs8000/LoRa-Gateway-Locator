Based on Version 01/02
- Teensy 3.2
- GPS u-blox PAM-7Q with battary backup
- RFM95W (LoRa Radio 868MHz)
- OLED display (128x64) look careful when ordering. some have Vcc ans GND switched!
- navigation switches

this Version (03) added the following features
- debug/status LED (Common Anode RGB-LED, "Piranha diffused")
- PPS LED  (pulse per second)
- board mounted switches (wires could be connected for box mounted ones instead of them)
- second antenna connector (u.fl)
- GPS backup battery (CR1220)
- OpenLog SDcard based logger, connected via Serial3
  all sent positions will be logged, so it could be compared from where no connection to a given gateway could be established.

some explanations to the power supply:
getting the most out of 3 x 1.5V AA batteries and having stable 5V therefor I've added a xV to 5V step-up boost converter
The suggested Pololu step-up does not work correctly in contrary to the specification.
So I changed to another (bulkier) type found at eBay ( DC-DC Boost Converter Step Up 1-5V to 5V 500mA Module ) 
Because the layout is totally different, you have to solder it inbetween the batter case and the input pins and to bridge Vin-Vout.
If you plan to use a 5V power bank you just can bridge Vin and Vout and connect your supply instead of batteries to JPOWER
  
  
Tested on final boards, made by ITEAD Studio

