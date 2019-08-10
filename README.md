# Scooter-RPM-meter-for-Kymco-with-Nextion-TFT
Scooter-RPM-meter-for-Kymco-with-Nextion-TFT, display has yellow Ferrari RPM display, RTC on board, Temp and humid meter and oil meter.
Of course this can be used for any brand of scooter, i have a kymco agility 50cc.
This RPM meter goes to 8000 RPM and has been tested for many months and works great.

This project contains the Arduino code, all STL files for printing, the images file for your Nextion Display and a circuit for the pickup detector.
For the pickup detector i have PCB's available which you have to built yourself, you can have one PCB (without components for 2 euro excluding shipping.
The schematic file is on this page.

The Arduino i used is an Arduina Nano, further I used a temp.hum sensor and a rtc clock and of course a Nextion Display.

You also need a buck convertor to convert the 12V bat supply to 5V for the Arduina, the sensor runs on 12V and is interface with an opto-coupler on the main board. Schematic for the main board i dont have, should be easy to figure out from the code.
If you get stuck while building this you can contact me for help.

The TFT file need to be loaded into the Nextion display with an SD card, place the file in the root of the SD card and power the dispay, the file should be loaded automatically.

Temp/Hum meter:  https://nl.aliexpress.com/item/32759158558.html?spm=a2g0s.9042311.0.0.27424c4dwhILO9

RTC: https://nl.aliexpress.com/item/2037934408.html?spm=a2g0s.9042311.0.0.27424c4dwhILO9

Oil Temp meter: https://nl.aliexpress.com/item/1297739612.html?spm=a2g0s.9042311.0.0.27424c4d40bAqC

Nextion Display:  https://nl.aliexpress.com/item/32810467897.html?spm=a2g0s.9042311.0.0.27424c4dopPIMa




