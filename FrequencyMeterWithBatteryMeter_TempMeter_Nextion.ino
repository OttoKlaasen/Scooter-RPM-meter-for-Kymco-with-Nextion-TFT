// Base Code for pulse detection from https://engmousaalkaabi.blogspot.com/2016/11/ac-220v-frequency-counter-using-arduino.html
// Rewritten by Otto M.Klaasen (c) 2018/2019
// This code also has a battery voltage meter and an oil temperature sensor.
// This version also includes a Nextion Display so we can display the data to a Nextion Display
// In april 2019 we added a temp/hum meter and a rtc clock.

#include <LiquidCrystal.h>
#include <OneWire.h>
#include <stdlib.h>

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.

const int numReadings = 3;      // Size of the filter

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// DHT Temperature & Humidity Sensor
// Unified Sensor Library Example
// Written by Tony DiCola for Adafruit Industries
// Released under an MIT license.

// REQUIRES the following Arduino libraries:
// - DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
// - Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2      // Digital pin connected to the DHT sensor 
#define BLINKLED  7   // Shows interrupt running

// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.
// Uncomment the type of sensor in use:
//#define DHTTYPE    DHT11     // DHT 11
#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);
uint32_t delayMS;

// DS3231_Serial
// To use the hardware I2C (TWI) interface of the Arduino you must connect
// the pins as follows:
//
// Arduino Uno/2009:
// ----------------------
// DS3231:  SDA pin   -> Arduino Analog 4 or the dedicated SDA pin
//          SCL pin   -> Arduino Analog 5 or the dedicated SCL pin
//
// Arduino Leonardo:
// ----------------------
// DS3231:  SDA pin   -> Arduino Digital 2 or the dedicated SDA pin
//          SCL pin   -> Arduino Digital 3 or the dedicated SCL pin
//
// Arduino Mega:
// ----------------------
// DS3231:  SDA pin   -> Arduino Digital 20 (SDA) or the dedicated SDA pin
//          SCL pin   -> Arduino Digital 21 (SCL) or the dedicated SCL pin
//
// Arduino Due:
// ----------------------
// DS3231:  SDA pin   -> Arduino Digital 20 (SDA) or the dedicated SDA1 (Digital 70) pin
//          SCL pin   -> Arduino Digital 21 (SCL) or the dedicated SCL1 (Digital 71) pin
//
// The internal pull-up resistors will be activated when using the 
// hardware I2C interfaces.
//
// You can connect the DS3231 to any available pin but if you use any
// other than what is described above the library will fall back to
// a software-based, TWI-like protocol which will require exclusive access 
// to the pins used, and you will also have to use appropriate, external
// pull-up resistors on the data and clock signals.
//

// Driver for the RTC
#include <DS3231.h>
// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);


// OneWire DS18S20, DS18B20, DS1822 Temperature Example
//
// http://www.pjrc.com/teensy/td_libs_OneWire.html
//
// The DallasTemperature library can do all this work for you!
// https://github.com/milesburton/Arduino-Temperature-Control-Library

OneWire  ds(10);  // on pin 10 (a 4.7K resistor is necessary)

// setting for Nextion Display
/* 
Connection with Arduino Uno/Nano for the Nextion Display
* +5V = 5V
* TX  = pin 0 (RX)
* RX  = pin 1 (TX)
* GND = GND
*/

int CurrentPage = 0;  // Create a variable to store which page is currently loaded

#include <Nextion.h>  // Include the nextion library (the official one) https://github.com/itead/ITEADLIB_Arduino_Nextion
                      // Make sure you edit the NexConfig.h file on the library folder to set the correct serial port for the display.
                      // By default it's set to Serial1, which most arduino boards don't have.
                      // Change "#define nexSerial Serial1" to "#define nexSerial Serial" if you are using arduino uno, nano, etc.
                      
// Declare objects that we are going to read from the display. This includes buttons, sliders, text boxes, etc:
// Format: <type of object> <object name> = <type of object>(<page id>, <object id>, "<object name>");
/* ***** Types of objects:
 * NexButton - Button
 * NexDSButton - Dual-state Button
 * NexHotspot - Hotspot, that is like an invisible button
 * NexCheckbox - Checkbox
 * NexRadio - "Radio" checkbox, that it's exactly like the checkbox but with a rounded shape
 * NexSlider - Slider
 * NexGauge - Gauge
 * NexProgressBar - Progress Bar
 * NexText - Text box
 * NexScrolltext - Scroll text box
 * NexNumber - Number box
 * NexVariable - Variable inside the nextion display
 * NexPage - Page touch event
 * NexGpio - To use the Expansion Board add-on for Enhanced Nextion displays
 * NexRtc - To use the real time clock for Enhanced Nextion displays
 * *****
 */
 //This was used in orginal nextion demo to switch a LED on Page 1.
NexButton b3 = NexButton(2, 7, "b3");  // Button added for storing RTC data.
// Define number fields at page 2
NexNumber n0 = NexNumber(2,5,"n0");   // Number field on Page 2 for Hours.
NexNumber n1 = NexNumber(2,8,"n1");   // Number field on Page 2 for Minutes.
NexNumber n2 = NexNumber(2,9,"n2");   // Number field on Page 2 for Seconds.
NexNumber n3 = NexNumber(2,16,"n3");  // Number field on Page 2 for Day.
NexNumber n4 = NexNumber(2,17,"n4");  // Number field on Page 2 for Months.
NexNumber n5 = NexNumber(2,18,"n5");  // Number field on Page 2 for Year.
// Day of week
NexText DayOW = NexText(2,19,"t3_2");  // Day of week
// Wave form at page 3
NexWaveform s0  = NexWaveform(3, 1, "s0"); //Declare the WaveForm object
// Next is not used
NexDSButton bt0 = NexDSButton(1, 3, "bt0");  // Dual state button added

// Declare pages:
// Sending data to the display to nonexistent objects on the current page creates an error code sent by the display.
// Any error sent by the display creates lag on the arduino loop because arduino tries to read it, thinking it's a touch event.
// So to avoid this, I am only going to send data depending on the page the display is on.
// That's the reason I want the arduino to know which page is loaded on the display.
// To let arduino know what page is currently loaded, we are creating a touch event for each page.
// On the nextion project, each page must send a simulated "Touch Press Event" in the "Preinitialize Event" section so
// we can register that a new page was loaded.
NexPage page0 = NexPage(0, 0, "page0");  // Page added as a touch event
NexPage page1 = NexPage(1, 0, "page1");  // Page added as a touch event
NexPage page2 = NexPage(2, 0, "page2");  // Page added as a touch event
NexPage page3 = NexPage(3, 0, "page3");  // Page added as a touch event


// Declare touch event objects to the touch event list: 
// You just need to add the names of the objects that send a touch event.
// Format: &<object name>,

NexTouch *nex_listen_list[] = 
{
  &bt0,    // Dual state button added
  &b3,     // Button added
  &n0,     // Number
  &page0,  // Page added as a touch event
  &page1,  // Page added as a touch event
  &page2,  // Page added as a touch event
  &page3,  // Page added as a touch event
    NULL  // String terminated
};  // End of touch event list

int input=13;  // Input for the RPM counter

long high_time;
long low_time;
long time_period;
long freq;
long frequency;
long cRPM;            // Current RPM
long pRPM;            // Previous RPM
int dRPM = 500;       // Delta RPM
long maxRPM = 8000;   // The max RPM we use

float Temperature;

// ************************************************************************************************************************************

void setup()
{
  cRPM = 0;
  pRPM = 0;

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(BLINKLED, OUTPUT);
  
// setup serial communication
  
  Serial.begin(9600);  // Start serial comunication at baud=9600. For Arduino mega you would have to add a
                       // number (example: "Serial1.begin(9600);").
                       // If you use an Arduino mega, you have to also edit everything on this sketch that
                       // says "Serial" and replace it with "Serial1" (or whatever number you are using).

  // Temp and Hum Sensor
  // Initialize device.
  dht.begin();

  // Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
//  Serial.println(F("------------------------------------"));
//  Serial.println(F("Temperature Sensor"));
//  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
//  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
//  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
//  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
//  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
//  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
//  Serial.println(F("------------------------------------"));
//  Print humidity sensor details.

    dht.humidity().getSensor(&sensor);
//  Serial.println(F("Humidity Sensor"));
//  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
//  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
//  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
//  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
//  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
//  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
//  Serial.println(F("------------------------------------"));
// Set delay between sensor readings based on sensor details.

  delayMS = sensor.min_delay / 1000;
  
  // Initialize the rtc object
  rtc.begin();
  
  // The following lines can be uncommented to set the date and time
   //rtc.setDOW(FRIDAY);     // Set Day-of-Week to SUNDAY
   //rtc.setTime(16, 13, 0);     // Set the time to 12:00:00 (24hr format)
   //rtc.setDate(19, 4, 2019);   // Set the date to January 1st, 2014

// Register the event callback functions of each touch event:
// You need to register press events and release events seperatly.
// Format for press events: <object name>.attachPush(<object name>PushCallback);
// Format for release events: <object name>.attachPop(<object name>PopCallback);
b3.attachPush(b3PushCallback);  // Button press
b3.attachPop(b3PopCallback);  // Button release
bt0.attachPush(bt0PushCallback);  // Dual state button bt0 press
page0.attachPush(page0PushCallback);  // Page press event
page1.attachPush(page1PushCallback);  // Page press event
page2.attachPush(page2PushCallback);  // Page press event
page3.attachPush(page3PushCallback);  // Page press event

// End of registering the event callback functions

StartTestGauge(CurrentPage); // Start Gauge test which rotates the gauge at the RPM meter from 0 to 10000 rpm.
  
pinMode(input,INPUT);

 // Average buffer
 // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // Setting up interrupts

cli();//stop interrupts

//set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 7812; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

sei();//allow interrupts
}

void loop()
{

// Measure RPM in main loop.
 
low_time=pulseIn(input,LOW);  // Read pulse in ms when input change from high to low.
high_time=pulseIn(input,HIGH);

if (low_time >0 & high_time>0 )  // If both have a value we have measured something !
{
  time_period=high_time+low_time;  // Time period in ms, so if we read 1000 + 12000= 13000
  time_period=time_period/1000;    // Time perion in Arduin to get in ms, so devide 13000/1000= 13ms pulse which is duty cycle.
  frequency=1000/time_period;      // To get frequence devide 1000ms by 13ms which is 76.9 Hz
  cRPM = int(frequency * 60);     // Convert to integer value 
 
  // Dont move RPM to fast, dRPM is the stepping value which is depening on the cRPM
  // You can select two methos of filtering, an average filter of a delta filter.
if (0) { 
    // If 1 use average filter
cRPM = AvgFilter(cRPM);
}
else {
    // Use Delta Filter, this works best with a Kymco Agility Scooter
     if (cRPM > pRPM + dRPM) {
        cRPM = pRPM + dRPM;
        }
     else if (cRPM < pRPM - dRPM) {
        cRPM = pRPM - dRPM;
     }
}

  DisplayRPM(CurrentPage,cRPM); // Send RPM to display.
  DisplayRPMGauge(CurrentPage,cRPM);
  // Update previous value
  pRPM = cRPM;
  
} else {
  cRPM = 0;  // You can increase the value for testing
  DisplayRPM(CurrentPage,cRPM); // Send 0 RPM to display.
  DisplayRPMGauge(CurrentPage,cRPM);
 }
 
// We are going to check the list of touch events we enter previously to
// know if any touch event just happened, and excecute the corresponding instructions:

 nexLoop(nex_listen_list);  // Check for any touch event  

 if(CurrentPage == 0) {
        // Force text fields to correct text when page switching has been done.
        WriteTextFieldPage0();
        // Set the Temp and Humi on Nextion display
        SetTempHumNextion();
      
        // Read the battery voltage.
        int sensorValue = analogRead(A1); //read the A1 pin value
        float R1R2 = (9.82 + 4.62)/4.62;  // we use 9.9K and 4.63K resistor.
        float voltage = sensorValue * (5.00 / 1023.00) * R1R2; //convert the value to a true voltage.
                 
        DisplayBatteryVoltage(CurrentPage,voltage); // Send voltage to display.

        // Read the temperature of the oil
 
        Temperature = ReadtemperatuurSensor();
        if ( Temperature != 0)  DisplayOilTemperature(CurrentPage,Temperature); // Send voltage to display.
 }
    
  if (CurrentPage == 2)
  {
    // Setup for the clock setup
    // Clock will be set when the button is presses in the UI which will trigger subroutine.
   }
  if (CurrentPage == 3)
  {
    // Display Wave of RPM on page 4, the update of the screen depends on the RPM detected.
    if (cRPM > 0 and cRPM < maxRPM){
        DisplayWave(CurrentPage,cRPM);
    }
   }
 }


// ********************************************************************************

// Here we have the functions

// Timer interrupt routine.

ISR(TIMER1_COMPA_vect){  //change the 0 to 1 for timer1 and 2 for timer2
//interrupt commands here
// Run indicator
digitalWrite(BLINKLED, HIGH);   // turn the LED on (LOW is the voltage level)
       
// Set the clock on Nextion display
if (CurrentPage==0) {
  SetClockNextion(CurrentPage);
}
if (CurrentPage==2) {
  SetUpClockNextion(CurrentPage);
}
digitalWrite(BLINKLED, LOW);   // turn the LED OFF (HIGH is the voltage level)
}

// ********************************************************************************
float ReadtemperatuurSensor(){  //This function reads the temperature from the oil.
byte i;
  byte present = 0;
  byte type_s;
  byte data[12];
  byte addr[8];
  float celsius;
  
  if ( !ds.search(addr)) {
    //Serial.println("No more addresses.");
    //Serial.println();
    ds.reset_search();
    delay(5);
    return;
   }
  
  //Serial.print("ROM =");
  //for( i = 0; i < 8; i++) {
    //Serial.print(addr[i], HEX);
  //}

  if (OneWire::crc8(addr, 7) != addr[7]) {
      //Serial.println("CRC is not valid!");
      return;
  }
  //Serial.println();
 
  // the first ROM byte indicates which chip
  switch (addr[0]) {
    case 0x10:
      //Serial.println("  Chip = DS18S20");  // or old DS1820
      type_s = 1;
      break;
    case 0x28:
      //Serial.println("  Chip = DS18B20");
      type_s = 0;
      break;
    case 0x22:
      //Serial.println("  Chip = DS1822");
      type_s = 0;
      break;
    default:
      //Serial.println("Device is not a DS18x20 family device.");
      return;
  } 

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end
  
  delay(5);     // maybe 5ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  //Serial.print("  Data = ");
  //Serial.print(present, HEX);
  //Serial.print(" ");
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
    //Serial.print(data[i], HEX);
    //Serial.print(" ");
  }
  //Serial.print(" CRC=");
  //Serial.print(OneWire::crc8(data, 8), HEX);
  //Serial.println();

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  if (type_s) {
    raw = raw << 3; // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  celsius = (float)raw / 16.0;
  //Serial.print("  Temperature = ");
  //Serial.print(celsius);
  //Serial.print(" Celsius, ");

  return celsius;
}
// End of float ReadtemperatuurSensor() fucntion
// ********************************************************************************

////////////////////////// Touch events:
// Each of the following sections are going to run everytime the touch event happens:
// Is going to run the code inside each section only ones for each touch event.

void b3PushCallback(void *ptr)  // Press event for button b3
{
  digitalWrite(13, LOW);  // Turn ON internal LED
  // Get clock settings from page 2
  uint32_t vHour = 0;  // Create variable to store value we are going to get
  n0.getValue(&vHour);  // Read value of number field.
  
  uint32_t vMin = 0;  // Create variable to store value we are going to get
  n1.getValue(&vMin);  // Read value of number field.
  
  uint32_t vSec = 0;  // Create variable to store value we are going to get
  n2.getValue(&vSec); //Read value of number field. 
  
  uint32_t vDay = 0;  // Create variable to store value we are going to get
  n3.getValue(&vDay);  // Read value of number field. 
  
  uint32_t vMonth = 0;  // Create variable to store value we are going to get
  n4.getValue(&vMonth);  // Read value of number field. 
  
  uint32_t vYear = 0;  // Create variable to store value we are going to get
  n5.getValue(&vYear);  // Read value of number field.

   char text_char[12]; 
   memset(text_char, 0, sizeof(text_char)); // Clear buffer
   DayOW.getText(text_char, 12); // Read text field t3 on page 2
   String MyDow = text_char;
   // Set the day of the week       
   if (MyDow == "Monday"){ 
    rtc.setDOW(MONDAY);
   }
   if (MyDow == "Tuesday"){ 
    rtc.setDOW(TUESDAY);
   }
   if (MyDow == "Wednesday"){ 
    rtc.setDOW(WEDNESDAY);
   }
   if (MyDow == "Thursday"){ 
    rtc.setDOW(THURSDAY);
   } 
   if (MyDow == "Friday"){ 
    rtc.setDOW(FRIDAY);
   }
   if (MyDow == "Saturday"){ 
    rtc.setDOW(SATURDAY);
   }
   if (MyDow == "Sunday"){ 
    rtc.setDOW(SUNDAY);
   }
   
   rtc.setTime(vHour, vMin, vSec);     // Set the time to 12:00:00 (24hr format)
   rtc.setDate(vDay, vMonth, vYear);   // Set the date to January 1st, 2014
   digitalWrite(13, HIGH);  // Turn ON internal LED
}  // End of press event

// ********************************************************************************
void b3PopCallback(void *ptr)  // Release event for button b1
{
   // digitalWrite(13, LOW);  // Turn OFF internal LED
}  // End of release event

// ********************************************************************************
void bt0PushCallback(void *ptr)  // Press event for dual state button bt0
{
  uint32_t number5 = 0;  // Create variable to store value we are going to get
  bt0.getValue(&number5);  // Read value of dual state button to know the state (0 or 1)

  if(number5 == 1){  // If dual state button is equal to 1 (meaning is ON)...
    digitalWrite(13, HIGH);  // Turn ON internal LED
  }else{  // Since the dual state button is OFF...
    digitalWrite(13, LOW);  // Turn OFF internal LED
  }
}  // End of press event

// ********************************************************************************

// Page change event:
void page0PushCallback(void *ptr)  // If page 0 is loaded on the display, the following is going to execute:
{
  CurrentPage = 0;  // Set variable as 0 so from now on arduino knows page 0 is loaded on the display
}  // End of press event

// ********************************************************************************

// Page change event:
void page1PushCallback(void *ptr)  // If page 1 is loaded on the display, the following is going to execute:
{
  CurrentPage = 1;  // Set variable as 1 so from now on arduino knows page 1 is loaded on the display
}  // End of press event

// ********************************************************************************

// Page change event:
void page2PushCallback(void *ptr)  // If page 2 is loaded on the display, the following is going to execute:
{
  CurrentPage = 2;  // Set variable as 2 so from now on arduino knows page 2 is loaded on the display
}  // End of press event

// ********************************************************************************

// Page change event:
void page3PushCallback(void *ptr)  // If page 3 is loaded on the display, the following is going to execute:
{
  cli();//stop interrupts
  CurrentPage = 3;  // Set variable as 3 so from now on arduino knows page 3 is loaded on the display
  sei();//allow interrupts
  
}  // End of press event

// ********************************************************************************
 // Gauge full range check from 0-360-0  at startup
 void StartTestGauge(int CurrentPage)
 {
 if(CurrentPage == 0){  // If the display is on page 1, do the following:
  
        // We are going to send the value of the loop to the object called z0:
        // After the name of the object you need to put the dot val because val is the atribute we want to change on that object.

        // Set to Blue
        Serial.print("z0.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
        Serial.print(31);  // This is the value you want to send to that object and atribute mention before.
        Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial.write(0xff);
        Serial.write(0xff);  
        delay(100);
        
        for (int i = 270; i <= 360; i=i+2) {
            Serial.print("z0.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
            Serial.print(i);  // This is the value you want to send to that object and atribute mention before.
            Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial.write(0xff);
            Serial.write(0xff);
            }
         for (int i = 0; i <= 180; i=i+2) {
            Serial.print("z0.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
            Serial.print(i);  // This is the value you want to send to that object and atribute mention before.
            Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial.write(0xff);
            Serial.write(0xff);
            }
        for (int i = 180; i >= 0; i=i-2) {
            Serial.print("z0.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
            Serial.print(i);  // This is the value you want to send to that object and atribute mention before.
            Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial.write(0xff);
            Serial.write(0xff);
            }
        for (int i = 360; i >= 270; i=i-2) {
            Serial.print("z0.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
            Serial.print(i);  // This is the value you want to send to that object and atribute mention before.
            Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
            Serial.write(0xff);
            Serial.write(0xff);
            }
        // Set to Red
        Serial.print("z0.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
        Serial.print(63488);  // This is the value you want to send to that object and atribute mention before.
        Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
        Serial.write(0xff);
        Serial.write(0xff);  
  }
 }

// ********************************************************************************
// Set the Temp and Hum at nextion display

void SetTempHumNextion(){ 

if(CurrentPage == 0) {
// We use t10 for temp and t10 for huminity object in the display at page 0
// Delay between measurements.
// delay(delayMS);
// Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
//    Serial.println(F("Error reading temperature!"));
  }
  else {
    // Serial.print(event.temperature);
       Serial.print("t10.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t10.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(event.temperature);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
    }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
//    Serial.println(F("Error reading humidity!"));
  }
  else {
    // Serial.print(event.relative_humidity);
       Serial.print("t11.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t11.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(int(event.relative_humidity));
          Serial.print("%");
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
  }
 }
}

// ********************************************************************************


// Set the RTC Clock at nextion display

void SetClockNextion(int CurrentPage){ 
 // We use t3 for time and t4 for date object in the display at page 0
 // We use t12 for the day of the week
  // Send Day-of-Week
  // Serial.print(rtc.getDOWStr());
  // Serial.print(" ");
  
  // Send date
  // Serial.print(rtc.getDateStr());
  // Serial.print(" -- ");

  // Send time
  // Serial.println(rtc.getTimeStr());
if (CurrentPage == 0) {  // Only Write in case of page 0
          Serial.print("t3.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t3.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(rtc.getTimeStr());  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 

          Serial.print("t4.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t4.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(rtc.getDateStr());  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 

          Serial.print("t12.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t12.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(rtc.getDOWStr());          
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
          }
}
// ********************************************************************************


// Set the RTC Clock at nextion display for page 2

void SetUpClockNextion(int CurrentPage){ 
 // We use t1 for time and t2 for date object in the display at page 2
 // We use t0 for the day of the week
  // Send Day-of-Week
  // Serial.print(rtc.getDOWStr());
  // Serial.print(" ");
  
  // Send date
  // Serial.print(rtc.getDateStr());
  // Serial.print(" -- ");

  // Send time
  // Serial.println(rtc.getTimeStr());
  
if (CurrentPage == 2) {  // Only Write in case of page 2
          Serial.print("t1.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t1.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(rtc.getTimeStr());  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 

          Serial.print("t2.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t2.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(rtc.getDateStr());  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 

          Serial.print("t0.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t0.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(rtc.getDOWStr());          
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
          }
}

// ********************************************************************************

 // DisplayBatteryVoltage, if voltage is above 12V the text is GREEN , below 12B it's RED.
 void DisplayBatteryVoltage(int CurrentPage, float voltage)
 // We use t5 object in the display at page 0.
 {
 if(CurrentPage == 0){  // If the display is on page 1, do the following:

      char buffer[10];   // Converting float to text with two decimals behind the ,
      String VoltageText = dtostrf(voltage, 5, 2, buffer);

      if(voltage > 12)  // If the variable is greater than 12V...
        {
          Serial.print("t5.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(1032);       // Green
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t5.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(VoltageText);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
          }
        else  // If condition was false, do the following:
        {
          Serial.print("t5.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(63488);      // Red
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t5.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(VoltageText);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          }
       
    }
 }

  
// ********************************************************************************

 // Display Oil temperature, if temperature is above 75 degreesthe text is GREEN , else it's RED.
 void DisplayOilTemperature(int CurrentPage, float Temperature)
 // We use t6 object in the display at page 0.
 {
 if(CurrentPage == 0 and Temperature !=0) {  // If the display is on page 0, do the following:

      char buffer[10];   // Converting float to text with two decimals behind the ,
      String TemperatureText = dtostrf(Temperature, 5, 2, buffer);

      if(Temperature < 75 )  // If the variable is greater than 75  degrees...
        {
          Serial.print("t6.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(1032);       // Green
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t6.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(TemperatureText);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
          }
        else  // If condition was false, do the following:
        {
          Serial.print("t6.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(63488);      // Red
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t6.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(TemperatureText);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          }
       
    }
 }

// ********************************************************************************

 // Display RPM, if RPM is above 6000 he text is GREEN , else it's RED.
 void DisplayRPM(int CurrentPage, float RPM)
  // We use t7 object in the display at page 0.
 {
 if(CurrentPage == 0){  // If the display is on page 0, do the following:

      char buffer[10];   // Converting float to text with two decimals behind the ,
      String RPMText = dtostrf(RPM, 5, 0, buffer);

      if(RPM < 6500 )  // If the variable is smaller than 65000 RPM..
        {
          Serial.print("t7.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(1032);       // Green
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t7.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          if(RPM ==0 )Serial.print("Zero");  // This is the text we want to send to that object and atribute mention before.
          else        Serial.print(RPMText);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
          }
        else  // If condition was false, do the following:
        {
          Serial.print("t7.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(63488);      // Red
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t7.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print(RPMText);  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          }
       
    }
 }

// ********************************************************************************

 // Display RPM on Gauge.
 void DisplayRPMGauge(int CurrentPage, float RPM)
  // We use z0 object in the display at page 0.
 {
 if(CurrentPage == 0){  // If the display is on page 1, do the following:
      int i;
      int RPM1 = 3600;
      int RPM2 = 7000;
      char buffer[10];   // Converting float to text with two decimals behind the ,
      String RPMText = dtostrf(RPM, 5, 2, buffer);
      // RPM can be from 0 - 10000 rpm.
      // There are two ranges , first from 270 degrees to 360 degrees
      // Second range is from 0 to 180 degress
      // So we use 3/4 of out display.
      // The first range is 1/3 of total range (90 degrees from 270 total total range.
      // So RPM 0 - 3333 defines first range and 3333 - 10000 is second range.
      // than map the two ranges depending on RPM: map(value, fromLow, fromHigh, toLow, toHigh)
      // To create better tolerances i made three ranges defined ny RPM1 and RPM2
      if (RPM < RPM1) {
        i = map (RPM,0,RPM1,270,360);
      }
      else{
          if (RPM > RPM2) {
             i = map (RPM,RPM2,10000,90,170); 
             }
          else {
            i = map (RPM,RPM1,RPM2,0,90); 
          }
      }
      Serial.print("z0.val=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
      Serial.print(i);  // This is the value you want to send to that object and atribute mention before.
      Serial.write(0xff);  // We always have to send this three lines after each command sent to the nextion display.
      Serial.write(0xff);
      Serial.write(0xff);
    }
 }
 // ********************************************************************************

 // Display RPM on Scope page
 void DisplayWave(int CurrentPage, float RPM)
  // We use s0 waveform object which has ID=1 in the display at page 3.
 {
 if(CurrentPage == 3){  // If the display is on page 3, do the following:

  int WValue = map(RPM,0,maxRPM,0,255);  //map it to 0.255 (max value of waveform=255)

  s0.addValue(0,WValue);
//  String Tosend = "add ";                                       //We send the string "add "
//  Tosend += 1;                                                  //send the id of the block you want to add the value to
//  Tosend += ",";  
//  Tosend += 0;                                                  //Channel of the id, in this case channel 0 of the waveform
//  Tosend += ",";
//  Tosend += WValue;                                              //Send the value and 3 full bytes
//  Serial.print(Tosend);
//  Serial.write(0xff);
//  Serial.write(0xff);
//  Serial.write(0xff);
      }
 }


 // ********************************************************************************
 // Force text fields on page 0 to correct text

 void WriteTextFieldPage0 (){

          Serial.print("t0.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t0.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print("RPM");  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 
          
          Serial.print("t1.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t1.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print("Batt");  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 

          Serial.print("t2.pco=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print(65535);       // White
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff);
          Serial.print("t2.txt=");  // This is sent to the nextion display to set what object name (before the dot) and what atribute (after the dot) are you going to change.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.print("Oil");  // This is the text we want to send to that object and atribute mention before.
          Serial.print("\"");  // Since we are sending text we need to send double quotes before and after the actual text.
          Serial.write(0xff); 
          Serial.write(0xff);
          Serial.write(0xff); 

 }

  // ********************************************************************************
 // Average filter for RPM

int AvgFilter (int RPM){

  if (RPM > 0 and RPM <10000) {

     // subtract the last reading:
      total = total - readings[readIndex];
      // read from the sensor:
      readings[readIndex] = RPM;
      // add the reading to the total:
      total = total + readings[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;
    
      // if we're at the end of the array...
      if (readIndex >= numReadings) {
        // ...wrap around to the beginning:
        readIndex = 0;
      }
    
      // calculate the average:
      average = total / numReadings;
      return int(average);
  }
 }
