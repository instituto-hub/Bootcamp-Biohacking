/*
	This file is part of Waag Society's BioHack Academy Code.

	Waag Society's BioHack Academy Code is free software: you can 
	redistribute it and/or modify it under the terms of the GNU 
	General Public License as published by the Free Software 
	Foundation, either version 3 of the License, or (at your option) 
	any later version.

	Waag Society's BioHack Academy Code is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even 
	the implied warranty of MERCHANTABILITY or FITNESS FOR A 
	PARTICULAR PURPOSE.  See the GNU General Public License for more 
	details.

	You should have received a copy of the GNU General Public License
	along with Waag Society's BioHack Academy Code. If not, see 
	<http://www.gnu.org/licenses/>.
*/

/* PLEASE NOTE
 *  By default this code assume you will use an analog temperature sensor 
 *  (10K Thermistor). In case you are using an 1-wire digital sensor you 
 *  will have to uncomment a section in the Loop() function
 */


/* Attribution

MOSFET code derived from: 	http://bildr.org/2012/03/rfp30n06le-arduino/
button code derived from: 	http://arduino.cc/en/tutorial/button
Thermistor code derived from: http://computers.tutsplus.com/tutorials/how-to-read-temperatures-with-arduino--mac-53714

NOTE for first time users:
- Try to build the circuits and run the sketches in the above examples prior to using this code

*/

/* *******************************************************
/  Libraries
*/
#include <math.h>    // loads a library with more advanced math functions
#include <Wire.h> // Needed for I2C connection
#include "LiquidCrystal_I2C.h" // Needed for operating the LCD screen
#include <OneWire.h>
/* *******************************************************
*/

/* *******************************************************
/  LCD
*/
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27,16,2);
/* *******************************************************
*/

/* *******************************************************
// Define pins
*/
#define fanPin 5     // The mosfet that drives the 80mm fan is connected to pin 3
#define mosfetPin 6   // Pin for the relay that controls the heat pad
#define buttonPin1 11 // Pin for the push button left of the screen
#define buttonPin2 12 // Pin for the push button right of the screen
#define ledPin 13    // Arduino nboard LED pin used as indicator
/* *******************************************************
*/

/* *******************************************************
/  Define variables
*/
int targetTemp = 18;   // Initial target temperature
int button1State = 0;  // variable for reading the pushbutton status
int button2State = 0;  // variable for reading the pushbutton status
int val;                //Create an integer variable
double temp;            //Variable to hold a temperature value
/* *******************************************************
*/

/* *******************************************************
/  Time variables
*/
long switch_time = 2000; // switch mosfet on or off once every 3 seconds
long begin_switch_time = 0;
long temp_time = 2000; // read temperature once every 2 seconds
long begin_temp_time = 0;
/* *******************************************************
*/

// Temp sensor digital
int DS18S20_Pin = 10; //DS18S20 Signal pin on digital 10
OneWire ds(DS18S20_Pin); // on digital pin 10

// Temp sensor analog
int analogTempPin = A0;
  
void setup() {            //This function gets called when the Arduino starts
  Serial.begin(115200);   //This code sets up the Serial port at 115200 baud rate
  Serial.println(F("BioHack Academy Incubator"));
  
  // Set pin modes
  pinMode(fanPin, OUTPUT);  // Fanpin
  pinMode(ledPin, OUTPUT); // initialize the LED pin as an output:
  pinMode(buttonPin1, INPUT); // initialize the pushbutton pin as an input:  
  pinMode(buttonPin2, INPUT); // initialize the pushbutton pin as an input:
  pinMode(mosfetPin, OUTPUT); // Mosfetpin controling the heating  
  
  // Set time 
  begin_switch_time = millis();  
  
  // Initialize I2C
  Wire.begin();

  // Initialize the LCD and print a message
  lcd.init(); // start the LCD
  lcd.backlight(); // enable the backlight
  lcd.clear(); // Clear the screen from any character
  lcd.setCursor(0,0); // Start writing from position 0 row 0, so top left
  lcd.print(F("BioHack Academy"));
  lcd.setCursor(0,1);
  lcd.print(F("Incubator test"));
  delay(1000);
  lcd.clear();

  // The fan is always on
  digitalWrite(fanPin, HIGH);
}

/* *******************************************************
/  Thermistor function converts the raw signal into a temperature
*/
double Thermister(int RawADC) {  //Function to perform the fancy math of the Steinhart-Hart equation
  double Temp;
  Temp = log(((10240000/RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;              // Convert Kelvin to Celsius
  //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Celsius to Fahrenheit - comment out this line if you need Celsius
  return Temp;
}
/* *******************************************************
*/

/* *******************************************************
/  Loop, this code is constantly repeated
*/ 
void loop() {

  if((millis() - begin_temp_time) > temp_time) {

    // UNCOMMENT THESE LINES IN CASE YOU ARE USING A DIGITAL TEMP SENSOR 
    // float temperature = getTemp(); // Read temperature via digital sensor
    // temp=temperature;
    // UNCOMMENT UNTIL HERE

    /* THESE LINES ARE USED WHEN AN ANALOG TEMP SENSOR IS IN USE */
    val=analogRead(analogTempPin);      //Read the analog port 0 and store the value in val
    temp=Thermister(val);   //Runs the fancy math on the raw analog value
    /* END ANALOG SENSOR */
    
    Serial.println(temp);   //Print the value to the serial port
    begin_temp_time = millis();
  }
  
  if((millis() - begin_switch_time) > switch_time) {
    // Turn the heating pad on or off depending on temperature
    if (temp < targetTemp) {
      digitalWrite(mosfetPin, HIGH);
      begin_switch_time = millis(); 
    }
    else {
      digitalWrite(mosfetPin, LOW);
      begin_switch_time = millis(); 
    }
  }
  
  // Read the state of the pushbutton value:
  button1State = digitalRead(buttonPin1);
  button2State = digitalRead(buttonPin2); 
 
  // Check if the pushbutton is pressed.
  // If it is, the buttonState is HIGH:
  if (button1State == HIGH) {     
    // turn LED on:    
    digitalWrite(ledPin, HIGH);  
    ++targetTemp;
    if(targetTemp > 50) targetTemp = 50;  // Sanity check, do not allow temperatures higher than 50  
    delay(500);
  } 
  else if (button2State == HIGH) {
    digitalWrite(ledPin, HIGH);
    --targetTemp;
    if(targetTemp < 0) targetTemp = 0;  // Sanity check, do not allow temperatures lower than 0   
    delay(500);
  }
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW); 
    
  }  

  button1State = 0;
  button2State = 0; 
  
  // Print the temperatures to the Serial port
  Serial.print(F("Temp: "));
  Serial.print(temp);
  Serial.print(F(" Target temp: "));
  Serial.println(targetTemp);
  
  // Display the temperature on the LCD
  displayTemps(constrain(temp,0,99),constrain(targetTemp,0,99));
}
/* *******************************************************
*/

/* *******************************************************
/  displayTemps outputs the temperatures to the LCD display
*/
void displayTemps(int tempToDisplay, int targetToDisplay) {
    // Display the settings on the LCD
    lcd.setCursor(0,0); // Start writing characters on the display from position 0 row 0, so top left
    lcd.print(F("Temp "));
    lcd.setCursor(5,0);
    lcd.print(tempToDisplay);
    lcd.print(F(" C"));
    lcd.setCursor(0,1);
    lcd.print(F("Target "));
    lcd.setCursor(7,1);
    lcd.print(targetToDisplay);
    lcd.print(F(" C"));
}
/* *******************************************************
*/

float getTemp(){
 //returns the temperature from one DS18S20 in DEG Celsius

 byte data[12];
 byte addr[8];

 if ( !ds.search(addr)) {
   //no more sensors on chain, reset search
   ds.reset_search();
   return -1000;
 }

 if ( OneWire::crc8( addr, 7) != addr[7]) {
   Serial.println("CRC is not valid!");
   return -1000;
 }

 if ( addr[0] != 0x10 && addr[0] != 0x28) {
   Serial.print("Device is not recognized");
   return -1000;
 }

 ds.reset();
 ds.select(addr);
 ds.write(0x44,1); // start conversion, with parasite power on at the end

 byte present = ds.reset();
 ds.select(addr);  
 ds.write(0xBE); // Read Scratchpad

 
 for (int i = 0; i < 9; i++) { // we need 9 bytes
  data[i] = ds.read();
 }
 
 ds.reset_search();
 
 byte MSB = data[1];
 byte LSB = data[0];

 float tempRead = ((MSB << 8) | LSB); //using two's compliment
 float TemperatureSum = tempRead / 16;
 
 return TemperatureSum;
 
}
