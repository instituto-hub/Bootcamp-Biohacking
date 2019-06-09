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


 PLEASE NOTE:

 The thermocycler is perhaps the most complex device in the BioHack
 Academy thus far. The logic of the code is explained in the lecture.
 Please make sure you understand the logic, before changing the code.

 Once again this code is intended to demonstrate the minimal functions
 of a thermocycler. The code is able to run through multiple cycles of
 3 stage (Denaturing, Annealing, Elongation) PCR cycles.
*/

boolean toggleLidHeater = false; // False = lid heater off, True = lid heater on
int lidTemp = 85;       // Target lid temp

int arduinoType = 0; // Choose: 0 = Arduino UNO; 1 = Arduino Leonardo
int tempSensorType = 0; // Choose: 0 = 10K NTC thermistor; 1 = DS18B20 digital temperature sensor

class RotaryEncoder
{
  byte state;
  byte pin0, pin1;
  int value;

  byte readState()
  {
    return (digitalRead(pin0) == HIGH ? 1u : 0u)
         | (digitalRead(pin1) == HIGH ? 2u : 0u);
  }
public:
  RotaryEncoder(int p0, int p1) :
    pin0(p0), pin1(p1), value(0), state(0) {}
  
  void init()
  {
    pinMode(pin0, INPUT);
    pinMode(pin1, INPUT);
    digitalWrite(pin0, 1);  // enable internal pullup
    digitalWrite(pin1, 1);  // enable internal pullup
    value = 0;
    state = readState();
  }

  bool poll()
  {
    // State transition table
    static char tbl[16] =
    { 0, +1, -1, 0,
      // position 3 = 00 to 11, can't really do anythin, so 0
      -1, 0, -2, +1,
      // position 2 = 01 to 10, assume a bounce, should be 01 -> 00 -> 10
      +1, +2, 0, -1,
      // position 1 = 10 to 01, assume a bounce, should be 10 -> 00 -> 01
      0, -1, +1, 0
      // position 0 = 11 to 10, can't really do anything
    };
  
    byte t = readState();
    char movement = tbl[(state << 2) | t];
    if (movement != 0)
    {
      value += movement;
      state = t;
      return true;
    }
    return false;
  }
  
  int getValue()
  {
    return value;
  }
};


/* *******************************************************
/  Libraries
*/

#include <math.h>   // loads a library with more advanced math functions
#include <LiquidCrystal.h>
#include <OneWire.h>// Needed for the temperature sensors
/* *******************************************************
*/

/* *******************************************************
/  LCD
*/
// Set the LCD address to 0x27 for a 16 chars and 2 line display

#define luzfundo 7
LiquidCrystal lcd(13, 12, 5, 4, 3, 2);

/* *******************************************************
*/

/* *******************************************************
/  Thermocycler Settings
*/
String stageNames[3] = { "Denat", "Anneal", "Elon" }; // Names of Stages
int tempSettings[3] = { 0, 0, 0}; // Temperatures of each stage
int timeSettings[3] = { 0, 0, 0}; // Duration of each stage
int coolSettings[3] = { 1, 0, 0}; // Toggle to enable fan / cooling after stage (only after Denat stage)
int cycleSetting = 0;     // Max number of cycles

// Pins
#define fanPin 5       // The mosfet that drives the 80mm fan is connected to pin 6
#define heatPin 6      // Pin for the mosfet that controls the heating element
#define lidPin 7       // Pin for the mosfet that controls the lid heater

// Temperature read
int val;               // Create an integer variable to temporarily store the thermistor read
double currentTemp;    // Variable to hold the current temperature value
double currentLidTemp; // Variable to hold the current lid temperature value
#define TempPin1 9     // DS18S20 Signal pin on digital 9
#define TempPin2 10    // DS18S20 Signal pin on digital 10

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire1(TempPin1);
OneWire oneWire2(TempPin2);

// Pass our oneWire reference to Dallas Temperature. 
OneWire tempSensor1(&oneWire1);
OneWire tempSensor2(&oneWire2);

// PCR cycling variables
int stageTemp = 0;      // Target temperature of the current stage
int stageTime = 0;      // Duration of current stage
int cycleCounter = 0;   // Counter of number of cycles completed
int currentState = 0;   // 3 states: Denat, Anneal and Elon
unsigned long currentStageStartTime = 0; // Beginning of the current Stage
int currentStage = 0;   // In each stage, go through 3 states: Ramping, Steady, Cooling
int toggleCooling = 0;  // Toggle to skip or execute Stage 3: Cooling
boolean showtime = false; // Display time on display

/* *******************************************************
*/

/* *******************************************************
/  Set the initial STATE of the machine
/  In this code we will switch operation modes, from (programming time, to programming temp) x3, to cycling, to stopping/slowing down
*/
#define STATE_DENAT_TIMEPROG 1
#define STATE_DENAT_TEMPPROG 2
#define STATE_ANNEAL_TIMEPROG 3
#define STATE_ANNEAL_TEMPPROG 4
#define STATE_ELON_TIMEPROG 5
#define STATE_ELON_TEMPPROG 6
#define STATE_CYCLESPROG 7
#define STATE_CYCLING 8
#define STATE_STOP 9

byte state = STATE_DENAT_TIMEPROG;
/* *******************************************************
*/

/* *******************************************************
/  Machine User Interface
*/
boolean buttonState = 0;    // Start button
boolean lastButtonState = false; // Debounce variable
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 500;    // the debounce time; increase if the output flickers
int ledstate = false;       // Blinking indicator LED

// Pins
#define buttonPin 11   // the number of the pushbutton pin
#define ledPin 13      // the number of Arduino's onboard LED pin
/* *******************************************************
*/

/* *******************************************************
/  Rotary Encoder
*/

volatile int lastEncoded = 0;
volatile long encoderValue = 0;

long lastencoderValue = 0;

int lastMSB = 0;
int lastLSB = 0;
/* *******************************************************
*/

/* *******************************************************
/  Variables needed for keeping track of time
*/
uint32_t lastTick = 0;  // Global Clock variable
int LCDTime = 0;        // Time tracker for LCD update

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
 
/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  
/* *******************************************************
*/

#if arduinoType == 0 
    RotaryEncoder rotaryEncoderA (2,3);
#endif
#if arduinoType == 1
    RotaryEncoder rotaryEncoderA (0,1);
#endif

/* *******************************************************
/  Setup function, this code is only executed once
*/
void setup() {
  // Encoder pins
  rotaryEncoderA.init();
  
  // Update clock
  lastTick = millis();

  // Open serial connection with the computer and print a message
  Serial.begin(9600);
  Serial.println(F("BioHack Academy Thermocycler"));

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  // rotary encoder as an input
  /*
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
  attachInterrupt(1, updateEncoder, CHANGE);  */

  // fan and heating and set low
  pinMode(fanPin, OUTPUT);
  pinMode(heatPin, OUTPUT);
  pinMode(lidPin, OUTPUT);
  digitalWrite(fanPin, LOW);
  digitalWrite(heatPin, LOW);
  digitalWrite(lidPin, LOW);

  // Start up the temperature sensor library
 // tempSensor1.begin();
 // tempSensor2.begin();
  
  // Initialize the LCD and print a message
  lcd.begin(16, 2);
  pinMode(luzfundo,OUTPUT);
  digitalWrite(luzfundo,HIGH);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("BioHack Academy"));
  lcd.setCursor(0,1);
  lcd.print(F("Thermocycler"));
  delay(1000);
  lcd.clear();
}
/* *******************************************************
*/

/* *******************************************************
/  Thermistor function converts the raw signal into a temperature
/  NOTE: no longer used
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
/  Loop function, this code is constantly repeated
*/
void loop() {
  // Update clock
  uint32_t time = millis();     // current time since start of sketch
  uint16_t dt = time-lastTick;  // difference between current and previous time
  lastTick = time;
  
  // Poll rotary encoder
  if (rotaryEncoderA.poll()) {
    Serial.print("Encoder A: ");
    Serial.println(rotaryEncoderA.getValue());
    encoderValue = rotaryEncoderA.getValue();
  }

     val=analogRead(A0);            //Read the Analog port 0 and store the value in val
     currentTemp=Thermister(val);  //Runs the fancy math function on the raw analog value
     val=analogRead(1);            //Read the Analog port 1 and store the value in val
     currentLidTemp=Thermister(val);  //Runs the fancy math function on the raw analog value
    
  // Print temperature to computer via Serial
  Serial.print("Temperature: ");
  Serial.println(currentTemp);
  //Serial.print("Lid Temperature: ");
  //Serial.println(currentLidTemp);

  // Check whether the button is pressed using debounce timer
  int reading = digitalRead(buttonPin);

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == 1) {
      buttonState = 1;
      lastDebounceTime = millis();
    }
  }
  else {
    buttonState = 0;
  }
  
  // Blink the LED, indicating that the Arduino is working
  if (ledstate == false) {
    digitalWrite(ledPin, HIGH); // turn LED on
    ledstate = true;
  }
  else {
    digitalWrite(ledPin, LOW); // turn LED off
    ledstate = false;
  }
  
  // Do machine logic
  machineUpdate(dt);
  
  // Reset button state
  buttonState = 0;

  // Wait 10 microsconds before restarting the loop
  delay(10);
}
/* *******************************************************
*/

/* *******************************************************
/  machineUpdate, this function checks in which STATE the device is and executes the code that belongs to that STATE
/  starting with STATEs to allow the user to set the PCR paramters of the device, such as temperate and time of each stage and the number of cycles
/  the next STATE is to execute the PCR protocol
/  final STATE to stop the machine and reset the settings
*/
void machineUpdate(uint16_t dt) {

  // STATE_DENAT_TIMEPROG is the first state in which the user can set the time of the Denat stage should last
  if(state == STATE_DENAT_TIMEPROG) {
    
    // Sanitize the values of the Rotary encoder, no less than 0, no more than 100
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 120) encoderValue = 120;
  
    // Convert encoder value to seconds  
    timeSettings[0] = encoderValue;

    // Display time setting on the LCD
    lcd.setCursor(0,0);
    lcd.print(F("Denat Time"));
    lcd.setCursor(11,0);
    lcd.print(time(timeSettings[0]));    

    // In case the button is pressed, continue to next state
    if(buttonState > 0) {
      stateChange(STATE_DENAT_TEMPPROG);
      encoderValue = 0; // reset encoderValue
    }  
  } 
 
  // STATE_DENAT_TEMPPROG is similar to STATE_DENAT_TIMEPROG, but now the user can set the temperature of the Denat stage
  if(state == STATE_DENAT_TEMPPROG) {
    
    // Sanity check
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 100) encoderValue = 100;
  
    tempSettings[0] = encoderValue;

    // Display the settings on the LCD
    lcd.setCursor(0,1);
    lcd.print(F("Denat Temp"));
    lcd.setCursor(11,1);
    lcd.print(encoderValue);
    lcd.print("  ");

    // Continue to next state if the button is pressed
    if(buttonState > 0) {
      stateChange(STATE_ANNEAL_TIMEPROG);
      lcd.clear(); // reset LCD screen
      encoderValue = 0; // reset encoderValue
    }     
  } 

  // now the user can set the time of the Anneal stage should last
  if(state == STATE_ANNEAL_TIMEPROG) {
    
    // Sanitize the values of the Rotary encoder, no less than 0, no more than 100
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 120) encoderValue = 120;
  
    // Convert encoder value to seconds  
    timeSettings[1] = encoderValue;

    // Display time setting on the LCD
    lcd.setCursor(0,0);
    lcd.print(F("Anneal Time"));
    lcd.setCursor(11,0);
    lcd.print(time(timeSettings[1]));    

    // In case the button is pressed, continue to next state
    if(buttonState > 0) {
      stateChange(STATE_ANNEAL_TEMPPROG);
      encoderValue = 0; // reset encoderValue
    }  
  } 
 
  // now the user can set the temperature of the Anneal stage
  if(state == STATE_ANNEAL_TEMPPROG) {
    
    // Sanity check
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 100) encoderValue = 100;
  
    tempSettings[1] = encoderValue;

    // Display the settings on the LCD
    lcd.setCursor(0,1);
    lcd.print(F("Anneal Temp"));
    lcd.setCursor(11,1);
    lcd.print(encoderValue);
    lcd.print("  ");

    // Continue to next state if the button is pressed
    if(buttonState > 0) {
      stateChange(STATE_ELON_TIMEPROG);
      lcd.clear(); // reset LCD screen
      encoderValue = 0; // reset encoderValue
    }     
  } 

  // now the user can set the time of the Elon stage should last
  if(state == STATE_ELON_TIMEPROG) {
    
    // Sanitize the values of the Rotary encoder, no less than 0, no more than 100
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 120) encoderValue = 120;
  
    // Convert encoder value to seconds  
    timeSettings[2] = encoderValue;

    // Display time setting on the LCD
    lcd.setCursor(0,0);
    lcd.print(F("Elon Time"));
    lcd.setCursor(11,0);
    lcd.print(time(timeSettings[2]));    

    // In case the button is pressed, continue to next state
    if(buttonState > 0) {
      stateChange(STATE_ELON_TEMPPROG);
      encoderValue = 0; // reset encoderValue
    }  
  } 
 
  // now the user can set the temperature of the Elon stage
  if(state == STATE_ELON_TEMPPROG) {
    
    // Sanity check
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 100) encoderValue = 100;
  
    tempSettings[2] = encoderValue;

    // Display the settings on the LCD
    lcd.setCursor(0,1);
    lcd.print(F("Elon Temp"));
    lcd.setCursor(11,1);
    lcd.print(encoderValue);
    lcd.print("  ");

    // Continue to next state if the button is pressed
    if(buttonState > 0) {
      stateChange(STATE_CYCLESPROG);
      lcd.clear(); // reset LCD screen
      encoderValue = 0; // reset encoderValue
    }     
  } 

  // now the user can set the number of PCR cycles
  if(state == STATE_CYCLESPROG) {
    
    // Sanity check
    if(encoderValue < 0) encoderValue = 0;
    if(encoderValue > 100) encoderValue = 100;
  
    cycleSetting = encoderValue;

    // Display the settings on the LCD
    lcd.setCursor(0,1);
    lcd.print(F("Cycles"));
    lcd.setCursor(8,1);
    lcd.print(encoderValue);
    lcd.print("  ");

    // Continue to next state if the button is pressed
    if(buttonState > 0) {
      
      stateChange(STATE_CYCLING);
      lcd.clear(); // reset LCD screen
      encoderValue = 0; // reset encoderValue
      currentState = 1; // start at first state ramp up, steady, cool
      currentStage = 0; // start at first stage denat, anneal, elon
      cycleCounter = 1; // start at first cycle
    }     
  } 
 
  // State Cyling is the state in which the thermocycler is running
  if(state == STATE_CYCLING) {

    LCDTime += dt; // Update LCD once every second
    if(LCDTime > 1000) {
      LCDTime = 0;
      
      // Print to LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("C "));
      lcd.print(cycleCounter);
      lcd.print("/");
      lcd.print(cycleSetting);
      lcd.print(" ");
      if(currentStage == 0) lcd.print("D ");
      if(currentStage == 1) lcd.print("A ");
      if(currentStage == 2) lcd.print("E ");
      if(showtime) { lcd.print(round((stageTime-(millis()-currentStageStartTime))/1000)); }
      lcd.setCursor(0,1);
      lcd.print(F("Temp "));
      lcd.print(round(currentTemp));      
      Serial.print("temp on LCD: ");
      Serial.println(round(currentTemp));
      lcd.print("/");
      lcd.print(stageTemp);
      lcd.print("  ");
      lcd.print(currentLidTemp);
    }
    
    /* Debug info
    Serial.print("stage"); Serial.println(currentStage); // denat, anneal, elon
    Serial.print("state"); Serial.println(currentState); // ramp up, steady, cool
    Serial.print("cycle"); Serial.println(cycleCounter);
    Serial.print("stageTemp"); Serial.println(stageTemp);
    
    Serial.print("tempsettings 0");
    Serial.print(tempSettings[0]);
    Serial.print("tempsettings 1");
    Serial.print(tempSettings[1]);
    Serial.print("tempsettings 2");
    Serial.print(tempSettings[2]);
    Serial.print("timesettings 0");
    Serial.print(timeSettings[0]);
    Serial.print("timesettings 1");
    Serial.print(timeSettings[1]);
    Serial.print("timesettings 2");
    Serial.print(timeSettings[2]);
    */

    if(cycleCounter < cycleSetting) { // Check whether we have not completed all cycles
      // If not, go through 3 PCR stages: Denat, Anneal and Elon
      if(currentStage == 3) {
        currentStage = 0;
        cycleCounter++; // After completion of all three PCR stage, add 1 to the cycleCounter
      }
      stageTemp = tempSettings[currentStage]; // set the PCR stage target temperature
      stageTime = timeSettings[currentStage] * 1000; // set the PCR stage time
      toggleCooling = coolSettings[currentStage]; // set whether the machine needs to cool after completing the PCR stage
    }
    else { // all cycles are done!

      // Print a message to the LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Done! "));
      delay(1000);
      lcd.clear();
      
      stateChange(STATE_STOP);
    }
    
    // Change state if the user presses the button
    if(buttonState > 0) {
      stateChange(STATE_STOP);
      lcd.clear();
    }     
  }

  // StateStop stops the cycling
  if(state == STATE_STOP) {

    // Reset the PCR settings
    tempSettings[0] = 0; // Denat temp
    timeSettings[0] = 0; // Denat time
    tempSettings[1] = 0; // Anneal temp
    timeSettings[1] = 0; // Anneal time
    tempSettings[2] = 0; // Elon temp
    timeSettings[2] = 0; // Elon time
    cycleSetting = 0;    // Max number of cycles

    // PCR cycling variables
    stageTemp = 0;       // Target temperature of the current stage
    stageTime = 0;       // Duration of current stage
    cycleCounter = 0;    // Counter of number of cycles completed
    currentStage = 0;    // Go through 3 stages: Denat, Anneal and Elon
    currentStageStartTime = 0; // Beginning of the current Stage
    currentState = 0;    // In each stage, go through 3 states: Ramping, Steady, Cooling
    toggleCooling = 0;   // Toggle to skip or execute State 3: Cooling

    // Stop heating and fan
    digitalWrite(fanPin, LOW);
    digitalWrite(heatPin, LOW);
    digitalWrite(lidPin, LOW);
    
    // Go back to the first state
    stateChange(STATE_DENAT_TIMEPROG);
  }

/* *******************************************************
/  The actual PCR cycles
/  The code above manages the 3 stages: Denat, Anneal, Elongate
/  Now we need to go through 3 states: Ramping the temperature up, maintain a Steady State, and Cooling if necessary
*/
    
  if(currentState == 1) {
    // RAMPING UP
    if(currentTemp < stageTemp - 5) {
      digitalWrite(heatPin, HIGH);
      digitalWrite(fanPin, LOW);   
      showtime = false;   
    } 
    else if(currentTemp < stageTemp - 2){
      analogWrite(heatPin, 200);
      digitalWrite(fanPin, LOW);
      showtime = false;
    }
    else {
      Serial.println(F("Reached Steady State"));
      currentStageStartTime = millis(); // Set timer
      showtime = true;
      currentState = 2; // Continue STEADY STATE stage
    }
  }

  if(currentState == 2) { 
    //STEADY STATE
    if(millis()-currentStageStartTime < stageTime){ // Check whether we completed the state
      if(currentTemp > stageTemp){ // Temperature too high, so switch off heater
        digitalWrite(heatPin, LOW);
      }else{ // Temperature too low, so turn on heater
        digitalWrite(heatPin, HIGH);
      }
    }
    else {
      Serial.print(F("Steady state finished. Temp: ")); 
      Serial.println(currentTemp);
      digitalWrite(heatPin, LOW);
      currentState = 3; // Continue to COOLING stage
    }
  }

  if(currentState == 3) {
    // COOLING
    showtime = false;
    // Set target temp of the next stage
    stageTemp = tempSettings[1];
    
    if(currentTemp > stageTemp && toggleCooling == 1) { // Check whether we need to cool
      Serial.println(F("Cooling down"));
      digitalWrite(fanPin, HIGH);
    }
    else {
      Serial.println(F("currentStage done"));
      digitalWrite(fanPin, LOW);
      
      currentState = 1; // Back to RAMPING UP state
      currentStage ++; // Go from Denat, to Anneal to Elon
    }
  }

  if(currentState > 0 && toggleLidHeater) {
    // LID HEATER
    if(currentLidTemp < lidTemp - 5) {
      digitalWrite(lidPin, HIGH); 
    } 
    else if(currentLidTemp < lidTemp - 2){
      analogWrite(lidPin, 200);
    }
    else {
      digitalWrite(lidPin, LOW);
    }
  }


/* *******************************************************
*/
  
}
/* *******************************************************
*/

/* *******************************************************
/  stateChange switches the machine logic from one state to another
*/
void stateChange(byte newstate) {
  // set new state
  state = newstate;
  
  // reset button
  buttonState = 0;
}
/* *******************************************************
*/

/* *******************************************************
/  time converts seconds to minutes:seconds format
*/
String time(int val){  
  // calculate number of days, hours, minutes and seconds
  int days = elapsedDays(val);
  int hours = numberOfHours(val);
  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);
            
  String returnval = "";
            
  // digital clock display of current time 
  returnval = printDigits(minutes) + ":" + printDigits(seconds) + "   ";
  
  // return value      
  return returnval;
}
/* *******************************************************
*/

/* *******************************************************
/  printDigits adds an extra 0 if the integer is below 10
*/
String printDigits(int digits){
  // utility function for digital clock display: prints colon and leading 0
  String returnval = "";
  if(digits < 10)
    returnval += "0";
  returnval += digits; 
         
  return returnval; 
}
/* *******************************************************
*/

/* *******************************************************
/  updateEncoder is the function that reacts to the rotary encoder interrupts
*/
/*
void updateEncoder(){
  Serial.println("test");
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) { encoderValue --; Serial.println("min"); }
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) { encoderValue ++; Serial.println("plus"); }

  lastEncoded = encoded; //store this value for next time
}
/* *******************************************************
*/
