#include <Math.h>
//#include <SPI.h>
#include "Adafruit_MAX31855.h"

#define StartBTN_PIN 7
#define EffektBTN_PIN 8
#define Pump_PIN 12
#define Fan_PIN 9
#define HeatPlug_PIN 6
#define OverHeatSensor_PIN 10


// Thermocouple digital IO pins.
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5

// initialize the Thermocouple
Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);


//Definer variabler
unsigned long warmup_time = 270000; // time the Heaplugg are turnd on
unsigned long FanDry_time = 8000; //Time the fan should run to ventilate the heat chamber

int RunningTemperatur = 250; //Temperature off fire indicates the burning are OK
int CoolDownTemperatur = 40; //Temperature Cooldown sequence OK


// On and Off Times (as int, max=32secs)
const unsigned int PumpPulseTime = 150;
const unsigned int PumpWaitStdTime = 1000;
unsigned int PumpWaitTime = 0;

//Pumpspeed adjustment potentiometer
int PumpPotPin = 0;    // select the input pin for the potentiometer
unsigned int PumpPotVal = 0; 

//Pumpspeed adjustment potentiometer Control Panel
int PumpPotControlPin = 1;    // select the input pin for the potentiometer
float PumpPotControlVal = 0; 

//TempControl
int OverHeatSensorActive = 0;
int OverHeatStatus = 0;
 
// Interval is how long Pump puls should last
int interval = PumpPulseTime;

// Used to track if Pump should be on or off
boolean PumpState = true;

//Fan setting
int FanSpeed=0;


// Tracks the last time event fired
unsigned long previousMillis = 0;
unsigned long StartBTNPressedMillis = millis();

//Button status
int StartButtonState = 1;
int LastStartButtonState = 0;
int EffektButtonState = 1;
int StartSequenceFinish = 0;

int CooldownSequenceActive = 0;
int DryBurnerComplete = 0;
 
void StartHeater(){

//If cooldown active or overheat triggerd
if (CooldownSequenceActive == 1 || OverHeatStatus == 1){
 // Serial.print("CooldownSequenceActive: ");
 // Serial.println(CooldownSequenceActive);
 // Serial.println("---");
 // Serial.print("OverHeatStatus: ");
 // Serial.println(OverHeatStatus);
  return;
}


//Serial.println("StartHeater()");
//Serial.println(StartBTNPressedMillis);

//Serial.print("warmup_time: ");
//Serial.println(warmup_time);
//Serial.print("StartSequenceFinish: ");
//Serial.println(StartSequenceFinish);
//Serial.print("CooldownSequenceActive: ");
//Serial.println(CooldownSequenceActive);
  
  // Compare to BTNPressed time to see if enough time has passed
  if (((unsigned long)((millis() - StartBTNPressedMillis) <= warmup_time)) && StartSequenceFinish == 0 && CooldownSequenceActive == 0) {
      if(StartButtonState == 0){
      //Serial.println("HeatRun(1)");
      HeatRun(1);
      if(DryBurnerComplete == 1) {
        digitalWrite(HeatPlug_PIN, 1);  //Activate Heatplugg relay
      }
      }
      }  
     else  
      {
          digitalWrite(HeatPlug_PIN, 0);  // Update the actual LED
          //Serial.println("Heat Sequence finish");
          StartSequenceFinish = 1;    
      }
  }

void CoolDown(){
//Serial.print("CooldownSequenceActive: ");
//Serial.println(CooldownSequenceActive);




  if(CooldownSequenceActive == 1){
  Serial.println("Stop Pump! ");
  digitalWrite(Pump_PIN, 1);  // Stop Pump'
  digitalWrite(HeatPlug_PIN, 0);  //Stop heatplugg
  FanSpeed=255;
  analogWrite(Fan_PIN,FanSpeed); //set fanspeed to max to cooldown fast
  Serial.println("Vifte Kjører full hastighet");
   //Serial.println("Cooldown StartSequenceFinish to 0: ");
  StartSequenceFinish = 0;
  DryBurnerComplete = 0;

}

}

void HeatRun(int x){

//If cooldown active or overheat triggerd
if (CooldownSequenceActive == 1 || OverHeatStatus == 1){
  //Serial.print("CooldownSequenceActive: ");
  //Serial.println(CooldownSequenceActive);
  //Serial.println("---");
  //Serial.print("OverHeatStatus: ");
  //Serial.println(OverHeatStatus);
  return;
}


   // Read PotValue and adjust time accordenly
    PumpPotVal = analogRead(PumpPotPin);    // read the value from the sensor
    PumpWaitTime = PumpWaitStdTime + PumpPotVal - 500;

if (((unsigned long)((millis() - StartBTNPressedMillis) <= FanDry_time)) && x == 1) {
   
  //kjør vifte ved oppstart før fuel
   FanSpeed=255;
   analogWrite(Fan_PIN,FanSpeed);
   Serial.println("Vifte Kjører redusert hastighet");
   return;
}else{
   DryBurnerComplete = 1; //Dry Burner OK.
}


if (StartSequenceFinish == 1){
   //Sjekk om start er ok og om bryter er slått på
   // Read Status Effekt Button, if on set value to 50%
  EffektButtonState = digitalRead(EffektBTN_PIN);
    if (EffektButtonState == 0){
    PumpWaitTime = (PumpWaitTime * 1.5);
    FanSpeed=255*0.7;
    } 
}else{
  FanSpeed=255; //kjører max vifte i oppstart
}

 // Serial.println("Heat Run");

//Justering hastighet + (ikke hvis lav effet er aktivert.
if(EffektButtonState == 1) {

// Read PotControlValue and adjust % lower
    
    PumpPotControlVal = analogRead(PumpPotControlPin);    // read the value from the sensor
    //PumpOffTime = PumpOffStdTime + PumpPotVal - 500;

    //Pump adjust % lower
    PumpPotControlVal = ((1-((PumpPotControlVal+1)/2048)*2)+1);
    float z = PumpWaitTime * PumpPotControlVal;
    int a = (int) round (z);
    PumpWaitTime = a;

    //Fan adjust % lower
    float y = 255 * PumpPotControlVal/2;
    int b = (int) round (y);
    FanSpeed=b;

   // Serial.println("Effekt redusert");

//Vifte må styres tilsvarende effekt /Se på om det kan gjøres ulinjært

}


  analogWrite(Fan_PIN,FanSpeed); //Run Fan
  
  // Set Pin 13 to state of PumpState each timethrough loop()
  // If PumpState hasn't changed, neither will the pin
  digitalWrite(Pump_PIN, PumpState);
 
  // Grab snapshot of current time, this keeps all timing
  // consistent, regardless of how much code is inside the next if-statement
  unsigned long currentMillis = millis();
 
  // Compare to previous capture to see if enough time has passed
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    // Change wait interval, based on current LED state
    if (PumpState) {
      // LED is currently off, set time to stay off
      interval = PumpPulseTime;
    } else {
      // LED is currently on, set time to stay on
      interval = PumpWaitTime;
    }
    // Toggle the LED's state, Fancy, eh!?
    PumpState = !(PumpState);
 
    // Save the current time to compare "later"
    previousMillis = currentMillis;
  }
 }

 


void TempControl(){


OverHeatSensorActive = digitalRead(OverHeatSensor_PIN); //read overheatsensor normal closed


//endre etter start
if(OverHeatSensorActive == 0) {
  Serial.println("Overheat Active");
OverHeatStatus=1;
CooldownSequenceActive = 1;
return;
  
}

//--------------------------------------------------------------------------
   double MessuredTemp = thermocouple.readCelsius();
   if (isnan(MessuredTemp)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C = ");
     Serial.println(MessuredTemp);
   }

//--------------------------------------------------------------------------


//sjekk temp sensor ikke mottatt enda før aktivering av denne.
//Serial.println("Cooldown finish:");

if(MessuredTemp <= CoolDownTemperatur || CooldownSequenceActive == 1){

  FanSpeed=0; 
  analogWrite(Fan_PIN,FanSpeed); //Stop fan
  CooldownSequenceActive = 0;

  
}

CooldownSequenceActive = 0;
  
}

// Usual Setup Stuff
void setup() {
  Serial.begin(9600); // open the serial port at 9600 bps:
  while (!Serial) delay(1);
  
  pinMode(Pump_PIN, OUTPUT);
  digitalWrite(Pump_PIN, 1);  // Stop Pump'
  pinMode(StartBTN_PIN, INPUT_PULLUP);
  pinMode(EffektBTN_PIN, INPUT_PULLUP);
  pinMode(OverHeatSensor_PIN, INPUT_PULLUP);
  pinMode(Pump_PIN, OUTPUT);
  pinMode(Fan_PIN, OUTPUT);
  analogWrite(Fan_PIN, 0);
  TCCR1B = TCCR1B & B11111000 | B00000101;    // set timer 1 divisor to  1024 for PWM frequency of    30.64 Hz
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
  Serial.println("DONE.");

  
}
 
void loop() {

  TempControl();
  CoolDown();

//Serial.print("Overoppheting:");
//Serial.println(OverHeatStatus);
  
 StartButtonState = digitalRead(StartBTN_PIN); //read on/off button

//set time of button changed (used in StartHeater) 
  if(StartButtonState != LastStartButtonState){
    StartBTNPressedMillis = millis();
    //Serial.print("StartButtonChange:");
    //Serial.println(StartButtonState);

    if(StartButtonState == 1){
    //Serial.println("CoolDown aktivert");
    CooldownSequenceActive=1;
    CoolDown();
    if(OverHeatStatus == 1) {
      //Serial.println("OverheatStatus = 0");
      OverHeatStatus = 0;
    }
    }

    LastStartButtonState = StartButtonState;

 }
 
 if(StartButtonState == 0){
  //Serial.println("StartHeater");
  //Serial.println(StartButtonState);
    StartHeater();
    if (StartSequenceFinish == 1) {
      HeatRun(2);
    }
 } 
}
