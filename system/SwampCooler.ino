#include "dht.h"
#include <LiquidCrystal.h>
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Wire.h>

int waterLevel = 0;                                                 //water level initialization
int Spin = A0;                                                      //water level sensor pin

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);                            // LCD display
#define dht_analogPin A1                                            // humidity/temperature sensor
dht DHT;                                                            // analog pin A1

int motor = 4;                                                      // enable motor at pwm pin 1
int motorIN_1 = 3;                                                  // setup direction M1 direction pin 3
int motorIN_2 = 2;                                                  // setup direction M1 direction pin 2

const int buttonPin = 50;                                            //push button pin to be read from digital I/O pin 9
int buttonState = 0;

                                                                    // setting up LED pins
int latchPin = 6;                                                   // Latch pin (ST_CP)
int clockPin = 7;                                                   // clock pin (SH_CP) 
int dataPin = 5;                                                    // data pin (DS)


int ledPinYellow;                                         //LED Disabled
int ledPinGreen;                                          //LED IDLE
int ledPinRed;                                            //LED Error
int ledPinBlue;                                           //LED Running

const char *monthName[12] = {                                       
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;                                                    //time

bool getDate(const char *str)                                    
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm.Day = Day;
  tm.Month = monthIndex + 1;
  tm.Year = CalendarYrToTm(Year);
  return true;
}

bool getTime(const char *str)                            
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm.Hour = Hour;
  tm.Minute = Min;
  tm.Second = Sec;
  return true;
}

boolean shiftRegister[8];
void writeReg()
{
  digitalWrite(clockPin, LOW);
  
  for (int i = 0; i<4; i++)
  {
    digitalWrite(latchPin, LOW);
    digitalWrite(dataPin, shiftRegister[i]);
    digitalWrite(latchPin, HIGH);
  }
  digitalWrite(clockPin, HIGH);
}

void setup() {

  lcd.begin(16, 2);                                                 //LCD display
  
  pinMode(motor, OUTPUT);                                           //motor
  pinMode(motorIN_1, OUTPUT);                                       //direction of motor
  pinMode(motorIN_2, OUTPUT);
  digitalWrite(motorIN_1, LOW);                                     // motor should be off initially
  digitalWrite(motorIN_2, LOW);

  pinMode(latchPin,  OUTPUT);                                       //  latch pin 
  pinMode(dataPin,  OUTPUT);                                        //  data pin 
  pinMode(clockPin, OUTPUT);                                        // clock pin o
  pinMode(buttonPin, INPUT);                                        // push button

  bool parse=false;                                                 // boolean variables used for time functions
  bool config=false;


  if (getDate(__DATE__) && getTime(__TIME__)) {                     // record the date and time
    parse = true;
    if (RTC.write(tm)) {
      config = true;
     }
    }

    Serial.begin(9600);       
    while (!Serial) ;
    // print the date and time 
    lcd.setCursor(0,0);
    lcd.print("Time is: ");
    lcd.setCursor(0,1);
    lcd.print(__TIME__);
    delay(2000);
    lcd.setCursor(0,0);
    lcd.print("Today's date:");
    lcd.setCursor(0,1);
    lcd.print(__DATE__);
    delay(2000);
    lcd.clear();
  
} // end of initial setup

void loop() {
  // begin program
  
    digitalWrite(latchPin, LOW);
    digitalWrite(dataPin, shiftRegister[2]);
    digitalWrite(latchPin, HIGH);
 
  buttonState = digitalRead(buttonPin);                             //on and off
  buttonState = HIGH;
 
  waterLevel = analogRead(Spin);                                    //water sensor analog pin                                     


  //Humidity/Temperature
  DHT.read11(A2);
  lcd.setCursor(0,0); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature);
  lcd.print((char)223);                  
  lcd.print("C");
  lcd.setCursor(0,1); 
  lcd.print("Temp: ");
  lcd.print(DHT.temperature*9/5 + 32 );   
  lcd.print((char)223);                   
  lcd.print("F");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Humidity: ");
  lcd.setCursor(0,1);
  lcd.print(DHT.humidity);
  lcd.print("%");
  delay(3000);
  lcd.clear();
  
  
  // Motor threshold test code mock up
  if(DHT.temperature >= 20)                                         // when the temperature is above 20 degress Celsius,
  {                                                                // motor will pick up speed
    digitalWrite(motor, 200);
    digitalWrite(motorIN_1, HIGH);
    digitalWrite(motorIN_2, LOW);
  }
  else                                                             // else, motor will still run around 100 when idle
  {
    digitalWrite(motor, 0);
    digitalWrite(motorIN_1, HIGH);
    digitalWrite(motorIN_2, LOW);
  }

  if(buttonState == LOW)                      //disabled
  {
    lcd.setCursor(0,0);                                               // display system state indicator on LCD
    lcd.print("SWAMP COOLER");
    lcd.setCursor(0,1);
    lcd.print("MODE: DISABLED");
    delay(3000);
    lcd.clear();
    //bitSet(leds, ledPinYellow);                                       // Turn on Yellow LED 
    //updateShiftRegister();
    //bitClear(leds, 0);                                              // Turn off other LEDs
    //updateShiftRegister();                                          // reflect the state of the LEDs
    digitalWrite(motor, 0);                                           // turn off motor
    digitalWrite(motorIN_1, LOW);                                     
    digitalWrite(motorIN_2, LOW);
  }
  //else                                        //idle
  //{
    //bitSet(leds, ledPinGreen);                                        // Turn on Green LED for Idle until cooler starts running
    //bitClear(leds, 0);                                                // Turn off other LEDs
    //updateShiftRegister();                                            // reflect the state of the LEDs
  //}

  
  if(waterLevel <= 100)                       //error           
  {
         //bitSet(leds, ledPinRed);                                     // Turn on Red LED
         //bitClear(leds, 0);                                           // clear other LEDs
         //updateShiftRegister();                                       // reflect the state of the LEDs

         digitalWrite(motor, 0);                                      // 0 is the motor not moving, change up to 255 to switch
         digitalWrite(motorIN_1, LOW);                                // to switch the speed
         digitalWrite(motorIN_2, LOW);
         
         lcd.setCursor(0,0);
         lcd.print("ERROR: WATER");                             // indicate water level on LCD display
         lcd.setCursor(0,1);
         lcd.print("LEVEL IS LOW");
         delay(3000);
         lcd.clear();
  }
  
  else if (waterLevel > 100 && waterLevel <= 300){
         lcd.print("WATER: LOW");                             // indicate water level on LCD display
         delay(3000);
         lcd.clear();                                // indicate other water level ranges on display
  }
  else if (waterLevel > 300 && waterLevel <= 330){
         lcd.print("WATER: MEDIUM");
         delay(3000);
         lcd.clear();
  }
  else if (waterLevel > 330){
         lcd.print("WATER: HIGH");
         delay(3000);
         lcd.clear();
  }
  //delay(3000);
  

  if(DHT.temperature > 30)                     //run
  {
     lcd.setCursor(0,0);                                             // display system indicator on LCD
     lcd.print("SWAMP COOLER");
     lcd.setCursor(0,1);
     lcd.print("MODE: RUNNING");
     delay(3000);
     lcd.clear();
     //bitSet(leds, ledPinBlue);                                       // Blue LED for running
     //bitClear(leds, 0);                                              // Clear rest of LEDs
     //updateShiftRegister();                                          // reflect the state of the LEDs
  }

  
  //delay(3000);                                                      //setup normal delay
} // end of loop
