#include "dht.h"
#include <LiquidCrystal.h>
#include <Time.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Wire.h>

int waterLevel = 0;                                                 //water level initialization
int Spin = A0;                                                      //water level sensor pin

LiquidCrystal lcd(8, 9, 10, 11, 12, 13);                            // LCD display
#define dht_analogPin A1                                            // directing the humidity/temperature sensor to
dht DHT;                                                            // analog pin A1

int motor = 4;                                                      // enable motor at pwm pin 1
int motorIN_1 = 3;                                                  // setup direction M1 direction 0/1 of motor at pwm pin 3
int motorIN_2 = 2;                                                  // setup direction M1 direction 1/0 of motor at pwm pin 2

const int buttonPin = 50;                                            //push button pin to be read from digital I/O pin 9
int buttonState = 0;

                                                                    // setting up LED pins through 74HC595 IC
int latchPin = 6;                                                   // Latch pin (ST_CP) defined at digital I/O pin 6 on Arduino
int clockPin = 7;                                                   // clock pin (SH_CP) defined at digital I/O pin 7 on Arduino
int dataPin = 5;                                                    // data pin (DS) defined at digital I/O pin 5 on Arduino


int ledPinYellow;                                         //LED Disabled
int ledPinGreen;                                          //LED IDLE
int ledPinRed;                                            //LED Error
int ledPinBlue;                                           //LED Running

const char *monthName[12] = {                                       // establish time elements for RTC module
  "Jan", "Feb", "Mar", "Apr", "May", "Jun",
  "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

tmElements_t tm;                                                    // variable used for time

bool getDate(const char *str)                                       // function utilized from DS1307RTC/examples/SetTime/SetTime.ino
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

//test

void setup() {
    // setup the UART
    Serial.begin(9600);

    // setup the ADC
    adcInit();

    //setup the Servo
    servo.attach(9);
    servo.write(fanAngle);

    //setup the fan
    pinModeB(FAN_ENABLE, OUTPUT);
    pinModeB(DIRA, OUTPUT);
    pinModeB(DIRB, OUTPUT);
    digitalWriteB(DIRA, HIGH);

    //setup on/off button
    pinModeB(SYSTEM, INPUT_PULLUP);

    // setup the LCD:
    lcd.init();
    lcd.backlight();

    Serial.print("Set Fan Angle (45 - 135)\n");
}

void loop() {
    lastButtonState = currentButtonState;      // save the last state
    currentButtonState = digitalReadB(SYSTEM); // read new state

    if(lastButtonState == HIGH && currentButtonState == LOW) {      
        enable = !enable;
        if(enable) {
            Serial.println("System On");
        }
        else {
            Serial.println("System Off");

            lcd.setCursor(0,0);
            lcd.print(".               ");

            lcd.setCursor(0, 1);
            lcd.print(".               ");

            digitalWriteB(FAN_ENABLE, LOW);
        }
    }
    if(enable) {
        
        waterLevel = adcRead(WaterSensorPin);
        
        if(Serial.available() > 0) {
            String fanAngleS = Serial.readString();
            fanAngle = fanAngleS.toInt();
            Serial.print(fanAngleS);
            if(fanAngle <= 135 && fanAngle >= 45) {
                servo.write(fanAngle);
                Serial.print("Set Fan Angle (45 - 135)\n");
            }
        }
        
        if(((HistoryValue >= waterLevel) && ((HistoryValue - waterLevel) > 10)) || ((HistoryValue < waterLevel) && ((waterLevel - HistoryValue) > 10))) {
            // Serial.println("Water Level: " + waterLevel);
            HistoryValue = waterLevel;
        }

        /* Measure temperature and humidity.  If the functions returns
            true, then a measurement is available. */
        //if(measure_environment(&temperature, &humidity) == true) {
        if(dht_sensor.measure(&temperature, &humidity)) {
            lcd.setCursor(0,0);
            lcd.print("Temp      C     ");
            lcd.setCursor(6,0);
            lcd.print(temperature, 1);

            lcd.setCursor(0, 1);
            lcd.print("%Humidity       ");
            lcd.setCursor(10,1);
            lcd.print(humidity, 1);
        }

        if(temperature >= 21 || temperature <= 16) {
            digitalWriteB(FAN_ENABLE, HIGH);
        }
        else {
            digitalWriteB(FAN_ENABLE, LOW);
        }

        if(waterLevel < 40) {
            enable = 0;
            lcd.setCursor(0,0);
            lcd.print("ERROR           ");

            lcd.setCursor(0, 1);
            lcd.print("Water Level Low ");
            digitalWriteB(FAN_ENABLE, LOW);
        }
    }
}

void pinModeB(uint8_t pin, uint8_t mode) {
    if(mode == INPUT) {
        *ddrB &= ~(INPUT << pin);
    }
    else if(mode == OUTPUT) {
        *ddrB |= OUTPUT << pin;
    }
    else if(mode == INPUT_PULLUP) {
        *portB |= 0x1 << pin;
    }
}

void digitalWriteB(uint8_t pin, uint8_t val) {
    if(val == LOW) {
        *portB &= ~(0x01 << pin);
    }
    else if(val == HIGH) {
        *portB |= 0x01 << pin;
    }
}

bool digitalReadB(uint8_t pin) {
    if(*pinB & (0x1 << pin)) return HIGH;
    return LOW;
}

void adcInit() {
    // set up the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 3 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 2-0 to 0 to set prescaler selection to slow reading
  
  // set up the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111; // clear bit 2-0 to 0 to set free running mode
  
  // set up the MUX Register
  *my_ADMUX  &= 0b00000000; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adcRead(unsigned char adc_channel_num) {
    // reset the channel and gain bits
    *my_ADMUX &= 0b11100000;

    // clear the channel selection bits
    *my_ADCSRB &= 0b11110111;

    // set the channel number
    if(adc_channel_num > 7) {
        // set the channel selection bits, but remove the most significant bit (bit 3)
        adc_channel_num -= 8;
        
        // set MUX bit 
        *my_ADCSRB |= 0b00001000;
    }

    // set the channel selection bits
    *my_ADMUX  += adc_channel_num;

    // set bit ?? of ADCSRA to 1 to start a conversion
    *my_ADCSRA |= 0b01000000;
  
    // wait for the conversion to complete
    while(*my_ADCSRA & 0b000000000);
    
    // return the result in the ADC data register
    return *my_ADC_DATA;
}
