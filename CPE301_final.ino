#include <Wire.h>
#include <LiquidCrystal.h>
#include "DHT11.h"
#include "RTClib.h"
#include <Stepper.h>


#define DHT11_PIN 50

#define MOTOR_PORT PORTH
#define MOTOR_DDR DDRH
#define MOTOR_BIT PH6

#define LED_DDR DDRB
#define LED_PORT PORTB
#define YELLOW_LED PB4
#define GREEN_LED PB5
#define BLUE_LED PB6
#define RED_LED PB7

unsigned int period = 60000;
unsigned long time_now = 0;
unsigned long prev_time = 0;
unsigned int water_level = 0;
unsigned int prev_sample = 0;

volatile unsigned char* prev_update = "DISABLED";

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

const int STEPS_PER_REV = 2048;
const int POT_PIN = A1;
const int IN1_PIN = 23;
const int IN2_PIN = 25;
const int IN3_PIN = 27;
const int IN4_PIN = 29;
Stepper myStepper(STEPS_PER_REV, IN1_PIN, IN3_PIN, IN2_PIN, IN4_PIN);

DHT11 dht11(DHT11_PIN);

RTC_DS1307 rtc;

const int RS = 30, EN = 32, D4 = 28, D5 = 26, D6 = 24, D7 = 22;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

enum State {
  OFF = 0,
  IDLE = 1,
  ON = 2,
  ERROR = 3
} state;

void setup() {
  LED_DDR |= (1 << DDB4) | (1 << DDB5) | (1 << DDB6) | (1 << DDB7);
  LED_PORT &= ~((1 << YELLOW_LED) | (1 << GREEN_LED) | (1 << BLUE_LED) | (1 << RED_LED));
  initUART();
  adc_init();
  state = IDLE;
  //*ddr_e = 0b00111010;
  // put your setup code here, to run once:
  lcd.begin(16, 2); // set up number of columns and rows
  float humi = dht11.readHumidity();
  float tempC = dht11.readTemperature();
  // put your main code here, to run repeatedly:
  MOTOR_DDR |= (1 << MOTOR_BIT);
  MOTOR_PORT &= ~(1 << MOTOR_BIT);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(tempC);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humi);
  lcd.print("%");
  attachInterrupt(digitalPinToInterrupt(2), stopButton, RISING);
  attachInterrupt(digitalPinToInterrupt(3), startButton, RISING);
  Wire.begin();
  if (!rtc.begin()) {
    uartPrint("RTC Not Found\n");
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC to compile time if not running
  }
  recordEvent("IDLE");
  myStepper.setSpeed(10);  // Stepper Speed
}

float humi = 0;
float tempC = 0;

long currentPosition = 0;
long targetPosition = 0;
long previousTargetPosition = -1;

int potValue = 0;
long stepsToMove = 0;

void loop() {
  switch(state){
  case ERROR:
    MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
    LED_PORT &= ~((1 << YELLOW_LED) | (1 << GREEN_LED) | (1 << BLUE_LED));
    LED_PORT |= (1 << RED_LED);
    time_now = millis();
    if(time_now - prev_time >= 1000){
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Water level low!");
      prev_time = millis();
    }
    break;
  case OFF:
    if(prev_update != "DISABLED"){
      recordEvent("DISABLED");
    }
    stepsToMove = 0;   
    MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
    LED_PORT &= ~((1 << GREEN_LED) | (1 << BLUE_LED) | (1 << RED_LED));
    LED_PORT |= (1 << YELLOW_LED);
    //*port_e &= 0b00000000;
    //*port_e |= 0b00010000;
    time_now = millis();
    if(time_now - prev_time >= period){
      humi = dht11.readHumidity();
      tempC = dht11.readTemperature();
      // put your main code here, to run repeatedly:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humi);
      lcd.print("%");
      prev_time = millis();
    }
    break;
  case IDLE:
    if(prev_update != "IDLE"){
      recordEvent("IDLE");
    }
    stepsToMove = 0;
    MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
    LED_PORT &= ~((1 << YELLOW_LED) | (1 << BLUE_LED) | (1 << RED_LED));
    LED_PORT |= (1 << GREEN_LED);
    time_now = millis();
    potValue = adc_read(1);
    //generateDelay(1);
    targetPosition = map(potValue, 0, 1023, 0, STEPS_PER_REV - 1);
    stepsToMove = targetPosition - currentPosition;
    if (stepsToMove > 500 || stepsToMove < -500) {
      // Move the stepper motor
      myStepper.step(stepsToMove);

      // Update the current position
      currentPosition = targetPosition;
    }
    if(time_now - prev_time >= 5000){
      water_level = adc_read(15);
      humi = dht11.readHumidity();
      tempC = dht11.readTemperature();
      // put your main code here, to run repeatedly:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humi);
      lcd.print("%");
      prev_time = millis();
      if(water_level <= 300){
        state = ERROR;
        recordEvent("ERROR");
        break;
      }
    }
    if(tempC >= 22){
      state = ON;
      recordEvent("RUNNING");
    }
    break;
  case ON:
    if(prev_update != "RUNNING"){
      recordEvent("RUNNING");
    }
    stepsToMove = 0;
    MOTOR_PORT |= (1 << MOTOR_BIT);
    LED_PORT &= ~((1 << YELLOW_LED) | (1 << GREEN_LED) | (1 << RED_LED));
    LED_PORT |= (1 << BLUE_LED);
    time_now = millis();
    int potValue2 = adc_read(1);
    for(int i = 0; i < 100; i++){
      //generateDelay(400);
    }
      long targetPosition2 = map(potValue2, 0, 1023, 0, STEPS_PER_REV - 1);
      stepsToMove = targetPosition2 - currentPosition;
      if (stepsToMove > 500 || stepsToMove < -500) {
       // Move the stepper motor
        myStepper.step(stepsToMove);

        // Update the current position
        currentPosition = targetPosition2;
      }
    if(time_now - prev_time >= 5000){
      water_level = adc_read(15);
      if(water_level <= 300){
        state = ERROR;
        recordEvent("ERROR");
        break;
      }
      humi = dht11.readHumidity();
      tempC = dht11.readTemperature();
      // put your main code here, to run repeatedly:
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.print(tempC);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humi);
      lcd.print("%");
      prev_time = millis();
    }
    if(tempC < 22){
      state = IDLE;
      recordEvent("IDLE");
    }
    break;
}
}

void stopButton(){
  MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
  state = OFF;
}

void startButton(){
  water_level = adc_read(10);
  if(water_level > 100){
    state = IDLE;
    prev_time = 0;
  }
}

// Initialize UART
void initUART() {
  // Baud rate 9600 at 16MHz: UBRR0=103
  UBRR0H = 0;
  UBRR0L = 103;
  UCSR0A = 0;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Write over UART
void uartPrint(const char *str) {
  while (*str) {
    while (!(UCSR0A & (1 << UDRE0)))
      ;
    UDR0 = *str++;
  }
}

// Write integer as string over UART
void uartPrintNumber(int num) {
  char buf[10];
  itoa(num, buf, 10);
  uartPrint(buf);
}

// Record time stamp for events
void recordEvent(const char *event) {
  prev_update = event;
  DateTime now = rtc.now();
  // Format: YYYY-MM-DD HH:MM:SS EVENT
  uartPrintNumber(now.year());
  uartPrint("-");
  if (now.month() < 10) uartPrint("0");
  uartPrintNumber(now.month());
  uartPrint("-");
  if (now.day() < 10) uartPrint("0");
  uartPrintNumber(now.day());
  uartPrint(" ");
  if (now.hour() < 10) uartPrint("0");
  uartPrintNumber(now.hour());
  uartPrint(":");
  if (now.minute() < 10) uartPrint("0");
  uartPrintNumber(now.minute());
  uartPrint(":");
  if (now.second() < 10) uartPrint("0");
  uartPrintNumber(now.second());
  uartPrint(" ");
  uartPrint(event);
  uartPrint("\n");
}

void adc_init()
{
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11011111;
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000;
  *my_ADCSRB &= 0b11110111; 
  *my_ADCSRB &= 0b11111000; 
  *my_ADMUX  &= 0b01111111; 
  *my_ADMUX  |= 0b01000000; 
  *my_ADMUX  &= 0b11011111;
  *my_ADMUX  &= 0b11100000; 
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

uint16_t readWaterSensorADC() {
  ADCSRA |= (5 << ADSC);
  while (ADCSRA & (5 << ADSC))
    ;
  return ADC;
}

void generateDelay(unsigned int freq){
  double period = 1.0/double(freq);
  double half_period = period / 2.0f;
  double clk_period = 0.0000000625; 
  unsigned int ticks = half_period/clk_period;
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  *myTCCR1A = 0x0;
  *myTCCR1B |= 0x0F;
  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;
  *myTIFR1 |= 0x01;
}
