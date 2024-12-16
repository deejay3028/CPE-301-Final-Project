// CPE301 Final Project code- written by Cody Shrive and Daxton Johnson

// Add libraries
#include <Wire.h>
#include <LiquidCrystal.h>
#include "DHT11.h"
#include "RTClib.h"
#include <Stepper.h>

// DHT11 pin
#define DHT11_PIN 50 

// Motor pins
#define MOTOR_PORT PORTH
#define MOTOR_DDR DDRH
#define MOTOR_BIT PH6

// LED pins
#define LED_DDR DDRB
#define LED_PORT PORTB
#define YELLOW_LED PB4
#define GREEN_LED PB5
#define BLUE_LED PB6
#define RED_LED PB7

// Timing variables
unsigned int period = 60000;
unsigned long time_now = 0;
unsigned long prev_time = 0;

// Water level variables
unsigned int water_level = 0;
unsigned int prev_sample = 0;

// Track previous update state for logging
volatile unsigned char* prev_update = "IDLE";

// Register addresses for timer
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// ADC registers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// Stepper motor configuration
const int STEPS_PER_REV = 2048;
const int POT_PIN = A1; // Potentiometer pin
// Stepper motor and ULN2003 pins
const int IN1_PIN = 23;
const int IN2_PIN = 25;
const int IN3_PIN = 27;
const int IN4_PIN = 29;
Stepper myStepper(STEPS_PER_REV, IN1_PIN, IN3_PIN, IN2_PIN, IN4_PIN); // Create stepper object

DHT11 dht11(DHT11_PIN); // Create DHT object

RTC_DS1307 rtc; // Create RTC object

// LCD pins and object
const int RS = 30, EN = 32, D4 = 28, D5 = 26, D6 = 24, D7 = 22;
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

// System states
enum State {
  DISABLED = 0,
  IDLE = 1,
  RUNNING = 2,
  ERROR = 3
} state;

void setup() {
  // Configure LED pins as outputs and turn them off
  LED_DDR |= (1 << DDB4) | (1 << DDB5) | (1 << DDB6) | (1 << DDB7);
  LED_PORT &= ~((1 << YELLOW_LED) | (1 << GREEN_LED) | (1 << BLUE_LED) | (1 << RED_LED));
  
  initUART(); // Initialize UART for logging
  adc_init(); // Initialize ADC for reading
  state = IDLE; // Set initial system state to IDLE
  lcd.begin(16, 2); // Initialize LCD to 16x2
  
  float humi = dht11.readHumidity(); // Variable for humidity value
  float tempC = dht11.readTemperature(); // Variable for temp value

  // Set motor pin as output and turn motor off
  MOTOR_DDR |= (1 << MOTOR_BIT);
  MOTOR_PORT &= ~(1 << MOTOR_BIT);

  // Display initial temperature and humidity on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(tempC);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humi);
  lcd.print("%");

  // Attach interrupts for start and stop buttons
  attachInterrupt(digitalPinToInterrupt(2), stopButton, RISING);
  attachInterrupt(digitalPinToInterrupt(3), startButton, RISING);
  
  Wire.begin(); // Initialize I2C for RTC

  // Initialize RTC
  if (!rtc.begin()) {
    uartPrint("RTC Not Found\n");
  }
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  // Set RTC to compile time if not running
  }
  recordEvent("IDLE"); // Log that system is in IDLE state
  myStepper.setSpeed(10);  // Set stepper speed
}

// Global temperature and humidity variables
float humi = 0;
float tempC = 0;

// Stepper control variables
long currentPosition = 0;
long targetPosition = 0;
long previousTargetPosition = -1;
int potValue = 0;
long stepsToMove = 0;

void loop() {
  // Machine state handling
  switch(state){
  // Error state
  case ERROR:
    MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
    LED_PORT &= ~((1 << YELLOW_LED) | (1 << GREEN_LED) | (1 << BLUE_LED)); // LEDS off
    LED_PORT |= (1 << RED_LED); // Red LED on
    // Update LCD
    time_now = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Water level low!");
    generateDelay(100);
    }
    break;
  // Disabled state
  case DISABLED:
    // Log state change
    if(prev_update != "DISABLED"){
      recordEvent("DISABLED");
    }

    // Control stepper motor
    stepsToMove = 0;   
    potValue = adc_read(1);
    //generateDelay(1);
    targetPosition = map(potValue, 0, 1023, 0, STEPS_PER_REV - 1);
    stepsToMove = targetPosition - currentPosition;
    if (stepsToMove > 500 || stepsToMove < -500) {
      myStepper.step(stepsToMove);
      recordEvent("Stepper moved");
      currentPosition = targetPosition;
    }
    MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
    LED_PORT &= ~((1 << GREEN_LED) | (1 << BLUE_LED) | (1 << RED_LED)); // LEDS off
    LED_PORT |= (1 << YELLOW_LED); // Yellow LED on
    //*port_e &= 0b00000000;
    //*port_e |= 0b00010000;
    // Update LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("System Disabled");
    generateDelay(100);
    break;
  // Idle state
  case IDLE:
    // Log state change
    if(prev_update != "IDLE"){
      recordEvent("IDLE");
    }
    // Control stepper motor
    stepsToMove = 0;
    MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
    LED_PORT &= ~((1 << YELLOW_LED) | (1 << BLUE_LED) | (1 << RED_LED)); // LEDS off
    LED_PORT |= (1 << GREEN_LED); // Green LED on
    time_now = millis();
    potValue = adc_read(1);
    //generateDelay(1);
    targetPosition = map(potValue, 0, 1023, 0, STEPS_PER_REV - 1);
    stepsToMove = targetPosition - currentPosition;
    if (stepsToMove > 500 || stepsToMove < -500) {
      myStepper.step(stepsToMove); // Move the stepper motor
      recordEvent("Stepper moved");
      currentPosition = targetPosition; // Update the current position
    }

    water_level = adc_read(15); // Read water level
    // If water level below threshold change state to error
    if(water_level <= 300){
        state = ERROR;
        recordEvent("ERROR");
        break;
    }
    // Update readings every minute
    if(time_now - prev_time >= period){
      humi = dht11.readHumidity();
      tempC = dht11.readTemperature();

      // Update LCD
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

    // If temp goes above threshold change to running
    if(tempC >= 22){
      state = RUNNING;
      recordEvent("RUNNING");
    }
    break;
  //Running state
  case RUNNING:
    // Log state change
    if(prev_update != "RUNNING"){
      recordEvent("RUNNING");
    }
    
    stepsToMove = 0;
    MOTOR_PORT |= (1 << MOTOR_BIT); // Turn fan on
    LED_PORT &= ~((1 << YELLOW_LED) | (1 << GREEN_LED) | (1 << RED_LED)); // Turn LEDs off
    LED_PORT |= (1 << BLUE_LED); // Turn blue LED on
    
    time_now = millis();
    int potValue2 = adc_read(1);
    for(int i = 0; i < 100; i++){
      //generateDelay(400);
    }

    // Control stepper motor
    long targetPosition2 = map(potValue2, 0, 1023, 0, STEPS_PER_REV - 1);
    stepsToMove = targetPosition2 - currentPosition;
    if (stepsToMove > 500 || stepsToMove < -500) {
      myStepper.step(stepsToMove); // Move the stepper motor
      recordEvent("Stepper moved");
      currentPosition = targetPosition2; // Update the current position
    }
    
    water_level = adc_read(15); // Read water level
    // If water level below threshold change state to error
    if(water_level <= 300){
      state = ERROR;
      recordEvent("ERROR");
      break;
    }
    // Update readings every minute
    if(time_now - prev_time >= period){
      humi = dht11.readHumidity();
      tempC = dht11.readTemperature();
      // Update LCD
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
    // If temp drops below threshold change to idle
    if(tempC < 22){
      state = IDLE;
      recordEvent("IDLE");
    }
    break;
}
}

// Function for stop button
void stopButton(){
  MOTOR_PORT &= ~(1 << MOTOR_BIT);  // Motor off
  state = DISABLED; // Change to disabled state
}

// Function for start button
void startButton(){
  water_level = adc_read(10); // Read water level
  // If valid water level change to idle state
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

// Initialize ADC for read operations
void adc_init()
{
  *my_ADCSRA |= 0b10000000;  // ADEN = 1 (ADC Enable)
  *my_ADCSRA &= 0b11011111;  // ADATE = 0 (no auto trigger)
  *my_ADCSRA &= 0b11110111;  // ADIE = 0 (no ADC interrupt)
  *my_ADCSRA &= 0b11111000;  // Clear prescaler bits, set them later if needed

  *my_ADCSRB &= 0b11110111;  // MUX5 = 0
  *my_ADCSRB &= 0b11111000;  // Clear MUX for high channel select

  *my_ADMUX  &= 0b01111111;  // REFS1 = 0
  *my_ADMUX  |= 0b01000000;  // REFS0 = 1 (AVcc reference)
  *my_ADMUX  &= 0b11011111;  // ADLAR = 0 (right adjust)
  *my_ADMUX  &= 0b11100000;  // Clear MUX0-4 to select channel later
}

// Read from ADC on specific channel
unsigned int adc_read(unsigned char adc_channel_num)
{
  *my_ADMUX  &= 0b11100000; // Clear old channel selection
  *my_ADCSRB &= 0b11110111; // Clear MUX5
  if(adc_channel_num > 7)
  {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000; // Set MUX5 to channels 8 and up
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40; // Start conversation
  while((*my_ADCSRA & 0x40) != 0); // Wait until conversation done
  return *my_ADC_DATA; // Return ADC
}

// Custom delay using Timer1 registers
void generateDelay(unsigned int freq){
  double period = 1.0/double(freq);
  double half_period = period / 2.0f;
  double clk_period = 0.0000000625; 
  unsigned int ticks = half_period/clk_period;
  
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  *myTCCR1A = 0x0;
  *myTCCR1B |= 0x01;
  while((*myTIFR1 & 0x01)==0);
  *myTCCR1B &= 0xF8;
  *myTIFR1 |= 0x01;
}
