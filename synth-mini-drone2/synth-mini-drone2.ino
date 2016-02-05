/* display 128x64 SPI */
#include <SPI.h>
#include <Wire.h>
//#define SSD1306_LCDWIDTH                  128
//#define SSD1306_LCDHEIGHT                 64
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// If using software SPI (the default case):
//#define OLED_MOSI   11
//#define OLED_CLK   13
//#define OLED_DC    A4
//#define OLED_CS    9
//#define OLED_RESET A5
//Adafruit_SSD1306 display(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
// i2c
#define OLED_RESET 13 // 4
Adafruit_SSD1306 display(OLED_RESET);

const byte CLOCKOUT = 9;   // Uno, Duemilanove, etc. for clocking sn76489

//
///* 8x adc converter SPI */
//#include <MCP3008.h>
//
//#define MCP3008_MOSI   11
//#define MCP3008_CLK   13
//#define MCP3008_MISO    12
//#define MCP3008_CS    10   // see if this works::  by using the same pin as the display, but through the 74HCT14 hex inverter, the MCP3008 is normally selected
//
//// put pins inside MCP3008 constructor
//MCP3008 adc(MCP3008_CLK, MCP3008_MOSI, MCP3008_MISO, MCP3008_CS);
//
//// cd4021 testing
//#define CD4021_LATCH A3
//#define CD4021_CLOCK A1
//#define CD4021_DATA A2


// SN76489

#define PIN_D0 0
#define PIN_D1 1
#define PIN_D2 2
#define PIN_D3 3
#define PIN_D4 4
#define PIN_D5 5
#define PIN_D6 6
#define PIN_D7 7

#define PIN_NotWE 8

#define BUT0_PIN 13
#define BUT1_PIN 12
#define BUT2_PIN 11
#define BUT3_PIN 10

//#define CV1_PIN 0
//#define GATE1_PIN 1
//#define CV2_PIN 2
//#define GATE2_PIN 3
//
//#define POT1_MCP 4
//#define POT2_MCP 5
//#define POT3_MCP 6

#define POT0_PIN A0
#define POT1_PIN A1
#define POT2_PIN A2
#define POT3_PIN A3

#define OSC_FREQ 4000000

unsigned long current_millis;

unsigned int but0_value;
unsigned int but1_value;
unsigned int but2_value;
unsigned int but3_value;

unsigned int target_pitch1;
unsigned int current_pitch1;
unsigned int current_att1;
unsigned int decay1;
unsigned long decay1_last_updated;
unsigned int cv1_value;
unsigned int gate1_value;
float cv1;
float f1;

unsigned int target_pitch2;
unsigned int current_pitch2;
unsigned int current_att2;
unsigned int decay2;
unsigned long decay2_last_updated;
unsigned int cv2_value;
unsigned int gate2_value;
float cv2;
float f2;

//unsigned int target_pitch3;
//unsigned int current_pitch3;
//unsigned int current_att3;
//unsigned int decay3;
//unsigned long decay3_last_updated;
//unsigned int cv3_value;
//unsigned int gate3_value;
//float cv3;
//float f3;

unsigned int pot0_value;
unsigned int pot1_value;
unsigned int pot2_value;
unsigned int pot3_value;

void send_byte(byte b)
{
  // is fixed to D0-D7
  PORTD = b;
  digitalWrite(PIN_NotWE, HIGH);
  digitalWrite(PIN_NotWE, LOW);
  digitalWrite(PIN_NotWE, HIGH);
}

// ch is 0..3
// att is 0..15
void attenuation(byte ch, byte att)
{
  send_byte(0b10010000 | (ch << 5) | att);
}

// ch is 0..2 (3 is noise)
// tone is 0..1023 (10 bits)
void pitch(byte ch, unsigned int counter_reset)
{
  send_byte(0b10000000 | (ch << 5) | (counter_reset & 0b1111));
  send_byte(0b00000000 | ((counter_reset >> 4) & 0b111111));
}

// mode 1=white, 0=periodic
// shift_rate 2 bits 0..3
void noise(byte mode, byte shift_rate)
{
  send_byte(0b11100000 | mode << 2 | shift_rate);
}

// f = 13 .. about 7000 (will get more inaccurate at higher pitches)
unsigned int f_to_pitch(float f) {
//  return int(OSC_FREQ / 16 / f); 
  return int(250000 / f); 
}

//ISR(TIMER1_COMPA_vect){  //change the 0 to 1 for timer1 and 2 for timer2
//  // TODO: factor out as much as possible so we can increase the frequency
////  cv1_value = adc.readADC(CV1_PIN);  //analogRead(CV1_PIN);
//  cv1 = float(cv1_value) * 5 / 1024;
////  cv2_value = adc.readADC(CV2_PIN);  //analogRead(CV2_PIN);
//  cv2 = float(cv2_value) * 5 / 1024;
//  //cv3_value = analogRead(CV3_PIN);
//  //cv3 = float(cv3_value) * 5 / 1024;
//
//
//  f1 = 440 * pow(2, cv1);  // + float(pot1_value) / 1024);
//  target_pitch1 = f_to_pitch(f1);
//
//  if (target_pitch1 != current_pitch1) {
//    current_pitch1 = target_pitch1;    
//    pitch(0, current_pitch1);
//  }
//
//  f2 = 440 * pow(2, cv2); // + float(pot2_value) / 1024);
//  target_pitch2 = f_to_pitch(f2);
//  if (target_pitch2 != current_pitch2) {
//    current_pitch2 = target_pitch2;
//    pitch(1, current_pitch2);
//  }
//
//  current_millis = millis();
//  //but1_value = digitalRead(BUT1_PIN);
////  gate1_value = adc.readADC(GATE1_PIN); //digitalRead(GATE1_PIN);
////  gate2_value = adc.readADC(GATE2_PIN);  //digitalRead(GATE2_PIN);
//
//  if (gate1_value > 512) {
//    current_att1 = 0;
//    decay1_last_updated = current_millis;
//    attenuation(0, current_att1);
//  } else if ((current_att1 < 0xF) && (current_millis - decay1_last_updated > decay1)) {
//    current_att1 += 1;
//    decay1_last_updated = current_millis;
//    attenuation(0, current_att1);
//  }
//  if (gate2_value > 512) {
//    current_att2 = 0;
//    decay2_last_updated = current_millis;
//    attenuation(1, current_att2);
//  } else if ((current_att2 < 0xF) && (current_millis - decay2_last_updated > decay2)) {
//    current_att2 += 1;
//    decay2_last_updated = current_millis;
//    attenuation(1, current_att2);
//  }
//
//}


void setup() {

  // create clock signal for sn76489
  //SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));

  cli();
  // set up 4 MHz timer on CLOCKOUT (OC1A)
  pinMode (CLOCKOUT, OUTPUT); 
  // set up Timer 1
  TCCR1A = bit (COM1A0);  // toggle OC1A on Compare Match
  TCCR1B = bit (WGM12) | bit (CS10);   // CTC, no prescaling
  OCR1A =  1;       // output every cycle
  sei();//allow interrupts
  
  
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  //display.begin(SSD1306_SWITCHCAPVCC);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)  // init done
    
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.display();

  //pinMode(CV1_PIN, INPUT);

  digitalWrite(BUT0_PIN, HIGH);
  delay(1000);
  digitalWrite(BUT0_PIN, LOW);
  delay(1000);
  digitalWrite(BUT0_PIN, HIGH);
  delay(1000);
  digitalWrite(BUT0_PIN, LOW);
  delay(1000);

  
  pinMode(BUT0_PIN, INPUT_PULLUP);
  pinMode(BUT1_PIN, INPUT_PULLUP);
  pinMode(BUT2_PIN, INPUT_PULLUP);
  pinMode(BUT3_PIN, INPUT_PULLUP);
  pinMode(POT0_PIN, INPUT);
  pinMode(POT1_PIN, INPUT);
  pinMode(POT2_PIN, INPUT);
  pinMode(POT3_PIN, INPUT);

  DDRD = B11111111;
  pinMode(PIN_NotWE, OUTPUT);

  attenuation(0, 0x0);
  attenuation(1, 0xf);
  attenuation(2, 0xf);
  attenuation(3, 0xf);
  pitch(0, 200);
  pitch(1, 320);
  pitch(2, 420);
  noise(1, 0);
  current_pitch1 = 200;
  current_pitch2 = 300;
//  current_pitch3 = 200;
  
  decay1 = 50;
  decay2 = 50;
//  decay3 = 50;

  decay1_last_updated = millis();
  decay2_last_updated = millis();
//  decay3_last_updated = millis();

////set timer0 interrupt at 2kHz
//  TCCR0A = 0;// set entire TCCR0A register to 0
//  TCCR0B = 0;// same for TCCR0B
//  TCNT0  = 0;//initialize counter value to 0
//  // set compare match register for 2khz increments
//  OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
//  // turn on CTC mode
//  TCCR0A |= (1 << WGM01);
//  // Set CS01 and CS00 bits for 64 prescaler
//  TCCR0B |= (1 << CS01) | (1 << CS00);   
//  // enable timer compare interrupt
//  TIMSK0 |= (1 << OCIE0A);


//  // set timer1 to approx 500 Hz
//cli();//stop interrupts
//  TCCR1A = 0;// set entire TCCR1A register to 0
//  TCCR1B = 0;// same for TCCR1B
//  TCNT1  = 0;//initialize counter value to 0
//  // set compare match register
//  OCR1A = 30;// (must be <65536)
//  // turn on CTC mode
//  TCCR1B |= (1 << WGM12);
//  // Set prescaler CS10, CS11 CS12
//  //TCCR1B |= (1 << CS10);  // no prescaling  
//  //TCCR1B |= (1 << CS11);  // /8  
//  //TCCR1B |= (1 << CS10) + (1 << CS11);  // /64  
//  //TCCR1B |= (1 << CS12);  // 256  
//  TCCR1B |= (1 << CS10) | (1 << CS12);  // 1024  
//  //TCCR1B |= (1 << CS12) | (1 << CS11);  // external clock on t1 pin, falling edge
//  //TCCR1B |= (1 << CS12) | (1 << CS11) |  (1 << CS10);  // external clock on t1 pin, rising edge
//  // enable timer compare interrupt
//  TIMSK1 |= (1 << OCIE1A);
//sei();//allow interrupts



}

void loop() {

//  pot1_value = adc.readADC(POT1_MCP); // analogRead(POT1_PIN);
  //pot2_value = adc.readADC(POT2_MCP);
  //pot3_value = adc.readADC(POT3_MCP);

  pot0_value = analogRead(POT0_PIN);
  pot1_value = analogRead(POT1_PIN);
  pot2_value = analogRead(POT2_PIN);
  pot3_value = analogRead(POT3_PIN);
  but0_value = digitalRead(BUT0_PIN);
  but1_value = digitalRead(BUT1_PIN);
  but2_value = digitalRead(BUT2_PIN);
  but3_value = digitalRead(BUT3_PIN);
  pitch(0, pot1_value + (1-digitalRead(BUT1_PIN)) * 100 );
  //decay1 = pot3_value;

  display.clearDisplay();
  display.setCursor(0,0);
//  display.print(gate1_value);display.print(" ");display.println(cv1_value);
//  display.print(gate2_value);display.print(" ");display.println(cv2_value);
  //display.println(cv2_value);
  display.println(pot0_value);
  display.println(pot1_value);
  display.println(pot2_value);
  display.println(pot3_value);
  display.println();
  display.print(but0_value);
  display.print(but1_value);
  display.print(but2_value);
  display.println(but3_value);
  //display.println(pot2_value);
  //display.println(pot3_value);

  
//  // testing CD4021
//  digitalWrite(CD4021_LATCH, 1);
//  delayMicroseconds(20);
//  digitalWrite(CD4021_LATCH, 0);
//
//  display.println(shiftIn(CD4021_DATA, CD4021_CLOCK));
    
  display.display();

}
