/*************************************************** 
 Puntatrice SCR Control
 ***************************************************
 ***************************************************
 2016-02-05 : V=0.1 - Initial release
 ***************************************************
 ******   ATmega328 (arduino Nano 3.1)  ************
 ## Serial: 0 (RX) and 1 (TX).
 ## External Interrupts: 2 and 3
 ## SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK)
 ## LED: 13.
 ## I2C: I2C: A4 (SDA) and A5 (SCL)

 ## Timer      pin     chip    label
 ####################################
 ## OC0A	6	12	PD6
 ## OC0B	5	11	PD5
 ## OC1A	9	15	PB1
 ## OC1B	10	16	PB2
 ## OC2A	11	17	PB3
 ## OC2B	3	5	PD3

  pin   label   description
  --------------------------
   (1)- PD1 - D1/TX - TX232/RS422/RS485
   (0)- PD0 - D0/RX - RX232/RS422/RS485
      - RESET - 
      - GND   -
   (2)- PD2 - D2    - Zero Cross DETECT
   (3)- PD3 - D3    - TRIAC
   (4)- PD4 - D4    - Manual Switch StartPulse
   (5)- PD5 - D5    - 
   (6)- PD6 - D6    - 
   (7)- PD7 - D7    - 
   (8)- PB0 - D8    - 
   (9)- PB1 - D9    - 
  (10)- PB2 - D10   - 
  (11)- PB3 - D11   - 
  (12)- PB4 - D12   - 
  (13)- PB5 - D13   - PIN Internal LED
      - 3V3   
      - AREF  -
  (14)- PC0 - A0    - Current Flow Trasformer
  (15)- PC1 - A1    - Potentiometer Manual Position TimeOn First PULSE
  (16)- PC2 - A2    - 
  (17)- PC3 - A3    - 
  (18)- PC4 - A4    - I2C SDA
  (19)- PC5 - A5    - I2C SCL
  (20)- A6    - Potentiometer Manual Position Time ON (ExtPotSetTon)
  (21)- A7    - Potentiometer Wait Time Td from First Pulse and TimeON
      - +5V
      - RESET
      - GND
      - VIN
  -------------------------- */

#pragma GCC diagnostic ignored "-Wwrite-strings"

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <Wire.h>
#include "LiquidCrystal_I2C.h"

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

// Define cbi() and sbi() for clearing and setting bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define nanoV3_led     13
#define triacPin        3
#define PinStartPulse   4
#define PinPotWaitTD    7
#define PinPotSetTon    6
#define PinPotSetPower  2
#define PinPotSetUnoT   1
#define PinPotCurrent   0

#define DEBUG                   0x00

/*
 * Global Variable and ADC[0..7] = voltage read
 */
LiquidCrystal_I2C lcd(0x27,20,4);

#define repeatTimeDebug 200   //ms
volatile unsigned long now_time=0, timePidRefresh=0;
char cbuff[20];
uint8_t isZeroCross=false, autoSP=0;
volatile uint16_t tadc, msCounter=0, msDelay=0, msBoot=5, pulse=9000, Td=5, UnoT=5, EndCurrent=0, ExtPotSetTd=0, ExtPotSetTon=0, ExtPotUnoT=0;

/*************************************************** 
 ********* FILTER StartPulse ***********************
 ***************************************************/
uint8_t isStartPulse(void) {
  uint8_t tt,bt;
  //Read Ext Start Pulse
  do{ 
   tt = digitalRead(PinStartPulse);
   for(bt=240; (bt!=0) && (tt==digitalRead(PinStartPulse)); bt--) {
   __asm__("nop\n\t");
   }
  }while(bt!=0);
  return tt;
}

/*
 * TIMER3 SEETINGS
 */
#define timer1_hz      60
#define defPreLoadTimer1  (65536-(16000000/256/timer1_hz))
#define timer2_hz      20000
#define defPreLoadTimer2  (255-(16000000/8/timer2_hz))

/*************************************************** 
 ********* INIT PWM ********************************
 ***************************************************
 *TCCR1A : COM1A1 COM1A0 COM1B1 COM1B0 – – WGM11 WGM10
 *TCCR1B : ICNC1 ICES1 – WGM13 WGM12 CS12 CS11 CS10
 *TCCR1C : FOC1A FOC1B – – – – – –
 *Mode WGM13 WGM12 WGM11 WGM10 
 *0 0 0 0 0 Normal 0xFFFF Immediate MAX
 *1 0 0 0 1 PWM, Phase Correct, 8-bit 0x00FF TOP BOTTOM
 *2 0 0 1 0 PWM, Phase Correct, 9-bit 0x01FF TOP BOTTOM
 *3 0 0 1 1 PWM, Phase Correct, 10-bit 0x03FF TOP BOTTOM
 *CS12 CS11 CS10 Description
 *0 0 0 No clock source (Timer/Counter stopped).
 *0 0 1 clkI/O/1 (No prescaling)
 *0 1 0 clkI/O/8 (From prescaler)
 *0 1 1 clkI/O/64 (From prescaler)
 *1 0 0 clkI/O/256 (From prescaler)
 *1 0 1 clkI/O/1024 (From prescaler)
 *1 1 0 External clock source on T1 pin. Clock on falling edge.
 *1 1 1 External clock source on T1 pin. Clock on rising edge.
 */
void setupPwm(void)
{
  // PWM Fase-Correct
  TCCR1A = 0;                 // clear control register A 
  TCCR1B = 0;
  TCCR1C = 0;
  
  OCR1A = 0; //dutyCycle - valore iniziale setting
  OCR1B = 0; //dutyCycle - valore iniziale setting

  // Phase Correct PWM, 8-bit, TOP = 0x00FF, Update OCR3x at TOP, TOV3 Flag Set on BOTTOM  => WGM43|WGM42|WGM41|WGM40 = 0|0|0|1
  //TCCR1A = _BV(WGM11) | _BV(WGM10) |_BV(COM1A1);// | _BV(COM1B1); 
  TCCR1A = (1 << WGM10) | (1 << COM1A1);// | _BV(COM1B1); 
  //TCCR1B = (1 << CS11)  | (1 << CS10); 
  TCCR1B = (1 << CS10); // 16MHz / 256 = 62.5KHz
}

void setMotorPWM_a(unsigned int i) { OCR1A = i; }

// Interrupt Timer 1
ISR(TIMER1_OVF_vect) { }

/*************************************************** 
 ********* Init ADC ********************************
 ***************************************************/
float readInternalT(void) {
  float internalT=0.0;
  
  cli();//disable interrupts
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;    // Analog Input bank 1
  DIDR0  = 0xF0; // First A0|A1|A2|A3 used analog mode

  ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX3);
  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set ADC clock with 128 prescaler- 16mHz/128=125kHz
  delay(300);             // wait for voltages to become stable.
  ADCSRA |= (1 << ADSC);  //start ADC measurements

  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  //internalT = ((ADCL | (ADCH << 8)) - 324.31 ) * 0.8196;
  internalT = ((float)(ADCL | (ADCH << 8)) - 125.0 ) * 0.1075;

  setupInterruptAdc();
  delay(100);
  return internalT;
}

// ADCSRA : ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0
// ADCSRB :   –  ACME    –    –    –  ADTS2 ADTS1 ADTS0
// DIDR0  :   –     – ADC5D ADC4D ADC3D ADC2D ADC1D ADC0D
// ADMUX  : REFS1 REFS0 ADLAR – MUX3 MUX2 MUX1 MUX0
void setupInterruptAdc(void) {

  cli();//disable interrupts
  //clear ADCSRA and ADCSRB registers
  ADCSRA = 0;
  ADCSRB = 0;    // Analog Input bank 1
  DIDR0  = 0xF0; // First A0|A1|A2|A3 used analog mode

  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (0);

  ADCSRA |= (1 << ADEN); //enable ADC
  ADCSRA |= (1 << ADIE); //enable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); //set ADC clock with 128 prescaler- 16mHz/128=125kHz

  //sleep_enable(); //enable the sleep capability
  //set_sleep_mode(SLEEP_MODE_ADC); //set the type of sleep mode. We are using the ADC noise reduction sleep mode
  //Note that in this sleep mode the chip automatically starts an ADC measurement once the chip enters sleep mode
  //Note that the reason for making the ADC measurements and storing in arrays was to avoid using the arduino serial functions with sleep mode since serial uses interrupts
  
  ADCSRA |= (1 << ADSC); //start ADC measurements
  sei();//enable interrupts
  
}

boolean channelAdcRead(unsigned int ch) {
  //Set Analog Multiplexer and start conversion
  ADMUX = (0<<REFS1) | (1<<REFS0) | (0<<ADLAR) | (ch&0x07);
  sbi(ADCSRA, ADSC); // Start the ADC conversion.
  return true;
} 

// Defines an ADC interrupt processing routine that asks the currently active scanner to process a newly scanned value.
ISR(ADC_vect) {
  tadc = ADCL | (ADCH << 8);
}

/*
 *  MCUSR – MCU Status Register
 *  MCUSR  :  –    –    –    –  WDRF BORF EXTRF PORF
 *  PORF: Power-on Reset Flag : This bit is set if a Power-on Reset occurs.
 *  EXTRF: External Reset Flag: This bit is set if an External Reset occurs.
 *  BORF: Brown-out Reset Flag: This bit is set if a Brown-out Reset occurs.
 *  WDRF: Watchdog System Reset Flag : This bit is set if a Watchdog System Reset occurs
 *  
 *  WDTCSR – Watchdog Timer Control Register
 *  WDTCSR : WDIF WDIE WDP3 WDCE WDE WDP2 WDP1 WDP0
 *  WDIF: Watchdog Interrupt Flag
 *  WDTON WDE WDIE Mode               Action on Time-out
 *    1    0    0  Stopped              None
 *    1    0    1  Interrupt            Mode Interrupt
 *    1    1    0  System Reset         Mode Reset
 *    1    1    1  Interrupt and System Reset - Interrupt, then go to System Reset Mode
 *    0    x    x  System Reset Mode    Reset
 *  WDP3 WDP2 WDP1 WDP0
 *    0 0 0 0 2K (2048) cycles 16ms
 *    0 0 0 1 4K (4096) cycles 32ms
 *    0 0 1 0 8K (8192) cycles 64ms
 *    0 0 1 1 16K (16384) cycles 0.125 s
 *    0 1 0 0 32K (32768) cycles 0.25 s
 *    0 1 0 1 64K (65536) cycles 0.5 s
 *    0 1 1 0 128K (131072) cycles 1.0 s
 *    0 1 1 1 256K (262144) cycles 2.0 s
 *    1 0 0 0 512K (524288) cycles 4.0 s
 *    1 0 0 1 1024K (1048576) cycles 8.0 s
 *  WDIE = 1 :Interrupt Enable
 *  WDCE =  Watchdog Change Enable
 *  WDE = 1  :Reset Enable - I won't be using this on the 2560
 *  WDP3,WDP2,WDP1,WDP0 = 0110 :For 1000ms Time-out
 */
void setupWDT(void) {
  cli();            // disable all interrupts
  wdt_reset();      // reset the WDT timer
  MCUSR  &= ~(1<<WDRF);   //Clear the reset flag.
  WDTCSR = (1<<WDCE) | (1<<WDE);  // Enter Watchdog Configuration mode
  // Set Watchdog settings: interrupte enable, 0110 for timer
  //WDTCSR = (1<<WDIE) | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); // 1sec, Interrupt Mode
  //WDTCSR = (1<<WDE)  | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0); // 0.25sec, System Reset Mode
  WDTCSR = (1<<WDE)  | (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0); // 0.5sec, System Reset Mode
  //WDTCSR = (1<<WDE)  | (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (0<<WDP0); // 1sec, System Reset Mode
  //WDTCSR = (1<<WDE) | (1<<WDP3);  // 4sec, System Reset Mode
  sei();
}

void WDT_off(void) {
  cli();        // disable all interrupts
  wdt_reset();  // reset the WDT timer
  MCUSR  &= ~(1<<WDRF);              //Clear the reset flag.
  WDTCSR = (1<<WDCE) | (1<<WDE);  // Enter Watchdog Configuration mode
  // Set Watchdog settings: interrupte enable, 0110 for timer
  WDTCSR = 0; //Turn off WDT
  sei();
}

/*
 * SLEEP_MODE_IDLE        – least power savings
 * SLEEP_MODE_ADC
 * SLEEP_MODE_EXTENDED_STANDBY  
 * SLEEP_MODE_PWR_SAVE
 * SLEEP_MODE_STANDBY
 * SLEEP_MODE_PWR_DOWN    – most power savings
 */
void enterSleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);   // EDIT: could also use SLEEP_MODE_PWR_DOWN for lowest power consumption.
  sleep_enable();
  
  sleep_mode(); // Now enter sleep mode.
  
  // The program will continue from here after the WDT timeout
  sleep_disable();    // First thing to do is disable sleep.
  
  power_all_enable(); // Re-enable the peripherals.
}

// Watchdog Interrupt Service. This is executed when watchdog timed out.
// ISR(WDT_vect) {}
ISR(WDT_vect, ISR_NAKED) {}

/***********************************************
 * INIT TIMER2
 * Timer 0: 8-bit, PWM OC0A,B 6,5
 * Timer 1: 16-bit, PWM OC1A,B 9,10
 * Timer 2: 8-bit, PWM OC2A,B 11,3

 * Timer 2
 * Interrupt rate =  16MHz / (prescaler * (255 - TCNT2))
        TCCR2B[b2:0]   Prescaler    Freq [KHz], Period [usec] after prescale
          0x0            (TC stopped)     0         0
          0x1                1        16000.        0.0625
          0x2                8         2000.        0.500
          0x3               32          500.        2.000
          0x4               64          250.        4.000
          0x5              128          125.        8.000
          0x6              256           62.5      16.000
          0x7             1024           15.625    64.000
 ************************************************/
void setupTimer2(void)
{
  // initialize timer2 
  noInterrupts();           // disable all interrupts
  TCCR2A = 0;
  TCCR2B = 0;

  // Set timer2_counter to the correct value for our interrupt interval

  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);    // 8 prescaler => 0.5ms
  TCNT2 = defPreLoadTimer2; // preload timer
  TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt*/
 
  interrupts();             // enable all interrupts
}

ISR(TIMER2_OVF_vect)
{
//  TCNT2 = defPreLoadTimer2;   // preload timer
}

/*************************************************** 
 ********* FREE RAM ********************************
 **************************************************/
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}


/*************************************************** 
 ********* ZERO CROSS DETECT ***********************
 ***************************************************
 * 50Hz => Rectifier => 100Hz => 10ms = 1/2 onda sinusoidale
 * 10.000ys /100 (step) = 100ys 
 *  1% of 10ms = 0.10ms MAX POWER
 * 99% of 10ms = 9.95ms MIN POWER 
 * RANGE (1-99)% => (0.10-9,95)ms
*/
void zeroCross() {

  if(msBoot) {
    delayMicroseconds(pulse);    // Off cycle
    digitalWrite(triacPin, LOW);   // triac firing
    msBoot--;
    delayMicroseconds(10);         // triac On propogation delay
    //Led Change*/
    digitalWrite(nanoV3_led, !digitalRead(nanoV3_led));
  }
  
  if(msDelay && (msBoot==0)) {
    msDelay--;
    delayMicroseconds(10);         // triac On propogation delay
    //Led Change*/
    digitalWrite(nanoV3_led, !digitalRead(nanoV3_led));
  }

  if(msCounter && (msBoot==0) &&  (msDelay==0)) {
    delayMicroseconds(pulse);    // Off cycle
    digitalWrite(triacPin, LOW);   // triac firing
    msCounter--;
    delayMicroseconds(10);         // triac On propogation delay
    //Led Change*/
    digitalWrite(nanoV3_led, !digitalRead(nanoV3_led));
  }

  
  digitalWrite(triacPin, HIGH);    // triac Off
  isZeroCross=true;
}

/*************************************************** 
 ********* Control START PULSE *********************
 ***************************************************/
void autoStartPulse(void) {
  switch(autoSP) {
    case 0 : //Is Start_Botton_Pulse Pressed ?
             if(isStartPulse()) {
               lcd.setCursor(0, 3);
               lcd.print("-ready- Press START ");
               autoSP=1;
             }
      break;
    case 1 : //Is Start_Botton_Pulse Released ?
             if(isStartPulse()==0) {
               lcd.setCursor(0, 3);
               lcd.print("release-for  *PULSE*");
               autoSP=2;
             }
      break;
    case 2 : 
             if(isStartPulse()) {
               lcd.setCursor(0, 3);
               lcd.print("!!!!! PULSE !!!!!!!!");
#if(DEBUG)
  Serial.print(F("Power % = "));
  Serial.print(100-pulse);
  Serial.print(F(" --- NUmber of HalfWAVE = "));
  Serial.println(ExtPotSetTon);
#endif
               autoSP=0;
               pulse *= 90; //delay time start Triac
               msCounter = ExtPotSetTon;
               msDelay = Td;
               msBoot = UnoT;
             }
      break;
    default : autoSP=0;
      break;
  }
}

/*************************************************** 
 ********* SETUP & LOOP ****************************
 ***************************************************/
void setup() {

  //Init Wotch DOG
  wdt_reset();      // reset the WDT timer
  //wdt_enable(WDTO_4S);
  //setupWDT();

  //Init Analog PIN
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  //Init Input PIN
  pinMode(2, INPUT_PULLUP);
  pinMode(triacPin, OUTPUT);
  pinMode (triacPin , HIGH );
  pinMode(PinStartPulse, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);

  //Init Attach Interrupt
  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0,  zeroCross,  RISING); //RISING, CHANGE
  //PCICR = Pin change interrupt control register
  //PCICR |= (1 << PCIE2);
  //PCMSK2 = Pin change Mask register
  //PCMSK2 |= (1 << PCINT20); //enable PinChangeInterrupt on PCINT20
  //Init Output PIN Enable PWM
  pinMode(8, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  //Init Timer1 PWM
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  //Init Output PIN
  pinMode(12, INPUT_PULLUP);

  pinMode ( nanoV3_led , OUTPUT );
  pinMode ( nanoV3_led , HIGH );

  Serial.begin(115200);

  
#if(DEBUG)
          Serial.print(F("MCUSR – MCU Status Register [HEX]= "));
          Serial.println(MCUSR,HEX);
#endif

  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Puntatrice V1.0");

  setupInterruptAdc();
  //setupTimer2();
  //setupWDT();
}

void loop() {

  now_time = millis();

    //Clear WatchDog();
  wdt_reset();  // reset the WDT timer

  if(msCounter>1) {
    //LEttura TA
    channelAdcRead(PinPotCurrent);
    // Detect end-of-conversion
    while (bit_is_set(ADCSRA,ADSC));
    EndCurrent = tadc;
  }


  //Only in wait state
  if( (now_time > (timePidRefresh + 250)) && (msCounter==0) && (msDelay==0) && (msBoot==0)) {
    timePidRefresh = now_time;
    //Read External Potentiometer Time Wait First To TimeON
    channelAdcRead(PinPotWaitTD);
    // Detect end-of-conversion
    while (bit_is_set(ADCSRA,ADSC));
    ExtPotSetTd = tadc;
    Td = (uint16_t)(25.0*(float)ExtPotSetTd/1023.0);
    if(Td<3) Td=3;
    else if(Td>25) Td=25;
    //Read External Potentiometer Time ON
    channelAdcRead(PinPotSetTon);
    // Detect end-of-conversion
    while (bit_is_set(ADCSRA,ADSC));
    ExtPotSetTon = (uint16_t)(100.0*tadc/1023.0);
    if(ExtPotSetTon==0) ExtPotSetTon=1;
    //Power
    channelAdcRead(PinPotSetPower);
    // Detect end-of-conversion
    while (bit_is_set(ADCSRA,ADSC));
    pulse = (uint16_t)(80.0*(1023-tadc)/1023.0);
    if(pulse<9) pulse=8;
    else if(pulse>80) pulse=80;
    //pulse=8;
    //Read External Potentiometer TimeOn First PULSE
    channelAdcRead(PinPotSetUnoT);
    // Detect end-of-conversion
    while (bit_is_set(ADCSRA,ADSC));
    ExtPotUnoT = tadc;
    UnoT = (uint16_t)(20.0*(float)ExtPotUnoT/1023.0);
    if(UnoT<1) UnoT=1;
    else if(UnoT>20) UnoT=20;
    //Print on Display
    //sprintf(cbuff,"T1P=%3ims EndA=%4i",UnoT*10,EndCurrent);
    sprintf(cbuff,"T1P=%3ims pw=%4i",UnoT*10,100-100*(pulse-8)/72);
    lcd.setCursor(0, 1);
    lcd.print(cbuff);
    sprintf(cbuff,"Td=%3ims Ton=%4ims",Td*10,ExtPotSetTon*10);
    lcd.setCursor(0, 2);
    lcd.print(cbuff);
    //Read Ext Start Pulse
    autoStartPulse();
    //pulse *= 90; //delay time start Triac
    //msCounter = ExtPotSetTon;
  }

#if(DEBUG)
#endif

}
