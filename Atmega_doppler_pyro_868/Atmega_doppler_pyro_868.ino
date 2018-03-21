////////////////////////////////////////////////////////////////////////////////////////////////////
//  Configuration
////////////////////////////////////////////////////////////////////////////////////////////////////

//#define DEBUG
//#define LOOPDEBUG

//#define noRebootTimeout

#define pyroSenseTreshold 800      //on/off treshold for analogRead on pyro sense pin
#define armTreshold 15
#define maxPower 31   //maximum tx power=-18dBm + value

#define TX_FREQ_MHZ   868.75

#define shootTime 5   // how long burn pyro in seconds
#define countDownTime 100 //how long to wait before shoot in seconds

#define tmr1Ticks 10    //timer ticks per second
/*
 * Atmega fuses: (E:FF, H:C1, L:E2)
 * Internal oscillatopr 8 MHz, 3.3V, Brownout off, WDT on
 * Arduino pro mini board for development
*/

/*
 * Old fuses (without WDT)
 * Atmega fuses: (E:FF, H:D1, L:E2)
 * Internal oscillatopr 8 MHz, 3.3V, Brownout off
 * Arduino pro mini board for development
*/

////////////////////////////////////////////////////////////////////////////////////////////////////
//  System code
////////////////////////////////////////////////////////////////////////////////////////////////////

#include <avr/wdt.h> 

enum kstate{
  stateSafe,
  stateArm,
  stateShoot,
  stateCharge
};

byte state=stateSafe;

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif 

#ifndef toggle
#define toggle(sfr, bit) (_SFR_BYTE(sfr) ^= _BV(bit))
#endif

#include "RFM69OOK.h"
#include "RFM69OOKregisters.h"


//const int txpin = A2;     //RTTY pin
#define txDir DDRC
#define txPort PORTC
#define txPin PINC2

//const int txpin = A0;     //pyro out
#define pyroDir DDRC
#define pyroPort PORTC
#define pyroPin PINC0

#define pyroSensePin A7

#define pyroVoltagePin A6

#define radioCsPin 9
#define radioResetPin 2

#include <avr/sleep.h>

RFM69OOK radio(radioCsPin);

bool isArmed=false;
unsigned int armCount=0;
unsigned int shootCount=0;

volatile unsigned int cnt=0;
volatile bool doTick=false;
void setup()
{ 
  wdt_reset();
  wdt_enable(WDTO_8S);
  Serial.begin(9600);
//  #ifdef DEBUG
//    Serial.begin(9600);
//  #endif
//
//  #ifdef LOOPDEBUG
//    #ifndef DEBUG
//      Serial.begin(9600);
//    #endif
//  #endif
  
  analogRead(pyroSensePin);
  analogRead(pyroSensePin);
  
  // Setup Timer2: CTC, prescaler 1024, irq on compare1 called baudRate times per second
//  TCCR2A = _BV(WGM21);
//  TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
//  OCR2A = (F_CPU / 1024 / timer2Ticks) - 1;
  //OCR2A=OCR2A_50;     //for variable baudrate
//  TIMSK2 = _BV(OCIE2A);

  
  // Setup Timer1: CTC, prescaler 256, irq on compare1, called tmr1Ticks times per second
  TCCR1A = 0;
  TCCR1B = _BV(WGM12) | _BV(CS12); // | _BV(CS10);
  OCR1A = (F_CPU  / 256 / (tmr1Ticks)) - 1;
  cli();
  TIMSK1 = _BV(OCIE1A);
  sei();

  initRadio();

#ifdef noRebootTimeout
  isArmed=isPyroArmed();
  if (isArmed){
    state=stateCharge;
  }
#endif
  Serial.println("Run");
  wdt_reset();
}

bool shooting=false;
bool armed=false;

unsigned int pyroVoltage=0;

bool isPyroArmed(){
  bool tmp;
  if (state==stateShoot){
    return true;
  }
  cbi(pyroPort,pyroPin);  // set output LOW
  cbi(pyroDir,pyroPin);   // set direction INPUT
  //analogReference(INTERNAL);
  //delay(1);
  analogRead(pyroSensePin);
  delay(1);
  analogRead(pyroSensePin);
  delay(1);
  
  #ifdef DEBUG
    Serial.print("Arm: ");
    Serial.println(analogRead(pyroSensePin));
  #endif

  Serial.print("PyroV: ");
  Serial.println(analogRead(pyroVoltagePin));
    
  if (analogRead(pyroSensePin)>armTreshold){  // read arm pin state
    tmp= true;
  } else {
    tmp=false;
  }

  pyroVoltage=analogRead(pyroVoltagePin);
  delay(1);
  
  cbi(pyroPort,pyroPin);  // set output LOW
  sbi(pyroDir,pyroPin);  // set direction OUTPUT
  return tmp;            // return pyro pin state
}

void pyroShoot(){
  cbi(pyroPort,pyroPin);  // set output LOW
  sbi(pyroDir,pyroPin);  // set direction OUTPUT
  sbi(pyroPort,pyroPin);  // set output HIGH  
}


void loop()
{
  wdt_reset();
  if(doTick){
    doTick=false;    
    isArmed=isPyroArmed();
  
    if (!isArmed){
      state=stateSafe;
    } else {
      cbi(txPort,txPin);
    }
    #ifdef LOOPDEBUG
      Serial.print("State: ");
    #endif
    switch(state){
      case stateSafe:
        #ifdef LOOPDEBUG
          Serial.println("Safe");
        #endif
        if (isArmed){
          state=stateArm;
        }
        armCount=0;
        if ( (state==stateSafe) && (pyroVoltage<pyroSenseTreshold) ){
          toggle(txPort,txPin);
        } else {
          cbi(txPort,txPin);
        }
        break;
      case stateArm:
        #ifdef LOOPDEBUG
          Serial.println("Armed");
          Serial.print("Arm count: ");
          Serial.println(armCount);
        #endif
        armCount+=1;
        if ( armCount > (countDownTime*tmr1Ticks) ){
          state=stateCharge;
        }
        break;
      case stateCharge:
        #ifdef LOOPDEBUG
          Serial.println("Arm, Charge");
        #endif
        if (pyroVoltage>pyroSenseTreshold){
          state=stateShoot;
        }
        break;
      case stateShoot:
        #ifdef LOOPDEBUG
          Serial.println("Arm, SHOOT");
        #endif
        pyroShoot();
        shootCount+=1;
        if ( shootCount > (shootTime*tmr1Ticks) ){
          state=stateCharge;
          shootCount=0;
        }
        break;
      default:
        #ifdef LOOPDEBUG
          Serial.println("WTF???");
        #endif
        break;
    }
  }
}

ISR ( TIMER1_COMPA_vect ) { doTick=true; }

void initRadio(){
  //Radio init stuff
  //Reset radio
  sbi(txDir,txPin);  
  pinMode(radioResetPin,INPUT_PULLUP);
  delay(10);
  pinMode(radioResetPin,INPUT);
  delay(10);  
  
  //init radio with libary functions
  radio.initialize();  

 
  //Set radio frequency manual
    //radio.writeReg(REG_FRFMSB,0x6C);
    //radio.writeReg(REG_FRFMID,0x7C);
    //radio.writeReg(REG_FRFLSB,0xCC);
  radio.setFrequencyMHz(TX_FREQ_MHZ);
  

  // +/- deviation freq = 61*register (6*61=366Hz, RTTY FSK shift=732 hz
  radio.writeReg(REG_FDEVMSB,0);
  radio.writeReg(REG_FDEVLSB,6);      

  //radio.writeReg(REG_PARAMP,0);
  radio.writeReg(REG_PARAMP,0b1111);
    
  //set TX power
    radio.writeReg(REG_OCP,0b10111);
    radio.writeReg(REG_TESTPA1,0x55);
    radio.writeReg(REG_TESTPA2,0x70);
    radio.writeReg(REG_PALEVEL,( 0b01100000 | maxPower ) );


  cbi(txPort,txPin);
  //sbi(txPort,txPin);
  radio.transmitBegin();
  
}
