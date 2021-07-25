#include "Arduino.h"
#include <avr/interrupt.h>

// circular buffer for capture events
#define   MAXR      32     // must be a power of 2. 8/16/32 ...
#define   VALD      20     // to be considered as valid 2 successive measurement must have difference lower than VALD

volatile  uint8_t   ring;
volatile  int8_t    prot;
volatile  uint16_t  icp[MAXR];

volatile  uint16_t  pulseCount;
volatile  uint8_t   intCount, edgeCount;

uint32_t pediod = 0;
float freq      = .0;

void initCapture(void)
{
  // Input Capture setup
  // ICNC1: Enable Input Capture Noise Canceler
  // ICES1: =1 for trigger on rising edge
  // CS12 CS11 CS10
  //   0    0    1  : /1    No prescaler, CPU clock
  //   0    1    0  : /8    prescaler
  //   0    1    1  : /64   prescaler
  //   1    0    0  : /256  prescaler
  //   1    0    1  : /1024 prescaler

  TCCR1A = 0;
  TCCR1B = (1<<ICNC1) | (0<<ICES1)| (1<<CS11);
  TCCR1C = 0;

  // initialize to catch Falling Edge
  { TCCR1B &= ~(1<<ICES1); TIFR1 |= (1<<ICF1); }
  ring = MAXR-1; // so ring+1 -> MAXR -> 0
  intCount = 0;
  edgeCount = 0;

  // Interrupt setup
  // ICIE1: Input capture
  // TOIE1: Timer1 overflow
  TIFR1 = (1<<ICF1) | (1<<TOV1);    // clear pending interrupts
  TIMSK1 = (1<<ICIE1) | (1<<TOIE1); // enable interupts

  // Set up the Input Capture pin, ICP1, Arduino Uno pin 8
  pinMode(8, INPUT);
  digitalWrite(8, 0); // floating may have 50 Hz noise on it.
  //digitalWrite(8, 1); // or enable the pullup
}


// to check there is a least 4 input change every 65536 count of timer
//
ISR(TIMER1_OVF_vect)
{
  edgeCount = intCount;
  intCount = 0;
}

// Interrupt capture handler
//
ISR(TIMER1_CAPT_vect)
{
  ring = (ring+1)&(MAXR-1);

  icp[ring] = ICR1;

  if (prot==ring)       // if (prot==ring) there is a big problem. data will be overwritten.
    prot = -abs(ring);  // prot set to negative as error flag.
   
  // setup to catch falling edge
  if (ring&1) { TCCR1B &= ~(1<<ICES1); TIFR1 |= (1<<ICF1); }
  // setup to catch rising edge
  //else { TCCR1B |= (1<<ICES1); TIFR1 |= (1<<ICF1); }

  pulseCount += (ring&1) && pulseCount<0xFFFF;  //branch is bad when execution time is precious
  intCount += intCount<0xFF;
}



//period is the time it takes a signal to repeat
uint16_t Period(boolean edge)
{
  uint8_t   i;
  uint16_t  val1,val2;

  if ((intCount+edgeCount)>4)           // buff is full of readings
  {
    i = ring;                           // last writing in circular buffer
    prot = (i+(MAXR-5))&(MAXR-1);       // protected area. TIMER1_CAPT_vect should not write here

    if(edge ^ (i&1))                    // if edge XOR rising, it's not the right one take previous
      i = (i+(MAXR-1))&(MAXR-1);        // (i+3)&3 = i-1 range 0-3

    val1 = icp[i] - icp[(i+(MAXR-2))&(MAXR-1)]; // (i+2)&3 = i-2 range 0-3
    i = (i+(MAXR-2))&(MAXR-1);                  // (i+2)&3 = i-2 range 0-3
    val2 = icp[i] - icp[(i+(MAXR-2))&(MAXR-1)]; // (i+2)&3 = i-2 range 0-3

    if (prot>=0 && abs(val1-val2)<VALD) // No overwritting and 2 measurements quite same
    {
      return ((uint32_t)val1+(uint32_t)val2)>>1;  // Return mean       
    }
    else
      return 0;                         // return 0 as error flag
  }
  else
    return 0;                           // return 0 as error flag
}


//percentage of a period a signal is active 0-4095 = 0-100%
int16_t Duty(boolean edge)
{
  uint8_t   i;
  uint16_t  val1,val2;

  if ((intCount+edgeCount)>4)           // buff is full of readings
  {
    i = ring;                           // last writing in circular buffer
    prot = (i+(MAXR-5))&(MAXR-1);       // protected area. TIMER1_CAPT_vect should not write here

    if(!edge ^ (i&1))                   // if edge XOR rising, it's not the right one take previous
      i = (i+(MAXR-1))&(MAXR-1);        // (i+3)&3 = i-1 range 0-3

    val1 = icp[i] - icp[(i+(MAXR-1))&(MAXR-1)]; // (i+3)&3 = i-1 range 0-3
    val1 = (((uint32_t)val1)<<12) / (icp[i] - icp[(i+(MAXR-2))&(MAXR-1)]);  // (i+2)&3 = i-2 range 0-3
    i = (i+(MAXR-2))&(MAXR-1);          // (i+2)&3 = i-2 range 0-3
    val2 = icp[i] - icp[(i+(MAXR-1))&(MAXR-1)]; // (i+3)&3 = i-1 range 0-3
    val2 = (((uint32_t)val2)<<12) / (icp[i] - icp[(i+(MAXR-2))&(MAXR-1)]);  // (i+2)&3 = i-2 range 0-3

// delay(4); // test for overwritting error
           
    if (prot>=0)                        // No overwritting
    {
      if (abs(val1-val2)<VALD)          // 2 measurements quite same
      {
        return ((uint32_t)val1+(uint32_t)val2)>>1;  // Return mean       
      }
      else
        return -2;                      // return -2 as error flag
    }
    else
    {
      return -1;                        // return -1 as error flag
    }
  }
  else if (!(intCount+edgeCount))
  {
    if(!edge ^ digitalRead(8))
      return 4095;
    else
      return 0;
  }
  else
    return -3;                      // return -3 as error flag
}

uint16_t Count()
{
  uint16_t val=0;

  val = pulseCount;
  pulseCount = 0;

  return val;
}

void setup() {
  Serial.begin(115200);
  initCapture();
}

void loop() {
    pediod = Period(true);
    freq = 1e6/(float(pediod)*0.5);
    if (pediod > 0) {
        Serial.print("Freq : "); Serial.println(freq);
    }

    pediod = 0;
}