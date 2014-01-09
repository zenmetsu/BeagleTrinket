/* Trinket/Gemma compatible IR read sketch
This Sketch allows a trinket with a 38kHz IR receiver attached to GPIO pin #2
and a piezo buzzer attached to pin #4 to act as a proximity alert when the device
approaches an IR beacon.  The code allows for configurable power management timeouts
and the Trinket is put into a sleep state in the absence of a signal in order to 
reduce power consumption.  In sleep, power consumption will be approximately 3.7mA
due to the power LED.  Removing the power LED by cutting the trace or increasing the
resistance of the current limiting resistor will reduce power consumption to under 1mA.

Based on Adafruit tutorial http://learn.adafruit.com/ir-sensor/using-an-ir-sensor
*/
 
#include <avr/power.h>
#include <avr/sleep.h>
 
// We need to use the 'raw' pin reading methods because timing is very important here 
// and the digitalRead() procedure is slower!
#define IRpin_PIN  PINB // ATTiny85 has Port B pins
#define IRpin      2    // IR sensor - TSOP38238 on Pin GPIO #2 / D2
#define LEDpin     1    // LED heartbeat indicator
#define SPEAKERPIN 4    // Piezo speaker on Trinket/Gemma Pin GPIO #4 / D4
 
#define MAXPULSE     5000   // the maximum pulse we'll listen for - 5 milliseconds 
#define NUMPULSES     100   // max IR pulse pairs to sample
#define RESOLUTION      2   // time between IR measurements
#define sensitivity     6   // will wait until this many hits
#define sleepTimeout 4000   // timer for power management in milliseconds

#define BPM           260   // heartbeat rate of indicator LED
 
// End config

// we will store up to 100 pulse pairs (this is -a lot-, reduce if needed)
uint16_t pulses[100][2];             // pair is high and low pulse
uint16_t currentpulse = 0;           // index for pulses we're storing
uint32_t irCode = 0;

uint32_t ocr = (F_CPU)/(BPM/30); 
uint8_t prescalarBits = 1;

volatile uint32_t triggerTimer = 0; // keeps track of last time trigger occurred
uint8_t trigger = 0;                // counts up to sensitivity

 


void setPrescalar()
{
  // scan prescalars to find best fit
  while (ocr>255)
  {
    prescalarBits++;
    ocr /=2;
  }
  TCCR1 = 0x90 | prescalarBits;
  OCR1C = ocr-1;                        // Set the OCR 
  bitWrite(TIMSK, OCIE1A, 1);           // enable interrupt
}



void naptime()
{
  TCCR1 = 0x80 | prescalarBits;
  digitalWrite(LEDpin, 0);
  trigger = 0;
  GIMSK = _BV(PCIE);                    // Enable pin change interrupt
  power_all_disable();                  // All peripherals off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Choose sleep mode
  sleep_enable();                       
  sei();                                // Prevent interrupts from waking
  sleep_mode();                         // CPU down, pin 2 will awaken the beast...
  GIMSK = 0;                            // No really, disable the pin change interrupt
  power_timer0_enable();                // Re-enable timer
  power_timer1_enable();                // Re-enable timer
  TCCR1 = 0x90 | prescalarBits;         // Reset TCCR to toggle pin
  power_usi_enable();                   // Re-enable USI
}
 
 
 
void setup() {
  power_adc_disable();          // Disable unused peripherals
  PCMSK |= _BV(PCINT2);         // Set change mask for pin 2
  pinMode(IRpin, INPUT);        // Listen to IR receiver on Trinket/Gemma pin D2
  pinMode(LEDpin, OUTPUT);      // Heartbeat indicator
  pinMode(SPEAKERPIN, OUTPUT);  // Output tones on Trinket/Gemma pin D3
  setPrescalar();
  triggerTimer = millis();      // Initialize triggerTimer
}
 
 

// Generate a tone on speakerPin - Trinket/Gemma/ATTiny85 compatible
void beep (unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds)
{	 // http://web.media.mit.edu/~leah/LilyPad/07_sound_code.html
          int x;	 
          long delayAmount = (long)(1000000/frequencyInHertz);
          long loopTime = (long)((timeInMilliseconds*1000)/(delayAmount*2));
          for (x=0;x<loopTime;x++)	 
          {	 
              digitalWrite(speakerPin,HIGH);
              delayMicroseconds(delayAmount);
              digitalWrite(speakerPin,LOW);
              delayMicroseconds(delayAmount);
          }	 
}
 
 
 
uint16_t listenForIR()                 // IR receive code
{
  currentpulse = 0;
  while (1) 
  {
    unsigned int highpulse, lowpulse;  // temporary storage timing
    highpulse = lowpulse = 0;          // start out with no pulse length 
   
    while (IRpin_PIN & _BV(IRpin))     // got a high pulse
    {
      highpulse++; 
      delayMicroseconds(RESOLUTION);
      if (((highpulse >= MAXPULSE) && (currentpulse != 0))|| currentpulse == NUMPULSES) 
      {
        return currentpulse; 
      }
    }
    pulses[currentpulse][0] = highpulse;
 
    while (! (IRpin_PIN & _BV(IRpin)))   // got a low pulse
    {
      lowpulse++; 
      delayMicroseconds(RESOLUTION);
      if (((lowpulse >= MAXPULSE) && (currentpulse != 0))|| currentpulse == NUMPULSES) 
      {
        return currentpulse; 
      }
    }
    pulses[currentpulse][1] = lowpulse;
    currentpulse++;
  }   
}



void loop() {
  irCode=listenForIR();         // Wait for an IR Code
  
  // Process the pulses to get our code
  for (int i = 0; i < 32; i++) 
  {
    irCode=irCode<<1;
    if((pulses[i][0] * RESOLUTION)>0&&(pulses[i][0] * RESOLUTION)<500) 
    {
      irCode|=0; 
    } else {
      irCode|=1;
    }
  }
  
  if(irCode==0xFFFFFFFF)               // Roomba virtual wall is seen this way
    {                                  // with the configured IR resolution
      triggerTimer = millis();
      trigger = trigger + 2;
      if (trigger >= sensitivity)
        {
          beep(SPEAKERPIN,10000,500);  // Make a 10 kHz beep for 1/2 secon
        }
    }
  if (trigger >= 1)                    // If we have any trigger counts
    { 
      trigger--;                       // Begin decrementing
    }
} // end loop



ISR(PCINT0_vect) {} // Button tap



ISR(TIMER1_COMPA_vect) 
{
  if (millis() - triggerTimer >= sleepTimeout)
  {   
    naptime();                    // probably not good to do this from within ISR
    triggerTimer = millis();      // but I am admittedly a novice C coder
  }
}
