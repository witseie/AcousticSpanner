/*
 * ArduinoAudioOAM by Mitchell A. Cox (mitchell.cox@wits.ac.za)
 * https://github.com/witseie/AcousticSpanner
 * MIT License
 * 
 * 
 * This code is written for the Arduino Due. 
 * 8 PWMs are used to generate sine waves pi/4 out of phase with each other.
 * When these signals are amplified and the speakers placed correctly, a
 * wavefront with orbital angular momentum is generated which can apply a
 * torque to an object.
 * 
 * Hardware Interfaces:
 * A button is present which switches the OAM \ell=+-1
 * Three LEDs present a rotation animation to represent \ell=+-1
 * 
 * 
 * References:
 * http://www.analog.com/media/en/training-seminars/tutorials/MT-085.pdf
 * http://interface.khm.de/index.php/lab/interfaces-advanced/arduino-dds-sinewave-generator/
 * 
 */

#include <Arduino.h>

// Tweakable Parameters:
#define SINE_FREQUENCY 650 //Hz
#define NUM_OUTPUTS 8
#define OAM_l 1
#define VOLUME 255.0
#define VOLUME_REDUCTION 3 //used when in phase

#define SPK1 9 //PWM
#define SPK2 8 //PWM
#define SPK3 7 //PWM
#define SPK4 6 //PWM
#define SPK5 5
#define SPK6 4
#define SPK7 3
#define SPK8 2

#define BTN_PIN 13
#define LED1_PIN 10
#define LED2_PIN 11
#define LED3_PIN 12
#define AMP_EN_PIN 14

#define DEBOUNCE_DELAY 500 //ms

// For the direction LEDs
#define LED_DELAY_MS 100 //ms

// Calculated / Fixed Parameters:
#define PI_OVER_256 PI / 256.0

// Redefine internal Arduino stuff for analogWrite to use more than 1kHz freq.
//#undef TC_FREQUENCY
//#define TC_FREQUENCY PWM_FREQ
//#undef PWM_FREQUENCY
//#define PWM_FREQUENCY PWM_FREQ
/*
* NOTE: Modify variant.h, TC_FREQUENCY and PWM_FREQUENCY must both be set to 32768 from the default 1000. 
* This file can be found at: C:\Users\youruser\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.11\variants\arduino_due_x
*/
#define PWM_FREQ 328125 //32814 or 65626

/*
* Global Variables ------------------------------------------------------------
*/
// Lookup table for sine wave
uint16_t sine256[256];

double freq;

volatile uint8_t offset;
volatile uint32_t phaseAccum;
volatile uint32_t phaseShift[NUM_OUTPUTS];
volatile uint32_t tuningWord;

volatile bool updateSine = false;

uint8_t state = 3; //0 = +1, 1 = -1, 2 = 0, 3 = off

uint8_t whichLED = 1; //which LED is on (1,2 or 3)

/*
* Utility Functions -----------------------------------------------------------
*/

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
  //See https://forum.arduino.cc/index.php?topic=130423.0
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
  uint32_t rc = VARIANT_MCK / 128 / frequency; //128 because we selected TIMER_CLOCK4 above
  TC_SetRA(tc, channel, rc / 2);               //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}


void setup()
{
  Serial.begin(115200);
  Serial.println("Audio Wrench v1.0 by Mitchell A. Cox and Ermes Toninelli");

  //Set up DDS (Direct Digital Synthesis)
  freq = SINE_FREQUENCY;
  double refClk = PWM_FREQ/12;
  tuningWord = ((double)pow(2, 32) * (freq / refClk));
  phaseAccum = 0;
 
  for (int i = 0; i < NUM_OUTPUTS; ++i)
  {
    phaseShift[i] = i * (pow(2, 32) / NUM_OUTPUTS);
  }

  // Populate the LUT
  double v;
  for (int i = 0; i <= 255; ++i)
  {
    v = (sin((double)i * 2.0 * PI_OVER_256) + 1.0) / 2.0; //sin() is [-1,1] so shift to [0,2] and scale back to [0,1]
    v = v * VOLUME; //scale up to VOLUME
    sine256[i] = (uint16_t)v;
    //Serial.print(String(sine256[i]) + ",");
  }
  //Serial.println();
  analogWriteResolution(8);

  //Set up hardware
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SPK1, OUTPUT);
  pinMode(SPK2, OUTPUT);
  pinMode(SPK3, OUTPUT);
  pinMode(SPK4, OUTPUT);
  pinMode(SPK5, OUTPUT);
  pinMode(SPK6, OUTPUT);
  pinMode(SPK7, OUTPUT);
  pinMode(SPK8, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(AMP_EN_PIN, OUTPUT);
  digitalWrite(AMP_EN_PIN, LOW);

  //PWM
  /*pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(PWM_FREQ*PWM_MAX_DUTY_CYCLE, 0, VARIANT_MCK);
  setupPWM(SPK1);
  setupPWM(SPK2);
  setupPWM(SPK3);
  setupPWM(SPK4); */

  //Can't use TC0 or 2 (used for PWM, etc.)
  startTimer(TC1, 0, TC3_IRQn, PWM_FREQ/12);
}

/*
* Work Functions --------------------------------------------------------------
*/

void loop()
{
  static uint32_t lastTick = 0;
  static uint32_t lastMicros = 0;
  static uint32_t nextMicros = 0;
  static int lastButtonState = LOW;
  static int buttonState = LOW;
  static uint32_t lastBounceTime = 0;

  if (updateSine) {
    UpdateSine();
    updateSine = false;
  }

  //Do the LED rotation
  if (millis() >= (lastTick + LED_DELAY_MS))
  {

    //buttonState = !buttonState;
    //digitalWrite(LED1_PIN, buttonState);
    //turn off existing LED
    digitalWrite(LED1_PIN + (whichLED-1), LOW);

    if (state == 0) {
      digitalWrite(AMP_EN_PIN, HIGH);
      whichLED++;
      if (whichLED > 3) whichLED = 1;
      digitalWrite(LED1_PIN + (whichLED-1), HIGH); 
    } else if (state == 1) {
      digitalWrite(AMP_EN_PIN, HIGH);
      whichLED--;
      if (whichLED < 1) whichLED = 3;
      digitalWrite(LED1_PIN + (whichLED-1), HIGH); 
    } else if (state == 2) {
      digitalWrite(AMP_EN_PIN, HIGH);
      digitalWrite(LED1_PIN, HIGH); 
      digitalWrite(LED2_PIN, HIGH); 
      digitalWrite(LED3_PIN, HIGH); 
    } else if (state == 3) {
      //off
      digitalWrite(AMP_EN_PIN, LOW);
      digitalWrite(LED1_PIN, LOW); 
      digitalWrite(LED2_PIN, LOW); 
      digitalWrite(LED3_PIN, LOW); 
    }

    
    lastTick = millis();
  }

  //Check the button, swap direction
  //Debounce code: https://www.arduino.cc/en/Tutorial/Debounce
  
  if (millis() >= (lastBounceTime + DEBOUNCE_DELAY)) {
    int buttonReading = digitalRead(BTN_PIN);
    if (buttonReading == LOW) {
      state++;
      if (state >= 4) state = 0;
      lastBounceTime = millis();
    }
  }

}

/*
* Interrupts ------------------------------------------------------------------
*/

void UpdateSine()
{
  phaseAccum += tuningWord;
  offset = (uint8_t)(phaseAccum >> 24); //divide and round down to 0-255

  if (state == 0)
  { //positive
    //offset = (uint8_t)((phaseAccum) >> 24); //phaseShift[0] = 0
    analogWrite(SPK1, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK1].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[1]) >> 24);
    analogWrite(SPK2, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK2].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[2]) >> 24);
    analogWrite(SPK3, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK3].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[3]) >> 24);
    analogWrite(SPK4, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK4].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[4]) >> 24);
    analogWrite(SPK5, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[5]) >> 24);
    analogWrite(SPK6, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[6]) >> 24);
    analogWrite(SPK7, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[7]) >> 24);
    analogWrite(SPK8, sine256[offset]);
  }
  else if (state == 1)
  { //negative
    //offset = (uint8_t)((phaseAccum) >> 24); //phaseShift[0] = 0
    analogWrite(SPK1, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK1].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[1]) >> 24);
    analogWrite(SPK2, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK2].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[2]) >> 24);
    analogWrite(SPK3, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK3].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[3]) >> 24);
    analogWrite(SPK4, sine256[offset]);
    //PWMC_SetDutyCycle(PWM_INTERFACE, g_APinDescription[SPK4].ulPWMChannel, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[4]) >> 24);
    analogWrite(SPK5, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[5]) >> 24);
    analogWrite(SPK6, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[6]) >> 24);
    analogWrite(SPK7, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[7]) >> 24);
    analogWrite(SPK8, sine256[offset]);
  } else if (state == 2) { //all on
    analogWrite(SPK1, sine256[offset] >> VOLUME_REDUCTION); //reduce volume
    analogWrite(SPK2, sine256[offset] >> VOLUME_REDUCTION);
    analogWrite(SPK3, sine256[offset] >> VOLUME_REDUCTION);
    analogWrite(SPK4, sine256[offset] >> VOLUME_REDUCTION);
    analogWrite(SPK5, sine256[offset] >> VOLUME_REDUCTION);
    analogWrite(SPK6, sine256[offset] >> VOLUME_REDUCTION);
    analogWrite(SPK7, sine256[offset] >> VOLUME_REDUCTION);
    analogWrite(SPK8, sine256[offset] >> VOLUME_REDUCTION);
  } else if (state == 3) { //all off
    analogWrite(SPK1, 0);
    analogWrite(SPK2, 0);
    analogWrite(SPK3, 0);
    analogWrite(SPK4, 0);
    analogWrite(SPK5, 0);
    analogWrite(SPK6, 0);
    analogWrite(SPK7, 0);
    analogWrite(SPK8, 0);
  }
}

void TC3_Handler() //TC1,0
{
  //Update PWM duty cycles
  TC_GetStatus(TC1, 0);
  updateSine = true;
}
