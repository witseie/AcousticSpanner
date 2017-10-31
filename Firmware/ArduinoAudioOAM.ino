/*
 * ArduinoAudioOAM by Mitchell A. Cox (mitchell.cox@wits.ac.za)
 * https://github.com/witseie/ArduinoAudioOAM
 * MIT License
 * 
 * Designed to go with hardware: https://github.com/witseie/MegaSineShield
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

// Tweakable Parameters:
#define SINE_FREQUENCY 50 //Hz
#define NUM_OUTPUTS 8
#define OAM_l 1
#define PWM_FREQ 32812.5 //Hz (VARIANT_MCK/128/20)

#define SPK1 2
#define SPK2 3
#define SPK3 4
#define SPK4 5
#define SPK5 6
#define SPK6 7
#define SPK7 8
#define SPK8 9

#define BTN_PIN 13
#define LED1_PIN 10
#define LED2_PIN 11
#define LED3_PIN 12

#define DEBOUNCE_DELAY 50 //ms

// For the direction LEDs
#define LED_DELAY_MS 100 //ms

// Calculated / Fixed Parameters:
#define PHASE_SHIFT 2 * PI *OAM_l / NUM_OUTPUTS
#define PI_OVER_255 PI / 255

// Redefine internal Arduino stuff for analogWrite to use more than 1kHz freq.
#undef TC_FREQUENCY
#define TC_FREQUENCY PWM_FREQ
#undef PWM_FREQUENCY
#define PWM_FREQUENCY PWM_FREQ

/*
* Global Variables ------------------------------------------------------------
*/
// Lookup table for sine wave
uint8_t sine256[256];

const double refClk = PWM_FREQ;

double freq;

volatile uint8_t offset;
volatile uint32_t phaseAccum;
volatile uint32_t phaseShift[NUM_OUTPUTS];
volatile uint32_t tuningWord;

bool direction = true; //true = +, false = -

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
  tuningWord = pow(2, 32) * freq / refClk;
  phaseAccum = 0;

  for (int i = 0; i < NUM_OUTPUTS; ++i)
  {
    phaseShift[i] = i * (pow(2, 32) / NUM_OUTPUTS);
  }

  // Populate the LUT
  double v;
  for (int i = 0; i <= 255; ++i)
  {
    v = sin((double)i * PI_OVER_255) * 255.0;
    sine256[i] = (uint8_t)v;
  }

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

  //Can't use TC0 or 2 (used for PWM, etc.)
  startTimer(TC1, 0, TC3_IRQn, PWM_FREQ);
}

/*
* Work Functions --------------------------------------------------------------
*/

void loop()
{
  static uint32_t lastTick = 0;
  static int lastButtonState = LOW;
  static int buttonState = LOW;
  static uint32_t lastBounceTime = 0;

  //Do the LED rotation
  if (millis() >= (lastTick + LED_DELAY_MS))
  {
    lastTick = millis();

    //turn off existing LED
    digitalWrite(LED1_PIN + (whichLED-1), LOW);

    if (direction) {
      whichLED++;
      if (whichLED > 3) whichLED = 1;
    } else {
      whichLED--;
      if (whichLED < 1) whichLED = 3;
    }

    digitalWrite(LED1_PIN + (whichLED-1), HIGH);
  }

  //Check the button, swap direction
  //Debounce code: https://www.arduino.cc/en/Tutorial/Debounce
  int buttonReading = digitalRead(BTN_PIN);
  if (buttonReading != lastButtonState) {
    lastBounceTime = millis();
  }
  if ((millis() - lastBounceTime) >= DEBOUNCE_DELAY) {
    Serial.println("button!");
    if (buttonReading != buttonState) {
      buttonState = buttonReading;
      direction = !direction;
    }
  }

  if (Serial.available()) {
    Serial.read();
    Serial.println("Oh Hay!");
    direction = !direction;
  }
}

/*
* Interrupts ------------------------------------------------------------------
*/

void TC3_Handler() //TC1,0
{
  //Update PWM duty cycles
  TC_GetStatus(TC1, 0);

  phaseAccum += tuningWord;
  offset = (uint8_t)(phaseAccum >> 24); //divide and round down to 0-255
  analogWrite(LED_BUILTIN, sine256[offset]);

  if (direction)
  { //positive
    //offset = (uint8_t)((phaseAccum) >> 24); //phaseShift[0] = 0
    analogWrite(SPK1, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[1]) >> 24);
    analogWrite(SPK2, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[2]) >> 24);
    analogWrite(SPK3, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[3]) >> 24);
    analogWrite(SPK4, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[4]) >> 24);
    analogWrite(SPK5, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[5]) >> 24);
    analogWrite(SPK6, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[6]) >> 24);
    analogWrite(SPK7, sine256[offset]);
    offset = (uint8_t)((phaseAccum + phaseShift[7]) >> 24);
    analogWrite(SPK8, sine256[offset]);
  }
  else
  { //negative
    //offset = (uint8_t)((phaseAccum) >> 24); //phaseShift[0] = 0
    analogWrite(SPK1, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[1]) >> 24);
    analogWrite(SPK2, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[2]) >> 24);
    analogWrite(SPK3, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[3]) >> 24);
    analogWrite(SPK4, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[4]) >> 24);
    analogWrite(SPK5, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[5]) >> 24);
    analogWrite(SPK6, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[6]) >> 24);
    analogWrite(SPK7, sine256[offset]);
    offset = (uint8_t)((phaseAccum - phaseShift[7]) >> 24);
    analogWrite(SPK8, sine256[offset]);
  }
}