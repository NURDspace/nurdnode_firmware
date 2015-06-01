#include "FastLED.h"
#include <Servo.h>

//#include <Conceptinetics.h>
#include <SoftwareSerial.h>

#define DMX_SLAVE_CHANNELS 5

// How many leds in your strip?
#define NUM_LEDS 5

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN
#define DATA_PIN 11

// Define the array of leds
CRGB leds[NUM_LEDS];

const int PWM_LED_R = 3;
const int PWM_LED_G = 5;
const int PWM_LED_B = 6;
const int PWM_SRV_P = 9;
const int PWM_SRV_T = 10;
const int STATUS_LED = 13;


//SoftwareSerial mySerial(10, 11); // RX, TX
//DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS);
Servo pan_servo;
Servo tilt_servo;

void setup()
{
  // Setup DMX
  //  dmx_slave.enable ();
  //  dmx_slave.setStartAddress (1);

  // Setup outputs
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PWM_SRV_P, OUTPUT);
  pinMode(PWM_SRV_T, OUTPUT);

  // Setup servo's
  pan_servo.attach(PWM_SRV_P);
  tilt_servo.attach(PWM_SRV_T);

  // Setup leds
  FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  Serial.begin(9600);

  Serial.write("Start powercycle");
  // Perform powercycle
  power_cycle();



}

void power_cycle()
{
  int i;
  for (i = 0; i <= 255; i++) {
    analogWrite(PWM_LED_R, i);
    analogWrite(PWM_LED_G, i);
    analogWrite(PWM_LED_B, i);
    change_servo_pos(pan_servo, i);
    change_servo_pos(tilt_servo, i);
    delay(10);
  }
  for (i = 255; i >= 0; i--) {
    analogWrite(PWM_LED_R, i);
    analogWrite(PWM_LED_G, i);
    analogWrite(PWM_LED_B, i);
    change_servo_pos(pan_servo, i);
    change_servo_pos(tilt_servo, i);
    delay(10);
  }
}

void loop()
{
  handle_leds();
  power_cycle();
  /*
  int pan, tilt;

  analogWrite(PWM_LED_R, dmx_slave.getChannelValue(1));
  analogWrite(PWM_LED_G, dmx_slave.getChannelValue(2));
  analogWrite(PWM_LED_B, dmx_slave.getChannelValue(3));

  pan = map(dmx_slave.getChannelValue(4), 0, 255, 10, 179);     // scale it to use it with the servo (value between 0 and 180)
  pan_servo.write(pan);

  tilt = map(dmx_slave.getChannelValue(5), 0, 255, 10, 179);
  tilt_servo.write(tilt);
  */
}

void handle_leds()
{
  for (int i = 0; i < 2; i++) {
    // Turn the LED on, then pause
    leds[0].r = 255;
    leds[0].g = 0;
    leds[0].b = 0;

    leds[1].r = 255;
    leds[1].g = 255;
    leds[1].b = 0;

    leds[2].r = 0;
    leds[2].g = 255;
    leds[2].b = 0;

    /*
    leds[2] = CRGB::Red;
    leds[3] = CRGB::Red;
    leds[4] = CRGB::Red;*/
    FastLED.show();
    delay(500);
    // Now turn the LED off, then pause
    leds[0] = CRGB::Black;
    leds[1] = CRGB::Black;
    leds[2] = CRGB::Black;
    /*
    leds[2] = CRGB::Black;
    leds[3] = CRGB::Black;
    leds[4] = CRGB::Black;*/
    FastLED.show();
    delay(500);
  }
}

void change_servo_pos(Servo servo, int position)
{
  int mapped_pos = map(position, 0, 255, 1, 179);
  servo.write(mapped_pos);
}
