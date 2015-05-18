#include <Servo.h>

#include <Conceptinetics.h>
//#include <SoftwareSerial.h>

#define DMX_SLAVE_CHANNELS 5

const int PWM_LED_R = 3;
const int PWM_LED_G = 5;
const int PWM_LED_B = 6;
const int PWM_SRV_P = 9;
const int PWM_SRV_T = 10;
const int STATUS_LED = 13;


//SoftwareSerial mySerial(10, 11); // RX, TX
DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS);
Servo pan_servo;
Servo tilt_servo;

void setup() {
  // Setup DMX
  dmx_slave.enable ();  
  dmx_slave.setStartAddress (1);
  
  // Setup outputs
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PWM_SRV_P, OUTPUT);
  pinMode(PWM_SRV_T, OUTPUT);
  
  // Setup servo's
  pan_servo.attach(PWM_SRV_P);
  tilt_servo.attach(PWM_SRV_T);

}

void loop() {
  int pan, tilt;
  
  analogWrite(PWM_LED_R, dmx_slave.getChannelValue(1));
  analogWrite(PWM_LED_G, dmx_slave.getChannelValue(2));
  analogWrite(PWM_LED_B, dmx_slave.getChannelValue(3));
  
  pan = map(dmx_slave.getChannelValue(4), 0, 255, 10, 179);     // scale it to use it with the servo (value between 0 and 180)
  pan_servo.write(pan);
  
  tilt = map(dmx_slave.getChannelValue(5), 0, 255, 10, 179);
  tilt_servo.write(tilt);
}
