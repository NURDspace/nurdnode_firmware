#include "FastLED.h"
#include <Servo.h>
#include <Conceptinetics.h>

#define DMX_SLAVE_CHANNELS 6
#define PU_DELAY 250

// Micro program ids
#define SHOW_ADDRESS 1


// Pin constants
const int PWM_LED_R = 3;
const int PWM_LED_G = 5;
const int PWM_LED_B = 6;
const int PWM_SRV_P = 9;
const int PWM_SRV_T = 10;
const int UADDRTRIP = 14;
const int MADDRTRIP = 15;
const int LADDRTRIP = 16;

// Addressing init section
const int TABLESIZE = 8;

struct addrlookup {
  int limit;
  int address;
};

addrlookup addrmap[TABLESIZE]
{
  {10, 0},
  {275, 4},
  {391, 2},
  {477, 6},
  {541, 1},
  {592, 5},
  {634, 3},
  {839, 7},
};

int address;

// Init dmx_slave
DMX_Slave dmx_slave(DMX_SLAVE_CHANNELS);

// Init PT servo's
Servo pan_servo;
Servo tilt_servo;

// Home positions (these may change due to calibration)
int pan_home = 128;
int tilt_home = 128;

void setup()
{
  // Wait a bit to allow all the caps in the array to charge up, and not have the servo's and LED
  // pull extra current. Poor PSU.
  delay(PU_DELAY);

  // Setup led channel outputs
  pinMode(PWM_LED_R, OUTPUT);
  pinMode(PWM_LED_G, OUTPUT);
  pinMode(PWM_LED_B, OUTPUT);

  // Setup servo outputs
  pinMode(PWM_SRV_P, OUTPUT);
  pinMode(PWM_SRV_T, OUTPUT);

  // Setup addressing input pins
  pinMode(UADDRTRIP, INPUT);
  pinMode(MADDRTRIP, INPUT);
  pinMode(LADDRTRIP, INPUT);

  // Setup servo's
  pan_servo.attach(PWM_SRV_P);
  tilt_servo.attach(PWM_SRV_T);

  // Get the DMX adress of our device
  address = get_address();
  
  // Setup DMX
  dmx_slave.enable();
  dmx_slave.setStartAddress(address);

  // Set servos to their home position
  servos_go_home();
  turn_off_LED();
  
  show_address();
}

/*
void printval(char* label, int  value)
{
    char buf[4];
    itoa(value, buf, 10);
    Serial.write(label);
    Serial.write(buf);
    Serial.write("\n");
}
*/
void turn_off_LED()
{
  analogWrite(PWM_LED_R, 0);
  analogWrite(PWM_LED_G, 0);
  analogWrite(PWM_LED_B, 0);
}


void loop()
{
  show_address();
  
  /*
  int pan, tilt;

  analogWrite(PWM_LED_R, dmx_slave.getChannelValue(1));
  analogWrite(PWM_LED_G, dmx_slave.getChannelValue(2));
  analogWrite(PWM_LED_B, dmx_slave.getChannelValue(3));

  pan = map(dmx_slave.getChannelValue(4), 0, 255, 10, 179);     // scale it to use it with the servo (value between 0 and 180)
  pan_servo.write(pan);

  tilt = map(dmx_slave.getChannelValue(5), 0, 255, 10, 179);
  tilt_servo.write(tilt);

  switch (dmx_slave.getChannelValue(6))
  {
    case SHOW_ADDRESS:
      show_address();
      break;
  }*/
}

/**
* Show the DMX address of the node
*/
void show_address()
{

  analogWrite(PWM_LED_B, 255);  
  delay(1000);
  turn_off_LED();
  delay(500);
  int bitmask = 256;
  for (int i = 0; i < 9; i++) {
    if ((address & (bitmask >> i)) == (bitmask >> i)) {
      analogWrite(PWM_LED_G, 255);
    } else {
      analogWrite(PWM_LED_R, 255);
    }
    delay(500);
    turn_off_LED();
    delay(500);
  }
  turn_off_LED();
  analogWrite(PWM_LED_B, 255);  
  delay(1000);
}

/**
* OHM sweet OHM
*/
void servos_go_home()
{
  change_servo_pos(pan_servo, pan_home);
  change_servo_pos(tilt_servo, tilt_home);
}

void change_servo_pos(Servo servo, int position)
{
  int mapped_pos = map(position, 0, 255, 1, 179);
  servo.write(mapped_pos);
}

unsigned int get_address()
{
  unsigned int address = 0;
  address += (binread_triplet(UADDRTRIP) & 0x7) << 6;
  address += (binread_triplet(MADDRTRIP) & 0x7) << 3;
  address += (binread_triplet(LADDRTRIP) & 0x7);

  return address;
}

int binread_triplet(int pin)
{
  int analog_value = analogRead(pin);
  for (int i = 0; i < TABLESIZE; i++) {
    if (analog_value < addrmap[i].limit) return addrmap[i].address;
  }
}
