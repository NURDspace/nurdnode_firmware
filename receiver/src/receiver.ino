#include "FastLED.h"
#include <Servo.h>
#include <Conceptinetics.h>
#include <Timer.h>

//                87654321
//#define _ADDRESS B11111110

#define DMX_SLAVE_CHANNELS 6
#define PU_DELAY 250
#define BIT_DELAY_ON 400
#define BIT_DELAY_OFF 150

// Micro program ids
#define SHOW_ADDRESS 1
#define BEZERK 2
#define FULL_POWER 3
#define GO_HOME 4

//Macros
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

Timer t;
int timer2_counter;

// Pin constants
const int DMX_RE    = 2;
const int PWM_LED_R = 3;
const int PWM_LED_G = 5;
const int PWM_LED_B = 6;
const int PWM_SRV_P = 9;
const int PWM_SRV_T = 10;
const int UADDRTRIP = 14;
const int MADDRTRIP = 15;
const int LADDRTRIP = 16;
const int EXT1      = 17;
const float V_DROP_CORRECTION = 1.075268817;

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

/*
addrlookup addrmap[TABLESIZE]
{
  {10, 0},
  {295, 4},
  {420, 2},
  {513, 6},
  {582, 1},
  {636, 5},
  {681, 3},
  {902, 7},
};
*/

int address;
int mode;

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
  
  // Setup as receiver
  pinMode(DMX_RE, OUTPUT);
  digitalWrite(DMX_RE, LOW);

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

  // mode setting
  pinMode(EXT1, INPUT);
  mode = digitalRead(EXT1);

  // Setup servo's
  pan_servo.attach(PWM_SRV_P);
  tilt_servo.attach(PWM_SRV_T);

  // Get the DMX adress of our device
  address = get_address();

  // Set debug address if needed 
  #ifdef _ADDRESS
  address = _ADDRESS;
  #endif

  // Set servos to their home position
  servos_go_home();
  turn_off_LED();

  // Setup DMX
  if (mode == 0) {
    dmx_slave.enable();
    dmx_slave.setStartAddress(address);
    show_address();
  }
  else
  {
    // Setup the timer for doing things while the servo's are running 
    noInterrupts();
    TCCR2A = 0;
    TCCR2B = 0;
    timer2_counter = 34286;
    TCNT2 = timer2_counter;
    TCCR2B |= (1 << CS12);
    TIMSK2 |= (1 << TOIE2);
    interrupts();
  }
}

void turn_off_LED()
{
  analogWrite(PWM_LED_R, 0);
  analogWrite(PWM_LED_G, 0);
  analogWrite(PWM_LED_B, 0);
}

void turn_on_LED()
{
  analogWrite(PWM_LED_R, 255);
  analogWrite(PWM_LED_G, 255);
  analogWrite(PWM_LED_B, 255);
}

void loop()
{
  if (mode)
    autonomic_mode();
  else
    dmx_mode();
}

void dmx_mode()
{
  int pan, tilt;

  analogWrite(PWM_LED_R, dmx_slave.getChannelValue(1));
  analogWrite(PWM_LED_G, dmx_slave.getChannelValue(2));
  analogWrite(PWM_LED_B, dmx_slave.getChannelValue(3));

  pan = map(dmx_slave.getChannelValue(4), 0, 255, 10, 179);
  pan_servo.write(pan);

  tilt = map(dmx_slave.getChannelValue(5), 0, 255, 10, 179);
  tilt_servo.write(tilt);

  switch (dmx_slave.getChannelValue(6))
  {
    case SHOW_ADDRESS:
      show_address();
      break;
    case BEZERK:
      bezerk(1);
      break;
    case FULL_POWER:
      full_power();
      break;    
    case GO_HOME:
      servos_go_home();
      turn_off_LED();
      break;      
  }
}

void autonomic_mode()
{
  if (CHECK_BIT(address,1))
    t.every(500, led_color_random);
  if (CHECK_BIT(address,0))
    bezerk(0);
}

/**
* Show the DMX address of the node
*/
void show_address()
{

  analogWrite(PWM_LED_B, 255);
  delay(1000);
  turn_off_LED();
  delay(BIT_DELAY_OFF);
  int bitmask = 256;
  for (int i = 0; i < 9; i++) {
    if ((address & (bitmask >> i)) == (bitmask >> i)) {
      analogWrite(PWM_LED_G, 255);
    } else {
      analogWrite(PWM_LED_R, 255);
    }
    
    delay(BIT_DELAY_ON);
    turn_off_LED();
    delay(BIT_DELAY_OFF);
  }
  turn_off_LED();
  analogWrite(PWM_LED_B, 255);
  delay(1000);
  turn_off_LED();
}

void full_power()
{
    analogWrite(PWM_LED_R, 255);
    analogWrite(PWM_LED_G, 255);
    analogWrite(PWM_LED_B, 255);
}

void led_color_random()
{
  analogWrite(PWM_LED_R, random(255));
  analogWrite(PWM_LED_G, random(255));
  analogWrite(PWM_LED_B, random(255));
}

void bezerk(int mode)
{
  int i;
  int pan, tilt;

  for (i = 0; i < 256; i++) {
    if (mode) {
      analogWrite(PWM_LED_R, i);
      analogWrite(PWM_LED_G, i);
      analogWrite(PWM_LED_B, i);
    }

    pan = map(i, 0, 255, 10, 179);
    pan_servo.write(pan);

    tilt = map(i, 0, 255, 10, 179);
    tilt_servo.write(tilt);
    delay(5);
  }

  for (i = 255; i >= 0; i--) {
    if (mode) {
      analogWrite(PWM_LED_R, i);
      analogWrite(PWM_LED_G, i);
      analogWrite(PWM_LED_B, i);
    }

    pan = map(i, 0, 255, 10, 179);
    pan_servo.write(pan);

    tilt = map(i, 0, 255, 10, 179);
    tilt_servo.write(tilt);
    delay(5);
  }      
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

/**
* This function takes the analog value of the specified pin and multiplies it by 
* the VDROP correction. The reason for this is that the valuemap is comptuted with
* the pro-mini connected with a FTDI cable to USB. This yield a difference in the reference
* Voltage as when the pro mini is connected by the board's power supply. Therefore the input
* needs to be devided by a constant that corrects for the voltage difference between the PRO mini's supply
* and the board's supply.
* 
*/
int binread_triplet(int pin)
{
  int analog_value = analogRead(pin) / V_DROP_CORRECTION;
  for (int i = 0; i < TABLESIZE; i++) {
    if (analog_value < addrmap[i].limit) return addrmap[i].address;
  }
}

ISR(TIMER2_OVF_vect)
{
  TCNT2 = timer2_counter;
  t.update();
}
