#include <DmxSimple.h>

/* Welcome to DmxSimple. This library allows you to control DMX stage and
** architectural lighting and visual effects easily from Arduino. DmxSimple
** is compatible with the Tinker.it! DMX shield and all known DIY Arduino
** DMX control circuits.
**
** DmxSimple is available from: http://code.google.com/p/tinkerit/
** Help and support: http://groups.google.com/group/dmxsimple       */

/* To use DmxSimple, you will need the following line. Arduino will
** auto-insert it if you select Sketch > Import Library > DmxSimple. */



const char ON        = 1;
const char OFF       = 0;
const char RIGHT     = 1;
const char UP        = 1;
const char LEFT      = 0;
const char DOWN      = -1;
const unsigned char MAX_RANGE  = 255;

const char DMX_TX   = 2;
const char BUTTON_R = 3;
const char BUTTON_G = 4;
const char BUTTON_B = 5;

const char BUTTON_PAN = 7;
const char BUTTON_TILT = 8;

const char DELAY = 10;



char R_CHAN = 1;
char G_CHAN = 2;
char B_CHAN = 3;
char PAN_CHAN = 5;
char TILT_CHAN = 4;

signed char R_DIRECTION = UP;
signed char G_DIRECTION = UP;
signed char B_DIRECTION = UP;
signed char PAN_DIRECTION = RIGHT;
signed char TILT_DIRECTION = UP;

unsigned char R_STATUS = OFF;
unsigned char G_STATUS = OFF;
unsigned char B_STATUS = OFF;
unsigned char PAN_STATUS = 127;
unsigned char TILT_STATUS = 127;

char BTN_R = OFF;
char BTN_G = OFF;
char BTN_B = OFF;
char BTN_PAN = OFF;
char BTN_TILT = OFF;

void setup() {
  // Setup the buttons
  pinMode(BUTTON_R, INPUT);
  pinMode(BUTTON_G, INPUT);
  pinMode(BUTTON_B, INPUT);
  pinMode(BUTTON_PAN, INPUT);
  pinMode(BUTTON_TILT, INPUT);

  Serial.begin(9600);
  
  
  /* The most common pin for DMX output is pin 3, which DmxSimple
  ** uses by default. If you need to change that, do it here. */
  DmxSimple.usePin(DMX_TX);

  /* DMX devices typically need to receive a complete set of channels
  ** even if you only need to adjust the first channel. You can
  ** easily change the number of channels sent here. If you don't
  ** do this, DmxSimple will set the maximum channel number to the
  ** highest channel you DmxSimple.write() to. */
  DmxSimple.maxChannel(5);
}

void loop() {
  determineDirection(&R_STATUS, &R_DIRECTION);
  determineDirection(&G_STATUS, &G_DIRECTION);
  determineDirection(&B_STATUS, &B_DIRECTION);
  determineDirection(&PAN_STATUS, &PAN_DIRECTION);
  determineDirection(&TILT_STATUS, &TILT_DIRECTION);
  
  check_button(BUTTON_R, &R_STATUS,  R_DIRECTION);
  check_button(BUTTON_G, &G_STATUS,  G_DIRECTION);
  check_button(BUTTON_B, &B_STATUS,  B_DIRECTION);
  check_button(BUTTON_PAN, &PAN_STATUS,  PAN_DIRECTION);
  check_button(BUTTON_TILT, &TILT_STATUS,  TILT_DIRECTION);
  

  
  DmxSimple.write(R_CHAN, R_STATUS);
  DmxSimple.write(G_CHAN, G_STATUS);
  DmxSimple.write(B_CHAN, B_STATUS);
  DmxSimple.write(PAN_CHAN, PAN_STATUS);
  DmxSimple.write(TILT_CHAN, TILT_STATUS);
  
  

}

void check_button(char button, unsigned char *chan_status, signed char sweep_direction)
{
  
  if (digitalRead(button)) {
      *chan_status += sweep_direction;
      output_char(*chan_status);
      delay(DELAY);
      
  }
}

void determineDirection(unsigned char *chan_status, signed char *sweep_direction)
{
    //output_char(*chan_status);
    if (*chan_status == 0 || *chan_status == MAX_RANGE) *sweep_direction = *sweep_direction == UP ? *sweep_direction = DOWN : UP;
}

void output_char(unsigned char msg)
{
    char buffer[3];
    itoa(msg, buffer, 10);
    Serial.write(buffer);
    Serial.write("\n");
}
