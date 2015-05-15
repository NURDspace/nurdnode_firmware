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

#include <DmxSimple.h>

const int ON       = 255;
const int OFF      = 0;

const int DMX_TX   = 2;
const int BUTTON_R = 3;
const int BUTTON_G = 4;
const int BUTTON_B = 5;

int R_CHAN = 1;
int G_CHAN = 2;
int B_CHAN = 3;

int R_STATUS = OFF;
int G_STATUS = OFF;
int B_STATUS = OFF;

void setup() {
  // Setup the buttons
  pinMode(BUTTON_R, INPUT);
  pinMode(BUTTON_G, INPUT);
  pinMode(BUTTON_B, INPUT);
  
  Serial.begin(9600);
  
  
  /* The most common pin for DMX output is pin 3, which DmxSimple
  ** uses by default. If you need to change that, do it here. */
  DmxSimple.usePin(DMX_TX);

  /* DMX devices typically need to receive a complete set of channels
  ** even if you only need to adjust the first channel. You can
  ** easily change the number of channels sent here. If you don't
  ** do this, DmxSimple will set the maximum channel number to the
  ** highest channel you DmxSimple.write() to. */
  DmxSimple.maxChannel(1);
  
  
}

void loop() {
  
  if (digitalRead(BUTTON_R) == HIGH) {
    R_STATUS = ON;
    Serial.write("R");
  } else {
    R_STATUS = OFF;
  }

  if (digitalRead(BUTTON_G) == HIGH) {
    G_STATUS = ON;
    Serial.write("G");
  } else {
    G_STATUS = OFF;
  }
 
  if (digitalRead(BUTTON_B) == HIGH) {
    B_STATUS = ON;
    Serial.write("B");
  } else {
    B_STATUS = OFF;
  }
   
  DmxSimple.write(R_CHAN, R_STATUS); // Chan, value
  DmxSimple.write(G_CHAN, G_STATUS);
  DmxSimple.write(B_CHAN, B_STATUS);
}
