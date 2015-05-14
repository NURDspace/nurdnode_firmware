#include <DmxSimple.h>

const int PWM_LED_R = 3;
const int PWM_LED_G = 5;
const int PWM_LED_B = 6;
const int PWM_SRV_P = 7;
const int PWM_SRV_T = 10;
  
void setup() {
  // put your setup code here, to run once:
 

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.write("Hello world!");
  
  int i = 0;
  while (1) {
	analogWrite(PWM_LED_R, i);
	analogWrite(PWM_LED_G, i + 80);
	analogWrite(PWM_LED_B, i + 160);
	i++;
	delay(10);
    }
}
