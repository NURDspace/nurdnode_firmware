#include <DmxSimple.h>

void setup() {
}

void loop() {
  int brightness;  
  for (brightness = 0; brightness <= 255; brightness++) {
    DmxSimple.write(1, brightness); // Set DMX channel 1 to new value
    delay(10); // Wait 10ms
  }
}
