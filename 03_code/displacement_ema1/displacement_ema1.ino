/*
 This sketch shows how to retrieve displacement data.
 */

#include <ADNS3080.h>

// SPI pins:
#define PIN_RESET 9
#define PIN_CS 10

ADNS3080<PIN_RESET, PIN_CS> sensor;

// Initial position
int x = 0;
int y = 0;

void setup() {
  sensor.setup();
  Serial.begin(115200);
}
long tRead = 0;
long count = 0;
void loop() {
  int8_t dx, dy;  // Displacement since last function call
  while (micros() < tRead + 1000) {}
  tRead = micros();
  sensor.displacement(&dx, &dy);

  // Integrate displacements
  x += dx;
  y += dy;

  // Displacement:
  if (dx != 0 && dy != 0) {
    if(count > 100){
      x=0;
      y=0;
    }
    Serial.print(" dx=");
    Serial.print(dx);
    Serial.print(" dy=");
    Serial.print(dy);
    Serial.print(" x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.print(y);
    Serial.print(" count=");
    Serial.print(count);
    Serial.println();
    count = 0;
  } else {
    count++;
  }
}
