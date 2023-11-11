/* Firmware code for pendulinum */

#include <ADNS3080.h>
// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>

#include <math.h>

// SPI pins:
#define PIN_RESET 9
#define PIN_CS 10

ADNS3080<PIN_RESET, PIN_CS> sensor;

// Initial position
long x = 0;
long y = 0;
int dxSum = 0;
int dySum = 0;
TaskHandle_t taskRead;

void setup() {
  sensor.setup();
  Serial.begin(115200);
  Serial.println("Start");

  //   /**
  //    * Task creation
  //    */
  //   xTaskCreate(TaskVariableRead,  // Task function
  //               "Blink",           // Task name
  //               128,               // Stack size
  //               NULL,              // Arguments
  //               0,                 // Priority
  //               &taskRead);        // Task handler
}
unsigned long tRead = 0;
unsigned long tPrint = 0;
bool motionDetect = false;
bool motionDetect_Old = false;
unsigned long lastTimeStart = 0;
unsigned long lastTimeEnd = 0;
unsigned long dt_halfSwing = 0;
unsigned long dt_motion = 0;
bool printed = true;
int phase = 0;
float deg = 0;
int countRead = 0;
void loop() {
  int8_t dx, dy;  // Displacement since last function call
  while (millis() < tRead + 1) {}
  tRead = millis();
  sensor.displacement(&dx, &dy);
  countRead++;
  // dxSum += dx;
  // dySum += dy;

  motionDetect_Old = motionDetect;
  // Displacement:
  if (dx != 0 && dx != 0) {
    countRead = 0;
    // Integrate displacements
    x += dx;
    y += dy;
    dxSum += dx;
    dySum += dy;
    motionDetect = true;
  } else {
    motionDetect = false;
  }

  unsigned long now = millis();
  if (motionDetect && !motionDetect_Old) {
    // Start Reading
    dt_halfSwing = now - lastTimeEnd;
    lastTimeStart = now;
    phase = 0;
  } else if (motionDetect && motionDetect_Old) {
    phase = 1;
    // current motion
  } else if (!motionDetect && motionDetect_Old) {
    lastTimeEnd = now;
    dt_motion = now - lastTimeStart;
    phase = 2;
  } else if (!motionDetect && !motionDetect_Old) {
    phase = 3;
    if (dxSum != 0 && dySum != 0)
      deg = atan2f(dxSum, dySum);
    dxSum = 0;
    dySum = 0;

    // begin new half swing
  }

  if (millis() < tPrint + 25) {
    tPrint = millis();
    // Serial.print(" dx=");
    // Serial.print(dx);
    // Serial.print(" dy=");
    // Serial.print(dy);
    Serial.print(" x=");
    Serial.print(x);
    Serial.print(" y=");
    Serial.print(y);
    Serial.print(" dt_motion=");
    Serial.print(dt_motion);
    Serial.print(" dt_halfSwing=");
    Serial.print(dt_halfSwing);
    Serial.print(" phase=");
    Serial.print(phase);
    Serial.print(" deg=");
    Serial.print(deg);


    Serial.println();
  }
}

// /**
//  * Example of utilities usage
//  */
// void TaskVariableRead(void *pvParameters) {
//   (void)pvParameters;  // remove warnings
// }
