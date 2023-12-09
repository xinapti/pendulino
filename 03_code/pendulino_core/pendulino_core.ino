/* Firmware code for pendulinum */

#include "ADNS3080/src/ADNS3080.h"
// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>

#include <math.h>
#include "circularBuffer/CircularBuffer.h"

// SPI pins:
#define PIN_RESET 9
#define PIN_CS 10


void (*resetFunc)(void) = 0;  //declare reset function @ address 0


TaskHandle_t taskRead;
typedef struct _sample {
  int8_t dx, dy;  // Displacement since last function call
  unsigned long tSamp;
  void clear() {
    dx = 0;
    dy = 0;
    tSamp = 0;
  }
  double dist() {
    // Calcola la distanza euclidea utilizzando il teorema di Pitagora
    return sqrt(sq(dx) + sq(dy));
  }
  double rad() {
    return atan2f(dx, dy);
  }
  void print() {
    Serial.print("dx: ");
    Serial.print(dx);
    Serial.print(" dy: ");
    Serial.print(dy);
    Serial.print(" tSamp: ");
    Serial.print(tSamp);
    Serial.print(" dist(): ");
    Serial.print(dist());
    Serial.print(" rad(): ");
    Serial.print(rad());
  }
} sample;



enum swingPhase { Unknow,
                  MotionUp,
                  SwingUp,
                  MotionDw,
                  SwingDw };

void printPhase(swingPhase sp) {
  switch (sp) {
    case Unknow:
      Serial.print("Unknow");
      break;
    case MotionUp:
      Serial.print("MUp");
      break;
    case SwingUp:
      Serial.print("SUp");
      break;
    case MotionDw:
      Serial.print("MDw");
      break;
    case SwingDw:
      Serial.print("SDw");
      break;
  }
  Serial.print(" [");
  Serial.print(sp);
  Serial.print("]");
}

typedef struct _stateSample {
  swingPhase phase;
  sample smp;
  void clear() {
    smp.clear();
    phase = Unknow;
  }
  void print() {
    smp.print();
    Serial.print(" Phase: ");
    printPhase(phase);
  }
} stateSample;

#define circPrint(bufCirc, elem) \
  do { \
    for (int i = 0; i < elem; i++) { \
      Serial.print(i); \
      Serial.print(") "); \
      bufCirc.readFromHeadIndex(i).print(); \
      Serial.println(); \
    } \
  } while (false);


// Proj Const
#define MotionTrigger 0.5      //dist() res trigger
#define pendant_diameter 0.05  // 5cm
ADNS3080<PIN_RESET, PIN_CS> sensor;

CircularBuffer<sample, 50> read;
CircularBuffer<stateSample, 50> phaseBuf;
stateSample phaseCur;

void setup() {
  sensor.setup();
  sensor.writeRegister(ADNS3080_FRAME_PERIOD_LOW, 0x7E);
  sensor.writeRegister(ADNS3080_FRAME_PERIOD_HIGH, 0x0E);
  Serial.begin(115200);
  Serial.println("Start");
  read.memClean();
  phaseCur.phase = Unknow;
  phaseCur.smp.clear();
  phaseBuf.memClean();
}



// Initial position
unsigned long lastTimeStartMotion = 0;
unsigned long lastTimeEndMotion = 0;
unsigned long lastTimeStartSwing = 0;
unsigned long lastTimeEndSwing = 0;

unsigned long dt_motion = 0;
unsigned long dt_halfSwing = 0;

double deg = 0;
double vel = 0;

bool stateChange = false;
sample s;
sample sWind;
sample h;
stateSample s0, s1, s2, sSum;

void loop() {
  if (Serial.available()) {
    Serial.println("\n\nRESET!!!!");
    delay(1000);
    resetFunc();
  }
  while (millis() < s.tSamp + 1) {}
  s.tSamp = millis();
  sensor.displacement(&s.dx, &s.dy);
  read.putF(s);

  sWind.clear();
  for (int i = 0; i < 10; i++) {
    h = read.readFromHeadIndex(i);
    sWind.dx += h.dx;
    sWind.dy += h.dy;
    // h.print();
    // Serial.println();
  }
  sWind.tSamp = read.readFromHeadIndex(0).tSamp;  // Most recent time
  // sWind.print();
  // Serial.println();
  // circPrint(read, 10);

  switch (phaseCur.phase) {
    case Unknow:
      if (sWind.dist() < MotionTrigger) {
        read.memClean();
        phaseBuf.memClean();
        break;
      }
      phaseCur.phase = MotionUp;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case MotionUp:
      if (sWind.dist() > MotionTrigger)
        break;
      phaseCur.phase = SwingUp;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case SwingUp:
      if (sWind.dist() <= MotionTrigger)
        break;
      phaseCur.phase = MotionDw;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case MotionDw:
      //Serial.println("mDw");
      if (sWind.dist() > MotionTrigger)
        break;
      phaseCur.phase = SwingDw;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case SwingDw:
      if (sWind.dist() <= MotionTrigger)
        break;
      phaseCur.phase = MotionUp;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    default:
      phaseCur.phase = Unknow;
      break;
  }

  if (stateChange) {
    stateChange = false;
    //enum swingPhase { Unknow, MotionUp, SwingUp, MotionDw, SwingDw };
    // sWind.print();
    // Serial.print("; Currente Phase: ");
    // phaseCur.print();
    switch (phaseCur.phase) {
      case Unknow:
        break;
      case SwingUp:
        //dt_motion = lastTimeEndMotion - lastTimeStartMotion;
        s0 = phaseBuf.readFromHeadIndex(0);
        s1 = phaseBuf.readFromHeadIndex(1);
        s2 = phaseBuf.readFromHeadIndex(2);
        dt_motion = s0.smp.tSamp - s1.smp.tSamp;
        dt_halfSwing = s1.smp.tSamp - s2.smp.tSamp;
        sSum.clear();
        sSum = s1;
        for (int i = 1; i < 10; i++) {
          s0 = phaseBuf.readFromHeadIndex(i * 4);
          sSum.smp.dx += s0.smp.dx;
          sSum.smp.dy += s0.smp.dy;
        }
        deg = sSum.smp.rad();
        vel = (pendant_diameter * 100.0) / (1.0 * (double)dt_motion);
        sSum.print();
        Serial.print("  dt_halfSwing: ");
        Serial.print(dt_halfSwing);
        Serial.print(" (");
        Serial.print(deg);
        Serial.print(" °)");

        Serial.print("  dt_motion: ");
        Serial.print(dt_motion);
        Serial.print(" (");
        Serial.print(vel);
        Serial.print(" cm/ms)");

        break;
      case MotionUp:
      case MotionDw:
      case SwingDw:
        break;
    }
    Serial.println();
  }
}
