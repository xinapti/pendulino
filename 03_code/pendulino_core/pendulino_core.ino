/* Firmware code for pendulinum */

#include <ADNS3080.h>
// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>

#include <math.h>
#include "circularBuffer/CircularBuffer.h"

// SPI pins:
#define PIN_RESET 9
#define PIN_CS 10

// Proj Const
#define MotionTrigger 1        //dist() res trigger
#define pendant_diameter 0.05  // 5cm


void (*resetFunc)(void) = 0;  //declare reset function @ address 0


ADNS3080<PIN_RESET, PIN_CS> sensor;

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
} sample;
CircularBuffer<sample, 100> read;

enum swingPhase { Unknow,
                  MotionUp,
                  SwingUp,
                  MotionDw,
                  SwingDw };

typedef struct _stateSample {
  swingPhase phase;
  unsigned long tSet;
  void print() {
    Serial.print("Phase: ");
    Serial.print(phase);
    Serial.print(" tSet: ");
    Serial.println(tSet);
  }
} stateSample;

CircularBuffer<stateSample, 10> phaseBuf;
stateSample phaseCur;
void setup() {
  sensor.setup();
  Serial.begin(115200);
  Serial.println("Start");
  read.memClean();
  phaseCur.phase = Unknow;
  phaseCur.tSet = 0;
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

void loop() {
  if (Serial.available())
    resetFunc();
  while (millis() < s.tSamp + 1) {}
  s.tSamp = millis();
  sensor.displacement(&s.dx, &s.dy);
  read.putF(s);

  sWind.clear();
  for (int i = 0; i < 10; i++) {
    h = read.readFromHeadIndex(i);
    sWind.dx += h.dx;
    sWind.dy += h.dy;
  }
  sWind.tSamp = h.tSamp;  // Time last sample

  switch (phaseCur.phase) {
    case Unknow:
      if (sWind.dist() < MotionTrigger) {
        read.memClean();
        phaseBuf.memClean();
        // lastTimeStartMotion = 0;
        // lastTimeEndMotion = 0;
        // lastTimeStartSwing = 0;
        // lastTimeEndSwing = 0;
        break;
      }
      phaseCur.phase = MotionUp;
      phaseCur.tSet = sWind.tSamp;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      // lastTimeStartMotion = sWind.tSamp;
      // lastTimeEndMotion = 0;
      // lastTimeStartSwing = 0;
      // lastTimeEndSwing = 0;
      break;
    case MotionUp:
      //TODO: fare funzione distanza e mettere dP maggiore di 1 per movimento
      if (sWind.dist() > MotionTrigger)
        break;
      phaseCur.phase = SwingUp;
      phaseCur.tSet = sWind.tSamp;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      // lastTimeStartSwing = sWind.tSamp;
      break;
    case SwingUp:
      //TODO: fare funzione distanza e mettere dP minore di 1 per movimento
      if (sWind.dist() <= MotionTrigger)
        break;
      // state = MotionDw;
      phaseCur.phase = MotionDw;
      phaseCur.tSet = s.tSamp;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      // lastTimeEndSwing = s.tSamp;
      // lastTimeStartMotion = s.tSamp;
      break;
    case MotionDw:
      //TODO: fare funzione distanza e mettere dP maggiore di 1 per movimento
      if (sWind.dist() > MotionTrigger)
        break;
      phaseCur.phase = SwingDw;
      phaseCur.tSet = sWind.tSamp;
      phaseBuf.putF(phaseCur);
      stateChange = true;

      // lastTimeEndMotion = sWind.tSamp;
      // lastTimeStartSwing = sWind.tSamp;
      break;
    case SwingDw:
      //TODO: fare funzione distanza e mettere dP minore di 1 per movimento
      if (sWind.dist() <= MotionTrigger)
        break;
      phaseCur.phase = MotionUp;
      phaseCur.tSet = s.tSamp;
      phaseBuf.putF(phaseCur);
      stateChange = true;

      // state = MotionUp;
      // lastTimeEndSwing = s.tSamp;
      // lastTimeStartMotion = s.tSamp;
      break;
    default:
      phaseCur.phase = Unknow;
      break;
  }

  if (stateChange) {
    stateChange = false;
    //enum swingPhase { Unknow, MotionUp, SwingUp, MotionDw, SwingDw };
    // Serial.print("sWind.dx: ");
    // Serial.print(sWind.dx);
    // Serial.print(" sWind.dy: ");
    // Serial.print(sWind.dy);
    // Serial.print(" sWind.dist(): ");
    // Serial.print(sWind.dist());
    // Serial.print("; Currente Phase: ");
    // Serial.print(phaseCur.phase);
    // Serial.print("  detect ");
    stateSample s0, s1, s2, s3;
    switch (phaseCur.phase) {
      case Unknow:
        Serial.println("Unknow");
        break;
      case SwingUp:
        //dt_motion = lastTimeEndMotion - lastTimeStartMotion;
        s0 = phaseBuf.readFromHeadIndex(0);
        s1 = phaseBuf.readFromHeadIndex(1);
        s2 = phaseBuf.readFromHeadIndex(2);
        s3 = phaseBuf.readFromHeadIndex(3);
        phaseCur.print();
        s0.print();
        s1.print();
        s2.print();
        s3.print();
        Serial.println();
        dt_motion = s0.tSet - s1.tSet;
        dt_halfSwing = s1.tSet - s2.tSet;
        deg = sWind.rad();
        vel = (pendant_diameter * 100) / (1000 * dt_motion);  // 2cm Diametro rondella

        // Serial.print("  dt_halfSwing: ");
        // Serial.print(dt_halfSwing);
        // Serial.print(" (");
        // Serial.print(deg);
        // Serial.print(" °)");

        // Serial.print("  dt_motion: ");
        // Serial.print(dt_motion);
        // Serial.print(" (");
        // Serial.print(vel);
        // Serial.println(" cm/s)");

        break;
      case MotionUp:
      case MotionDw:
      case SwingDw:
        Serial.println();
        break;
    }
  }
}
