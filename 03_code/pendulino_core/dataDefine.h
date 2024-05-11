#ifndef DATA_DEFINE
#define DATA_DEFINE

#include "Arduino.h"

// TaskHandle_t taskRead;
typedef struct _sample {
  int8_t dx, dy;  // Displacement since last function call
  unsigned long tSamp;
  void clear() {
    dx = 0;
    dy = 0;
    tSamp = 0;
  }
  void add(_sample &addSmp) {
    dx += addSmp.dx;
    dy += addSmp.dy;
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

enum swingPhase {
  Unknow,
  MotionUp,
  SwingUp,
  MotionDw,
  SwingDw
};

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
  //TODO: Modificare smp affinchè diventi la sommatoria dei sample in quello stato
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

#endif