/* Firmware code for pendulinum */
#include "ADNS3080/src/ADNS3080.h"
// Include Arduino FreeRTOS library
// #include <Arduino_FreeRTOS.h>

#include <math.h>
#include "circularBuffer/CircularBuffer.h"

#include "dataDefine.h"

// Proj Const
#define MotionTrigger 0.5      //dist() res trigger
#define pendant_diameter 0.05  // 5cm
#define tSampleMillis 1

void (*resetFunc)(void) = 0;  //declare reset function @ address 0

// Print 0->elem item in circular buffer, use for debug
#define circPrint(bufCirc, elem) \
  do { \
    for (int i = 0; i < elem; i++) { \
      Serial.print(i); \
      Serial.print(") "); \
      bufCirc.readFromHeadIndex(i).print(); \
      Serial.println(); \
    } \
  } while (false);


void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  init_readSubSystem();
  init_phaseMngSubSystem();
  Serial.println("End Setup");
}


unsigned long counterNoChange = 0;
// Initial position
unsigned long dt_motion = 0;
unsigned long dt_halfSwing = 0;
double deg = 0;
double vel = 0;

//TODO: Alla versione attuale misuriamo abbastanza bene la velocità con questi 40 campioni.
// Vorremmo poter sommare tutti campioni nelle varie finestre di swing, per avere il dx e dy totale dello swing
stateSample s0, s1, s2, sSum;
sample sWind;
void loop() {
  // Reset from terminal
  if (Serial.available()) {
    Serial.println("\n\nRESET!!!!");
    delay(1000);
    resetFunc();
  }
  // Execute new read
  waitNextSample(tSampleMillis);
  cumulativeMove(&sWind, 100);
  // State update
  if (phaseUpdate(&sWind)) {
    counterNoChange = 0;
    stateSample phaseCur;
    getState(phaseCur, 0);

    switch (phaseCur.phase) {
      case Unknow:
        break;
      case SwingUp:
        getState(s0, 0);
        getState(s1, 1);
        getState(s2, 2);
        dt_motion = s0.smp.tSamp - s1.smp.tSamp;
        dt_halfSwing = s1.smp.tSamp - s2.smp.tSamp;
        vel = (pendant_diameter * 100.0) / (0.001 * (double)dt_motion);
        // Calcolo angolo su ultimi 10 stati
        // TODO: Modificare, meglio vedere tutti i sample di questo stato
        sSum.clear();
        sSum = s1;
        for (int i = 0; i < 10; i++) {
          getState(s0, i * 4);
          sSum.smp.dx += s0.smp.dx;
          sSum.smp.dy += s0.smp.dy;
        }
        deg = sSum.smp.rad();

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
        Serial.print(" cm/s)");
        break;
      case MotionUp:
      case MotionDw:
      case SwingDw:
        printPhase(phaseCur.phase);
        break;
    }
    Serial.println();
  } else {
    counterNoChange++;
    if (counterNoChange > 10000) {
      Serial.println("No status change detect for 10000 iteration!");
      counterNoChange = 0;
    }
  }
}


bool phaseUpdate(sample *decisionWindows) {
  //TODO: modificare la logica, quando il trigger è raggiunto .smp deve
  //      diventare la sommatoria degli spostamenti letti dall'ultimo stato ad ora
  bool stateChange = false;
  stateSample newState;
  newState.clear();

  stateSample nowPhase;
  getState(nowPhase, 0);

  switch (nowPhase.phase) {
    case Unknow:
      if (decisionWindows->dist() < MotionTrigger) {
        restart_readSubSystem();
        break;
      }
      newState.phase = MotionUp;
      newState.smp = *decisionWindows;
      stateChange = true;
      break;
    case MotionUp:
      if (decisionWindows->dist() > MotionTrigger)
        break;
      newState.phase = SwingUp;
      newState.smp = *decisionWindows;
      stateChange = true;
      break;
    case SwingUp:
      if (decisionWindows->dist() <= MotionTrigger)
        break;
      newState.phase = MotionDw;
      newState.smp = *decisionWindows;
      stateChange = true;
      break;
    case MotionDw:
      if (decisionWindows->dist() > MotionTrigger)
        break;
      newState.phase = SwingDw;
      newState.smp = *decisionWindows;
      stateChange = true;
      break;
    case SwingDw:
      if (decisionWindows->dist() <= MotionTrigger)
        break;
      newState.phase = MotionUp;
      newState.smp = *decisionWindows;
      stateChange = true;
      break;
    default:
      break;
  }
  if (stateChange) {
    storeNewState(newState);
  }

  return stateChange;
}
