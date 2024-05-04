/* Firmware code for pendulinum */
#include "ADNS3080/src/ADNS3080.h"
// Include Arduino FreeRTOS library
// #include <Arduino_FreeRTOS.h>

#include <math.h>
#include "circularBuffer/CircularBuffer.h"

// SPI pins:
#define PIN_RESET 26
#define PIN_CS 5

#define tSampleMillis 1


void (*resetFunc)(void) = 0;  //declare reset function @ address 0


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
  unsigned int nSmp{ 0 };
  void clear() {
    smp.clear();
    phase = Unknow;
    nSmp = 0;
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

CircularBuffer<sample, 10000> readBuf;
CircularBuffer<stateSample, 50> phaseBuf;
stateSample phaseCur;


sample s;
sample sWind;
sample h;
stateSample s0, s1, s2, sSum;

void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  sensor.setup();
  sensor.writeRegister(ADNS3080_FRAME_PERIOD_LOW, 0x7E);
  sensor.writeRegister(ADNS3080_FRAME_PERIOD_HIGH, 0x0E);
  readBuf.memClean();
  phaseCur.phase = Unknow;
  phaseCur.smp.clear();
  phaseBuf.memClean();
  s.clear();
  Serial.println("End Setup");
}



// Initial position
unsigned long dt_motion = 0;
unsigned long dt_halfSwing = 0;
unsigned long counterNoChange = 0;
double deg = 0;
double vel = 0;

//TODO: Alla versione attuale misuriamo abbastanza bene la velocità con questi 40 campioni.
// Vorremmo poter sommare tutti campioni nelle varie finestre di swing, per avere il dx e dy totale dello swing

void loop() {

  // Reset from terminal
  if (Serial.available()) {
    Serial.println("\n\nRESET!!!!");
    delay(1000);
    resetFunc();
  }
  // Wait new samle moment
  while (millis() < s.tSamp + tSampleMillis) {}
  s.tSamp = millis();

  // Read and store the new sample
  sensor.displacement(&s.dx, &s.dy);
  readBuf.putF(s);

  // Generate current decision window samble sum, and set time to most recent sample
  sWind.clear();
  sWind.tSamp = readBuf.readFromHeadIndex(0).tSamp;  // Most recent time
  for (int i = 0; i < 100; i++) {
    h = readBuf.readFromHeadIndex(i);
    sWind.add(h);
  }

  // sWind.print();
  // Serial.println();
  // circPrint(read, 10);


  if (phaseUpdate()) {
    //enum swingPhase { Unknow, MotionUp, SwingUp, MotionDw, SwingDw };
    // sWind.print();
    // Serial.print("; Currente Phase: ");
    // phaseCur.print();
    counterNoChange = 0;
    switch (phaseCur.phase) {
      case Unknow:
        break;
      case SwingUp:
        s0 = phaseBuf.readFromHeadIndex(0);
        s1 = phaseBuf.readFromHeadIndex(1);
        s2 = phaseBuf.readFromHeadIndex(2);
        dt_motion = s0.smp.tSamp - s1.smp.tSamp;
        dt_halfSwing = s1.smp.tSamp - s2.smp.tSamp;
        vel = (pendant_diameter * 100.0) / (0.001 * (double)dt_motion);
        // Calcolo angolo su ultimi 10 stati
        sSum.clear();
        sSum = s1;
        for (int i = 0; i < 10; i++) {
          s0 = phaseBuf.readFromHeadIndex(i * 4);
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
        Serial.print(" nSmp: ");
        Serial.print(phaseCur.nSmp);
        break;
    }
    Serial.println();
  } else {
    phaseCur.nSmp++;
    counterNoChange++;
    if (counterNoChange > 10000) {
      Serial.println("No status change detect for 10000 iteration!");
      counterNoChange = 0;
    }
  }
}


bool phaseUpdate() {
  bool stateChange = false;
  switch (phaseCur.phase) {
    case Unknow:
      if (sWind.dist() < MotionTrigger) {
        readBuf.memClean();
        phaseBuf.memClean();
        break;
      }
      phaseCur.clear();
      phaseCur.phase = MotionUp;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case MotionUp:
      if (sWind.dist() > MotionTrigger)
        break;
      phaseCur.clear();
      phaseCur.phase = SwingUp;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case SwingUp:
      if (sWind.dist() <= MotionTrigger)
        break;
      phaseCur.clear();
      phaseCur.phase = MotionDw;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case MotionDw:
      //Serial.println("mDw");
      if (sWind.dist() > MotionTrigger)
        break;
      phaseCur.clear();
      phaseCur.phase = SwingDw;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    case SwingDw:
      if (sWind.dist() <= MotionTrigger)
        break;
      phaseCur.clear();
      phaseCur.phase = MotionUp;
      phaseCur.smp = sWind;
      phaseBuf.putF(phaseCur);
      stateChange = true;
      break;
    default:
      phaseCur.clear();
      break;
  }
  return stateChange;
}
