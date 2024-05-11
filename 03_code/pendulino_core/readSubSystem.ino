
// SPI pins:
#define PIN_RESET 26
#define PIN_CS 5


ADNS3080<PIN_RESET, PIN_CS> sensor;
CircularBuffer<sample, 10000> readBuf;

void init_readSubSystem() {
  // Optical mouse sensor
  sensor.setup();
  sensor.writeRegister(ADNS3080_FRAME_PERIOD_LOW, 0x7E);
  sensor.writeRegister(ADNS3080_FRAME_PERIOD_HIGH, 0x0E);
  restart_readSubSystem();
}

void restart_readSubSystem() {
  // Store reading buffer
  readBuf.memClean();
}


void waitNextSample(unsigned long millisDelay) {
  sample lastSample = readBuf.readHead();
  // Wait new samle moment
  while (millis() < lastSample.tSamp + millisDelay) {}
  lastSample.tSamp = millis();
  // Read and store the new sample
  sensor.displacement(&lastSample.dx, &lastSample.dy);
  readBuf.putF(lastSample);
}


sample *cumulativeMove(sample *sSum, unsigned int nOldSample) {
  sample h;
  // Generate current decision window samble sum, and set time to most recent sample
  sSum->clear();
  sSum->tSamp = readBuf.readFromHeadIndex(0).tSamp;  // Most recent time
  for (int i = 0; i < 100; i++) {
    h = readBuf.readFromHeadIndex(i);
    sSum->add(h);
  }
  return sSum;
}