
CircularBuffer<stateSample, 50> phaseBuf;

void init_phaseMngSubSystem() {
  phaseBuf.memClean();
}

void storeNewState(stateSample &newState) {
  phaseBuf.putF(newState);
}

void getState(stateSample &getState, unsigned int nOldState) {
  getState.clear();
  getState = phaseBuf.readFromHeadIndex(nOldState);
}
