
//#include "src/AccelStepper/AccelStepper.h"
#include "FlowAdapter.h"

FlowAdapter flowAdapter;

void setup() {
  flowAdapter.init();
}

void loop() {
  flowAdapter.readFlow();
  delay(100);
}
