#include "FlowAdapter.h"
#include <Wire.h>
#include "src/flow/SFM3200/sfm3000wedo.h"

SFM3000wedo measflow(64);

int offset = 32000; // Offset for the sensor
float scale = 140.0; // Scale factor for Air and N2 is 140.0, O2 is 142.8


FlowAdapter::FlowAdapter(void) {
  
}

void FlowAdapter::init (void) {
  Wire.begin();
  #ifdef DEBUG_SERIAL
  Serial.begin(9600);
  #endif
  delay(500); // let serial console settle


  // initialize the sesnor
  measflow.init();
  #ifdef DEBUG_SERIAL
  Serial.println("Sensor initialized!");
  #endif

}

float FlowAdapter::readFlow(void) {
  unsigned int result = measflow.getvalue();

  float flow = ((float)result - offset) / scale;
#ifdef DEBUG_SERIAL
  Serial.print("Flow: ");
  if (result >= 0) {
    Serial.print(" ");
  }
  Serial.print(flow, 5);
  Serial.print("\n");
#endif
  return flow;

}
