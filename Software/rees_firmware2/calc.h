/** Calculation functions
 *
 * @file calc.h
 *
 *
 */
#ifndef CALC_H
#define CALC_H

#include "defaults.h"
#include "Arduino.h"
//#include <math.h>// o <cmath>

int calcularVolumenTidal(int estatura, int sexo);
void calcularCicloInspiratorio(float *speedIns, float *speedEsp,
                               float *tIns, float *tEsp, float *tCiclo,
                               float porcentajeInspiratorio, int rpm);
float getCurrentFlow(float pressure1, float pressure2);
float constrainFloat(float value, float lowLimit, float highLimit);

float computePID(float setpoint, float feedbackInput);
void refreshWatchDogTimer();
float vol2pos(float volume);
float pos2vol(float position);
float flow2speed(float flow);
int integratorFlowToVolume(float currentFlow);
void integratorFlowToVolume(float* currentVolume, float currentFlow);
void resetPID();
float curveInterpolator(float inValue, float currentProgressFactor);


#endif // CALC_H
