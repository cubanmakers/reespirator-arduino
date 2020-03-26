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

/**
 * @brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * @param estatura en cm, del paciente
 * @param sexo 0: varón, 1: mujer, sexo del paciente
 * @return *volumenTidal volumen tidal estimado, en mililitros
 */
int calcularVolumenTidal(int estatura, int sexo);

/**
 * @brief Constrains the value within the limits
 */
float constrainFloat(float value, float lowLimit, float highLimit);

float computePID(float setpoint, float feedbackInput);
/**
 * @brief Refresca el WDT (Watch Dog Timer)
 */
void refreshWatchDogTimer();

float pos2vol(float position);
float flow2speed(float flow);
int integratorFlowToVolume(float currentFlow);
void integratorFlowToVolume(float* currentVolume, float currentFlow);
float curveInterpolator(float inValue, float currentProgressFactor);

#endif // CALC_H
