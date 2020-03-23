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
 * @brief calcula los tiempos de ciclo, inspiratorio y espiratorio, en seg.
 *
 * Calcula a partir de las respiraciones por minuto, los tiempos de ciclo,
 * inspiratorio y espiratorio, y las velocidades uno y dos.
 * @param speedIns TODO: explicación?
 * @param speedEsp TODO: explicación?
 * @param tIns tiempo de inspiracion, en segundos
 * @param tEsp tiempo de espiracion, en segundos
 * @param tCiclo tiempo de ciclo, en segundos
 * @param porcentajeInspiratorio fraccion del ciclo en la que se inspira, tIns/tCiclo*100
 * @param rpm respiraciones por minuto
 */
void calcularCicloInspiratorio(float *speedIns, float *speedEsp,
                               float *tIns, float *tEsp, float *tCiclo,
                               float porcentajeInspiratorio, int rpm);

/**
 * @brief estima el caudal a partir de la diferencia de presión
 *
 * @param pressure1 presión a un lado
 * @param pressure2 presión a otro lado
 * @param flux caudal resultante
 */
float getCurrentFlow(float pressure1, float pressure2);

/**
 * @brief Constrains the value within the limits
 */
float constrainFloat(float value, float lowLimit, float highLimit);

float computePID(float setpoint, float feedbackInput);
/**
 * @brief Refresca el WDT (Watch Dog Timer)
 */
void refreshWatchDogTimer();

float vol2pos(float volume);
float pos2vol(float position);
float flow2speed(float flow);
int integratorFlowToVolume(float currentFlow);
void integratorFlowToVolume(float* currentVolume, float currentFlow);
void resetPID();
float curveInterpolator(float inValue, float currentProgressFactor);

#endif // CALC_H
