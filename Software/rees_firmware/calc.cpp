#include "calc.h"

/**
 * @brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * @param estatura en cm, del paciente
 * @param sexo 0: varón, 1: mujer, sexo del paciente
 * @return *volumenTidal volumen tidal estimado, en mililitros
 */
int calcularVolumenTidal(int estatura, int sexo) {
  float peso0, pesoIdeal, volumenEstimado;
  if (sexo == 0) { // Varón
    peso0 = 50.0;
  } else if (sexo == 1) { // Mujer
    peso0 = 45.5;
  }
  pesoIdeal = peso0 + 0.91 * (estatura - 152.4); // en kg

  return ((int)(round(pesoIdeal * DEFAULT_ML_POR_KG_DE_PESO_IDEAL)));
}

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
 * @param pasosPorRevolucion TODO: explicación?
 * @param microStepper TODO: explicación?
 * @param porcentajeInspiratorio fraccion del ciclo en la que se inspira, tIns/tCiclo*100
 * @param rpm respiraciones por minuto
 */
void calcularCicloInspiratorio(float* speedIns, float* speedEsp,
                               float* tIns, float* tEsp, float* tCiclo,
                               int pasosPorRevolucion, int microStepper,
                               int porcentajeInspiratorio, int rpm) {
  *tCiclo = 60 / rpm; // Tiempo de ciclo en segundos
  *tIns = *tCiclo * porcentajeInspiratorio/100;
  *tEsp = *tCiclo - *tIns;

  *speedIns = (pasosPorRevolucion * microStepper / 2) / *tIns; // TODO: unidades?
  *speedEsp = (pasosPorRevolucion * microStepper / 2) / *tEsp; // TODO: unidades?
}

/**
 * @brief estima el caudal a partir de la diferencia de presión
 *
 * @param pressure1 presión a un lado
 * @param pressure2 presión a otro lado
 * @param flux caudal resultante
 */
float getCurrentFlow(float pressure1, float pressure2) {
  float flow = (pressure1 - pressure2) * DEFAULT_PRESSURE_V_FLUX_K1;
  return flow;
}

/**
 * @brief Constrains the value within the limits
 */
float constrainFloat(float value, float lowLimit, float highLimit) {
  if (value < lowLimit) {
    value = lowLimit;
  } else if (value > highLimit) {
    value = highLimit;
  }
  return value;
}

/**
 * PID step calculation
 */
float proportional = 0;
float integral = 0;
float derivative = 0;
float previous_feedbackInput = 0;

float computePID(float setpoint, float feedbackInput) {
  //dt is fixed to 1 msec by timer interrupt

  float error = setpoint - feedbackInput;

  proportional = PID_KP * error;
  integral += PID_KI * error;
  integral = constrainFloat(integral, PID_MIN, PID_MAX);
  derivative = -PID_KD * (feedbackInput - previous_feedbackInput);

  float output = constrainFloat((proportional + integral + derivative), PID_MIN, PID_MAX);

  // Guardar ultimo tiempo y error
  previous_feedbackInput = feedbackInput;
  return output;
}

/**
 * @brief Refresca el WDT (Watch Dog Timer)
 */
void refreshWatchDogTimer() {
  //TODO implementar
}

// float flow2Position(float flow) { //returns position
//   //TODO implementar
//   return flow;
// }

float vol2pos(float volume) { //converts volume [litres] to position [steps]
  //TODO improve with LUT to linearize if needed
  float position = volume * K_VOL2POS;

  return position;
}

float pos2vol(float position) { //converts volume [litres] to position [steps]
  //TODO improve with LUT to linearize if needed
  float volume = position * K_POS2VOL;

  return volume;
}



