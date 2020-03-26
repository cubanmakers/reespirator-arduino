#include "calc.h"

/**
 * @brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * @param height - height of the patient, cm
 * @param sex - 0: male patient, 1: female patient
 * @return tidalVolume - estimated tidal volume, in ml
 */
int calculateTidalVolume(int height, int sex)
{
  int tidalVolume;
  float weight0, idealWeight;
  if (sex == 0)
  {
    weight0 = 50.0;
  }
  else if (sex == 1)
  {
    weight0 = 45.5;
  }

  idealWeight = weight0 + 0.91 * (height - 152.4); // en kg
  tidalVolume = ((int)(round(idealWeight * DEFAULT_ML_TO_KG_IDEAL_WEIGHT)));

  return tidalVolume;
}

/**
 * @brief calcula los tiempos de ciclo, inspiratorio y espiratorio, en seg.
 *
 * Calcula a partir de las respiraciones por minuto, los tiempos de ciclo,
 * inspiratorio y espiratorio, y las velocidades uno y dos.x
 * @param tIns tiempo de inspiracion, en segundos
 * @param tEsp tiempo de espiracion, en segundos
 * @param tCycle tiempo de ciclo, en segundos
 * @param inspiratoryFraction fraccion del ciclo en la que se inspira, tIns/tCycle
 * @param rpm respiraciones por minuto
 */
void calculateInspiratoryCycle(float *tIns, float *tEsp, float *tCycle,
                               float inspiratoryFraction, int rpm)
{
  *tCycle = 60.0 / float(rpm); // Tiempo de ciclo en segundos
  *tIns = *tCycle * inspiratoryFraction;
  *tEsp = *tCycle - *tIns;
}

/**
 * @brief Filtro de media móvil, de low-pass filter, para la diferencia de presiones
 *
 * @param value variable a filtrar
 * @param lpfArray array con muestras anteriores
 * @return float filtered value
 */
float computeLPF(float value, float *lpfArray, int samples)
{
  int k = samples - 1;        // tamaño de la matriz
  float cumParameter = value; // el acumulador suma ya el valor
  lpfArray[0] = value;
  while (k > 0)
  {
    lpfArray[k] = lpfArray[k - 1]; // desplaza el valor una posicion
    cumParameter += lpfArray[k];   // acumula valor para calcular la media
    k--;
  }
  float result = cumParameter / samples;
  return result;
}


// ----------------------------------------------------------------
// @dc A partir de aquí, estas funciones son de la regulación PID,
// estas funciones no están implementadas
// ----------------------------------------------------------------


/**
 * @brief Constrains the value within the limits
 */
float constrainFloat(float value, float lowLimit, float highLimit)
{
  if (value < lowLimit)
  {
    value = lowLimit;
  }
  else if (value > highLimit)
  {
    value = highLimit;
  }
  return value;
}

/**
 * PID step calculation
 */
float proportional = 0;
static float integral = 0;
float derivative = 0;
float previous_feedbackInput = 0;

void resetPID()
{
  integral = 0;
}

float computePID(float setpoint, float feedbackInput)
{
  //dt is set to TIME_BASE msec by timer interrupt

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
void refreshWatchDogTimer()
{
  //TODO implementar
}

// float flow2Position(float flow) { //returns position
//   //TODO implementar
//   return flow;
// }

float vol2pos(float volume)
{ //converts volume [ml] to position [steps]
  //TODO improve with LUT to linearize if needed
  //float position  = volume * (1/800)                 * (100/1);
  //      [steps]      [ml]  [fully_pressed_ambu/ml]   [steps/fully_pressed_ambu]
  //float position  = volume * (1/8);
  //  #define K_VOL2POS (STEPS_FOR_TOTALLY_PRESSED_AMBU / VOLUME_FOR_TOTALLY_PRESSED_AMBU)
  //  K_VOL2POS = (1/8);

  float position = volume * K_VOL2POS;

  return position;
}

float pos2vol(float position)
{ //converts position [steps] to volume [ml]
  float volume = position * K_POS2VOL;

  return volume;
}

void integratorFlowToVolume(float *currentVolume, float currentFlow)
{
  //We add to currentVolume the ml that flowed in TIME_BASE msec;
  *currentVolume += currentFlow * 60 * TIME_BASE;
  //currentVolume += currentFlow * (1000/1) * (60/1) * (1/1000)   * TIME_BASE;
} //  [l/m]       [l]/[ml]  [min]/[s]  [s]/[msec]   [msec]

//float curveInterpolator(int[] curve, float min, float max, float currentProgressFactor);
//float curveInterpolator(float maxValue, float currentProgressFactor);
float curveInterpolator(float inValue, float currentProgressFactor)
{
  float outValue = 0;

  if (currentProgressFactor < 0.03)
  {
    outValue = 0.7 * inValue;
  }
  else if ((currentProgressFactor >= 0.03) && (currentProgressFactor < 0.97))
  {
    outValue = 1.2 * inValue;
  }
  else if (currentProgressFactor >= 0.97)
  {
    outValue = 0.7 * inValue;
  }

  return outValue;
}

float flow2speed(float flow)
{ //converts flow [LPM] to speed [steps/sec]
  //float speed  = flow *  (1/60) * (1000/1) * (1/800)                 * (100/1);
  //     [steps/s]  [l/m]  [min/sec] [ml/l]    [fully_pressed_ambu/ml]   [steps/fully_pressed_ambu]
  //float position  = volume * (25/12);
  //  #define K_VOL2POS (STEPS_FOR_TOTALLY_PRESSED_AMBU / VOLUME_FOR_TOTALLY_PRESSED_AMBU)
  //  K_FLOW2SPEED = (25/12);

  float speed = flow * K_FLOW2SPEED;

  return speed;
}
