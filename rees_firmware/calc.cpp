#include "calc.h"

/**
 * @brief estima el volumen tidal en función de estatura y sexo, en ml.
 *
 * @param estatura en cm, del paciente
 * @param sexo 0: varón, 1: mujer, sexo del paciente
 * @return *volumenTidal volumen tidal estimado, en mililitros
 */
int estimateTidalVolume(int estatura, int sexo) {
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
 * @brief Refresca el WDT (Watch Dog Timer)
 */
void refreshWatchDogTimer() {
  //TODO implementar
}

// float flow2Position(float flow) { //returns position
//   //TODO implementar
//   return flow;
// }

void integratorFlowToVolume(float* currentVolume, float currentFlow) {
  //We add to currentVolume the ml that flowed in TIME_BASE msec;
  *currentVolume += currentFlow * 60 * TIME_BASE;
  //currentVolume += currentFlow * (1000/1) * (60/1) * (1/1000)   * TIME_BASE;
}                 //  [l/m]       [l]/[ml]  [min]/[s]  [s]/[msec]   [msec]


#if 0
/**
 * @brief Filtro de media móvil, de low-pass filter, para la diferencia de presiones
 *
 * @param parameter variable a filtrar
 * @param lpfArray array con muestras anteriores
 * @return float filtered value
 */
float computeLPF(int parameter, int lpfArray[])
{
  int samples = sizeof(lpfArray);
  int k = samples - 1;           // tamano de la matriz
  int cumParameter = parameter; // el acumulador suma ya el param
  lpfArray[0] = parameter;
  while (k > 0)
  {
    lpfArray[k] = lpfArray[k - 1];   // desplaza el valor una posicion
    cumParameter += lpfArray[k];     // acumula valor para calcular la media
    k--;
  }
  float result = cumParameter / samples;
  return result;
}
#endif
