#ifndef DEFAULTS_H
#define DEFAULTS_H

#undef I2C // definido = Display i2c, sin definir Display parallel

// Valores motor
#define DEFAULT_PASOS_POR_REVOLUCION 200 // Suponiendo un motor de 200 pasos/rev sin microstepper
#define DEFAULT_ACELERACION 6000
#define DEFAULT_MICROSTEPPER 16

// Valores por defecto
#define DEFAULT_ESTATURA 170 // cm
#define DEFAULT_SEXO 0 // 0: varón, 1: mujer
#define DEFAULT_ML_POR_KG_DE_PESO_IDEAL 7
#define DEFAULT_MAX_VOLUMEN_TIDAL 800
#define DEFAULT_MIN_VOLUMEN_TIDAL 240
#define DEFAULT_FLUJO_TRIGGER 3
#define DEFAULT_RPM 15
#define DEFAULT_MAX_RPM 24
#define DEFAULT_MIN_RPM 14
#define DEFAULT_POR_INSPIRATORIO 60


#endif // DEFAULTS_H
