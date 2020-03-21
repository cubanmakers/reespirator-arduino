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

#define DEFAULT_PRESSURE_V_FLUX_K1 1   //Constante proporcional que relaciona presión con caudal

#define FLOW__INSUFLATION_TRIGGER_LEVEL 3.0   //LPM

// Ventilation cycle timing
#define VENTILATION_CYCLE__STOP_INSUFLATION_TIME 500    //msec

// PID limits and constants
#define PID_MIN 0
#define PID_MAX 0

#define PID_P 1.0
#define PID_I 1.0
#define PID_D 1.0
#define PID_dt 1        //msec

#define PID_KP PID_P
#define PID_KI (PID_I * PID_dt)
#define PID_KD (PID_D / PID_dt)


#endif // DEFAULTS_H
