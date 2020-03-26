
#ifndef _DEFAULTS_H
#define _DEFAULTS_H

#undef I2C // definido = Display i2c, sin definir Display parallel

// Time base. Periodo de llamada a mechVentilation.update
#define TIME_BASE 5                        // milliseconds
#define SENSORS_PERIOD_READING 3           // milliseconds

// Stepper motor
#define STEPPER_MICROSTEPS 4               // microsteps per step
#define STEPPER_STEPS_PER_REVOLUTION 200   // steps
#define STEPPER_HOMING_DIRECTION -1        // +1 or -1
#define STEPPER_HOMING_SPEED 50            // steps/sec
#define STEPPER_LOWEST_POSITION 77         // steps
#define STEPPER_HIGHEST_POSITION -128      // steps
#define STEPPER_SPEED_INSUFFLATION 12000L  // steps/sec
#define STEPPER_SPEED_EXSUFFLATION 1000L   // steps/sec
#define STEPPER_ACC_INSUFFLATION 900       // steps/sec2
#define STEPPER_ACC_EXSUFFLATION 500       // steps/sec2

// Valores paciente por defecto
#define DEFAULT_HEIGHT 170                 // cm
#define DEFAULT_SEX 0                      // 0: male, 1: female
#define DEFAULT_ML_TO_KG_IDEAL_WEIGHT 7    // ml per kg
#define DEFAULT_MAX_TIDAL_VOLUME 800       // ml
#define DEFAULT_MIN_TIDAL_VOLUME 240       // ml
#define DEFAULT_TRIGGER_THRESHOLD 3.0      // liters per minute

// Control de ciclos
#define DEFAULT_RPM 15                     // breaths per minute
#define DEFAULT_MAX_RPM 28                 // breaths per minute
#define DEFAULT_MIN_RPM 3                  // breaths per minute
#define DEFAULT_INSPIRATORY_FRACTION (1/3) // 0-1
#define DEFAULT_RETAIN_INSIPIRATION 1000   // mseg

// Presión
#define DEFAULT_PA_TO_CM_H20 0.0102F
// @dc esta constante no se usará cuando tengamos un tercer
// sensor de presión que mida la presión absoluta en tiempo real.
#define DEFAULT_ABSOLUTE_PRESSURE 98290.0F // Pa
// @dc esta constante ha de revisarse cuando se modele correctamente
// el flujo en función de la diferencia de presión
#define DEFAULT_K_PA_TO_LPM 0.9423        // liters per minute


// ----------------------------------------------------------------
// @dc A partir de aquí, estas constantes son de la regulación PID,
// estas constantes no están implementadas
// ----------------------------------------------------------------


// PID constants
#define PID_MIN 0
#define PID_MAX 100

#define PID_P 1.0
#define PID_I 0.0 //TODO To be adjusted
#define PID_D 0.0 //TODO To be adjusted
#define PID_dt TIME_BASE //msec. I hast to match the MechVentilation.update() refresh period.

#define PID_KP PID_P
#define PID_KI (PID_I * PID_dt)
#define PID_KD (PID_D / PID_dt)

//Volume to Position and viceversa constants
#define STEPS_FOR_TOTALLY_PRESSED_AMBU (STEPPER_STEPS_PER_REVOLUTION / 2) //steps
#define VOLUME_FOR_TOTALLY_PRESSED_AMBU 800                               //ml
#define K_VOL2POS (STEPS_FOR_TOTALLY_PRESSED_AMBU / VOLUME_FOR_TOTALLY_PRESSED_AMBU)
#define K_POS2VOL (VOLUME_FOR_TOTALLY_PRESSED_AMBU / STEPS_FOR_TOTALLY_PRESSED_AMBU)
#define K_FLOW2SPEED (25/12)

// @fm superñapa. parametrizar desde el inicio. se usa en mechVentilation
#define ventilationCycle_WaitBeforeInsuflationTime  800 //ms TODO parameter to mechVent
#define totalPatientVolume 0.8 // liters

#endif // DEFAULTS_H
