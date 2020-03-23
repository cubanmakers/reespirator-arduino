// /** Mechanical ventilation.
//  *
//  * @file MechVentilation.h
//  *
//  * This is the mechanical ventilation software module.
//  * It handles the mechanical ventilation control loop.
//  */
// #ifndef INC_MECHANICAL_VENTILATION_H
// #define INC_MECHANICAL_VENTILATION_H

// #include <inttypes.h>
// #include "src/FlexyStepper/FlexyStepper.h"
// #include "Sensors.h"
// #include "pinout.h"
// #include "defaults.h"

// /** States of the mechanical ventilation. */
// enum State {
//     Init_WaitBeforeInsuflation = 0,
//     State_WaitBeforeInsuflation = 1,    /**< Wait for trigger or timer, if running is true. */
//     Init_Insufflation = 2,
//     State_Insufflation = 3,             /**< Insufflating (PID control). */
//     Init_WaitBeforeExsufflation = 4,
//     State_WaitBeforeExsufflation = 5,   /**< Wait for timer. */
//     Init_Exsufflation = 6,
//     State_Exsufflation = 7,              /**< Return to position 0 and wait for the patient to exsufflate. */
//     State_HOMING = 8
// };

// /**
//  * This is the mechanical ventilation class.
//  */
// class MechVentilation {
// public:
// 	/**
// 	 * Constructor (no trigger).
// 	 *
//      * @param[in]   mlTidalVolume             Tidal volume in millilitres.
// 	 * @param[in]   secTimeoutInsufflation    Insufflation timeout in seconds.
// 	 * @param[in]   secTimeoutExsufflation    Exsufflation timeout in seconds.
//      * @param[in]   speedInsufflation         Insufflation speed. @todo Denote units.
//      * @param[in]   speedExsufflation         Exsufflation speed. @todo Denote units.
//      *
// 	 */
// 	MechVentilation(
//         FlexyStepper stepper,
//         Sensors* sensors,
//         int mlTidalVolume,
//         float secTimeoutInsufflation,
//         float secTimeoutExsufflation,
//         float speedInsufflation,
//         float speedExsufflation,
//         int ventilationCyle_WaitTime
//     );

//     /**
// 	 * Constructor (triggered mechanical ventilation).
// 	 *
//      * @param[in]   mlTidalVolume             Tidal volume in millilitres.
// 	 * @param[in]   secTimeoutInsufflation    Insufflation timeout in seconds.
// 	 * @param[in]   secTimeoutExsufflation    Exsufflation timeout in seconds.
//      * @param[in]   speedInsufflation         Insufflation speed. @todo Denote units.
//      * @param[in]   speedExsufflation         Exsufflation speed. @todo Denote units.
//      * @param[in]   lpmFluxTriggerValue       Flux trigger value in Litres Per Minute.
//      *
// 	 */
// 	MechVentilation(
//         FlexyStepper stepper,
//         Sensors* sensors,
//         int mlTidalVolume,
//         float secTimeoutInsufflation,
//         float secTimeoutExsufflation,
//         float speedInsufflation,
//         float speedExsufflation,
//         int ventilationCyle_WaitTime,
//         float lpmFluxTriggerValue
//     ); 

//     /* Setters/getters */
//     // TODO: Add stepper, bme1, bme2 setters
//     /** Set tidal volume */
//     void setTidalVolume(float mlTidalVolume);
//     /** Set insufflation timeout. */
//     void setTimeoutInsufflation(float secTimeoutInsufflation);
//     /** Set exsufflation timeout. */
//     void setTimeoutExsufflation(float secTimeoutExsufflation);
//     /** Set insufflation speed. */
//     void setSpeedInsufflation(float speedInsufflation);
//     /** Set exsufflation speed. */
//     void setSpeedExsufflation(float speedExsufflation);
//     boolean getStartWasTriggeredByPatient();
//     void setVentilationCyle_WaitTime(float speedExsufflation);
//     /** Start mechanical ventilation. */
//     void start(void);
//     /** Stop mechanical ventilation. */
//     void stop(void);
//     /** Update mechanical ventilation.
//      *
//      * If any control variable were to change, new value
//      * would be applied at the beginning of the next ventilation
//      * cycle.
//      *
//      * @note This method must be called on the main loop.
//      */
//     void update(void);
//     bool getSensorErrorDetecte();

// private:
//     /** Initialization. */
//     void _init(
//         FlexyStepper stepper,
//         Sensors* sensors,
//         int mlTidalVolume,
//         float secTimeoutInsufflation,
//         float secTimeoutExsufflation,
//         float speedInsufflation,
//         float speedExsufflation,
//         int ventilationCyle_WaitTime,
//         float lpmFluxTriggerValue
//     );

//     /** Set state. */
//     void _setState(State state);

//     /* Configuration parameters */
//     FlexyStepper _cfgStepper;
//     Sensors* _sensors;

//     /** Tidal volume in millilitres. */
//     float _cfgmlTidalVolume;
//     /** Flux trigger value in litres per minute. */
//     float _cfgLpmFluxTriggerValue;
//     /**  Insufflation timeout in seconds. */
//     float _cfgSecTimeoutInsufflation;
//     /* Exsufflation timeout in seconds. */
//     float _cfgSecTimeoutExsufflation;
//     /** Insufflation speed. @todo Denote units. */
//     float _cfgSpeedInsufflation;
//     /** Exsufflation speed. @todo Denote units. */
//     float _cfgSpeedExsufflation;

//     /* Internal state */
//     /** Previous state. @todo Consider removing. */
//     //State _previousState;
//     /** Current state. */
//     State _currentState;
//     /** Next state. @todo Consider removing. */
//     //State _nextState;
//     /** Timer counter in seconds. */
//     uint64_t _secTimerCnt;
//     /**  Insufflation timeout in seconds. */
//     float _secTimeoutInsufflation;
//     /** Exsufflation timeout in seconds. */
//     float _secTimeoutExsufflation;
//     /** Insufflation speed. @todo Denote units. */
//     float _speedInsufflation;
//     /** Exsufflation speed. @todo Denote units. */
//     float _speedExsufflation;
//     bool _running = false;
//     bool _sensor_error_detected;
//     bool _startWasTriggeredByPatient = false;
//     float _currentVolume = 0;



//     /** Estimated flux accross the bmes. @todo Denote units. */
//     //float _flux;

//     /* @todo PID stuff */

// };

// #endif /* INC_MECHANICAL_VENTILATION_H */
