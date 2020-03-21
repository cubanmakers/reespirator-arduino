/** Mechanical ventilation.
 *
 * @file MechVentilation.cpp
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */
#include <float.h>
#include "calc.h"
#include "MechVentilation.h"
#include "src/AccelStepper/AccelStepper.h"

/** No trigger. */
#define LPM_FLUX_TRIGGER_VALUE_NONE     FLT_MAX

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
bool running = false;
float positionSetpoint = 0;

MechVentilation::MechVentilation(
    AccelStepper stepper,
    Sensors sensors, 
    float mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    int ventilationCyle_WaitTime
)
{
    _init(stepper,
          sensors,
          mlTidalVolume,
          secTimeoutInsufflation,
          secTimeoutExsufflation,
          speedInsufflation,
          speedExsufflation,
          ventilationCyle_WaitTime,
          LPM_FLUX_TRIGGER_VALUE_NONE);
}

MechVentilation::MechVentilation(
    AccelStepper stepper,
    Sensors sensors,
    float mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    int ventilationCyle_WaitTime,
    float lpmFluxTriggerValue
)
{
    _init(stepper,
          sensors,
          mlTidalVolume,
          secTimeoutInsufflation,
          secTimeoutExsufflation,
          speedInsufflation,
          speedExsufflation,
          ventilationCyle_WaitTime,
          lpmFluxTriggerValue);
}

void MechVentilation::setTidalVolume(float mlTidalVolume) {
    _cfgmlTidalVolume = mlTidalVolume;
}

void MechVentilation::setTimeoutInsufflation(float secTimeoutInsufflation) {
    _cfgSecTimeoutInsufflation = secTimeoutInsufflation;
}

void MechVentilation::setTimeoutExsufflation(float secTimeoutExsufflation) {
    _cfgSecTimeoutExsufflation = secTimeoutExsufflation;
}

void MechVentilation::setSpeedInsufflation(float speedInsufflation) {
    _speedInsufflation = _cfgSpeedInsufflation;
}

void MechVentilation::setSpeedExsufflation(float speedExsufflation) {
    _speedExsufflation = _cfgSpeedExsufflation;
}

void MechVentilation::setVentilationCyle_WaitTime(float speedExsufflation) {
    //TODO _speedExsufflation = _cfgSpeedExsufflation;
}

void MechVentilation::start(void) {
    running = true;

    // if(_cfgLpmFluxTriggerValue == LPM_FLUX_TRIGGER_VALUE_NONE) {
    //     _setState(State_StartInsufflation);
    // } else {
    //     /* Triggered */
    //     _setState(State_WaitTrigger);
    // }
}

void MechVentilation::stop(void) {
    running = false;
    
    //_setState(State_Shutdown);
}

void MechVentilation::stop(void) {
    running = false;
    
    //_setState(State_Shutdown);
}

/**
 * I's called from timer1Isr
 */
void MechVentilation::update(void) {
    //getCurrentFlow(*getPressure1(), *getPressure2(), *flux);
    getCurrentFlow(*getPressure1(), *pressure2, *currentFlow); //TODO Must calculate Flow using the last measured pressure couple,
                      //but the pressure reading must be done as non blocking in the main loop

    refreshWatchDogTimer();

    switch(_currentState) {
        case State_WaitBeforeInsuflation : { //Wait Trigger or Time
            stepper.setSpeed(0);
            if (running && ((currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) || (currentWaitTriggerTime > ventilationCyle_WaitTime))) {
                
                /* Status update, for next time */
                _setState(State_Insufflation);
            }
            currentWaitTriggerTime++;
        }
        break;

        // case State_StartInsufflation : {
        //     _secTimerCnt            = 0;
        //     _secTimeoutInsufflation = _cfgSecTimeoutInsufflation;
        //     _speedInsufflation      = _cfgSpeedInsufflation;
        //     _secTimeoutExsufflation = _secTimeoutExsufflation;
        //     _speedExsufflation      = _speedExsufflation;

        //     /* @todo start PID stuff? */

        //     /* Status update, for next time */
        //     _setState(State_Insufflation);
        // }
        // break;

        case State_Insufflation : {
            float output = 0;
            float pidOutput_PositionSetpoint = 0;

            /* Calculate position/flow PID */
            computePID(*positionSetpoint, float* feedbackInput, *output);

            pidOutput_PositionSetpoint = flow2Position(output);
    
            /* Steeper control (Position)*/
            stepper.setPosition(pidOutput_PositionSetpoint);


            

            if(_secTimerCnt < _secTimeoutInsufflation) {
                /* @todo Keep on the PID work */

                // Acquire sensors data
                SensorValues_t sensorValues = _sensors.getPressure();
                
                if (sensorValues.state == SensorStateOK) {
                    // Calculate flux from delta pressure
                    // TODO: Check sign of results!
                    calcularCaudal(sensorValues.pressure1, sensorValues.pressure2, &_flux);

                    // TODO: Control stepper velocity from flux
                } else {
                    //TODO do something
                }

            } else {
                /* Insufflation timeout expired */
                _secTimerCnt = 0;

                /* Status update, for next time */
                _setState(State_StopInsufflation);
            }
        }
        break;

        case State_StopInsufflation : {
            /* Steeper control (Speed) */
            stepper.setSpeed(0);

            /* Start exsufflation timer */
            currentWaitTime = 0;

            if (currentStopInsufflationTime > VENTILATION_CYCLE__STOP_INSUFLATION_TIME) {
                currentStopInsufflationTime++;

                /* Status update, for next time */
                _setState(State_StartInsufflation);
            }


            /* Status update, for next time */
            _setState(State_WaitExsufflation);
        }
        break;

        case State_WaitExsufflation : {
            if(_secTimerCnt < _secTimeoutExsufflation) {
                /* @todo Mover leva hacia arriba sin controlar y/o esperar */
            } else {
                /* Exsufflation timeout expired */
                if(_cfgLpmFluxTriggerValue == LPM_FLUX_TRIGGER_VALUE_NONE) {

                    /* Status update, for next time */
                    _setState(State_StartInsufflation);
                } else {

                    /* Status update, for next time */
                    _setState(State_WaitTrigger);
                }
            }
        }
        break;

        // case State_Shutdown : {
        //     /* @todo Shutdown in a safe way!!! */

        //     //TODO @fm init????
        //     _init(_cfgStepper,
        //             _sensors,
        //             _cfgmlTidalVolume,
        //             _cfgSecTimeoutInsufflation,
        //             _cfgSecTimeoutExsufflation,
        //             _cfgSpeedInsufflation,
        //             _cfgSpeedExsufflation,
        //             _cfgLpmFluxTriggerValue);
        // }

        default :{
            /* State_Init
            * State_Idle
            *
            * Do nothing.
            */
        }
        break;
    }

}

void MechVentilation::_init(
    AccelStepper stepper,
    Sensors sensors,
    float mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    int ventilationCyle_WaitTime,
    float lpmFluxTriggerValue
)
{
    /* Set configuration parameters */
    _cfgStepper                 = stepper;
    _sensors                    = sensors;
    _cfgmlTidalVolume           = mlTidalVolume;
    _cfgSecTimeoutInsufflation  = secTimeoutInsufflation;
    _cfgSecTimeoutExsufflation  = secTimeoutExsufflation;
    _cfgSpeedInsufflation       = speedInsufflation;
    _cfgSpeedExsufflation       = speedExsufflation;
    _cfgLpmFluxTriggerValue     = lpmFluxTriggerValue;

    /* Initialize internal state */
    _previousState          = State_Init;
    _currentState           = State_Idle;
    _nextState              = State_Idle;
    _secTimerCnt            = 0;
    _secTimeoutInsufflation = 0;
    _secTimeoutExsufflation = 0;
    _speedInsufflation      = 0;
    _speedExsufflation      = 0;
}

void MechVentilation::_setState(State state) {
    _previousState  = _currentState;
    _currentState   = state;
}