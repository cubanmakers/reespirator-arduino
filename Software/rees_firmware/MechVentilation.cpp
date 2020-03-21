/** Mechanical ventilation.
 *
 * @file MechVentilation.cpp
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */
#include <float.h>
#include "MechVentilation.h"

/** No trigger. */
#define LPM_FLUX_TRIGGER_VALUE_NONE     FLT_MAX

MechVentilation::MechVentilation(
    float mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation
)
{
    _init(mlTidalVolume,
          secTimeoutInsufflation,
          secTimeoutExsufflation,
          speedInsufflation,
          speedExsufflation,
          LPM_FLUX_TRIGGER_VALUE_NONE);
}

MechVentilation::MechVentilation(
    float mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    float lpmFluxTriggerValue
)
{
    _init(mlTidalVolume,
          secTimeoutInsufflation,
          secTimeoutExsufflation,
          speedInsufflation,
          speedExsufflation,
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

void MechVentilation::start(void) {
    if(_cfgLpmFluxTriggerValue == LPM_FLUX_TRIGGER_VALUE_NONE) {
        _setState(State_StartInsufflation);
    } else {
        /* Triggered */
        _setState(State_WaitTrigger);
    }
}

void MechVentilation::stop(void) {
    _setState(State_Shutdown);
}

void MechVentilation::update(void) {
    bool bContinue = false;

    do {
        bContinue = false;

        switch(_currentState) {
            case State_WaitTrigger : {
                /* @todo Uncomment on trigger condition */
#if 0
                _setState(State_StartInsufflation);
                /* Step directly into next case */
                bContinue = true;
#endif
            }
            break;

            case State_StartInsufflation : {
                _secTimerCnt            = 0;
                _secTimeoutInsufflation = _cfgSecTimeoutInsufflation;
                _speedInsufflation      = _cfgSpeedInsufflation;
                _secTimeoutExsufflation = _secTimeoutExsufflation;
                _speedExsufflation      = _speedExsufflation;

                /* @todo start PID stuff? */
                _setState(State_Insufflation);
                /* Step directly into next case */
                bContinue = true;
            }
            break;

            case State_Insufflation : {
                if(_secTimerCnt < _secTimeoutInsufflation) {
                    /* @todo Keep on the PID work */
                } else {
                    /* Insufflation timeout expired */
                    _secTimerCnt = 0;
                    _setState(State_StopInsufflation);
                    /* Step directly into next case */
                    bContinue = true;
                }
            }
            break;

            case State_StopInsufflation : {
                /* @todo Stop PID stuff? */
                /* @todo Start exsufflation timer */
                _setState(State_WaitExsufflation);
                /* Step directly into next case */
                bContinue = true;
            }
            break;

            case State_WaitExsufflation : {
                if(_secTimerCnt < _secTimeoutExsufflation) {
                    /* @todo Mover leva hacia arriba sin controlar y/o esperar */
                } else {
                    /* Exsufflation timeout expired */
                    if(_cfgLpmFluxTriggerValue == LPM_FLUX_TRIGGER_VALUE_NONE) {
                        _setState(State_StartInsufflation);
                    } else {
                        /* Triggered */
                        _setState(State_WaitTrigger);
                    }
                    bContinue = true;
                }
            }
            break;

            case State_Shutdown : {
                /* @todo Shutdown in a safe way!!! */

                _init(_cfgmlTidalVolume,
                      _cfgSecTimeoutInsufflation,
                      _cfgSecTimeoutExsufflation,
                      _cfgSpeedInsufflation,
                      _cfgSpeedExsufflation,
                      _cfgLpmFluxTriggerValue);
            }

            default :{
                /* State_Init
                * State_Idle
                *
                * Do nothing.
                */
            }
            break;
        }
    } while (bContinue = true);

}

void MechVentilation::_init(
    float mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    float lpmFluxTriggerValue
)
{
    /* Set configuration parameters */
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