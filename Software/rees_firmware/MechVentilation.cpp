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
#include "Sensors.h"
//#include "src/AccelStepper/AccelStepper.h"
#include "src/FlexyStepper/FlexyStepper.h"

/** No trigger. */
#define LPM_FLUX_TRIGGER_VALUE_NONE FLT_MAX

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

MechVentilation::MechVentilation(
    FlexyStepper stepper,
    Sensors * sensors,
    int mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    int ventilationCyle_WaitTime
) {

    _init(
        stepper,
        sensors,
        mlTidalVolume,
        secTimeoutInsufflation,
        secTimeoutExsufflation,
        speedInsufflation,
        speedExsufflation,
        ventilationCyle_WaitTime,
        LPM_FLUX_TRIGGER_VALUE_NONE
    );

}

MechVentilation::MechVentilation(
    FlexyStepper stepper,
    Sensors * sensors,
    int mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    int ventilationCyle_WaitTime,
    float lpmFluxTriggerValue
) {
    _init(
        stepper,
        sensors,
        mlTidalVolume,
        secTimeoutInsufflation,
        secTimeoutExsufflation,
        speedInsufflation,
        speedExsufflation,
        ventilationCyle_WaitTime,
        lpmFluxTriggerValue
    );
}

//TODO: use this method to play a beep in main loop, 1 second long for example.
boolean MechVentilation::getStartWasTriggeredByPatient() { //returns true if last respiration cycle was started by patient trigger. It is cleared when read.
    if (_startWasTriggeredByPatient) {
        return true;
    } else {
        return false;
    }
}

//TODO: use this method to play a beep in main loop, 2 second long for example.
boolean MechVentilation::getSensorErrorDetecte() { //returns true if there was an sensor error detected. It is cleared when read.
    if (_sensor_error_detected) {
        return true;
    } else {
        return false;
    }
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
    _running = true;
}

void MechVentilation::stop(void) {
    _running = false;
}

/**
 * I's called from timer1Isr
 */
void MechVentilation::update(void) {
    static int currentWaitInsuflationTime = 0; //TODO static function in cpp. check
    static int currentInsuflationTime = 0; //TODO static function in cpp. check
    static int currentWaitExsufflationTime = 0; //TODO static function in cpp. check
    static int currentExsufflationTime = 0; //TODO static function in cpp. check
    static int totalCyclesInThisState = 0;
    static int waitBeforeInsuflationTime = 0;
    static int currentTime = 0;
    static int insuflationTime = 0;
    static int flowSetpoint = 0;

    // TODO: meter algo como esto en loop ppal (creo que ya está)      Acquire
    // sensors data     SensorValues_t sensorValues = _sensors.getPressure();

    SensorValues_t values = _sensors->getPressure();
    if (values.state != SensorStateOK) { // Sensor error detected: return to zero position and continue from there

        _sensor_error_detected = true; //An error was detected in sensors
        /* Status update, for this time */
        _setState(State_Exsufflation);
    } else {
        _sensor_error_detected = false; //clear flag
    }
    currentFlow = getCurrentFlow(values.pressure1, values.pressure2); //TODO Must calculate Flow using the last measured pressure couple,
    //but the pressure reading must be done as non blocking in the main loop
    integratorFlowToVolume(&_currentVolume, currentFlow);

    refreshWatchDogTimer();

    switch (_currentState) {

        case Init_WaitBeforeInsuflation:
            {

                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000;
                //											[1000msec/1sec]*[1sec/1cycle]

                /* Calculate wait time */
                waitBeforeInsuflationTime = _cfgSecTimeoutExsufflation * 1000;
                //                                              [1000msec/1sec]

                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeInsuflation);
                //currentTime = 0;  MUST BE COMMENTED

                /* Stepper control*/
                _cfgStepper.setTargetPositionInSteps(0);
                _startWasTriggeredByPatient = false;

            }
            //break;  MUST BE COMMENTED
        case State_WaitBeforeInsuflation:
            { //Wait Trigger or Time.  Stepper is stopped in this state

                if (_running) {

                    /* Stepper control*/
                    if (!_cfgStepper.motionComplete()) {
                        _cfgStepper.processMovement();
                    }

                    if (currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) { //The start was triggered by patient
                        _startWasTriggeredByPatient = true;

                        /* Status update, for next time */
                        _setState(Init_Insufflation);

                    } else if (currentTime > waitBeforeInsuflationTime) {

                        /* Status update, for next time */
                        _setState(Init_Insufflation);
                    }
                }
                currentTime++;
            }
            break;

        case Init_Insufflation:
            {
                totalCyclesInThisState = _cfgSecTimeoutInsufflation * 1000;
                //											[1000msec/1sec]*[1sec/1cycle]

                /* Calculate wait time */
                insuflationTime = _cfgSecTimeoutInsufflation * 1000;
                //                                              [1000msec/1sec]

                flowSetpoint = (_cfgmlTidalVolume / insuflationTime);
                /////¿¿¿¿¿ === ???? _cfgSpeedInsufflation [step/sec]

                /* Stepper control: set acceleration and end-position */
                _cfgStepper.setAccelerationInStepsPerSecondPerSecond(INSUFFLATION_ACCEL);
                _cfgStepper.setTargetPositionInSteps(vol2pos(_cfgmlTidalVolume));

                /* Status update, reset timer, for next time, and reset PID integrator to zero */
                _setState(State_Insufflation);
                resetPID();
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_Insufflation:
            {
                float curveOutput_FlowSetpoint = 0;
                float pidOutput_FlowSetpoint = 0;
                float pidOutput_StepperSpeedSetpoint = 0;
                float stepperSpeed = 0;

                float currentProgressFactor = currentTime / totalCyclesInThisState;

                /* Calculate FlowSetpoint */
                curveOutput_FlowSetpoint = curveInterpolator(
                    flowSetpoint,
                    currentProgressFactor
                );

                /* Calculate Flow PID */
                pidOutput_FlowSetpoint = computePID(curveOutput_FlowSetpoint, currentFlow);

                /* Conver Flow to stepper speed
            stepperSpeed = flow2speed(pidOutput_FlowSetpoint);

            /* Stepper control: set end position */
                _cfgStepper.setSpeedInStepsPerSecond(stepperSpeed);
                if (!_cfgStepper.motionComplete()) {
                    _cfgStepper.processMovement();
                }

                if ((_currentVolume >= _cfgmlTidalVolume) || (currentTime > insuflationTime)) {

                    /* Status update and reset timer, for next time */
                    _setState(Init_WaitBeforeExsufflation);

                    currentTime = 0;
                }

            }
            break;

        case Init_WaitBeforeExsufflation:
            {
                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000;
                //											[1000msec/1sec]*[1sec/1cycle]

                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeExsufflation);
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_WaitBeforeExsufflation:
            { //Stepper is stopped in this state
                _cfgStepper.setTargetPositionInSteps(vol2pos(_cfgmlTidalVolume));

                if (currentTime > WAIT_BEFORE_EXSUFLATION_TIME) {

                    /* Status update, for next time */
                    _setState(Init_Exsufflation);
                }
                currentTime++;
            }
            break;

        case Init_Exsufflation:
            {
                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000;
                //											[1000msec/1sec]*[1sec/1cycle]

                /* Calculate wait time */
                //exsuflationTime = _cfgSecTimeoutExsufflation * 1000;
                //                                              [1000msec/1sec]

                /* Status update and reset timer, for next time */
                _setState(State_Exsufflation);
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_Exsufflation:
            {
                _cfgStepper.setAccelerationInStepsPerSecondPerSecond(EXSUFFLATION_ACCEL); //TODO
                _cfgStepper.setSpeedInStepsPerSecond(EXSUFFLATION_SPEED); //TODO
                _cfgStepper.setTargetPositionInSteps(0);

                //TODO @fm read hall sensor
                
                
                // if (stepperIsInZeroPoint) { //Hall sensor boolean

                //     /* Status update and reset timer, for next time */
                //     currentWaitInsuflationTime = 0;
                //     if (_sensor_error_detected) {
                //         // error sensor reading
                //         _running = false;
                //         //TODO buzzer & display
                //     } else {
                //         _setState(State_WaitBeforeInsuflation);
                //     }
                // }
                
                /* Stepper control*/
                if (!_cfgStepper.motionComplete()) {
                    _cfgStepper.processMovement();
                } else {

                    /* Status update and reset timer, for next time */
                    currentWaitInsuflationTime = 0;
                    if (_sensor_error_detected) {
                        // error sensor reading
                        _running = false;
                        //TODO buzzer & display
                    } else {
                        _setState(Init_WaitBeforeInsuflation);
                    }
                }

                currentTime++;
            }
            break;

        default:
            {}
            break;
    }
}

void MechVentilation::_init(
    FlexyStepper stepper,
    Sensors * sensors,
    int mlTidalVolume,
    float secTimeoutInsufflation,
    float secTimeoutExsufflation,
    float speedInsufflation,
    float speedExsufflation,
    int ventilationCyle_WaitTime,
    float lpmFluxTriggerValue
) {
    /* Set configuration parameters */
    _cfgStepper = stepper;
    _sensors = sensors;
    _cfgmlTidalVolume = mlTidalVolume;
    _cfgSecTimeoutInsufflation = secTimeoutInsufflation;
    _cfgSecTimeoutExsufflation = secTimeoutExsufflation;
    _cfgSpeedInsufflation = speedInsufflation;
    _cfgSpeedExsufflation = speedExsufflation;
    _cfgLpmFluxTriggerValue = lpmFluxTriggerValue;

    /* Initialize internal state */
    //_previousState = Init_WaitBeforeInsuflation;
    _currentState = Init_WaitBeforeInsuflation;
    //_nextState = Init_WaitBeforeInsuflation;
    _secTimerCnt = 0;
    _secTimeoutInsufflation = 0;
    _secTimeoutExsufflation = 0;
    _speedInsufflation = 0;
    _speedExsufflation = 0;

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    stepper.setSpeedInStepsPerSecond(STEPPER_SPEED);
    stepper.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCELERATION);

    _sensor_error_detected = false;
}

void MechVentilation::_setState(State state) {
    //_previousState = _currentState;
    _currentState = state;
}
