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
#include "pinout.h"
#include "defaults.h"

/** No trigger. */
#define LPM_FLUX_TRIGGER_VALUE_NONE FLT_MAX

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

MechVentilation::MechVentilation(
    FlexyStepper * stepper,
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
    FlexyStepper * stepper,
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

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

#if DEBUG_STATE_MACHINE
    extern String debugMsg[];
    extern byte debugMsgCounter;
#endif

    #if DEBUG_STATE_MACHINE
    // debugMsg[debugMsgCounter++] = "Starting update state: " +
    // String(_currentState);
    #endif

    SensorPressureValues_t values = _sensors->getPressure();
  
#ifndef PRUEBAS
    if (values.state != SensorStateOK) { // Sensor error detected: return to zero position and continue from there
        _sensor_error_detected = true; //An error was detected in sensors
        /* Status update, for this time */
        _setState(State_Exsufflation);
    } else {
        _sensor_error_detected = false; //clear flag
    }
#endif
    currentFlow = _sensors->getVolume().volume;
    //but the pressure reading must be done as non blocking in the main loop
    integratorFlowToVolume(&_currentVolume, currentFlow);

                        #if 0
                        if (currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) { //The start was triggered by patient
                            _startWasTriggeredByPatient = true;

                            #if DEBUG_STATE_MACHINE
                            debugMsg[debugMsgCounter++] = "!!!! Trigered by patient";
                            #endif
                            /* Status update, for next time */
                            _setState(Init_Insufflation);

                        } else 
                        #endif


    refreshWatchDogTimer();
    switch (_currentState) {
        case Init_Insufflation:
            {
                // Close Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_CLOSED);

                totalCyclesInThisState = ((_cfgSecTimeoutInsufflation * 1000) - WAIT_BEFORE_EXSUFLATION_TIME)/ TIME_BASE;
                // [1000msec/1sec]*[1sec/1cycle] /* Calculate wait time */ insuflationTime =
                // _cfgSecTimeoutInsufflation * 1000 / TIME_BASE;
                // [1000msec/1sec]

                flowSetpoint = (_cfgmlTidalVolume / totalCyclesInThisState);
                /////¿¿¿¿¿ === ???? _cfgSpeedInsufflation [step/sec]

                /* Stepper control: set acceleration and end-position */
                _cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED_INSUFFLATION);
                _cfgStepper->setAccelerationInStepsPerSecondPerSecond(
                    STEPPER_ACC_INSUFFLATION
                );
                _cfgStepper->setTargetPositionInSteps( _positionInsufflated);

                #if DEBUG_STATE_MACHINE
                debugMsg[debugMsgCounter++] = "Motor: to insuflation at " + String(millis());
                #endif

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
                #if 0
                //@fm enable for production
                float currentProgressFactor = currentTime / totalCyclesInThisState;

                /* Calculate FlowSetpoint */
                curveOutput_FlowSetpoint = curveInterpolator(
                    flowSetpoint,
                    currentProgressFactor
                );

                /* Calculate Flow PID */
                pidOutput_FlowSetpoint = computePID(curveOutput_FlowSetpoint, currentFlow);

                /* Conver Flow to stepper speed */
                stepperSpeed = flow2speed(pidOutput_FlowSetpoint);

                /* Stepper control: set end position */

                #if DEBUG_UPDATE
                Serial.println("Motor:speed=" + String(stepperSpeed));
                #endif
                _cfgStepper->setSpeedInStepsPerSecond(stepperSpeed);
                #endif

                if ((_currentVolume >= _cfgmlTidalVolume) || (currentTime > totalCyclesInThisState)) {

                    /* Status update and reset timer, for next time */
                    _setState(Init_WaitBeforeExsufflation);

                    currentTime = 0;
                } else {
                    currentTime++;
                }

            }
            break;

        case Init_WaitBeforeExsufflation:
            {
                // Open Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_OPEN);
                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeExsufflation);
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_WaitBeforeExsufflation:
            { //Stepper is stopped in this state

                if (currentTime > WAIT_BEFORE_EXSUFLATION_TIME / TIME_BASE) {

                    /* Status update, for next time */
                    _setState(Init_Exsufflation);
                } else {
                    currentTime++;
                }
            }
            break;

        case Init_Exsufflation:
            {
                // Open Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_OPEN);

                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE

                #if DEBUG_STATE_MACHINE
                debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
                #endif
                /* Stepper control*/
                _cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFFLATION);
                _cfgStepper->setAccelerationInStepsPerSecondPerSecond(
                    STEPPER_ACC_EXSUFFLATION
                );
                _cfgStepper->setTargetPositionInSteps(
                    STEPPER_DIR * (STEPPER_LOWEST_POSITION + 0)
                );
                #if DEBUG_STATE_MACHINE
                debugMsg[debugMsgCounter++] = "Motor: to exsuflation at " + String(millis());
                #endif

                /* Status update and reset timer, for next time */
                _setState(State_Exsufflation);
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_Exsufflation:
            {
                
                if (currentTime > totalCyclesInThisState) {

                    /* Status update and reset timer, for next time */
                    _setState(Init_Insufflation);
                    _startWasTriggeredByPatient = false;

                    currentTime = 0;
                } else {
                    currentTime++;
                }
            }
            break;

        case State_Homming:
            {
                // Open Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_OPEN);

                if (_sensor_error_detected) {
                    // error sensor reading
                    _running = false;
                    #if DEBUG_UPDATE
                    Serial.println("Sensor: FAILED");
                    #endif
                }

                if (!digitalRead(ENDSTOPpin)) { //If not in HOME, do Homming

                    /* Stepper control: homming */
                    // bool moveToHomeInMillimeters(long directionTowardHome, float
                    // speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int
                    // homeLimitSwitchPin)

                    #ifndef PRUEBAS

                    while (
                        _cfgStepper->moveToHomeInSteps(1, STEPPER_HOMING_SPEED, STEPPER_MICROSTEPS_PER_REVOLUTION, ENDSTOPpin)
                    );
                    

                    #endif
                }

                /* Status update and reset timer, for next time */
                currentTime = 0;
                _setState(Init_Exsufflation);
            }
            break;

        default:
            {}
            break;
    }

}

void MechVentilation::_init(
    FlexyStepper * stepper,
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
    _currentState = State_Homming;
    _secTimeoutInsufflation = 0;
    _secTimeoutExsufflation = 0;
    _speedInsufflation = 0;
    _speedExsufflation = 0;
    _positionInsufflated = _calculateInsuflationPosition();

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    _cfgStepper->connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    // _cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED);
    // _cfgStepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCELERATION);
    _cfgStepper->setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION);

    _sensor_error_detected = false;
}

int MechVentilation::_calculateInsuflationPosition (void) {
    short volumeTarget = (short) _cfgmlTidalVolume;
    short lastPosition = volumeCalibration[0].stepperPos;
    for (byte i = 1; i < sizeof(CalibrationVolume_t); i++) {
        if (volumeTarget > volumeCalibration[i].mlVolume) {
            lastPosition = volumeCalibration[i].stepperPos;
        } else {
            break;
        }
    }
    return lastPosition;
}


void MechVentilation::_setState(State state) {
    //_previousState = _currentState;
    _currentState = state;
}
