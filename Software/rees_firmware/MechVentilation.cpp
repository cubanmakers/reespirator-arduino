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

/** No trigger. */
#define LPM_FLUX_TRIGGER_VALUE_NONE FLT_MAX

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
float currentFlow = 0;

MechVentilation::MechVentilation(
    FlexyStepper *stepper,
    Sensors *sensors,
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
    FlexyStepper *stepper,
    Sensors *sensors,
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
    static int exsuflationTime = 0;
    static int flowSetpoint = 0;

    // TODO: meter algo como esto en loop ppal (creo que ya está)      Acquire
    // sensors data     SensorValues_t sensorValues = _sensors.getPressure();

#if DEBUG_UPDATE
    Serial.println("Starting update state: " + String(_currentState));
#endif

    SensorValues_t values = _sensors->getPressure();
    //Serial.println("Sensors state=" + String(values.state) + ",pres1=" + String(values.pressure1) + ",pres2=" + String(values.pressure2));
    if (false /*values.state != SensorStateOK*/) { // Sensor error detected: return to zero position and continue from there
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
#if DEBUG_UPDATE
                Serial.println("totalCyclesInThisState: " + String(totalCyclesInThisState));
#endif
            {
                // Close Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_CLOSED);

                totalCyclesInThisState = (int)(_cfgSecTimeoutExsufflation * 1000 / TIME_BASE);
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE
#if DEBUG_UPDATE
                Serial.println("totalCyclesInThisState: " + String(totalCyclesInThisState));
#endif
                /* Calculate wait time */
                waitBeforeInsuflationTime = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //                                              [1000msec/1sec] / TIME_BASE
#if DEBUG_UPDATE
                Serial.println("waitBeforeInsuflationTime: " + String(waitBeforeInsuflationTime));
#endif
                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeInsuflation);
                //currentTime = 0;  MUST BE COMMENTED

                /* Stepper control*/
                _cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFFLATION);
                _cfgStepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_EXSUFFLATION);
                _cfgStepper->setTargetPositionInSteps(STEPPER_DIR * (STEPPER_LOWEST_POSITION + 0));

#if DEBUG_UPDATE
                Serial.println("Motor:Process movement position=" + String(_cfgStepper->getCurrentPositionInSteps()));
#endif
                _startWasTriggeredByPatient = false;
            }
            //break;  MUST BE COMMENTED
        case State_WaitBeforeInsuflation:
            { //Wait Trigger or Time.  Stepper is stopped in this state
                
                // Close Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_CLOSED);

                if (_running) {

                    /* Stepper control*/
                    if (_cfgStepper->motionComplete()) {
//@dm enable for production
                        if (false /*currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL*/) { //The start was triggered by patient
                            _startWasTriggeredByPatient = true;
#if DEBUG_UPDATE
                            Serial.println("!!!! Trigered by patient");
#endif
                            /* Status update, for next time */
                            _setState(Init_Insufflation);

                         } else if (currentTime > waitBeforeInsuflationTime) {

                            /* Status update, for next time */
                            _setState(Init_Insufflation);
                        }
#if DEBUG_UPDATE
                        Serial.println("Motor:Process movement position=" + String(_cfgStepper->getCurrentPositionInMillimeters()));
#endif
                    }
                    //TODO check motor error
                }
                //This is a "currentTime++" but avoiding overflow, that could happen if (_running = false).
                //And MUST be here, to be counting even when (_running = false).
                if (currentTime <= waitBeforeInsuflationTime) { currentTime++; }
            }
            break;

        case Init_Insufflation:
            {
                totalCyclesInThisState = _cfgSecTimeoutInsufflation * 1000 / TIME_BASE;
                //											[1000msec/1sec]*[1sec/1cycle]

                // /* Calculate wait time */
                // insuflationTime = _cfgSecTimeoutInsufflation * 1000 / TIME_BASE;
                // //                                              [1000msec/1sec]

                flowSetpoint = (_cfgmlTidalVolume / insuflationTime);
                /////¿¿¿¿¿ === ???? _cfgSpeedInsufflation [step/sec]

                /* Stepper control: set acceleration and end-position */
                _cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED_INSUFFLATION);
                _cfgStepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_INSUFFLATION);
                _cfgStepper->setTargetPositionInSteps(STEPPER_DIR * (STEPPER_LOWEST_POSITION + vol2pos(_cfgmlTidalVolume)));
// TEST            _cfgStepper->setTargetPositionInSteps(-128 * STEPPER_MICROSTEPS); // this line for testing

#if DEBUG_UPDATE
                Serial.println("Motor:Process movement position=" + String(_cfgStepper->getCurrentPositionInSteps()));
                Serial.println("Motor:targetPos (tidalVol)" + String(_cfgmlTidalVolume));
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
            
#if DEBUG_UPDATE
                Serial.println("Motor:Process movement position=" + String(_cfgStepper->getCurrentPositionInSteps()));
#endif

                if ((_currentVolume >= _cfgmlTidalVolume) || (currentTime > insuflationTime)) {

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

                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE
#if DEBUG_UPDATE
                Serial.println("totalCyclesInThisState" + String(totalCyclesInThisState));
#endif
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
                totalCyclesInThisState = _cfgSecTimeoutExsufflation * (1000 / TIME_BASE);
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE

                /* Calculate wait time */
                exsuflationTime = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //                                              [1000msec/1sec] / TIME_BASE

                /* Stepper control*/
                _cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFFLATION);
                _cfgStepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_EXSUFFLATION);
                _cfgStepper->setTargetPositionInSteps(STEPPER_DIR * (STEPPER_LOWEST_POSITION + 0));
                               
                /* Status update and reset timer, for next time */
                _setState(State_Exsufflation);
                currentTime = 0;
           }
            //break;  MUST BE COMMENTED
        case State_Exsufflation:
            {
                // Open Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_OPEN);

                if (currentTime > exsuflationTime) {

                    /* Status update and reset timer, for next time */
                    _setState(Init_WaitBeforeInsuflation);

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
                
//#if DEBUG_UPDATE
                    Serial.println("H");
//#endif

                    /* Stepper control: homming */
                    //bool moveToHomeInMillimeters(long directionTowardHome, float speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int homeLimitSwitchPin)
 
                    while (
                        _cfgStepper->moveToHomeInSteps(1, STEPPER_HOMING_SPEED, STEPPER_MICROSTEPS_PER_REVOLUTION, ENDSTOPpin)
                    ) ;
 
                }

                /* Status update and reset timer, for next time */
                currentTime = 0;
                _setState(Init_WaitBeforeInsuflation);
            }
            break;

        default:
            {}
            break;
    }
}

void MechVentilation::_init(
    FlexyStepper *stepper,
    Sensors *sensors,
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
    _secTimerCnt = 0;
    _secTimeoutInsufflation = 0;
    _secTimeoutExsufflation = 0;
    _speedInsufflation = 0;
    _speedExsufflation = 0;

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    _cfgStepper->connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    //_cfgStepper->setSpeedInStepsPerSecond(STEPPER_SPEED);
    //_cfgStepper->setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCELERATION);
    _cfgStepper->setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION);

    _sensor_error_detected = false;
}

void MechVentilation::_setState(State state) {
    //_previousState = _currentState;
    _currentState = state;
}
