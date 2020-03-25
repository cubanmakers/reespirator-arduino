/** Mechanical ventilation.
 *
 * @file MechVentilation.cpp
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */

#include "MechVentilation.h"
#include "defaults.h"

void MechVentilation::update(bool insuflate, int positionInSteps) {
    // Inspiración

        if (insuflate) {
        _sensors -> getPressureInPascals();

        // Apply velocity and acceleration depending on the volume tidal
        _stepper -> setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT * STEPPER_MICROSTEPS);
        _stepper -> setAccelerationInStepsPerSecondPerSecond(
            STEPPER_ACC_DEFAULT * STEPPER_MICROSTEPS
        );
        _stepper -> setTargetPositionInSteps(positionInSteps * STEPPER_MICROSTEPS);

        // Espiración
    } else {
        _stepper -> setSpeedInStepsPerSecond(
            STEPPER_SPEED_EXSUFFLATION * STEPPER_MICROSTEPS
        );
        _stepper -> setAccelerationInStepsPerSecondPerSecond(
            STEPPER_ACC_EXSUFFLATION * STEPPER_MICROSTEPS
        );
        _stepper -> setTargetPositionInSteps(
            STEPPER_LOWEST_POSITION * STEPPER_MICROSTEPS
        );

    }

}

MechVentilation::MechVentilation(FlexyStepper * stepper, Sensors * sensors) {
    _stepper = stepper;
    _sensors = sensors;
}

#if 0
#include < float.h > #include "calc.h"
#include "MechVentilation.h"
#include "Sensors.h"
//#include "src/AccelStepper/Accelstepper->h"
#include "src/FlexyStepper/Flexystepper->h"

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
    // sensors data     SensorValues_t sensorValues = _sensors.getPressureInPascals();
    Serial.println("Starting update state: " + String(_currentState));

    SensorValues_t values = _sensors - >getPressureInPascals();
    // Serial.println("Sensors state=" + String(values.state) + ",pres1=" +
    // String(values.pressure1) + ",pres2=" + String(values.pressure2));
    if (values.state != SensorStateOK) { // Sensor error detected: return to zero position and continue from there

        _sensor_error_detected = true; //An error was detected in sensors
        /* Status update, for this time */
        _setState(State_Exsufflation);
    } else {
        _sensor_error_detected = false; //clear flag
    }
    currentFlow = getCurrentFlow(values.pressure1, values.pressure2); //TODO Must calculate Flow using the last measured pressure couple,
    //but the pressure reading must be done as non blocking in the main loop
    integratorFlowToVolume(& _currentVolume, currentFlow);

    refreshWatchDogTimer();
    switch (_currentState) {

        case Init_WaitBeforeInsuflation:
            {
                totalCyclesInThisState = (int)(_cfgSecTimeoutExsufflation * 1000 / TIME_BASE);
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE
                Serial.println("totalCyclesInThisState: " + String(totalCyclesInThisState));
                /* Calculate wait time */
                waitBeforeInsuflationTime = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //                                              [1000msec/1sec] / TIME_BASE
                Serial.println(
                    "waitBeforeInsuflationTime: " + String(waitBeforeInsuflationTime)
                );
                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeInsuflation);
                //currentTime = 0;  MUST BE COMMENTED

                /* Stepper control*/
                _cfgstepper -> setTargetPositionInSteps(STEPPER_DIR * STEPPER_HOMING_OFFSET);
                while (!_cfgstepper -> motionComplete()) {
                    _cfgstepper -> processMovement();
                }
                Serial.println(
                    "Motor:Process movement position=" + String(_cfgstepper -> getCurrentPositionInSteps())
                );
                _startWasTriggeredByPatient = false;
            }
            //break;  MUST BE COMMENTED
        case State_WaitBeforeInsuflation:
            { //Wait Trigger or Time.  Stepper is stopped in this state

                if (_running) {

                    /* Stepper control*/
                    if (!_cfgstepper -> motionComplete()) {
                        _cfgstepper -> processMovement();
                        Serial.println(
                            "Motor:Process movement position=" + String(_cfgstepper -> getCurrentPositionInMillimeters())
                        );
                    }

                    if (currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) { //The start was triggered by patient
                        _startWasTriggeredByPatient = true;
                        Serial.println("!!!! Trigered by patient");
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
                totalCyclesInThisState = _cfgSecTimeoutInsufflation * 1000 / TIME_BASE;
                //											[1000msec/1sec]*[1sec/1cycle]

                /* Calculate wait time */
                insuflationTime = _cfgSecTimeoutInsufflation * 1000 / TIME_BASE;
                //                                              [1000msec/1sec]

                flowSetpoint = (_cfgmlTidalVolume / insuflationTime);
                /////¿¿¿¿¿ === ???? _cfgSpeedInsufflation [step/sec]

                /* Stepper control: set acceleration and end-position */
                _cfgstepper -> setAccelerationInStepsPerSecondPerSecond(
                    STEPPER_SPEED_INSUFFLATION
                );
                _cfgstepper -> setTargetPositionInSteps(
                    STEPPER_DIR * (vol2pos(_cfgmlTidalVolume) + STEPPER_HOMING_OFFSET)
                );
                while (!_cfgstepper -> motionComplete()) {
                    _cfgstepper -> processMovement();
                }
                Serial.println(
                    "Motor:Process movement position=" + String(_cfgstepper -> getCurrentPositionInSteps())
                );
                Serial.println("Motor:targetPos" + String(_cfgmlTidalVolume));

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

                /* Conver Flow to stepper speed */
                stepperSpeed = flow2speed(pidOutput_FlowSetpoint);

                /* Stepper control: set end position */
                Serial.println("Motor:speed=" + String(stepperSpeed));
                _cfgstepper -> setSpeedInStepsPerSecond(stepperSpeed);
                while (!_cfgstepper -> motionComplete()) {
                    _cfgstepper -> processMovement();
                }
                Serial.println(
                    "Motor:Process movement position=" + String(_cfgstepper -> getCurrentPositionInSteps())
                );

                if ((_currentVolume >= _cfgmlTidalVolume) || (currentTime > insuflationTime)) {

                    /* Status update and reset timer, for next time */
                    _setState(Init_WaitBeforeExsufflation);

                    currentTime = 0;
                }

            }
            break;

        case Init_WaitBeforeExsufflation:
            {
                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE
                Serial.println("totalCyclesInThisState" + String(totalCyclesInThisState));
                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeExsufflation);
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_WaitBeforeExsufflation:
            { //Stepper is stopped in this state
                /* Stepper control*/
                // _cfgstepper->setTargetPositionInSteps(STEPPER_DIR *
                // vol2pos(_cfgmlTidalVolume)); while (!_cfgstepper->motionComplete()) {
                // _cfgstepper->processMovement(); }
                // Serial.println("Motor:Process movement position=" +
                // String(_cfgstepper->getCurrentPositionInSteps()));

                if (currentTime > WAIT_BEFORE_EXSUFLATION_TIME / TIME_BASE) {

                    /* Status update, for next time */
                    _setState(Init_Exsufflation);
                }
                currentTime++;
            }
            break;

        case Init_Exsufflation:
            {
                totalCyclesInThisState = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                //											[1000msec/1sec]*[1sec/1cycle] / TIME_BASE

                /* Calculate wait time */
                // exsuflationTime = _cfgSecTimeoutExsufflation * 1000 / TIME_BASE;
                // [1000msec/1sec] / TIME_BASE

                /* Status update and reset timer, for next time */
                _setState(State_Exsufflation);
                currentTime = 0;
            }
            //break;  MUST BE COMMENTED
        case State_Exsufflation:
            {
                /* Stepper control*/
                _cfgstepper -> setAccelerationInStepsPerSecondPerSecond(
                    STEPPER_ACC_EXSUFFLATION
                );
                _cfgstepper -> setSpeedInStepsPerSecond(STEPPER_SPEED_EXSUFFLATION);
                _cfgstepper -> setTargetPositionInSteps(STEPPER_DIR * STEPPER_HOMING_OFFSET);

                while (!_cfgstepper -> motionComplete()) {
                    _cfgstepper -> processMovement();
                }
                /* Status update and reset timer, for next time */
                currentTime = 0;
                _setState(Init_WaitBeforeInsuflation);
            }
            break;

        case State_HOMING:
            {
                if (_sensor_error_detected) {
                    // error sensor reading
                    _running = false;
                    Serial.println("Sensor: FAILED");
                }

                if (!digitalRead(ENDSTOP_PIN)) { //If not in HOME, do HOMING

                    /* Stepper control: HOMING */
                    // bool moveToHomeInMillimeters(long directionTowardHome,  float
                    // speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters,  int
                    // homeLimitSwitchPin)
                    Serial.println("**********  HOMING  **********");

                    while (
                        !_cfgstepper -> moveToHomeInSteps(1, HOMING_SPEED, (105 * STEPPER_MICROSTEPS), ENDSTOP_PIN)
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
    _currentState = State_HOMING;
    _secTimerCnt = 0;
    _secTimeoutInsufflation = 0;
    _secTimeoutExsufflation = 0;
    _speedInsufflation = 0;
    _speedExsufflation = 0;

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    _cfgstepper -> connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    _cfgstepper -> setSpeedInStepsPerSecond(STEPPER_SPEED_DEFAULT);
    _cfgstepper -> setAccelerationInStepsPerSecondPerSecond(STEPPER_ACC_DEFAULT);
    _cfgstepper -> setStepsPerRevolution(
        STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS
    );

    _sensor_error_detected = false;
}

void MechVentilation::_setState(State state) {
    //_previousState = _currentState;
    _currentState = state;
}
#endif