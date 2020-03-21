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
#define LPM_FLUX_TRIGGER_VALUE_NONE     FLT_MAX

int currentWaitTriggerTime = 0;
int currentStopInsufflationTime = 0;
bool running = false;
bool _startWasTriggeredByPatient = false;
float positionSetpoint = 0;

MechVentilation::MechVentilation (
    FlexyStepper stepper,
    Sensors* sensors,
    int mlTidalVolume,
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
    FlexyStepper stepper,
    Sensors *sensors,
    int mlTidalVolume,
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

//TODO: use this method to play a beep in main loop, 1 second long for example.
boolean MechVentilation::getStartWasTriggeredByPatient() { //returns true if last respiration cycle was started by patient trigger. It is cleared when read.
    if (_startWasTriggeredByPatient) {
        _startWasTriggeredByPatient = false;
    }
    return true;
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
}

void MechVentilation::stop(void) {
    running = false;
 }

/**
 * I's called from timer1Isr
 */
void MechVentilation::update(void)
{
    static int currentWaitInsuflationTime = 0; //TODO static function in cpp. check
    static int currentWaitExsufflationTime = 0; //TODO static function in cpp. check
 
    //TODO: meter algo como esto en loop ppal (creo que ya está)
    //     // Acquire sensors data
    //     SensorValues_t sensorValues = _sensors.getPressure();


    SensorValues_t values = _sensors->getPressure();
    if (values.state != SensorStateOK) {
        //TODO sensor failed. do something
    } else {

        float currentFlow = getCurrentFlow(values.pressure1, values.pressure2); //TODO Must calculate Flow using the last measured pressure couple,
            //but the pressure reading must be done as non blocking in the main loop

        refreshWatchDogTimer();

        switch(_currentState) {
            case State_WaitBeforeInsuflation : { //Wait Trigger or Time.  Stepper is stopped in this state
                _cfgStepper.setTargetPositionInSteps(0);
                _startWasTriggeredByPatient = false;
                if (running && ((currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) || (currentWaitTriggerTime > ventilationCycle_WaitBeforeInsuflationTime))) {

                    if (currentWaitTriggerTime > ventilationCycle_WaitBeforeInsuflationTime) { //The start was triggered by patient
                        _startWasTriggeredByPatient = true;
                    }

                    /* Status update, for next time */
                    _setState(State_Insufflation);
                }
                currentWaitInsuflationTime++;
            }
            break;

            case State_Insufflation : {
                float output = 0;
                float pidOutput_PositionSetpoint = 0;
                float feedbackInput = 0; // TODO feedbackInput

                /* Calculate position/flow PID */
                output = computePID(positionSetpoint, feedbackInput);

                pidOutput_PositionSetpoint = 0; //flow2Position(output); //TODO

                /* Steeper control (Position)*/
                _cfgStepper.setTargetPositionInSteps(pidOutput_PositionSetpoint);

                /* Status update and reset timer, for next time */
                _setState(State_WaitBeforeExsufflation);
                currentWaitExsufflationTime = 0;
            }
            break;

            case State_WaitBeforeExsufflation : { //Stepper is stopped in this state
                _cfgStepper.setTargetPositionInSteps(vol2pos(totalPatientVolume));

                if (currentWaitExsufflationTime > VENTILATION_CYCLE__WAIT_BEFORE_EXSUFLATION_TIME) {

                    /* Status update, for next time */
                    _setState(State_Exsufflation);
                }
                currentWaitTriggerTime++;
            }
            break;

            case State_Exsufflation : {
                _cfgStepper.setSpeedInStepsPerSecond(0); //TODO
                //_cfgStepper.setTargetPositionInSteps(XXX);
                
                //TODO @fm read hall sensor
                #if 0
                if (steeperIsInZeroPoint) { //Hall sensor boolean

                    /* Status update and reset timer, for next time */
                    currentWaitInsuflationTime = 0;
                    _setState(State_WaitBeforeInsuflation);
                }
                #endif
            }
            break;

            default : {
            }
            break;
        }
    }
}

void MechVentilation::_init(
    FlexyStepper stepper,
    Sensors* sensors,
    int mlTidalVolume,
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

    //
    // connect and configure the stepper motor to its IO pins
    //
    stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
    stepper.setSpeedInStepsPerSecond(STEPPER_SPEED);
    stepper.setAccelerationInStepsPerSecondPerSecond(STEPPER_ACCELERATION);

}

void MechVentilation::_setState(State state) {
    _previousState  = _currentState;
    _currentState   = state;
}