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
float currentFlow = 0;
float currentVolume = 0;

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
        _startWasTriggeredByPatient = false; //clear flag
        return true;
    } else {
        return false;
    }    
}

//TODO: use this method to play a beep in main loop, 2 second long for example.
boolean MechVentilation::getSensorErrorDetecte() { //returns true if there was an sensor error detected. It is cleared when read.
    if (_sensor_error_detected) {
        _sensor_error_detected = false; //clear flag
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
    static int currentInsuflationTime = 0; //TODO static function in cpp. check
    static int currentWaitExsufflationTime = 0; //TODO static function in cpp. check
    static int currentExsufflationTime = 0; //TODO static function in cpp. check
 
    //TODO: meter algo como esto en loop ppal (creo que ya está)
    //     // Acquire sensors data
    //     SensorValues_t sensorValues = _sensors.getPressure();

    SensorValues_t values = _sensors->getPressure();
 
    if (values.state != SensorStateOK) { // Sensor error detected: return to zero position and continue from there

        _sensor_error_detected = true; //An error was detected in sensors
       /* Status update, for this time */
        _setState(State_Exsufflation);
    }

    currentFlow = getCurrentFlow(values.pressure1, values.pressure2); //TODO Must calculate Flow using the last measured pressure couple,
        //but the pressure reading must be done as non blocking in the main loop
    currentVolume = integratorFlowToVolumen(currentFlow);

    refreshWatchDogTimer();

    switch(_currentState) {

        case State_WaitBeforeInsuflation : { //Wait Trigger or Time.  Stepper is stopped in this state
            _cfgStepper.setTargetPositionInSteps(0);
            _startWasTriggeredByPatient = false;
            if (running && ((currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) || (currentWaitTriggerTime > ventilationCycle_WaitBeforeInsuflationTime))) {

                if (currentWaitTriggerTime > ventilationCycle_WaitBeforeInsuflationTime) { //The start was triggered by patient
                    _startWasTriggeredByPatient = true;
                }

                /* Status update, for next time, and reset PID integrator to zero */
                _setState(State_Insufflation);
                resetPID();
                currentInsuflationTime = 0;
            }
            currentWaitInsuflationTime++;
        }
        break;

        case State_Insufflation : {
            float pidOutput_VolumeSetpoint = 0;
            float pidOutput_PositionSetpoint = 0;


            // float curveInterpolator(int[] curve, float min, float max, float currentProgressPercent);

            /* Calculate VolumeSetpoint */
            volumeSetpoint = curveInterpolator(insufflationCurve, 0, _cfgmlTidalVolume, currentProgress);

            /* Calculate position/flow PID */
            //pidOutput_PositionSetpoint = computePID(_cfgmlTidalVolume, currentVolume);
            pidOutput_VolumeSetpoint = computePID(volumeSetpoint, currentVolume);

            pidOutput_PositionSetpoint = vol2pos(pidOutput_VolumeSetpoint);

            /* Steeper control (Position)*/
            _cfgStepper.setTargetPositionInSteps(pidOutput_PositionSetpoint);

            /* Status update and reset timer, for next time */
            _setState(State_WaitBeforeExsufflation);
            
            if (currentVolume >= mlTidalVolume) {
                currentWaitExsufflationTime = 0;
            }
                _setState(State_WaitBeforeInsuflation);
            }
            currentInsuflationTime++;
        }
        break;

        default : {
        }
        break;
    }
}

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