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
    AutoPID * pid,
    VentilationOptions_t options) {

    _init(
        stepper,
        sensors,
        pid,
        options,
        LPM_FLUX_TRIGGER_VALUE_NONE
    );

}

MechVentilation::MechVentilation(
    FlexyStepper * stepper,
    Sensors * sensors,
    AutoPID * pid,
    VentilationOptions_t options,
    float lpmFluxTriggerValue
) {
    _init(
        stepper,
        sensors,
        pid,
        options,
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
boolean MechVentilation::getSensorErrorDetected() { //returns true if there was an sensor error detected. It is cleared when read.
    if (_sensor_error_detected) {
        return true;
    } else {
        return false;
    }
}

void MechVentilation::start(void) {
    _running = true;
}

void MechVentilation::stop(void) {
    _running = false;
}

uint8_t MechVentilation::getRPM(void) {
    return _rpm;
}
short MechVentilation::getExsuflationTime(void) {
    return _cfg_msecTimeoutExsufflation;
}
short MechVentilation::getInsuflationTime(void) {
    return _cfg_msecTimeoutInsufflation;
}
void MechVentilation::reconfigParameters (uint8_t newRpm) {
    _rpm = newRpm;
    _setInspiratoryCycle();
    _positionInsufflated = _calculateInsuflationPosition();
}

void MechVentilation::_setInspiratoryCycle(void) {
  float tCiclo = ((float)60) * 1000/ ((float)_rpm); // Tiempo de ciclo en msegundos
  _cfg_msecTimeoutInsufflation = tCiclo * DEFAULT_POR_INSPIRATORIO/100;
  _cfg_msecTimeoutExsufflation = (tCiclo) - _cfg_msecTimeoutInsufflation;
}

/**
 * I's called from timer1Isr
 */
void MechVentilation::update(void) {

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

    #if DEBUG_STATE_MACHINE
        extern volatile String debugMsg[];
        extern volatile byte debugMsgCounter;
    #endif


    
    #ifndef PRUEBAS
        SensorPressureValues_t values = _sensors->getRelativePressureInCmH20();
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

    refreshWatchDogTimer();
    switch (_currentState) {
        case Init_Insufflation:
            {
                // Close Solenoid Valve
                digitalWrite(SOLENOIDpin, SOLENOID_CLOSED);

                totalCyclesInThisState = (_cfg_msecTimeoutInsufflation - WAIT_BEFORE_EXSUFLATION_TIME)/ TIME_BASE;

                /* Stepper control: set acceleration and end-position */
                _stepper->setSpeedInStepsPerSecond(_speedInsufflation);
                _stepper->setAccelerationInStepsPerSecondPerSecond(
                    STEPPER_ACC_INSUFFLATION
                );
                _stepper->setTargetPositionInSteps( _positionInsufflated);

                #if DEBUG_STATE_MACHINE
                debugMsg[debugMsgCounter++] = "State: InitInsuflation at " + String(millis());
                debugMsg[debugMsgCounter++] = "Motor to " + String(_positionInsufflated);
                #endif

                /* Status update, reset timer, for next time, and reset PID integrator to zero */
                _setState(State_Insufflation);
                
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
                _stepper->setSpeedInStepsPerSecond(stepperSpeed);
                #endif
                if (currentTime > totalCyclesInThisState) {
                    // time expired
                    if (!_stepper->motionComplete()) {
                        // motor not finished
                        _increaseInsuflationSpeed(1);
                    }

                    _setState(Init_WaitBeforeExsufflation);
                    currentTime = 0;
                }  else {
                    #if ENABLED_SENSOR_VOLUME
                    if (_currentVolume > _cfgmlTidalVolume) {
                        //TODO stop motor

                        /* Status update and reset timer, for next time */
                        _setState(Init_WaitBeforeExsufflation);
                        currentTime = 0;
                    } else 
                    #endif
                    {
                        currentTime++;
                    }
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

                totalCyclesInThisState = _cfg_msecTimeoutExsufflation / TIME_BASE;

                #if DEBUG_STATE_MACHINE
                debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
                #endif
                /* Stepper control*/
                _stepper->setSpeedInStepsPerSecond(_speedExsufflation);
                _stepper->setAccelerationInStepsPerSecondPerSecond(
                    STEPPER_ACC_EXSUFFLATION
                );
                _stepper->setTargetPositionInSteps(
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

                if (_stepper->motionComplete()) {
                    if (currentFlow < FLOW__INSUFLATION_TRIGGER_LEVEL) { //The start was triggered by patient
                        _startWasTriggeredByPatient = true;

                        #if DEBUG_STATE_MACHINE
                            debugMsg[debugMsgCounter++] = "!!!! Trigered by patient";
                        #endif

                        /* Status update, for next time */
                        _setState(Init_Insufflation);

                    }
                }
                                        
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
                        _stepper -> moveToHomeInSteps(
                            STEPPER_HOMING_DIRECTION,
                            STEPPER_HOMING_SPEED,
                            STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
                            ENDSTOPpin
                        ));
                    #endif
                }

                /* Status update and reset timer, for next time */
                currentTime = 0;
                _setState(Init_Exsufflation);
            }
            break;

        case State_Error:
        default:
            //TODO
            break;
    }

}

void MechVentilation::_init(
    FlexyStepper * stepper,
    Sensors * sensors,
    AutoPID * pid,
    VentilationOptions_t options,
    float lpmFluxTriggerValue
) {
    /* Set configuration parameters */
    _stepper = stepper;
    _sensors = sensors;
    _pid = pid;
    _rpm = options.respiratory_rate;
    _pip = options.peak_inspiratory_pressure;
    _pep = options.peak_espiratory_pressure;
    reconfigParameters(mlTidalVolume, rpm);
    _cfgLpmFluxTriggerValue = lpmFluxTriggerValue;

    /* Initialize internal state */
    _currentState = State_Homming;
    _speedInsufflation = STEPPER_SPEED_INSUFFLATION;
    _speedExsufflation = STEPPER_SPEED_EXSUFFLATION;

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    _stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    _stepper->setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS);

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

void MechVentilation::_increaseInsuflationSpeed (byte factor)
{
    _speedInsufflation += factor;
}
void MechVentilation::_decreaseInsuflationSpeed (byte factor) 
{
    _speedInsufflation -= factor;
}
void MechVentilation::_increaseInsuflation (byte factor) 
{
    _positionInsufflated -= factor;
}
void MechVentilation::_decreaseInsuflation (byte factor) 
{
    _positionInsufflated += factor;
}

void MechVentilation::_setState(State state) {
    //_previousState = _currentState;
    _currentState = state;
}
