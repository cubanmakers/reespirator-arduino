/** Mechanical ventilation.
 *
 * @file MechVentilation.cpp
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */

#include "MechVentilation.h"

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
        options
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
    return _timeoutEsp;
}
short MechVentilation::getInsuflationTime(void) {
    return _timeoutIns;
}

float MechVentilation::getPeakInspiratoryPressure(void) {
    return _pip;
}

float MechVentilation::getPeakEspiratoryPressure(void) {
    return _peep;
}

void MechVentilation::setRPM(uint8_t rpm) {
    _rpm = rpm;
    _setInspiratoryCycle();
}

void MechVentilation::setPeakInspiratoryPressure(float pip) {
    _pip = pip;
}

void MechVentilation::setPeakEspiratoryPressure(float peep) {
    _peep = peep;
}

void MechVentilation::_setInspiratoryCycle(void) {
  float timeoutCycle = ((float)60) * 1000 / ((float)_rpm); // Tiempo de ciclo en msegundos
  _timeoutIns = timeoutCycle * DEFAULT_POR_INSPIRATORIO/100;
  _timeoutEsp = (timeoutCycle) - _timeoutIns;
}

/**
 * It's called from timer1Isr
 */
void MechVentilation::update(void) {

    static int totalCyclesInThisState = 0;
    static int currentTime = 0;
    static int flowSetpoint = 0;

    #if DEBUG_STATE_MACHINE
        extern volatile String debugMsg[];
        extern volatile byte debugMsgCounter;
    #endif
    

    SensorPressureValues_t pressures = _sensors -> getRelativePressureInCmH20();
    _currentVolume = _sensors->getVolume().volume;
    _currentFlow = _sensors->getFlow();
    if (pressures.state != SensorStateOK) { // Sensor error detected: return to zero position and continue from there
        _sensor_error_detected = true; //An error was detected in sensors
        /* Status update, for this time */
        // TODO: SAVE PREVIOUS CYCLE IN MEMORY AND RERUN IT
        Serial.println("fail sensor");
        _setState(State_Exsufflation);
    } else {
        _sensor_error_detected = false; //clear flag
    }


    refreshWatchDogTimer();

    switch (_currentState) {
        case Init_Insufflation:
            {
#if DEBUG_UPDATE
Serial.println("Starting insuflation");
#endif
        // Close Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);

        totalCyclesInThisState = (_timeoutIns) / TIME_BASE;

        /* Stepper control: set acceleration and end-position */
        _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        _stepper->setAccelerationInStepsPerSecondPerSecond(
            STEPPER_ACC_INSUFFLATION);
        _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);

#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "State: InitInsuflation at " + String(millis());
#endif

        /* Status update, reset timer, for next time, and reset PID integrator to zero */
        _setState(State_Insufflation);

        currentTime = 0;
    }
    break;
    case State_Insufflation:
    {

        /* Stepper control: set end position */

#if DEBUG_UPDATE
//Serial.println("Motor:speed=" + String(_stepperSpeed));
#endif

        // time expired
        if (currentTime > totalCyclesInThisState)
        {
            if (!_stepper->motionComplete())
            {
                // motor not finished, force motor to stop in current position
                _stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
            }
            _setState(Init_Exsufflation);
            currentTime = 0;
        }
        else
        {
            _currentPressure = pressures.pressure1;
            _pid->run(&_currentPressure, &_pip, &_stepperSpeed);

            // TODO: if _currentPressure > _pip + 5, trigger alarm
            _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
            if (_stepperSpeed >= 0)
            {
                _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
            }
            else
            {
                _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
            }
            currentTime++;
        }
    }
    break;
    case Init_Exsufflation:
    {
#if DEBUG_UPDATE
        Serial.println("Starting exsuflation");
#endif
        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

        totalCyclesInThisState = _timeoutEsp / TIME_BASE;
        _sensors->saveVolume();

#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "ExsuflationTime=" + String(totalCyclesInThisState);
#endif
        /* Stepper control*/
        _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
        _stepper->setAccelerationInStepsPerSecondPerSecond(
            STEPPER_ACC_EXSUFFLATION);
        _stepper->setTargetPositionInSteps(
            STEPPER_DIR * (STEPPER_LOWEST_POSITION));
#if DEBUG_STATE_MACHINE
        debugMsg[debugMsgCounter++] = "Motor: to exsuflation at " + String(millis());
#endif

        /* Status update and reset timer, for next time */
        _setState(State_Exsufflation);
        currentTime = 0;
    }
    break;
    case State_Exsufflation:
    {
#if 0
        if (_stepper->motionComplete())
        {
            if (currentFlow < _triggerThreshold && _hasTrigger)
            { // The start was triggered by patient
                _startWasTriggeredByPatient = true;

#if DEBUG_STATE_MACHINE
                debugMsg[debugMsgCounter++] = "!!!! Trigered by patient";
#endif

                /* Status update, for next time */
                _setState(Init_Insufflation);
            }
        }
#endif
        // Time has expired
        if (currentTime > totalCyclesInThisState)
        {
            if (!_stepper->motionComplete())
            {
                // motor not finished, force motor to stop in current position
                _stepper->setTargetPositionInSteps(_stepper->getCurrentPositionInSteps());
            }
            /* Status update and reset timer, for next time */
            _setState(Init_Insufflation);
            _startWasTriggeredByPatient = false;

            currentTime = 0;
        }
        else
        {
            _currentPressure = pressures.pressure1;
            _pid->run(&_currentPressure, &_peep, &_stepperSpeed);

            _stepper->setSpeedInStepsPerSecond(abs(_stepperSpeed));
            if (_stepperSpeed >= 0)
            {
                _stepper->setTargetPositionInSteps(STEPPER_HIGHEST_POSITION);
            }
            else
            {
                _stepper->setTargetPositionInSteps(STEPPER_LOWEST_POSITION);
            }
            currentTime++;
        }
    }
    break;

    case State_Homing:
    {
        // Open Solenoid Valve
        digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

        if (_sensor_error_detected)
        {
            // error sensor reading
            _running = false;
#if DEBUG_UPDATE
            Serial.println("Sensor: FAILED");
#endif
        }

        /*
                 * If not in home, do Homing.
                 * 0: stepper is in home
                 * 1: stepper is not in home
                 */

        if (digitalRead(PIN_ENDSTOP))
        {

            /* Stepper control: homming */
            // bool moveToHomeInMillimeters(long directionTowardHome, float
            // speedInMillimetersPerSecond, long maxDistanceToMoveInMillimeters, int
            // homeLimitSwitchPin)
            Serial.println("Homing attempt!!");
            if (_stepper->moveToHomeInSteps(
                    STEPPER_HOMING_DIRECTION,
                    STEPPER_HOMING_SPEED,
                    STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS,
                    PIN_ENDSTOP) != true)
            {
                Serial.println("Homing has failed");
            }
        }

        /* Status update and reset timer, for next time */
        currentTime = 0;
        _setState(Init_Exsufflation);
    }
    break;

    case State_Error:
        break;
    default:
        //TODO
        break;
    }

}

void MechVentilation::_init(
    FlexyStepper * stepper,
    Sensors * sensors,
    AutoPID * pid,
    VentilationOptions_t options
) {
    /* Set configuration parameters */
    _stepper = stepper;
    _sensors = sensors;
    _pid = pid;
    _rpm = options.respiratoryRate;
    _pip = options.peakInspiratoryPressure;
    _peep = options.peakEspiratoryPressure;
    setRPM(_rpm);
    _hasTrigger = options.hasTrigger;
    if (_hasTrigger) {
        _triggerThreshold = options.triggerThreshold;
    } else {
        _triggerThreshold = FLT_MAX;
    }

    /* Initialize internal state */
    _currentState = State_Homing;
    _stepperSpeed = STEPPER_SPEED_DEFAULT;

    //
    // connect and configure the stepper motor to its IO pins
    //
    //;
    _stepper->connectToPins(PIN_STEPPER_STEP, PIN_STEPPER_DIRECTION);
    _stepper->setStepsPerRevolution(STEPPER_STEPS_PER_REVOLUTION * STEPPER_MICROSTEPS);

    _sensor_error_detected = false;
}

void MechVentilation::_setState(State state) {
    _currentState = state;
}