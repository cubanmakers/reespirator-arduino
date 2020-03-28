/** Mechanical ventilation.
 *
 * @file MechVentilation.h
 *
 * This is the mechanical ventilation software module.
 * It handles the mechanical ventilation control loop.
 */
#ifndef INC_MECHANICAL_VENTILATION_H
#define INC_MECHANICAL_VENTILATION_H

#include <inttypes.h>
#include "src/AutoPID/AutoPID.h"
#include "src/FlexyStepper/FlexyStepper.h"
#include "Sensors.h"
#include "pinout.h"
#include "defaults.h"


typedef struct {
    short mlVolume;
    short stepperPos;
} CalibrationVolume_t;

const CalibrationVolume_t volumeCalibration[] = {{500,100}, {600, 200}, {700, 300}, {800, 400}};

/** States of the mechanical ventilation. */
enum State {

    Init_Insufflation = 1,
    State_Insufflation = 2,             /**< Insufflating (PID control). */
    Init_WaitBeforeExsufflation = 3,
    State_WaitBeforeExsufflation = 4,   /**< Wait for timer. */
    Init_Exsufflation = 5,
    State_Exsufflation = 6,              /**< Return to position 0 and wait for the patient to exsufflate. */
    State_Homing = 0,
    State_Error = -1
};

/**
 * This is the mechanical ventilation class.
 */
class MechVentilation {
public:
	/**
	 * @brief Construct a new Mech Ventilation object
	 * 
	 * @param stepper 
	 * @param sensors 
	 * @param pid 
	 * @param rpm 
	 */
	MechVentilation(
        FlexyStepper* stepper,
        Sensors* sensors,
        AutoPID* pid,
        VentilationOptions_t options
    );

    /**
	 * @brief Construct a new Mech Ventilation object
	 * 
	 * @param stepper 
	 * @param sensors 
	 * @param pid 
	 * @param rpm 
	 */
	MechVentilation(
        FlexyStepper* stepper,
        Sensors* sensors,
        AutoPID* pid,
        VentilationOptions_t options,
        float lpmFluxTriggerValue
    ); 

    boolean getStartWasTriggeredByPatient();
    void setVentilationCyle_WaitTime(float speedExsufflation);
    /** Start mechanical ventilation. */
    void start(void);
    /** Stop mechanical ventilation. */
    void stop(void);
    /** Update mechanical ventilation.
     *
     * If any control variable were to change, new value
     * would be applied at the beginning of the next ventilation
     * cycle.
     *
     * @note This method must be called on the main loop.
     */
    void update(void);
    bool getSensorErrorDetected();
    /**
     * Get tidal volume. In ml
     */
    uint8_t getRPM(void);
    short getExsuflationTime(void);
    short getInsuflationTime(void);
    void reconfigParameters (uint8_t newRpm);

private:
    /** Initialization. */
    void _init(
        FlexyStepper* stepper,
        Sensors* sensors,
        AutoPID* pid,
        VentilationOptions_t options,
        float lpmFluxTriggerValue
    );
    #if 0
    int _calculateInsuflationPosition (void);
    #endif

    /** Set state. */
    void _setState(State state);
    #if 0
    void _increaseInsuflationSpeed (byte factor);
    void _decreaseInsuflationSpeed (byte factor);
    void _increaseInsuflation (byte factor);
    void _decreaseInsuflation (byte factor);
    #endif
    void _setInspiratoryCycle(void);

    /* Configuration parameters */
    FlexyStepper* _stepper;
    Sensors* _sensors;
    AutoPID* _pid;

    /** Flux trigger value in litres per minute. */
    float _trigger_threshold;
    /**  Insufflation timeout in seconds. */
    short _timeout_ins;
    /** Exsufflation timeout in seconds. */
    short _timeout_esp;
    /** Breaths per minute */
    uint8_t _rpm;
    /** Peak inspiratory pressure */
    float _pip;
    /** Peak espiratory pressure */
    float _pep;

    /* Internal state */
    /** Current state. */
    State _currentState;


    /** Stepper speed. @todo Denote units. */
    float _stepperSpeed;
    /** stepper insuflation position */
    short _positionInsufflated;

    bool _running = false;
    bool _sensor_error_detected;
    bool _startWasTriggeredByPatient = false;
    float _currentPressure = 0.0;

    /** Estimated flux accross the bmes. @todo Denote units. */
    //float _flux;

    /* @todo PID stuff */

};

#endif /* INC_MECHANICAL_VENTILATION_H */
