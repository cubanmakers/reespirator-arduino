/** Ventilation control.
 *
 * @file ventilation_control.cpp
 *
 * This is the ventilation control software module.
 * It handles the ventilation control state machine and the control loop.
 */

#include "ventilation_control.h"

ventilation_control::ventilation_control(

    static _Data_t _data;
    
    _init();

}

void ventilation_control::_init {
    _data._currentState = State_Homing;
    _data._mech_goal_position = MECH_LOWEST_POSITION;
    _data._mech_goal_speed = 0;
    _data._mech_goal_accel = 0;
    _data._current_cycle_was_triggered = false;
    _data._time_in_current_state = 0;
    _data._flow_setpoint = 0;
}

void ventilation_control::_calc_cycle_timeouts(void) {
    PCD.timeout_cycle = ((60 * 1000) / PCD.rpm); // Respiratoty cycle time [msec]
    PCD._timeout_ins = (u16)((u32)PCD.timeout_cycle * PCD.t_ins_percent) / 100);
    PCD._timeout_esp = PCD.timeout_cycle - PCD._timeout_ins;
}

/**
 * It's called from timer1Isr
 */
void ventilation_control::update(void) {

    if (PCD.errors != NO_ERRORS) {
        //TODO
    }

    if (PCD.mode = demo) {
        //TODO
        // Populate PCD with simulated data 
    }

    //// General tasks (Must be performe every cycle)
    // Calculations
    PCD.calculate_current_flow(VENTILATION_CONTROL_CYCLE);
    PCD.calculate_current_volume(VENTILATION_CONTROL_CYCLE);

    PCD.alarm_battery_min.flag = (PCD.current_battery.level_percent < PCD.alarm_battery_min.value);
    // if (PCD.current_battery.level_percent < PCD.alarm_battery_min.value) {
    //     PCD.alarm_battery_min.flag = true;
    // } else {
    //     PCD.alarm_battery_min.flag = false;
    // }

    switch (_data._currentState) {
        case Init_Insufflation:
        {
            _current_cycle_was_triggered = false;

            _calc_cycle_timeouts();
            _calc_last_cycle_values(); // (Volume), PIP, PEEP, RPM

            // Alarm check (TODO: mejor sólo al inicio de cada ciclo)
            PCD.alarm_volume_max.flag = (PCD.current_volume > PCD.alarm_volume_max.value);
            PCD.alarm_volume_min.flag = (PCD.current_volume < PCD.alarm_volume_min.value);
            PCD.alarm_pip_max.flag =    (PCD.current_pip    > PCD.alarm_pip_max.value);
            PCD.alarm_pip_min.flag =    (PCD.current_pip    < PCD.alarm_pip_min.value);
            PCD.alarm_peep_max.flag =   (PCD.current_peep   > PCD.alarm_peep_max.value);
            PCD.alarm_peep_min.flag =   (PCD.current_peep   < PCD.alarm_peep_min.value);
            PCD.alarm_rpm_max.flag =    (PCD.current_rpm    > PCD.alarm_rpm_max.value);
            PCD.alarm_rpm_min.flag =    (PCD.current_rpm    < PCD.alarm_rpm_min.value);

            // Close Solenoid Valve
            ins_exp_solenoid.set(SOLENOID_INS); //digitalWrite(PIN_SOLENOID, SOLENOID_CLOSED);

            /* Status update, reset timer, for next time, and reset PID integrator to zero */
            _data._currentState = State_Insufflation;
            _data._time_in_current_state = 0;
            pid_ins.reset();

            if (PCD.mode == MODE_VOLUME_CONTROL) {
                mech.set_position(calc_vol_2_pos_LUT(PCD.setting_volume));
            } else {
                // MODE_PRESSURE_CONTROL
                mech.set_position(mech.MECH_HIGHEST_POSITION);
            }
        }
        //break; // MUST be commented

        case State_Insufflation:
        {
            /* Stepper control: set end position */

            // Time out
            if (_data._time_in_current_state > _timeout_ins)
            {
                mech.set_speed(0);
                _data._currentState = Init_Exsufflation;
                _data._time_in_current_state = 0;
            } else {
                if (PCD.mode == MODE_VOLUME_CONTROL) {
                    if (_data._current_volume < PCD.setting_volume) {
                        mech.set_position(calc_vol_2_pos_LUT(PCD.setting_volume));
                    } else {
                        mech.set_speed(0);
                    }
                } else {
                    // MODE_PRESSURE_CONTROL
                    if (!mech.motionCompleted()) {
                        if (mech.current_speed >= 0) {
                            mech.set_position(mech.MECH_HIGHEST_POSITION);
                        } else {
                            mech.set_position(mech.MECH_LOWEST_POSITION);
                        }
                        mech.set_speed(pid_ins.update());
                    } else {
                        mech.set_speed(0);
                    }
                }

                _data._time_in_current_state + VENTILATION_CONTROL_CYCLE;
            }
        }
        break;

        case Init_Exsufflation:
        {
            // Open Solenoid Valve
            ins_exp_solenoid.set(SOLENOID_EXS); //digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

            totalCyclesInThisState = _timeoutEsp / TIME_BASE;
            _sensors->saveVolume();

            /* Stepper control*/
            _stepper->setSpeedInStepsPerSecond(_stepperSpeed);
            _stepper->setAccelerationInStepsPerSecondPerSecond(
                STEPPER_ACC_EXSUFFLATION);
            _stepper->setTargetPositionInSteps(
                STEPPER_DIR * (STEPPER_LOWEST_POSITION));

            /* Status update and reset timer, for next time */
            _data._currentState = State_Exsufflation;
            _data._time_in_current_state = 0;
            pid_exs.reset();
        }
        //break; // MUST be commented

        case State_Exsufflation:
        {
            // Time out
            if (_data._time_in_current_state > _timeout_ins)
            {
                mech.set_speed(0);
                _data._currentState = Init_Insufflation;
                _data._time_in_current_state = 0;
            } else {
                if (!mech.motionCompleted()) {
                    mech.set_position(MECH_LOWEST_POSITION);
                    mech.set_speed(pid_exs.update());
                } else {
                    mech.set_speed(0);
                    _data._currentState = State_Waiting_For_Trigger;
                }
                    
                _data._time_in_current_state + VENTILATION_CONTROL_CYCLE;
            }
        }
        break;

        case State_Waiting_For_Trigger:
        {
            if ((_data._time_in_current_state > _timeout_ins) || check_trigger())
            {
                mech.set_speed(0);
                _data._currentState = Init_Exsufflation;
                _data._time_in_current_state = 0;
            }
        }
        break;

        case State_Homing:
        {
            // Open Solenoid Valve
            ins_exp_solenoid.set(SOLENOID_EXS); //digitalWrite(PIN_SOLENOID, SOLENOID_OPEN);

            if (!mech.in_lower_possition())
            {
                mech.do_homming();
            }

            /* Status update and reset timer, for next time */
            _data._time_in_current_state = 0;
            _data._currentState = Init_Exsufflation;
        }
        break;

        case State_Error:
            //TODO
        break;

        default:
            //TODO
        break;
    }

}

