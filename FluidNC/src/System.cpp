// Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
  System.cpp - Header for system level commands and real-time processes
*/

#include "System.h"
#include "Report.h"                 // report_ovr_counter
#include "Config.h"                 // MAX_N_AXIS
#include "Machine/MachineConfig.h"  // config
#include "Stepping.h"               // config
#include "Serial.h"                 // AllChannels

#include <cstring>  // memset
#include <cmath>    // roundf

// Declare system global variable structure
system_t sys;
steps_t  probe_steps[MAX_N_AXIS];  // Last probe position in steps.

void system_reset() {
    // Reset system variables.
    if (state_is(State::Starting)) {
        set_state(State::Idle);
    }

    State prior_state = sys.state();
    bool  prior_abort = sys.abort();
    sys.reset();  // Clear system struct variable.
    set_state(prior_state);
    sys.set_abort(prior_abort);
    sys.set_f_override(FeedOverride::Default);                 // Set to 100%
    sys.set_r_override(RapidOverride::Default);                // Set to 100%
    sys.set_spindle_speed_ovr(SpindleSpeedOverride::Default);  // Set to 100%
    memset(probe_steps, 0, sizeof(probe_steps));               // Clear probe position.
    report_ovr_counter = 0;
    report_wco_counter = 0;
}

// Individual axis versions
float steps_to_motor_pos(steps_t steps, size_t motor) {
    return float(steps / Axes::_axis[axis_t(motor)]->_stepsPerMm);
}
steps_t motor_pos_to_steps(float mpos, size_t motor) {
    return lroundf(mpos * Axes::_axis[axis_t(motor)]->_stepsPerMm);
}

// Array of axes versions
void motor_pos_to_steps(steps_t* steps, float* motor_pos) {
    auto   a      = config->_axes;
    axis_t n_axis = a ? a->_numberAxis : X_AXIS;
    for (size_t motor = 0; motor < size_t(n_axis); motor++) {
        steps[motor] = motor_pos_to_steps(motor_pos[motor], motor);
    }
}

void steps_to_motor_pos(float* motor_pos, steps_t* steps) {
    auto   a      = config->_axes;
    axis_t n_axis = a ? a->_numberAxis : X_AXIS;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        motor_pos[axis] = steps_to_motor_pos(steps[axis], axis);
    }
}

// Convert motor step count to mm and transform back to Cartesian (GCode) space
void steps_to_mpos(float* position, steps_t* steps) {
    auto   a      = config->_axes;
    axis_t n_axis = a ? a->_numberAxis : X_AXIS;
    float  motor_pos[MAX_N_AXIS];
    steps_to_motor_pos(motor_pos, steps);
    config->_kinematics->motors_to_cartesian(position, motor_pos, n_axis);
}

void set_steps(axis_t axis, steps_t steps) {
    Stepping::setSteps(axis, steps);
}

void set_motor_pos(size_t motor, float motor_pos) {
    set_steps(axis_t(motor), motor_pos_to_steps(motor_pos, motor));
}

void set_motor_pos(float* motor_pos, size_t n_motors) {
    for (size_t motor = 0; motor < n_motors; motor++) {
        set_steps(axis_t(motor), motor_pos_to_steps(motor_pos[motor], motor));
    }
}

// Position in motor steps for one axis
steps_t get_axis_steps(axis_t axis) {
    return Stepping::getSteps(axis);
}

// Position in motor steps for all axes
void get_steps(steps_t* steps) {
    auto axes   = config->_axes;
    auto n_axis = axes->_numberAxis;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        steps[axis] = Stepping::getSteps(axis);
    }
}

// The conversions below fill only the configured axes; zero the rest so a
// caller that copies the whole array sees what the old zero-filled static
// buffers used to give it.
static void zero_unused_axes(float* position) {
    for (axis_t axis = Axes::_numberAxis; axis < MAX_N_AXIS; axis++) {
        position[axis] = 0;
    }
}

// Position in mm in motor space for all axes
void get_motor_pos(float* motor_pos) {
    steps_t steps[MAX_N_AXIS] = { 0 };
    get_steps(steps);
    steps_to_motor_pos(motor_pos, steps);
    zero_unused_axes(motor_pos);
}

// Position in mm in Cartesian space for all axes, accounting for kinematics
void get_mpos(float* position) {
    steps_t steps[MAX_N_AXIS] = { 0 };
    get_steps(steps);
    steps_to_mpos(position, steps);
    zero_unused_axes(position);
}

void get_wco(float* wco) {
    auto n_axis = Axes::_numberAxis;
    for (axis_t axis = X_AXIS; axis < n_axis; axis++) {
        // Apply work coordinate offsets and tool length offset to current position.
        wco[axis] = gc_state.coord_system[axis] + gc_state.coord_offset[axis];
        wco[axis] += gc_state.tool_length_offset[axis];
    }
    zero_unused_axes(wco);
}

const std::map<State, const char*> StateName = {
    { State::Idle, "Idle" },
    { State::Alarm, "Alarm" },
    { State::CheckMode, "CheckMode" },
    { State::Homing, "Homing" },
    { State::Cycle, "Cycle" },
    { State::Hold, "Hold" },
    { State::Jog, "Jog" },
    { State::SafetyDoor, "SafetyDoor" },
    { State::Sleep, "Sleep" },
    { State::ConfigAlarm, "ConfigAlarm" },
    { State::Critical, "Critical" },
};

void set_state(State s) {
    if (!state_is(s)) {
        sys.set_state(s);
        allChannels.notifyState();
    }
}
bool IRAM_ATTR state_is(State s) {
    return sys.state() == s;
}

bool inMotionState() {
    return state_is(State::Cycle) || state_is(State::Homing) || state_is(State::Jog) ||
           (state_is(State::Hold) && !sys.suspend().bit.holdComplete);
}
