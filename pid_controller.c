//*********************************************************************************
// Arduino PID Library Version 1.0.1 Modified Version for C -
// Platform Independent
//
// Revision: 1.1
//
// Description: The PID Controller module originally meant for Arduino made
// platform independent. Some small bugs present in the original Arduino source
// have been rectified as well.
//
// For a detailed explanation of the theory behind this library, go to:
// http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
//
// Revisions can be found here:
// https://github.com/tcleg
//
// Modified by: Trent Cleghorn , <trentoncleghorn@gmail.com>
//
// Copyright (C) Brett Beauregard , <br3ttb@gmail.com>
//
//                                 GPLv3 License
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
// PARTICULAR PURPOSE.  See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <http://www.gnu.org/licenses/>.
//*********************************************************************************

//*********************************************************************************
// Headers
//*********************************************************************************
#include "pid_controller.h"

//*********************************************************************************
// Macros and Globals
//*********************************************************************************
#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

//*********************************************************************************
// Functions
//*********************************************************************************
void PIDInit(PIDControl *pid, float kp, float ki, float kd,
             float minOutput, float maxOutput, float offset,
             PIDMode mode, PIDDirection controllerDirection)
{
    pid->controllerDirection = controllerDirection;
    pid->mode = mode;
    pid->iTerm = 0.0f;
    pid->input = 0.0f;
    pid->lastInput = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    pid->offset = offset;

    PIDOutputLimitsSet(pid, minOutput, maxOutput);
    PIDTuningsSet(pid, kp, ki, kd);
}

bool
PIDCompute(PIDControl *pid, float dt)
{
    float error, dInput;

    if(pid->mode == MANUAL)
    {
        return false;
    }

    // The classic PID error term
    error = (pid->setpoint) - (pid->input);

    // Compute the integral term separately ahead of time
    pid->iTerm += pid->alteredKi * dt * error;

    // Constrain the integrator to make sure it does not exceed output bounds
    pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );

    // Take the "derivative on measurement" instead of "derivative on error"
    dInput = (pid->input) - (pid->lastInput);

    // Run all the terms together to get the overall output
    pid->output = pid->alteredKp * error + (pid->iTerm) - pid->alteredKd *
	    dInput / dt;

    // Bound the output
    pid->output = CONSTRAIN( (pid->output), (pid->outMin), (pid->outMax) );

    // Make the current input the former input
    pid->lastInput = pid->input;

    return true;
}

void
PIDModeSet(PIDControl *pid, PIDMode mode)
{
    // If the mode changed from MANUAL to AUTOMATIC
    if(pid->mode != mode && mode == AUTOMATIC)
    {
        // Initialize a few PID parameters to new values
        pid->iTerm = pid->output;
        pid->lastInput = pid->input;

        // Constrain the integrator to make sure it does not exceed output bounds
        pid->iTerm = CONSTRAIN( (pid->iTerm), (pid->outMin), (pid->outMax) );
    }

    pid->mode = mode;
}

void
PIDOutputLimitsSet(PIDControl *pid, float min, float max)
{
    // Check if the parameters are valid
    if(min >= max)
    {
        return;
    }

    // Save the parameters
    pid->outMin = min - pid->offset;
    pid->outMax = max - pid->offset;

    // If in automatic, apply the new constraints
    if(pid->mode == AUTOMATIC)
    {
        pid->output = CONSTRAIN(pid->output, pid->outMin, pid->outMax);
        pid->iTerm  = CONSTRAIN(pid->iTerm,  pid->outMin, pid->outMax);
    }
}

void
PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd)
{
    // Check if the parameters are valid
    if(kp < 0.0f || ki < 0.0f || kd < 0.0f)
    {
        return;
    }

    // Save the parameters for displaying purposes
    pid->dispKp = kp;
    pid->dispKi = ki;
    pid->dispKd = kd;

    // Alter the parameters for PID
    pid->alteredKp = kp;
    pid->alteredKi = ki;
    pid->alteredKd = kd;

    // Apply reverse direction to the altered values if necessary
    if(pid->controllerDirection == REVERSE)
    {
        pid->alteredKp = -(pid->alteredKp);
        pid->alteredKi = -(pid->alteredKi);
        pid->alteredKd = -(pid->alteredKd);
    }
}

void
PIDTuningKpSet(PIDControl *pid, float kp)
{
    PIDTuningsSet(pid, kp, pid->dispKi, pid->dispKd);
}

void
PIDTuningKiSet(PIDControl *pid, float ki)
{
    PIDTuningsSet(pid, pid->dispKp, ki, pid->dispKd);
}

void
PIDTuningKdSet(PIDControl *pid, float kd)
{
    PIDTuningsSet(pid, pid->dispKp, pid->dispKi, kd);
}
