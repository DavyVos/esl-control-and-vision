#include "jiwy_controller.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "pansubmod.h"
#include "tiltsubmod.h"
#include "stdlib.h"
#include <fcntl.h>
#include <unistd.h>

uint32_t Jiwy_setPWM(int enable, int dir, int duty)
{
    return (enable << 9) | (dir << 8) | (duty & 0xFF);
}

int16_t sanitizeEncoder(uint32_t raw)
{
    return (int16_t)(raw & 0xFFFF);
}

void Jiwy_Init(Jiwy *jiwy,
               uint32_t *encoderPan,
               uint32_t *encoderTilt,
               uint32_t *pwmPan,
               uint32_t *pwmTilt)
{
    jiwy->encoderPanPtr  = encoderPan;
    jiwy->encoderTiltPtr = encoderTilt;
    jiwy->pwmPanPtr      = pwmPan;
    jiwy->pwmTiltPtr     = pwmTilt;

    jiwy->panMin  = 0;
    jiwy->panMax  = 0;
    jiwy->tiltMin = 0;
    jiwy->tiltMax = 0;

    XXDouble pan_inputs[2]  = {(XXDouble)(jiwy->pan_target), (XXDouble)(jiwy->pan_current)};
    XXDouble pan_outputs[2] = {0.0, 0.0};

    PanInitializeSubmodel(pan_inputs, pan_outputs, jiwy->time);

    XXDouble tilt_inputs[3]  = {pan_outputs[0], (XXDouble)(jiwy->tilt_target), (XXDouble)(jiwy->tilt_current)};
    XXDouble tilt_outputs[1] = {0.0};
    jiwy->dt = Pan_step_size;

    TiltInitializeSubmodel(tilt_inputs, tilt_outputs, jiwy->time);
}

void Jiwy_SetTiltPWM(Jiwy *jiwy)
{
    int dir = 0;
    if (jiwy->tilt_velocity <= 0)
    {
        dir = 1;
    }
    int enable = 1;
    uint32_t duty = (uint8_t)(fabs(jiwy->tilt_velocity * 64));
    *jiwy->pwmTiltPtr = Jiwy_setPWM(enable, dir, duty);
}

void Jiwy_SetPanPWM(Jiwy *jiwy)
{
    int dir = 0;
    if (jiwy->pan_velocity <= 0)
    {
        dir = 1;
    }
    int enable = 1;
    uint32_t duty = (uint8_t)(fabs(jiwy->pan_velocity * 255));
    *jiwy->pwmPanPtr = Jiwy_setPWM(enable, dir, duty);
}

double Jiwy_getTilt(Jiwy *jiwy)
{
    double mid  = (jiwy->tiltMax + jiwy->tiltMin) / 2.0;
    double half = (jiwy->tiltMax - jiwy->tiltMin) / 2.0;
    return (sanitizeEncoder(*jiwy->encoderTiltPtr) - mid) / (half / 0.99);
}

double Jiwy_getPan(Jiwy *jiwy)
{
    double mid  = (jiwy->panMax + jiwy->panMin) / 2.0;
    double half = (jiwy->panMax - jiwy->panMin) / 2.0;
    return (sanitizeEncoder(*jiwy->encoderPanPtr) - mid) / (half / 0.99);
}

// Sweeps one way set min value, sweeps the other way sets a max
void Jiwy_CalibratePan(Jiwy *jiwy)
{
    *jiwy->pwmPanPtr = Jiwy_setPWM(1, 1, 30);
    usleep(1000000);
    jiwy->panMin = sanitizeEncoder(*jiwy->encoderPanPtr);
    // sweep to min
    *jiwy->pwmPanPtr = Jiwy_setPWM(1, 0, 30);
    // sweep to max
    usleep(1000000);
    jiwy->panMax = sanitizeEncoder(*jiwy->encoderPanPtr);
    *jiwy->pwmPanPtr = Jiwy_setPWM(0, 0, 0); // brake to stop the motor
}

// Sweeps one way set min value, sweeps the other way sets a max
void Jiwy_CalibrateTilt(Jiwy *jiwy)
{
    // sweep to min
    *jiwy->pwmTiltPtr = Jiwy_setPWM(1, 1, 30);
    usleep(1000000);
    *jiwy->pwmTiltPtr = Jiwy_setPWM(0, 1, 30);
    jiwy->tiltMin = sanitizeEncoder(*jiwy->encoderTiltPtr);
    *jiwy->pwmTiltPtr = Jiwy_setPWM(1, 0, 30);
    usleep(1000000);
    *jiwy->pwmTiltPtr = Jiwy_setPWM(0, 1, 30);
    jiwy->tiltMax = sanitizeEncoder(*jiwy->encoderTiltPtr);
    // sweep to max
    *jiwy->pwmTiltPtr = Jiwy_setPWM(0, 0, 0); // break to stop the motor
}

void Jiwy_Update(Jiwy *jiwy)
{
    // set outputs
    Jiwy_SetTiltPWM(jiwy);
    Jiwy_SetPanPWM(jiwy);

    // measure and set current positions
    jiwy->tilt_current = Jiwy_getTilt(jiwy);
    jiwy->pan_current  = Jiwy_getPan(jiwy);

    XXDouble pan_inputs[2]  = {(XXDouble)(jiwy->pan_target), (XXDouble)(jiwy->pan_current)};
    XXDouble pan_outputs[2] = {0.0, 0.0};

    // Calculate pan first to get correction value for tilt
    PanCalculateSubmodel(pan_inputs, pan_outputs, (XXDouble)(jiwy->time));

    // Feed pan's corr output into tilt's corr input
    XXDouble tilt_inputs[3]  = {pan_outputs[0], (XXDouble)(jiwy->tilt_target), (XXDouble)(jiwy->tilt_current)};
    XXDouble tilt_outputs[1] = {0.0};

    TiltCalculateSubmodel(tilt_inputs, tilt_outputs, (XXDouble)(jiwy->time));
    jiwy->time += jiwy->dt;

    // Save output values: pan velocity = pan_outputs[1], tilt velocity = tilt_outputs[0]
    jiwy->pan_velocity  = pan_outputs[1];
    jiwy->tilt_velocity = tilt_outputs[0];
}

void Jiwy_Disable(Jiwy *jiwy) {
    *jiwy->pwmPanPtr = Jiwy_setPWM(0,0,0);
    *jiwy->pwmTiltPtr = Jiwy_setPWM(0,0,0);
}