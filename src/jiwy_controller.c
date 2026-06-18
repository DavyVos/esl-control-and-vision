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

uint32_t waitForEncoderEndStop(uint32_t *numberPtr)
{
    uint32_t temp, prevTemp;
    prevTemp = 0;
    while (1)
    {
        temp = *numberPtr;
        if (abs(temp - prevTemp) < 2)
        {
            return temp;
        }
        prevTemp = temp;
        usleep(10000);
    }
}

void Jiwy_Init(Jiwy *jiwy,
               uint32_t *encoderPan,
               uint32_t *encoderTilt,
               uint32_t *pwmPan,
               uint32_t *pwmTilt)
{
    jiwy->encoderPanPtr = encoderPan;
    jiwy->encoderTiltPtr = encoderTilt;
    jiwy->pwmPanPtr = pwmPan;
    jiwy->pwmTiltPtr = pwmTilt;

    jiwy->panMin = 0;
    jiwy->panMax = 0;
    jiwy->tiltMin = 0;
    jiwy->tiltMax = 0;

    TiltModelInitialize();
    PanModelInitialize();
}

void Jiwy_SetTiltPWM(Jiwy *jiwy)
{
    int dir = 0;
    if (jiwy->tilt_velocity <= 0)
    {
        dir = 1;
    }
    int enable = 1;
    uint32_t duty = (uint32_t)(abs(jiwy->tilt_velocity * 128));
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
    uint32_t duty = (uint32_t)(abs(jiwy->pan_velocity * 128));
    *jiwy->pwmPanPtr = sJiwy_etPWM(enable, dir, duty);
}

double Jiwy_getTilt(Jiwy *jiwy) //convert uint32_t to a callibrated double
{
    return (*((int32_t *)jiwy->encoderTiltPtr) - jiwy->tiltMin) / ((double)(jiwy->tiltMax - jiwy->tiltMin));
}

double Jiwy_getPan(Jiwy *jiwy) //convert uint32_t to a callibrated double
{
    return (*((int32_t *)jiwy->encoderPanPtr) - jiwy->panMin) / ((double)(jiwy->panMax - jiwy->panMin));
}

// Sweeps one way set min value, sweeps the other way sets a max TODO: check if not other way around
void Jiwy_CalibratePan(Jiwy *jiwy)
{
    *jiwy->pwmPanPtr = Jiwy_setPWM(1, 1, 24);
    // sweep to min
    jiwy->panMin = waitForEncoderEndStop((uint32_t *)jiwy->encoderPanPtr);
    *jiwy->pwmPanPtr = setPWM(1, 0, 24);
    // sweep to max
    jiwy->panMax = waitForEncoderEndStop((uint32_t *)jiwy->encoderPanPtr);
    *jiwy->pwmPanPtr = setPWM(0, 0, 0); // brake to stop the motor
}

// Sweeps one way set min value, sweeps the other way sets a max TODO: check if not other way around
void Jiwy_CalibrateTilt(Jiwy *jiwy)
{
    *jiwy->pwmTiltPtr = setPWM(1, 1, 24);
    uint32_t temp, prevTemp;
    prevTemp = 0;
    // sweep to min
    jiwy->tiltMin = waitForEncoderEndStop((int32_t *)jiwy->encoderTiltPtr);
    *jiwy->pwmTiltPtr = Jiwy_setPWM(1, 0, 24);
    // sweep to max
    jiwy->tiltMax = waitForEncoderEndStop((int32_t *)jiwy->encoderTiltPtr);
    *jiwy->pwmTiltPtr = Jiwy_setPWM(0, 0, 0); // break to stop the motor
}

void Update(Jiwy *jiwy)
{
    // set outputs
    Jiwy_SetTiltPWM(jiwy);
    Jiwy_SetPanPWM(jiwy);

    // measure and set current positions
    jiwy->tilt_current = Jiwy_getTilt(jiwy);
    jiwy->pan_current = Jiwy_getPan(jiwy);

    XXDouble tilt_inputs[3]  = {0.0, (XXDouble)(jiwy->tilt_target), (XXDouble)(jiwy->tilt_current)};
    XXDouble tilt_outputs[3] = {0.0, 0.0, 0.0};
    XXDouble pan_inputs[3] = {0.0, (XXDouble)(jiwy->pan_target), (XXDouble)(jiwy->pan_current)};
    XXDouble pan_outputs[3] = {0.0, 0.0, 0.0};

    // calculate
    TiltCalculateSubmodel(tilt_inputs, tilt_outputs, (XXDouble)(jiwy->time));
    PanCalculateSubmodel(pan_inputs, pan_outputs, (XXDouble)(jiwy->time));
    jiwy->time += jiwy->dt;

    //Save output values for next iteration
    jiwy->tilt_velocity = tilt_outputs[2];
    jiwy->pan_velocity = pan_outputs[2];
}