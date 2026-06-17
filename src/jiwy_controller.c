#include "jiwy_controller.h"
#include <stdint.h>
#include <stdio.h>
#include "math.h"
#include "panmodel.h"
#include "tiltmodel.h"
#include "stdlib.h"

uint32_t setPWM(int enable, int dir, int duty)
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

    // tilt_V[9]  = input for the tilt model
    // tilt_V[10] = position of the tilt jiwy (from encoder)
    // tilt_V[11] = output -> range from -0.99 to 0.99

    // initialize arrays to all 0
    // for (int i = 0; i < 3; i++)
    //     jiwy->panInput[i] = 0.0;
    // for (int i = 0; i < 3; i++)
    //     jiwy->panOutput[i] = 0.0;
    // for (int i = 0; i < 4; i++)
    //     jiwy->tiltInput[i] = 0.0;
    // for (int i = 0; i < 2; i++)
    //     jiwy->tiltOutput[i] = 0.0;

    TiltodelInitialize();
    PanModelInitialize();
}

void SetTiltPWM(Jiwy *jiwy)
{
    int dir = 0;
    if (tilt_V[11] <= 0)
    {
        dir = 1;
    }
    int enable = 1;
    uint8_t duty = (uint8_t)(abs(tilt_V[11] * 128));
    *jiwy->pwmTiltPtr = setPWM(enable, dir, duty);
}

void SetPanPWM(Jiwy *jiwy)
{
    int dir = 0;
    if (Pan_V[11] <= 0)
    {
        dir = 1;
    }
    int enable = 1;
    uint8_t duty = (uint8_t)(abs(Pan_V[11] * 128));
    *jiwy->pwmPanPtr = setPWM(enable, dir, duty);
}

double getTilt(Jiwy *jiwy)
{
    return (*((int32_t *)jiwy->encoderTiltPtr) - jiwy->tiltMin) / ((double)(jiwy->tiltMax - jiwy->tiltMin));
}

double getPan(Jiwy *jiwy)
{
    return (*((int32_t *)jiwy->encoderPanPtr) - jiwy->panMin) / ((double)(jiwy->panMax - jiwy->panMin));
}

// Sweeps one way set min value, sweeps the other way sets a max TODO: check if not other way around
void CalibratePan(Jiwy *jiwy)
{
    *jiwy->pwmPanPtr = setPWM(1, 1, 24);
    // sweep to min
    jiwy->panMin = waitForEncoderEndStop((uint32_t *)jiwy->encoderPanPtr);
    *jiwy->pwmPanPtr = setPWM(1, 0, 24);
    // sweep to max
    jiwy->panMax = waitForEncoderEndStop((uint32_t *)jiwy->encoderPanPtr);
    *jiwy->pwmPanPtr = setPWM(0, 0, 0); // brake to stop the motor
}

// Sweeps one way set min value, sweeps the other way sets a max TODO: check if not other way around
void CalibrateTilt(Jiwy *jiwy)
{
    *jiwy->pwmTiltPtr = setPWM(1, 1, 24);
    uint32_t temp, prevTemp;
    prevTemp = 0;
    // sweep to min
    jiwy->tiltMin = waitForEncoderEndStop((int32_t *)jiwy->encoderTiltPtr);
    *jiwy->pwmTiltPtr = setPWM(1, 0, 24);
    // sweep to max
    jiwy->tiltMax = waitForEncoderEndStop((int32_t *)jiwy->encoderTiltPtr);
    *jiwy->pwmTiltPtr = setPWM(0, 0, 0); // break to stop the motor
}

void Update(Jiwy *jiwy)
{
    // set outputs
    SetTiltPWM(jiwy);
    SetPanPWM(jiwy);

    // measure
    tilt_V[10] = getTilt(jiwy);
    Pan_V[10]  = getPan(jiwy);

    // calculate
    TiltCalculateDynamic();
    PanCalculateDynamic();
}