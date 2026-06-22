#ifndef JIWY_CONTROLLER_H
#define JIWY_CONTROLLER_H

#include <stdint.h>
#include "common/xxtypes.h"

typedef struct
{
  int16_t *encoderPanPtr;
  int16_t *encoderTiltPtr;
  int16_t *pwmPanPtr;
  int16_t *pwmTiltPtr;

  // pitch and pan extremes for conversion to double
  int16_t panMin, panMax;
  int16_t tiltMin, tiltMax;

  // converted pitch and pan positions to feed into submodels
  double tilt_target, tilt_current;
  double pan_target, pan_current;

  // submodel outputs
  double tilt_velocity, pan_velocity;

  double time;
  double dt;

} Jiwy;

void Jiwy_Init(Jiwy *jiwy,
               uint32_t *encoderPan,
               uint32_t *encoderTilt,
               uint32_t *pwmPan,
               uint32_t *pwmTilt);

//calibration motions
void Jiwy_CalibratePan(Jiwy *jiwy);
void Jiwy_CalibrateTilt(Jiwy *jiwy);

//void Jiwy_SetTargetPan(Jiwy *jiwy, double target);
void Jiwy_SetTargetTilt(Jiwy *jiwy, double target);

// Set the PWM output according to current Jiwy
void Jiwy_SetPanPWM(Jiwy *jiwy);
void Jiwy_SetTiltPWM(Jiwy *jiwy);

// Disable the motors (safety brake)
void Jiwy_Disable(Jiwy *jiwy);
void Jiwy_Update(Jiwy *jiwy);

double Jiwy_getPan(Jiwy *jiwy);
double Jiwy_getTilt(Jiwy *jiwy);

uint32_t Jiwy_setPWM(int enable, int dir, int duty);

#endif