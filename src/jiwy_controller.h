#ifndef JIWY_CONTROLLER_H
#define JIWY_CONTROLLER_H

#include <stdint.h>
#include "common/xxtypes.h"

typedef struct
{
  uint32_t *encoderPanPtr;
  uint32_t *encoderTiltPtr;
  uint32_t *pwmPanPtr;
  uint32_t *pwmTiltPtr;

  // pitch and pan extremes for conversion to double
  int32_t panMin, panMax;
  int32_t tiltMin, tiltMax;

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

void CalibratePan(Jiwy *jiwy);
void CalibrateTilt(Jiwy *jiwy);

void SetTargetPan(Jiwy *jiwy, double target);
void SetTargetTilt(Jiwy *jiwy, double target);

// Set the PWM output according to current Jiwy
void SetPanPWM(Jiwy *jiwy);
void SetTiltPWM(Jiwy *jiwy);

void Disable(Jiwy *jiwy);
void Update(Jiwy *jiwy);

double getPan(Jiwy *jiwy);
double getTilt(Jiwy *jiwy);

uint32_t setPWM(int enable, int dir, int duty);

#endif