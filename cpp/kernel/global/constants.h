#pragma once

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define _USE_MATH_DEFINES
#include <math.h>

/*! Ускорение свободного падения, м/с^2
 * (в модели Simulink используется 9.81, а не 9.80665)
 * \ingroup kernel
*/
const double g_acc        = 9.81;
//! Плотность воздуха у земли, кг/м^3 (блок "altitude air density" = 1.23) \ingroup kernel
const double rho_0        = 1.23;
//! коэффициент преобразования градусы в радианы \ingroup kernel
const double gradToRad    = M_PI / 180.0;
//! коэффициент преобразования радианы в градусы \ingroup kernel
const double radToGrad    = 1.0 / gradToRad;
//! количество винтов квадрокоптера \ingroup kernel
const int    OS4_NUM_ROTORS = 4;

#endif // CONSTANTS_H
