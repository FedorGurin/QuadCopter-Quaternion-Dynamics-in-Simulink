#pragma once

#ifndef OS4_FUSELAGE_H
#define OS4_FUSELAGE_H

#include "ICalculateElement.h"
#include "globalData.h"

/*! \brief Аэродинамика корпуса (подсистема "fuselage aerodynamics")
 *
 *  q_S = 1/2 * rho * Vinf^2 * Ab
 *  CD  = 0.65 - 0.45 * cos(theta)
 *  CL  = 0.5 * sin(2*theta)
 *  F_air = [-q_S*CD, 0, -q_S*CL] - в скоростной СК
 *  F     = Rz(psi) * Ry(theta) * F_air - поворот к связанной СК
 *  Fe    = q * F * q~
 *
 *  Момент корпуса равен нулю (как в Simulink).
 * \ingroup os4
*/
class OS4Fuselage : public ICalculateElement
{
public:
    OS4Fuselage(uint32_t idClass);

    virtual void calculate() override;
    virtual void finite() override {}

private:
    //! выход модуля
    TOS4ForceMoment fuselage;
    //! физические параметры
    TOS4Physical   *physical;
    //! вектор состояния
    TOS4State      *state;
    //! параметры потока
    TOS4Aero       *aero;
};

#endif // OS4_FUSELAGE_H
