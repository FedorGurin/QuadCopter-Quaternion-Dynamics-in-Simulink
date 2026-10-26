#pragma once

#ifndef OS4_PROPELLER_H
#define OS4_PROPELLER_H

#include "ICalculateElement.h"
#include "os4Data.h"

/*! \brief Винты и моторы (подсистема "Propeller & motor dynamics")
 *
 *  Обозначения формул как в Simulink: ( ) - Pounds 2007,
 *  [ ] - Bouabdallah 2007, { } - Simplicio 2011.
 *
 *  Для каждого винта i:
 *   - скорость потока у винта:  V = Vb + w x r_i  ("Tangent velocity")
 *     Vz = -V.z, Vh = sqrt(V.x^2 + V.y^2)          ("vertical speed")
 *   - влияние земли ("ground effect"):
 *     h/R = -z_i/R, lambda_IGE = lambda0 - lambda0 / (16*(h/R)^2) * cutoff(h/R)
 *   - коэффициент тяги [5.13]:
 *     C_T = sigma*a0/4 * (theta_t - (Vz + Vi)/(w*R))
 *   - индуктивный поток по Глауэрту {2-58},{2-59}:
 *     C_T^Gl = 2*lambda0*sqrt((Vh/wR)^2 + ((Vz+Vi)/wR)^2)
 *     dlambda0/dt = (C_T - C_T^Gl) / tau_lambda0
 *   - коэффициент момента (3.4): C_Q = max(C_T*sqrt(C_T/2), 0.001)
 *   - тяга [5.5]:   T = rho*pi*R^2*R^2*C_T*w^2
 *   - момент [5.6]: Q = rho*pi*R^2*R^3*C_Q*w*|w|
 *   - мотор [2.30]: dw/dt = (tau - Q) / Ip
 *
 *  Сила и моменты на корпус:
 *   Fe = q * [0, 0, -sum(T)] * q~
 *   Mb = sum(r_i x [0, 0, -T_i]) + M_gyro + [0, 0, sum(-dir_i*tau_i)]
 *   M_gyro = [-sum(dir_i*q*w_i), sum(dir_i*p*w_i), 0]  (* Ip, если gyroWithIp)
 * \ingroup os4
*/
class OS4Propeller : public ICalculateElement
{
public:
    OS4Propeller(uint32_t idClass);

    virtual void calculate() override;
    virtual void finite() override {}

private:
    //! выход модуля
    TOS4Propeller prop;
    //! физические параметры
    TOS4Physical *physical;
    //! настройки модели
    TOS4Options  *options;
    //! вектор состояния
    TOS4State    *state;
    //! параметры потока
    TOS4Aero     *aero;
    //! входные параметры: команда на моторы, Н*м
    TOS4Input    *input;
};

#endif // OS4_PROPELLER_H
