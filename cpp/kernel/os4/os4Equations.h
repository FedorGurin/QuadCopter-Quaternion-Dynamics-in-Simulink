#pragma once

#ifndef OS4_EQUATIONS_H
#define OS4_EQUATIONS_H

#include "ICalculateElement.h"
#include "globalData.h"

/*! \brief Правые части уравнений движения (подсистема "12 ODEs")
 *
 *  dXe/dt = Ve                                   ("position dot")
 *  dVe/dt = (F_prop + F_fuselage + F_gravity)/m  ("acceleration")
 *  dp/dt  = ((Iyy - Izz)*q*r + Mx) / Ixx         ("rotational acceleration")
 *  dq/dt  = ((Izz - Ixx)*p*r + My) / Iyy
 *  dr/dt  = ((Ixx - Iyy)*p*q + Mz) / Izz
 *  dQ/dt  = 1/2 * Q * [0, p, q, r]               ("Quaternion dot")
 *
 *  Сюда же собираются производные скоростей винтов и индуктивного
 *  потока, посчитанные в OS4Propeller.
 * \ingroup os4
*/
class OS4Equations : public ICalculateElement
{
public:
    OS4Equations(uint32_t idClass);

    virtual void calculate() override;
    virtual void finite() override {}

private:
    //! выход модуля: производные вектора состояния
    TOS4State        dx;
    //! физические параметры
    TOS4Physical    *physical;
    //! вектор состояния
    TOS4State       *state;
    //! силы
    TOS4Gravity     *gravity;
    TOS4ForceMoment *fuselage;
    TOS4Propeller   *prop;
};

#endif // OS4_EQUATIONS_H
