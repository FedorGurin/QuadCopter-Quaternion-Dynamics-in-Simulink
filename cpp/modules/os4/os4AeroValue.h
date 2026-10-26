#pragma once

#ifndef OS4_AERO_VALUE_H
#define OS4_AERO_VALUE_H

#include "ICalculateElement.h"
#include "os4Data.h"

/*! \brief Параметры набегающего потока (подсистемы "vel to Body" и "aero value")
 *
 *  Vb    = q~ * Ve * q
 *  Vinf  = |Vb|
 *  psi   = atan2(Vby, Vbx)
 *  theta = asin(-Vbz / Vinf)
 *  rho   = 1.23 (от высоты не зависит)
 *
 *  Ветра нет: воздушная скорость равна скорости корпуса.
 * \ingroup os4
*/
class OS4AeroValue : public ICalculateElement
{
public:
    OS4AeroValue(uint32_t idClass);

    virtual void calculate() override;
    virtual void finite() override {}

private:
    //! выход модуля
    TOS4Aero   aero;
    //! вектор состояния
    TOS4State *state;
};

#endif // OS4_AERO_VALUE_H
