#pragma once

#ifndef OS4_GRAVITY_H
#define OS4_GRAVITY_H

#include "ICalculateElement.h"
#include "globalData.h"

/*! \brief Сила тяжести (подсистема "gravity")
 *
 *  Fe = [0, 0, m*g] в земной СК (ось z вниз)
 *  gB = q~ * [0, 0, -g] * q - добавка для акселерометра (IMU_acc_g)
 *
 *  Переключатель "Attitude Switch" из Simulink (вариант [-m*g, 0, 0]
 *  от модели самолета-tail-sitter) не переносился.
 * \ingroup os4
*/
class OS4Gravity : public ICalculateElement
{
public:
    OS4Gravity(uint32_t idClass);

    virtual void calculate() override;
    virtual void finite() override {}

private:
    //! выход модуля
    TOS4Gravity   gravity;
    //! физические параметры
    TOS4Physical *physical;
    //! вектор состояния
    TOS4State    *state;
};

#endif // OS4_GRAVITY_H
