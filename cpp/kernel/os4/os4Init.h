#pragma once

#ifndef OS4_INIT_H
#define OS4_INIT_H

#include "globalData.h"

/*! \defgroup os4 Модель квадрокоптера OS4
 *  \brief Перенос Simulink-модели OS4dynamics_01.slx на C++
*/

/*! Физические параметры (функция OS4physical_init из OS4dynamics_01_init.m)
 *  Источник: S. Bouabdallah, 2007, с. 144
 * \ingroup os4
*/
TOS4Physical os4PhysicalInit();

/*! Начальные условия (функции OS4dynamics_init_state и OS4actuator_init)
 * \ingroup os4
*/
TOS4InitState os4InitState(const TOS4Physical &physical);

/*! Настройки модели по умолчанию (поведение как у Simulink-модели)
 * \ingroup os4
*/
TOS4Options os4DefaultOptions();

#endif // OS4_INIT_H
