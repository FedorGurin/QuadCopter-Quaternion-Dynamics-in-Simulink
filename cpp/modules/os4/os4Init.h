#pragma once

#ifndef OS4_INIT_H
#define OS4_INIT_H

#include "os4Data.h"

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
TOS4Setup os4SetupInit();

/*! Входные параметры по умолчанию (0.1 Н*м на каждый мотор)
 * \ingroup os4
*/
TOS4Input os4InputInit();

/*! Настройки модели по умолчанию (исправленный вариант, см. TOS4Options)
 * \ingroup os4
*/
TOS4Options os4DefaultOptions();

#endif // OS4_INIT_H
