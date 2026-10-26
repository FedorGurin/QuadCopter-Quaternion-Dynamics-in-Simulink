#pragma once

#ifndef OS4_VEHICLE_H
#define OS4_VEHICLE_H

#include "ICalculateElement.h"
#include "globalData.h"

/*! \brief Квадрокоптер OS4 (верхний уровень модели OS4dynamics_01.slx)
 *
 *  Модуль хранит вектор состояния (все интеграторы Simulink)
 *  и содержит подчиненные модули в порядке расчета:
 *   1. OS4AeroValue  - "aero value"
 *   2. OS4Gravity    - "gravity"
 *   3. OS4Fuselage   - "fuselage aerodynamics"
 *   4. OS4Propeller  - "Propeller & motor dynamics"
 *   5. OS4Equations  - "12 ODEs"
 *
 *  Интегрирование - метод Рунге-Кутты 4-го порядка с постоянным шагом
 *  (в Simulink - ode45 с переменным шагом).
 * \ingroup os4
*/
class OS4Vehicle : public ICalculateElement
{
public:
    OS4Vehicle(uint32_t idClass, double step_ = 0.001);

    //! установка начальных условий
    virtual void init() override;
    //! один шаг интегрирования
    virtual void calculate() override;
    //! расчет выходов модели на текущем шаге
    virtual void finite() override;

    //! задать команду на моторы, Н*м
    void setControl(const double tau[OS4_NUM_ROTORS]);
    //! вычислить производные вектора состояния x
    void derivatives(const TOS4State &x, TOS4State &dx);

    //! физические параметры (можно менять до init())
    TOS4Physical  physical;
    //! начальные условия (можно менять до init())
    TOS4InitState initState;
    //! настройки модели
    TOS4Options   options;
    //! выходы модели
    TOS4Output    output;
    //! модельное время, с
    double        time;

private:
    //! текущее состояние (результат интегрирования)
    TOS4State  x;
    //! состояние, по которому подчиненные модули считают производные
    TOS4State  xEval;
    //! команда на моторы
    double     control[OS4_NUM_ROTORS];

    //! выходы подчиненных модулей
    TOS4Aero      *aero;
    TOS4Gravity   *gravity;
    TOS4Propeller *prop;
    TOS4State     *dxEval;
};

#endif // OS4_VEHICLE_H
