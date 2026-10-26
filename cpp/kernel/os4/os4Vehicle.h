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
 *  Интегрирование - метод Рунге-Кутты 4-го порядка, функция math_func::rk4
 *  из SimTVRS: один шаг на каждый вызов calculate(). Шаг задает диспетчер
 *  через setFreq() (в Simulink - ode45 с переменным шагом).
 * \ingroup os4
*/
class OS4Vehicle : public ICalculateElement
{
public:
    OS4Vehicle(uint32_t idClass);

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
    /*! правые части ОДУ в виде массивов (интерфейс для math_func::rk4)
     * \param in  - вектор состояния, OS4_STATE_SIZE чисел
     * \param out - производные,       OS4_STATE_SIZE чисел
    */
    void SDU(const double *in, double *out);

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
