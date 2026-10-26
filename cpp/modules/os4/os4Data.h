#pragma once

#ifndef OS4_DATA_H
#define OS4_DATA_H

#include <stdint.h>
#include "constants.h"
#include "math_func.h"

using math_func::TVector3;
using math_func::TQuaternion;

/*! \defgroup data Структуры данных
 *  \brief Данные, которыми обмениваются модули (аналог шин Simulink)
 *  \ingroup kernel
*/

//! Параметры инерции (подсистема "inertia", OS4_physical.inertia) \ingroup data
typedef struct
{
    //! масса, кг
    double mass;
    //! осевые моменты инерции, кг*м^2
    double Ixx;
    double Iyy;
    double Izz;
    //! центробежные моменты инерции, кг*м^2 (в уравнениях не используются, как и в Simulink)
    double Ixy;
    double Ixz;
    double Iyz;
    //! момент инерции винта с ротором мотора, кг*м^2
    double Ip;
}TOS4Inertia;

//! Параметры винта (OS4_physical.prop) \ingroup data
typedef struct
{
    //! хорда лопасти, м
    double chord;
    //! радиус винта R, м
    double rad;
    //! угол установки на конце лопасти, рад
    double thetaTip;
    //! коэффициент заполнения диска sigma = A_лопастей / A_диска
    double sigma;
    //! наклон кривой подъемной силы профиля a0, 1/рад
    double a0;
}TOS4Prop;

/*! Физические параметры квадрокоптера (структура OS4_physical из OS4dynamics_01_init.m)
 * \ingroup data
*/
typedef struct
{
    TOS4Inertia inertia;
    TOS4Prop    prop;
    //! размеры корпуса, м
    TVector3    fuselageSize;
    //! площадь корпуса (миделя), м^2
    double      fuselageArea;
    //! длина луча (от центра до оси мотора), м
    double      armLength;
    //! положение винтов в связанной СК, м
    TVector3    rotorPos[OS4_NUM_ROTORS];
    //! направление вращения винтов: -1 тяга вдоль -z, +1 вдоль +z
    double      rotorDir[OS4_NUM_ROTORS];
    //! постоянная времени индуктивного потока tau_lambda0, с
    double      downwashTau;
}TOS4Physical;

/*! \brief Настроечные параметры модуля OS4: начальные условия
 *
 * Аналог структур OS4_initX, OS4_initR, OS4_initC. Доступны через МППМ
 * под именем "OS4_Setup". Чтобы новые начальные условия вступили в силу
 * во время работы, нужно записать reset = 1: модель перезапустится
 * с этими условиями и сбросит признак.
 *
 * Только double и один байт в конце - раскладка в памяти простая
 * (смещения полей приведены в README).
 * \ingroup data
*/
typedef struct
{
    //! положение в земной СК (NED, ось z вниз), м
    TVector3    pos;
    //! скорость в земной СК, м/с
    TVector3    vel;
    //! углы Эйлера 3-2-1 [крен, тангаж, курс], град
    TVector3    euler;
    //! угловая скорость [p q r] в связанной СК, рад/с
    TVector3    w;
    //! угловые скорости винтов, рад/с
    double      omega[OS4_NUM_ROTORS];
    //! начальная индуктивная скорость винтов, м/с (lambda0 = vInd/(omega*R))
    double      vInd;
    //! 1 - перезапустить модель с этими начальными условиями
    uint8_t     reset;
}TOS4Setup;

/*! \brief Входные параметры модуля OS4 (вход control модели Simulink)
 *
 * Доступны через МППМ под именем "OS4_Input".
 * По умолчанию на все моторы подается 0.1 Н*м (блок Constant
 * [0.1 0.1 0.1 0.1] перед "Input Switch").
 * \ingroup data
*/
typedef struct
{
    //! команда на моторы (момент), Н*м; ограничивается диапазоном [0, 0.2]
    double tau[OS4_NUM_ROTORS];
}TOS4Input;

/*! \brief Настройки модели
 *
 * По умолчанию gyroWithIp и useThetaTip включены: это исправленный
 * вариант модели. Если их выключить, модель на C++ повторяет
 * Simulink-модель OS4dynamics_01.slx один в один, включая ее
 * особенности (ключ --simulink программы os4sim).
 * \ingroup data
*/
typedef struct
{
    /*! Умножать гироскопический момент винтов на Ip (по умолчанию да).
     *  В Simulink множителя нет: M_gyro = [-sum(dir*q*w), sum(dir*p*w), 0]
    */
    bool gyroWithIp;
    /*! Брать угол на конце лопасти из OS4_physical.prop.thetatip (по умолчанию да).
     *  В Simulink блок "theta_t" (Fcn u(1)) берет первый элемент шины
     *  geometry, а это радиус винта R = 0.075, т.е. угол ~4.3 град.
    */
    bool useThetaTip;
    //! останавливать расчет при касании винтом земли (Stop Simulation)
    bool stopOnCollision;
}TOS4Options;

//! размер вектора состояния: 3+3+3+4 твердое тело, 4 скорости винтов, 4 индуктивных потока \ingroup data
const int OS4_STATE_SIZE = 13 + 2 * OS4_NUM_ROTORS;

/*! \brief Вектор состояния модели (все интеграторы Simulink)
 *
 * 13 состояний твердого тела + 4 скорости винтов + 4 индуктивных потока = 21
 * \ingroup data
*/
class TOS4State
{
public:
    TOS4State()
    {
        clear();
    }
    void clear()
    {
        pos = TVector3();
        vel = TVector3();
        w   = TVector3();
        q   = TQuaternion();
        for(int i = 0; i < OS4_NUM_ROTORS; i++)
        {
            omega[i]   = 0.0;
            lambda0[i] = 0.0;
        }
    }
    //! интегратор "integrate X.pos": положение в земной СК, м
    TVector3    pos;
    //! интегратор "integrate X.vel": скорость в земной СК, м/с
    TVector3    vel;
    //! интегратор "Integrate R.w": угловая скорость [p q r], рад/с
    TVector3    w;
    //! интегратор "Integrate R.q": кватернион (до нормализации)
    TQuaternion q;
    //! интеграторы "|w|": угловые скорости винтов, рад/с
    double      omega[OS4_NUM_ROTORS];
    //! интеграторы "lambda_0": индуктивный поток винтов
    double      lambda0[OS4_NUM_ROTORS];

    //! упаковать в массив из OS4_STATE_SIZE чисел (для math_func::rk4)
    void toArray(double *v) const
    {
        int n = 0;
        v[n++] = pos.x; v[n++] = pos.y; v[n++] = pos.z;
        v[n++] = vel.x; v[n++] = vel.y; v[n++] = vel.z;
        v[n++] = w.x;   v[n++] = w.y;   v[n++] = w.z;
        v[n++] = q.q0;  v[n++] = q.q1;  v[n++] = q.q2;  v[n++] = q.q3;
        for(int i = 0; i < OS4_NUM_ROTORS; i++) v[n++] = omega[i];
        for(int i = 0; i < OS4_NUM_ROTORS; i++) v[n++] = lambda0[i];
    }
    //! распаковать из массива из OS4_STATE_SIZE чисел
    void fromArray(const double *v)
    {
        int n = 0;
        pos.x = v[n++]; pos.y = v[n++]; pos.z = v[n++];
        vel.x = v[n++]; vel.y = v[n++]; vel.z = v[n++];
        w.x   = v[n++]; w.y   = v[n++]; w.z   = v[n++];
        q.q0  = v[n++]; q.q1  = v[n++]; q.q2  = v[n++]; q.q3  = v[n++];
        for(int i = 0; i < OS4_NUM_ROTORS; i++) omega[i]   = v[n++];
        for(int i = 0; i < OS4_NUM_ROTORS; i++) lambda0[i] = v[n++];
    }
};

/*! Аэродинамические параметры движения (сигнал Aero, подсистема "aero value")
 * \ingroup data
*/
typedef struct
{
    //! скорость в связанной СК, м/с
    TVector3 Vb;
    //! модуль воздушной скорости, м/с
    double   Vinf;
    //! азимут набегающего потока в связанной СК, рад
    double   psi;
    //! угол набегающего потока к плоскости xy связанной СК, рад
    double   theta;
    //! плотность воздуха, кг/м^3
    double   rho;
}TOS4Aero;

//! Выход подсистемы "gravity" \ingroup data
typedef struct
{
    //! сила тяжести в земной СК, Н
    TVector3 Fe;
    //! ускорение -g в связанной СК (для акселерометра, IMU_acc_g), м/с^2
    TVector3 gB;
}TOS4Gravity;

//! Силы и моменты (выход подсистем аэродинамики) \ingroup data
typedef struct
{
    //! сила в земной СК, Н
    TVector3 Fe;
    //! момент в связанной СК, Н*м
    TVector3 Mb;
}TOS4ForceMoment;

/*! Выход подсистемы "Propeller & motor dynamics"
 * \ingroup data
*/
typedef struct
{
    //! сила тяги в земной СК, Н
    TVector3 Fe;
    //! суммарный момент винтов в связанной СК, Н*м
    TVector3 Mb;
    //! составляющие момента: от тяги (R cross F), гироскопический, реактивный
    TVector3 M_thrust;
    TVector3 M_gyro;
    TVector3 M_react;
    //! команда (момент мотора) после ограничения, Н*м
    double tau[OS4_NUM_ROTORS];
    //! тяга винта, Н
    double T[OS4_NUM_ROTORS];
    //! аэродинамический момент винта, Н*м
    double Q[OS4_NUM_ROTORS];
    //! коэффициенты тяги и момента
    double CT[OS4_NUM_ROTORS];
    double CQ[OS4_NUM_ROTORS];
    //! индуктивный поток с учетом влияния земли
    double lambdaIGE[OS4_NUM_ROTORS];
    //! высота винта над землей, м
    double h[OS4_NUM_ROTORS];
    //! производные: domega/dt и dlambda0/dt
    double dOmega[OS4_NUM_ROTORS];
    double dLambda0[OS4_NUM_ROTORS];
    //! признак касания земли (Compare To Constant, h/R <= 1/4)
    bool   collision;
}TOS4Propeller;

/*! Выходные параметры модуля OS4 (порты stateX, stateB, stateQ, aero V,
 * IMU, Motor Sensor и переменные os4sim_* рабочего пространства MATLAB).
 * Доступны через МППМ под именем "OS4_Output".
 * \ingroup data
*/
typedef struct
{
    //! модельное время, с
    double      time;
    //! stateX: скорость и положение в земной СК
    TVector3    Ve;
    TVector3    Xe;
    //! stateB: скорость в связанной СК и угловая скорость
    TVector3    Vb;
    TVector3    pqr;
    //! stateQ: нормализованный кватернион
    TQuaternion q;
    //! углы Эйлера 3-2-1 [крен, тангаж, курс], рад (для удобства изучения)
    TVector3    euler;
    //! aero V
    TOS4Aero    aero;
    //! IMU: гироскоп и акселерометр (кажущееся ускорение) в связанной СК
    TVector3    gyro;
    TVector3    acc;
    //! stateHeading: орты осей x и y корпуса в земной СК
    TVector3    headingX;
    TVector3    headingY;
    //! Motor Sensor: [tau(4) T(4) omega(4)]
    double      tau[OS4_NUM_ROTORS];
    double      T[OS4_NUM_ROTORS];
    double      omega[OS4_NUM_ROTORS];
}TOS4Output;

#endif // OS4_DATA_H
