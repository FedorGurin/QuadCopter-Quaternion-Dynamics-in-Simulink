#pragma once

#ifndef GLOBALDATA_H
#define GLOBALDATA_H

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

/*! Начальные условия (структуры OS4_initX, OS4_initR, OS4_initC)
 * \ingroup data
*/
typedef struct
{
    //! положение в земной СК (NED), м
    TVector3    pos;
    //! скорость в земной СК, м/с
    TVector3    vel;
    //! угловая скорость [p q r] в связанной СК, рад/с
    TVector3    w;
    //! кватернион ориентации
    TQuaternion q;
    //! угловые скорости винтов, рад/с
    double      omega[OS4_NUM_ROTORS];
    //! безразмерная индуктивная скорость lambda_0 = Vi/(omega*R)
    double      lambda0[OS4_NUM_ROTORS];
}TOS4InitState;

/*! \brief Настройки модели
 *
 * По умолчанию все признаки выключены и модель на C++ повторяет
 * Simulink-модель OS4dynamics_01.slx один в один, включая ее
 * особенности. Включая признаки, можно исправить найденные неточности
 * и сравнить поведение.
 * \ingroup data
*/
typedef struct
{
    /*! Умножать гироскопический момент винтов на Ip.
     *  В Simulink множителя нет: M_gyro = [-sum(dir*q*w), sum(dir*p*w), 0]
    */
    bool gyroWithIp;
    /*! Брать угол на конце лопасти из OS4_physical.prop.thetatip.
     *  В Simulink блок "theta_t" (Fcn u(1)) берет первый элемент шины
     *  geometry, а это радиус винта R = 0.075, т.е. угол ~4.3 град.
    */
    bool useThetaTip;
    //! останавливать расчет при касании винтом земли (Stop Simulation)
    bool stopOnCollision;
    //! использовать постоянную команду вместо внешнего входа (Manual Switch "Input Switch")
    bool constControl;
    //! значение постоянной команды на каждый мотор, Н*м (блок Constant [0.1 0.1 0.1 0.1])
    double constTau;
}TOS4Options;

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

    //! x + k*dx (нужно для метода Рунге-Кутты)
    TOS4State addScaled(const TOS4State &dx, double k) const
    {
        TOS4State r;
        r.pos = pos + dx.pos * k;
        r.vel = vel + dx.vel * k;
        r.w   = w   + dx.w   * k;
        r.q   = q   + dx.q   * k;
        for(int i = 0; i < OS4_NUM_ROTORS; i++)
        {
            r.omega[i]   = omega[i]   + dx.omega[i]   * k;
            r.lambda0[i] = lambda0[i] + dx.lambda0[i] * k;
        }
        return r;
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

/*! Выходы модели (порты stateX, stateB, stateQ, aero V, IMU, Motor Sensor
 * и переменные os4sim_* рабочего пространства MATLAB)
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

#endif // GLOBALDATA_H
