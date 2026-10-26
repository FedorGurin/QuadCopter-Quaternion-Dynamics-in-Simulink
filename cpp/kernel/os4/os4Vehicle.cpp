#include "os4Vehicle.h"
#include "os4Init.h"
#include "os4AeroValue.h"
#include "os4Gravity.h"
#include "os4Fuselage.h"
#include "os4Propeller.h"
#include "os4Equations.h"
#include "globalNameID.h"
#include <iostream>

using namespace math_func;

OS4Vehicle::OS4Vehicle(uint32_t idClass, double step_):ICalculateElement(idClass, step_)
{
    stringUId = "OS4dynamics_01";
    time      = 0.0;

    // параметры по умолчанию (аналог OS4dynamics_01_init.m)
    physical  = os4PhysicalInit();
    initState = os4InitState(physical);
    options   = os4DefaultOptions();
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
        control[i] = 0.0;

    // подчиненные модули в порядке расчета
    addElement(new OS4AeroValue(ID_OS4AeroValueModel));
    addElement(new OS4Gravity  (ID_OS4GravityModel));
    addElement(new OS4Fuselage (ID_OS4FuselageModel));
    addElement(new OS4Propeller(ID_OS4PropellerModel));
    addElement(new OS4Equations(ID_OS4EquationsModel));

    // указываем зависимость
    bindTo(ID_OS4_Aero,       &aero);
    bindTo(ID_OS4_Gravity,    &gravity);
    bindTo(ID_OS4_Propeller,  &prop);
    bindTo(ID_OS4_Derivative, &dxEval);
    // открываем интерфейсы
    share(ID_OS4_Physical,   &physical);
    share(ID_OS4_Options,    &options);
    share(ID_OS4_State,      &xEval);
    share(ID_OS4_Control,    control);
    share(ID_OS4_Output,     &output);
    share(ID_OS4_Time,       &time);
    share(ID_OS4_VehicleObj, this);
}
void OS4Vehicle::setControl(const double tau[OS4_NUM_ROTORS])
{
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
        control[i] = tau[i];
}
void OS4Vehicle::init()
{
    ICalculateElement::init();

    // начальные условия интеграторов
    x.pos = initState.pos;
    x.vel = initState.vel;
    x.w   = initState.w;
    x.q   = initState.q;
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        x.omega[i]   = initState.omega[i];
        x.lambda0[i] = initState.lambda0[i];
    }
    time = 0.0;
    setStart();
    finite();
}
void OS4Vehicle::derivatives(const TOS4State &state, TOS4State &dx)
{
    // "Input Switch": постоянная команда или внешний вход
    if(options.constControl)
    {
        for(int i = 0; i < OS4_NUM_ROTORS; i++)
            control[i] = options.constTau;
    }
    // все подчиненные модули считают по состоянию xEval
    xEval = state;
    for(auto ele : *returnElements())
        ele->calculate();
    dx = *dxEval;
}
void OS4Vehicle::calculate()
{
    if(isStop())
        return;
    double h = step;
    TOS4State k1, k2, k3, k4;

    // метод Рунге-Кутты 4-го порядка
    derivatives(x,                       k1);
    derivatives(x.addScaled(k1, h / 2.0), k2);
    derivatives(x.addScaled(k2, h / 2.0), k3);
    derivatives(x.addScaled(k3, h),       k4);

    x = x.addScaled(k1, h / 6.0)
         .addScaled(k2, h / 3.0)
         .addScaled(k3, h / 3.0)
         .addScaled(k4, h / 6.0);
    time += h;
}
void OS4Vehicle::finite()
{
    // пересчитать все сигналы по текущему состоянию
    TOS4State dx;
    derivatives(x, dx);

    TQuaternion q = normalize(x.q);

    output.time     = time;
    output.Ve       = x.vel;
    output.Xe       = x.pos;
    output.Vb       = aero->Vb;
    output.pqr      = x.w;
    output.q        = q;
    output.euler    = qToEuler321(q);
    output.aero     = *aero;
    // IMU: гироскоп и акселерометр ("acc to Body" + "IMU_acc_g")
    output.gyro     = x.w;
    output.acc      = earthToBody(q, dx.vel) + gravity->gB;
    // "Vectors": орты корпуса в земной СК
    output.headingX = bodyToEarth(q, TVector3(1.0, 0.0, 0.0));
    output.headingY = bodyToEarth(q, TVector3(0.0, 1.0, 0.0));
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        output.tau[i]   = prop->tau[i];
        output.T[i]     = prop->T[i];
        output.omega[i] = x.omega[i];
    }

    // "ground strike": касание винтом земли
    if(prop->collision && options.stopOnCollision && !isStop())
    {
        std::cout << "Ground Collision!! t = " << time << " c" << std::endl;
        setStop();
    }
}
