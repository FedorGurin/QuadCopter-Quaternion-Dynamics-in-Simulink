#include "os4Vehicle.h"
#include "os4Init.h"
#include "os4AeroValue.h"
#include "os4Gravity.h"
#include "os4Fuselage.h"
#include "os4Propeller.h"
#include "os4Equations.h"
#include "escModel.h"
#include "globalNameID.h"
#include "IMsg.h"
#include <iostream>

using namespace math_func;

OS4Vehicle::OS4Vehicle(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "OS4dynamics_01";
    time      = 0.0;

    // параметры по умолчанию (аналог OS4dynamics_01_init.m)
    physical  = os4PhysicalInit();
    setup     = os4SetupInit();
    input     = os4InputInit();
    options   = os4DefaultOptions();

    // подчиненные модули в порядке расчета
    dynElements.push_back(new OS4AeroValue(ID_OS4AeroValueModel));
    dynElements.push_back(new OS4Gravity  (ID_OS4GravityModel));
    dynElements.push_back(new OS4Fuselage (ID_OS4FuselageModel));
    dynElements.push_back(new OS4Propeller(ID_OS4PropellerModel));
    dynElements.push_back(new OS4Equations(ID_OS4EquationsModel));
    for(auto ele : dynElements)
        addElement(ele);
    // ESC: подчиненный модуль, считается один раз за шаг
    esc = new ESCModel(ID_ESCModel);
    addElement(esc);

    // указываем зависимость
    bindTo(ID_OS4_Aero,       &aero);
    bindTo(ID_OS4_Gravity,    &gravity);
    bindTo(ID_OS4_Propeller,  &prop);
    bindTo(ID_OS4_Derivative, &dxEval);
    // открываем интерфейсы
    share(ID_OS4_Physical,   &physical);
    share(ID_OS4_Options,    &options);
    share(ID_OS4_State,      &xEval);
    share(ID_OS4_Input,      &input);
    share(ID_OS4_Setup,      &setup);
    share(ID_OS4_Output,     &output);
    share(ID_OS4_Time,       &time);
    share(ID_OS4_VehicleObj, this);
}
void OS4Vehicle::setControl(const double tau[OS4_NUM_ROTORS])
{
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
        input.tau[i] = tau[i];
}
void OS4Vehicle::init()
{
    // регистрация параметров в МППМ (как MsgRegMPPM в SimTVRS)
    IRegMPPM *mppm = (IRegMPPM *)findShared(ID_MPPM);
    if(mppm != nullptr)
    {
        mppm->addReq(ptrToReq("OS4_Setup",  &setup));
        mppm->addReq(ptrToReq("OS4_Input",  &input));
        mppm->addReq(ptrToReq("OS4_Output", &output));
    }
    // подчиненные модули (ESC регистрирует ESC_Setup и ESC_Output)
    ICalculateElement::init();
    applySetup();
}
void OS4Vehicle::applySetup()
{
    // начальные условия интеграторов
    x.pos = setup.pos;
    x.vel = setup.vel;
    x.w   = setup.w;
    x.q   = euler321ToQ(setup.euler.x * gradToRad,
                        setup.euler.y * gradToRad,
                        setup.euler.z * gradToRad);
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        x.omega[i] = setup.omega[i];
        double omegaR = setup.omega[i] * physical.prop.rad;
        x.lambda0[i]  = (fabs(omegaR) > 1e-9) ? setup.vInd / omegaR : 0.0;
    }
    time        = 0.0;
    setup.reset = 0;
    esc->restart();
    setStart();
    finite();
}
void OS4Vehicle::derivatives(const TOS4State &state, TOS4State &dx)
{
    // все подчиненные модули считают по состоянию xEval
    xEval = state;
    for(auto ele : dynElements)
        ele->calculate();
    dx = *dxEval;
}
void OS4Vehicle::SDU(const double *in, double *out)
{
    TOS4State state, dx;
    state.fromArray(in);
    derivatives(state, dx);
    dx.toArray(out);
}
void OS4Vehicle::calculate()
{
    // команда перезапуска (например, записанная через МППМ)
    if(setup.reset != 0)
    {
        applySetup();
        return;
    }
    if(isStop())
        return;
    // ESC: напряжение на моторах на этот шаг по газу и текущим оборотам
    if(options.useESC)
    {
        xEval = x;
        esc->calculate();
    }

    double in [OS4_STATE_SIZE];
    double out[OS4_STATE_SIZE];

    // метод Рунге-Кутты 4-го порядка
    x.toArray(in);
    rk4<OS4Vehicle, OS4_STATE_SIZE>(this, in, out, OS4_STATE_SIZE, step);
    x.fromArray(out);
    time += step;
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
