#include "os4Init.h"

TOS4Physical os4PhysicalInit()
{
    TOS4Physical physical;

    // инерция
    physical.inertia.mass = 0.65;
    physical.inertia.Ixx  = 7.5e-3;
    physical.inertia.Iyy  = 7.5e-3;
    physical.inertia.Izz  = 1.3e-2;
    physical.inertia.Ixy  = 0.0;
    physical.inertia.Ixz  = 0.0;
    physical.inertia.Iyz  = 0.0;
    physical.inertia.Ip   = 6e-5;

    // винт
    physical.prop.chord    = 0.04;
    physical.prop.rad      = 0.075;
    physical.prop.thetaTip = 12.0 * gradToRad;
    physical.prop.sigma    = 0.054;   // Ablade/Adisk
    physical.prop.a0       = 5.5;     // наклон кривой подъемной силы

    // корпус
    physical.fuselageSize = TVector3(0.08, 0.08, 0.07);
    physical.fuselageArea = physical.fuselageSize.x * physical.fuselageSize.y;

    // расположение винтов (X-схема)
    double al = 0.23;
    physical.armLength   = al;
    physical.rotorPos[0] = TVector3( al / sqrt(2.0),  al / sqrt(2.0), -0.05); // вперед-вправо
    physical.rotorPos[1] = TVector3(-al / sqrt(2.0), -al / sqrt(2.0), -0.05); // назад-влево
    physical.rotorPos[2] = TVector3( al / sqrt(2.0), -al / sqrt(2.0), -0.05); // вперед-влево
    physical.rotorPos[3] = TVector3(-al / sqrt(2.0),  al / sqrt(2.0), -0.05); // назад-вправо
    physical.rotorDir[0] = -1.0;
    physical.rotorDir[1] = -1.0;
    physical.rotorDir[2] =  1.0;
    physical.rotorDir[3] =  1.0;

    // индуктивный поток
    physical.downwashTau = 0.01;
    return physical;
}

TOS4Setup os4SetupInit()
{
    TOS4Setup setup;

    // поступательное движение
    setup.vel = TVector3(0.1, 0.0, 0.0);
    setup.pos = TVector3(0.1, 0.1, -4.0);   // ось z вниз: высота 4 м

    // вращательное движение
    setup.w     = TVector3(0.0, 0.0, 0.0);
    setup.euler = TVector3(0.0,      // крен,   град
                           -30.0,    // тангаж, град
                           0.0);     // курс,   град

    // моторы и индуктивный поток
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
        setup.omega[i] = 2100.0;
    setup.vInd  = 6.0;                // индуктивная скорость, м/с
    setup.reset = 0;
    return setup;
}

TOS4Input os4InputInit()
{
    TOS4Input input;
    for(int i = 0; i < OS4_NUM_ROTORS; i++)
    {
        input.throttle[i] = 0.5;
        input.tau[i]      = 0.1;
    }
    return input;
}

TOS4Options os4DefaultOptions()
{
    TOS4Options opt;
    opt.gyroWithIp      = true;     // исправление: в Simulink false
    opt.useThetaTip     = true;     // исправление: в Simulink false
    opt.stopOnCollision = true;
    opt.useESC          = true;     // в Simulink вход - момент мотора
    return opt;
}
