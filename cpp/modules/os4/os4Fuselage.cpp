#include "os4Fuselage.h"
#include "globalNameID.h"

using namespace math_func;

OS4Fuselage::OS4Fuselage(uint32_t idClass):ICalculateElement(idClass)
{
    stringUId = "fuselage aerodynamics";
    // указываем зависимость
    bindTo(ID_OS4_Physical, &physical);
    bindTo(ID_OS4_State,    &state);
    bindTo(ID_OS4_Aero,     &aero);
    // открываем интерфейсы
    share(ID_OS4_Fuselage, &fuselage);
}
void OS4Fuselage::calculate()
{
    TQuaternion q = normalize(state->q);

    // скоростной напор на площадь корпуса
    double qS = 0.5 * aero->rho * aero->Vinf * aero->Vinf * physical->fuselageArea;
    // подсистема "body coefficients"
    double CL = 0.5 * sin(2.0 * aero->theta);
    double CD = 0.65 - 0.45 * cos(aero->theta);

    // сила в скоростной СК: сопротивление против потока, подъемная сила вверх
    TVector3 F_air(-qS * CD, 0.0, -qS * CL);

    // "Air to Body": поворот вокруг оси y на угол theta (Body 2-3-1 to Q, beta = 0)
    TVector3 F1 = bodyToEarth(euler231ToQ(aero->theta, 0.0, 0.0), F_air);
    // поворот вокруг оси z на угол psi
    TVector3 Fb(F1.x * cos(aero->psi) - F1.y * sin(aero->psi),
                F1.x * sin(aero->psi) + F1.y * cos(aero->psi),
                F1.z);

    // "Force Body to Earth w/R"
    fuselage.Fe = bodyToEarth(q, Fb);
    fuselage.Mb = TVector3(0.0, 0.0, 0.0);
}
